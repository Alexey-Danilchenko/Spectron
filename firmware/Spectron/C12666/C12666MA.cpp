/*
 *  C12666MA.cpp - Hamamatsu C12666MA driver for Spectron board.
 *                 This is quite generic in handling spectrometer
 *                 read cycles. The AD7980 16 bit ADC is used to
 *                 read spectrometer output. All ADC interfaces
 *                 are tuned to run as fast as possible on Photon
 *                 hardware (STM32F205) at the price of portability.
 *
 *  Copyright 2015-2020 Alexey Danilchenko, Iliah Borg
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3, or (at your option)
 *  any later version with ADDITION (see below).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, 51 Franklin Street - Fifth Floor, Boston,
 *  MA 02110-1301, USA.
 */

#include "C12666MA.h"
#include <math.h>

// Gain indexes
#define NO_GAIN    0
#define HIGH_GAIN  1

// Mask to set
#define GAIN_BIT_MASK    0x80

// Number of pixels in Hamamatsu C12666 spectrometer
#define SPEC_PIXELS 256

// standard SPI pins
#define SPI_MOSI    A5
#define SPI_MISO    A4
#define SPI_SCK     A3

//
// State flow with single read integration cycle:
//    Lead -> Reset -> Reset2 -> Integration -> Read -> Trail -> Stop
//
enum spec_state_t {
    SPEC_LEAD,
    SPEC_RESET,
    SPEC_RESET2,
    SPEC_INTEGRATION,
    SPEC_READ,
    SPEC_TRAIL,
    SPEC_STOP
};

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//    The following values are calculated and working on Photon only
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// Timer prescaler - this is what CPU counter clock frequency is divided by to get the frequency
//      generally prescaler is calculated as SYSCORECLOCK (60000000 for Photon for base timers)
//      divided by the frequency of the timer counter. For example:
//
//      TIMER_PRESCALER = (SYSCORECLOCK / 1000000) - 1 to get TIM counter clock = 1MHz
//
#define TIMER_PRESCALER   5    // for basic timers at 60MHz it gives 100ns timer unit counter
#define TIMER_US_FACTOR   10   // conversion factor to/from timer units and microseconds

// This determines minimal clock tick duration for spectrometer,
// in above timer units (100ns units). It should be larger than
// ADC conversion times for AD7980.
#define SPEC_CLK_100KHZ   50   // enough for 2 averaging ADC reads with min integration time 10.3 ms
#define SPEC_CLK_58_8KHZ  85   // enough for 4 averaging ADC reads with min integration time 17.51 ms

#ifdef ADC_AVG_4
    // 4 ADC averaging reads
    #define SPEC_CLK_TICK_TIMER  SPEC_CLK_58_8KHZ
#else
    // 2 ADC averaging reads
    #define SPEC_CLK_TICK_TIMER  SPEC_CLK_100KHZ
#endif

// Macro to convert ticks to uSec with rounding to closer
#define ticksToUsec(x) (((x)*SPEC_CLK_TICK_TIMER)/TIMER_US_FACTOR)
#define uSecToTicks(x) (((x)*TIMER_US_FACTOR)/SPEC_CLK_TICK_TIMER)

// C12666MA:
//      Integration time = (INTEG_TICKS + READ_TICKS)/2/frequency
//      Integration time limits from datasheet: 0.01 sec to 10 sec
//      Each read takes 4 CLK cycles; 6 CLK cycles min. after the last read
//           for 256 pixels train is 4*256+6=1030 CLK cycles
//      Numbers below are in ticks: 2 ticks (h/l and l/h) per clock cycle
//      even numbers only!
#define TICKS_PER_PIXEL      8              // Sensor spec - ticks per single pixel readout
#define MAX_INTEG_TIME_US    10000000UL     // 10s maximum integration time
#define DEF_LEAD_TICKS       64             // anything greater than 38 seems OK up to 200KHz clock, room temperature
#define TRAIL_TICKS          12             // anything greater than 2 seems OK up to 200KHz clock, room temperature
#define READ_TICKS           (SPEC_PIXELS*TICKS_PER_PIXEL + TRAIL_TICKS)
#define EXT_TRG_HIGH_TICKS   uSecToTicks(1000)  // duration of ext TRG pin high signal - 1mSec

// Integration ticks set to minimum by default - integration time is
// formed by INTEG_TICKS + READ_TICKS
static uint32_t INTEG_TICKS = 0;

// Lead ticks - this can be larger to accomodate external trigger delay
static uint32_t LEAD_TICKS = DEF_LEAD_TICKS;

// Timing arrays and correspinding bitmasks
#define ST_BIT         1    // ST pin state bitmask
#define READY_BIT      2    // data ready state (for ADC conversion to start)

static uint8_t specRead[READ_TICKS];

// high value bitmask for CLK
#define CLK_HIGH       1

// ADC conversion delay as per AD7980 spec sheet - CS mode-3 wire without Busy ind
static const uint32_t adcConvTimeTicks  = (71*System.ticksPerMicrosecond())/100;

// spectrometer states and trigger variables
static volatile bool         timerOn = false;
static volatile spec_state_t specState;
static volatile uint16_t     specReadCycleCounter = 0; // reading cycles counter
static volatile bool         specDataReady = false;    // trigger for ADC conversion
static uint8_t               specCLK = LOW;            // current clock pin state
static uint8_t               specST  = LOW;            // current ST pin state
static uint32_t              specCounter = 0;          // counter
static uint32_t              extTRGCounter = 0;        // ext trigger counter
static uint32_t* volatile    specData = 0;             // pointer to current data for ADC reads
static uint16_t* volatile    specDataCounter = 0;      // pointer to current data for ADC reads counter

// spectrometer pins used by timer
uint8_t adcPinCNV   = NO_PIN;
uint8_t specPinCLK  = NO_PIN;
uint8_t specPinST   = NO_PIN;
uint8_t extPinTRG   = NO_PIN;
uint8_t extPinLIGHT = NO_PIN;

// spectrometer trigger pin - hardware access
uint16_t specPinTRG = 0;    STM32_Pin_Info* specPinTRG_Info = 0;

// pin set/read macros
#define specTrgSoftInterrupt() EXTI->SWIER = specPinTRG

// set pin fast
#define pinSetVal(pin,val) if (val) pinSetFast(pin); else pinResetFast(pin)

// Static internal sensor readings arrays - these hold
// aggregated sensor measurements and measurement counts
uint32_t data[SPEC_PIXELS];
uint16_t dataCounts[SPEC_PIXELS];

// Time to usec multipliers
static const double usecToTimeUnits[MAX_TIME_UNITS] = { 1.0, 1/1000.0, 1/1000000.0 };

// ------------------------------
//   Hardware specific routines
// ------------------------------
#include "gpio_hal.h"
#include "pinmap_impl.h"
#include "stm32f2xx.h"

// IRQ numbers for all 16 GPIO pin levels
static const uint8_t GPIO_IRQn[] = {
    EXTI0_IRQn,     //0
    EXTI1_IRQn,     //1
    EXTI2_IRQn,     //2
    EXTI3_IRQn,     //3
    EXTI4_IRQn,     //4
    EXTI9_5_IRQn,   //5
    EXTI9_5_IRQn,   //6
    EXTI9_5_IRQn,   //7
    EXTI9_5_IRQn,   //8
    EXTI9_5_IRQn,   //9
    EXTI15_10_IRQn, //10
    EXTI15_10_IRQn, //11
    EXTI15_10_IRQn, //12
    EXTI15_10_IRQn, //13
    EXTI15_10_IRQn, //14
    EXTI15_10_IRQn  //15
};

typedef void (*EXT_IRQ_Handler)(void);

// existing IRQ handler
EXT_IRQ_Handler sysIrqHandler = 0;

// this is needed because wiring undefines SPIn definitions
#define SPI_BASE         ((SPI_TypeDef *) SPI1_BASE)

// SPI registers Masks
#define CR1_CLEAR_MASK   ((uint16_t)0x3040)

// initialise ADC (AD7980) and setup SPI
inline void startADC(uint8_t adc_cnv_pin)
{
    // disable whatever else might have SPI enabled via HAL
    SPI.end();

    // Enable SPI Clock
    RCC->APB2ENR |= RCC_APB2Periph_SPI1;

    // Connect SPI pins to AF
    STM32_Pin_Info* PIN_MAP = HAL_Pin_Map();
    GPIO_PinAFConfig(PIN_MAP[SCK].gpio_peripheral,  PIN_MAP[SCK].gpio_pin_source,  GPIO_AF_SPI1);
    GPIO_PinAFConfig(PIN_MAP[MISO].gpio_peripheral, PIN_MAP[MISO].gpio_pin_source, GPIO_AF_SPI1);
    GPIO_PinAFConfig(PIN_MAP[MOSI].gpio_peripheral, PIN_MAP[MOSI].gpio_pin_source, GPIO_AF_SPI1);

    HAL_Pin_Mode(SCK,  AF_OUTPUT_PUSHPULL);
    HAL_Pin_Mode(MISO, AF_OUTPUT_PUSHPULL);
    HAL_Pin_Mode(MOSI, AF_OUTPUT_PUSHPULL);

    // Ensure that there is no glitch on SS pin
    PIN_MAP[adc_cnv_pin].gpio_peripheral->BSRRL = PIN_MAP[adc_cnv_pin].gpio_pin;
    HAL_Pin_Mode(adc_cnv_pin, OUTPUT);

    // Get the SPIx CR1 value */
    uint16_t tmpreg = SPI_BASE->CR1;

    // Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, MSTR, CPOL and CPHA bits
    tmpreg &= CR1_CLEAR_MASK;

    tmpreg |= SPI_Direction_2Lines_RxOnly |
              SPI_Mode_Master |
              SPI_DataSize_16b |
              SPI_BaudRatePrescaler_2 | // absolute max for SPI1 = 30Mhz (with APB2 at its allowed maximum 60Mhz)
              SPI_NSS_Soft |
              SPI_CPOL_Low | SPI_CPHA_1Edge |  // SPI_MODE0
              SPI_FirstBit_MSB;

    // Write to SPIx CR1
    SPI_BASE->CR1 = tmpreg;

    // Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register)
    SPI_BASE->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);

    // CRC polynomial
    SPI_BASE->CRCPR = 7;

    // set conversion pin low
    pinResetFast(adc_cnv_pin);
}

// force inlining
inline void readADC(uint32_t* data, uint16_t* dataCounter) __attribute__((always_inline));

// function to read 16 bit value from ADC7980
inline void readADC(uint32_t* data, uint16_t* dataCounter)
{
    // 1st read
    // initiate conversion and wait for max conversion time
    pinSetFast(adcPinCNV);
    System.ticksDelay(adcConvTimeTicks);
    pinResetFast(adcPinCNV);

    // SPI enable
    SPI_BASE->CR1 |= SPI_CR1_SPE;

    // Wait for SPI data reception
    while ((SPI_BASE->SR & SPI_I2S_FLAG_RXNE) == 0) ;

    // Read SPI received data into local vars (registers)
    uint32_t rData = SPI_BASE->DR;
    uint16_t rCount = 1;

    // disable
    SPI_BASE->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);

    // 2nd read
    // initiate conversion and wait for max conversion time
    if (specDataReady)
    {
        pinSetFast(adcPinCNV);
        System.ticksDelay(adcConvTimeTicks);
        pinResetFast(adcPinCNV);

        // SPI enable
        SPI_BASE->CR1 |= SPI_CR1_SPE;

        // Wait for SPI data reception
        while ((SPI_BASE->SR & SPI_I2S_FLAG_RXNE) == 0) ;

        // Read SPI received data
        if (specDataReady)
        {
            rData += SPI_BASE->DR;
            ++rCount;
        }

        // disable
        SPI_BASE->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);

#ifdef ADC_AVG_4
        // 3rd read
        // initiate conversion and wait for max conversion time
        if (specDataReady)
        {
            pinSetFast(adcPinCNV);
            System.ticksDelay(adcConvTimeTicks);
            pinResetFast(adcPinCNV);

            // SPI enable
            SPI_BASE->CR1 |= SPI_CR1_SPE;

            // Wait for SPI data reception
            while (SPI_BASE->SR & SPI_I2S_FLAG_RXNE == 0) ;

            // Read SPI received data
            if (specDataReady)
            {
                rData += SPI_BASE->DR;
                ++rCount;
            }

            // disable
            SPI_BASE->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);

            // 4th read
            // initiate conversion and wait for max conversion time
            if (specDataReady)
            {
                pinSetFast(adcPinCNV);
                System.ticksDelay(adcConvTimeTicks);
                pinResetFast(adcPinCNV);

                // SPI enable
                SPI_BASE->CR1 |= SPI_CR1_SPE;

                // Wait for SPI data reception
                while (SPI_BASE->SR & SPI_I2S_FLAG_RXNE == 0) ;

                // Read SPI received data
                if (specDataReady)
                {
                    rData += SPI_BASE->DR;
                    ++rCount;
                }

                // disable
                SPI_BASE->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);
            }
        }
#endif
    }

    // update data and counter
    *data += rData;
    *dataCounter += rCount;
}

// deinitialise ADC SPI
inline void endADC()
{
    // Enable SPI1 reset state
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);
    // Release SPI1 from reset state
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);
}

// init timer structures
void initSpecTimerData()
{
    // initialize timing arrays for inactive levels for ST
    // ST  inactive High
    for (int i=0; i<READ_TICKS; i++)
        specRead[i] = 0 | ST_BIT;

    // filling ST
    specRead[0] &= ~ST_BIT;
    specRead[1] &= ~ST_BIT;

    // filling READY
    for (int i=7; i<SPEC_PIXELS*TICKS_PER_PIXEL+1; i+=TICKS_PER_PIXEL)
        specRead[i] |= READY_BIT;
}

// --------------------------------------------------
//   Timer and spectrometer clock handling routines
// --------------------------------------------------
// TRG pin handling interrupt
void spectroTRGInterrupt(void)
{
    if (EXTI->PR & specPinTRG)
    {
        EXTI->PR = specPinTRG;

        if (specDataReady && specData)
            readADC(specData++, specDataCounter++);
    }

    // call system interrupt
    if (sysIrqHandler)
        sysIrqHandler();
}

// Spectrometer timer interrupt call. All is controlled by a state machine:
//     Lead -> Reset -> Reset2 -> Integration -> Read -> Trail -> Stop
//
// The timer basically triggers clock, sets the states from pre-populated array
// and advances array pointers to the data being read
void spectroClockInterrupt(void)
{
    // HAL version of this would be
    //   if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    if ((TIM7->SR & TIM_IT_Update) && (TIM7->DIER & TIM_IT_Update))
    {
        // HAL version of this would be
        //   TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        TIM7->SR = (uint16_t)~TIM_IT_Update;

        // only proceed if timer is enabled
        if (!timerOn)
            return;

        // write CLK and ST - immediately
        pinSetVal(specPinCLK, specCLK);
        pinSetVal(specPinST,  specST);

        // flip CLK
        specCLK ^= CLK_HIGH;

        // state machine
        switch (specState) {
            case SPEC_LEAD:
                ++specCounter;
                if (specCounter == LEAD_TICKS) {
                    specCounter = 0;
                    specState = SPEC_RESET;
                    specST  = specRead[0] & ST_BIT;
                }
                break;

            case SPEC_RESET:
                ++specCounter;
                if (specCounter == READ_TICKS) {
                    // second reset cycle
                    specCounter = 0;
                    specState = SPEC_RESET2;
                    // enable external light if defined
                    if (extPinLIGHT != NO_PIN)
                        pinSetFast(extPinLIGHT);
                }
                specST  = specRead[specCounter] & ST_BIT;
                break;

            case SPEC_RESET2:
                ++specCounter;
                if (specCounter == READ_TICKS) {
                    specCounter = 0;
                    if (INTEG_TICKS)
                        specState = SPEC_INTEGRATION;
                    else {
                        specState = SPEC_READ;
                        specST  = specRead[0] & ST_BIT;
                    }
                }
                else
                    specST  = specRead[specCounter] & ST_BIT;
                break;

            case SPEC_INTEGRATION:
                ++specCounter;
                if (specCounter == INTEG_TICKS) {
                    specCounter = 0;
                    specState = SPEC_READ;
                    specST  = specRead[0] & ST_BIT;
                }
                break;

            case SPEC_READ:
                if (specRead[specCounter] & READY_BIT)
                {
                    specDataReady = true;
                    specTrgSoftInterrupt();
                }
                else
                    specDataReady = false;
                ++specCounter;
                if (specCounter == READ_TICKS) {
                    --specReadCycleCounter;
                    if (specReadCycleCounter) {
                        // initialise data variables and start another cycle
                        specData = data;
                        specDataCounter = dataCounts;
                        specCounter = 0;
                        if (INTEG_TICKS)
                            specState = SPEC_INTEGRATION;
                        else {
                            specState = SPEC_READ;
                            specST  = specRead[0] & ST_BIT;
                        }
                    } else {
                        specCounter = 0;
                        specState = SPEC_TRAIL;
                        specData = 0;
                        specDataCounter = 0;
                    }
                }
                else
                    specST  = specRead[specCounter] & ST_BIT;
                break;

            case SPEC_TRAIL:
                ++specCounter;
                 if (specCounter == TRAIL_TICKS) {
                    specCounter = 0;
                    specState = SPEC_STOP;
                    specCLK = LOW;
                    specST = LOW;
                    // disable external light if defined
                    if (extPinLIGHT != NO_PIN)
                        pinResetFast(extPinLIGHT);
                }
                break;

            case SPEC_STOP:
            default:
                specCLK = LOW;
                break;
        }

        // process trigger
        if (extTRGCounter)
        {
            --extTRGCounter;
            if (extTRGCounter == EXT_TRG_HIGH_TICKS)
                pinSetFast(extPinTRG);
            else if (extTRGCounter == 0)
                pinResetFast(extPinTRG);
        }
    }
}

// start active timer
void startSpecTimer(uint32_t extTrgDelayTicks, bool doExtTriggering)
{
    TIM_TimeBaseInitTypeDef timerInit = {0};
    NVIC_InitTypeDef nvicInit = {0};

    // set in timer guard
    if (timerOn)
        return;

    timerOn = true;

    // init state
    specCounter = 0;

    // init lead and ext counter states
    if (doExtTriggering && extPinTRG != NO_PIN && extTrgDelayTicks > 0)
    {
        // making it even
        extTrgDelayTicks &= ~1;

        // calculate LEAD_TICKS
        if (extTrgDelayTicks > DEF_LEAD_TICKS + READ_TICKS + READ_TICKS)
            LEAD_TICKS = extTrgDelayTicks - READ_TICKS - READ_TICKS;
        else
            LEAD_TICKS = DEF_LEAD_TICKS;

        // set trigger cycles
        extTRGCounter = EXT_TRG_HIGH_TICKS + LEAD_TICKS + READ_TICKS + READ_TICKS - extTrgDelayTicks;
    }
    else
    {
        extTRGCounter = 0;
        LEAD_TICKS = DEF_LEAD_TICKS;
    }

    specState = SPEC_LEAD;
    specDataReady = false;

    // initial and next values of CLK and ST
    specST  = HIGH;
    specCLK = CLK_HIGH;
    pinResetFast(specPinCLK);  // CLK initially low
    pinSetFast(specPinST);     // ST  initially high

    // reset triggers
    if (extPinTRG != NO_PIN)
        pinResetFast(extPinTRG);   // Ext trigger initially low
    if (extPinLIGHT != NO_PIN)
        pinResetFast(extPinLIGHT); // Ext light trigger initially low

    // enable TIM7 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    // enable timer IRQ
    nvicInit.NVIC_IRQChannel                   = TIM7_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
    nvicInit.NVIC_IRQChannelSubPriority        = 0;
    nvicInit.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvicInit);

    // setup timer
    timerInit.TIM_Prescaler         = TIMER_PRESCALER;
    timerInit.TIM_CounterMode       = TIM_CounterMode_Up;
    timerInit.TIM_Period            = SPEC_CLK_TICK_TIMER;
    timerInit.TIM_ClockDivision     = TIM_CKD_DIV1;
    timerInit.TIM_RepetitionCounter = 0;

    // enable timer
    TIM_TimeBaseInit(TIM7, &timerInit);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM7, ENABLE);

    // setup spec TRG pin interrupts
    uint8_t trgPortNumber = 0;  // port A by default

    // clear pending EXTI interrupt flag for the TRG pin
    EXTI->PR = specPinTRG;

    // set the port number
    if (specPinTRG_Info->gpio_peripheral == GPIOB)
        trgPortNumber = 1;
    else if (specPinTRG_Info->gpio_peripheral == GPIOC)
        trgPortNumber = 2;
    else if (specPinTRG_Info->gpio_peripheral == GPIOD)
        trgPortNumber = 3;

    // connect EXTI Line to TRG pin
    SYSCFG_EXTILineConfig(trgPortNumber, specPinTRG_Info->gpio_pin_source);

    // enable TRG pin - software interrupt
    EXTI->IMR |= specPinTRG;    // enable interrupt

    // enable spec TRG pin IRQ
    nvicInit.NVIC_IRQChannel                   = GPIO_IRQn[specPinTRG_Info->gpio_pin_source];
    nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
    nvicInit.NVIC_IRQChannelSubPriority        = 0;
    nvicInit.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvicInit);
}

// stop active timer
void stopSpecTimer()
{
    NVIC_InitTypeDef nvicInit = {0};

    // disable timer
    TIM_Cmd(TIM7, DISABLE);

    // disable timer IRQ
    nvicInit.NVIC_IRQChannel    = TIM7_IRQn;
    nvicInit.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&nvicInit);

    // disable timer peripheral
    TIM_DeInit(TIM7);

    // disable TRG pin interrupts
    EXTI->PR = specPinTRG;      // clear pending
    EXTI->IMR &= ~specPinTRG;   // mask interrupt
    EXTI->EMR &= ~specPinTRG;   // mask event

    // disable NVIC IRQ line if it is not shared
    if (sysIrqHandler == 0)
    {
        nvicInit.NVIC_IRQChannel    = GPIO_IRQn[specPinTRG_Info->gpio_pin_source];
        nvicInit.NVIC_IRQChannelCmd = DISABLE;
        NVIC_Init(&nvicInit);
    }

    // reset pins
    pinResetFast(specPinCLK);
    pinResetFast(specPinST);
    if (extPinTRG != NO_PIN)
        pinResetFast(extPinTRG);
    if (extPinLIGHT != NO_PIN)
        pinResetFast(extPinLIGHT);

    timerOn = false;
}

// Constructor
C12666MA::C12666MA(uint8_t spec_gain, uint8_t spec_eos, uint8_t spec_trg, uint8_t spec_clk,
             uint8_t spec_st, uint8_t adc_ref_sel1, uint8_t adc_ref_sel2, uint8_t adc_cnv,
             uint8_t ext_trg, uint8_t ext_trg_ls,
             const double *wvCalibration, const int baseEEPROM)
        : CSpectrometer(adc_ref_sel1, adc_ref_sel2, wvCalibration, baseEEPROM),
          spec_gain_(spec_gain), spec_eos_(spec_eos), spec_trg_(spec_trg),
          spec_clk_(spec_clk), spec_st_(spec_st), adc_cnv_(adc_cnv), ext_trg_(ext_trg),
          ext_trg_ls_(ext_trg_ls), extTrgMeasDelayTicks_(0), gainFactor_(1.0)
{
    rawMeas_ = data;
    rawMeasCounts_ = dataCounts;

    timerOn = false;
    specState = SPEC_STOP;
    specDataReady = false;
    INTEG_TICKS = 0;

    // initialise timing data
    initSpecTimerData();
}

// Destructor
C12666MA::~C12666MA()
{
}

// Setup methods
bool C12666MA::begin()
{
    bool success = CSpectrometer::begin();

    // Initialise integration time
    // only read saved spectrometer state it if enabled
    if (baseEEPROM_>=0)
    {
        EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_GAIN_FACTOR],      gainFactor_);
        EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_TRG_MEAS_DELAY],   extTrgMeasDelayTicks_);
        EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_INTEGRATION_TIME], INTEG_TICKS);

        if (isnan(gainFactor_) || gainFactor_ < 1.0 || gainFactor_ > 100.0)
            gainFactor_ = 1.0;
    }

    // checks and set defaults
    if (extTrgMeasDelayTicks_ < EXT_TRG_HIGH_TICKS
        || extTrgMeasDelayTicks_ < uSecToTicks(50000000))  // 5 sec as top limit
        extTrgMeasDelayTicks_ = 0;

    if (INTEG_TICKS > uSecToTicks(MAX_INTEG_TIME_US))
        INTEG_TICKS = 0;

    // Setup pins
    pinMode(adc_cnv_,   OUTPUT);
    pinMode(SPI_MOSI,   OUTPUT);
    pinMode(SPI_MISO,   INPUT);
    pinMode(SPI_SCK,    OUTPUT);
    pinMode(spec_trg_,  INPUT_PULLDOWN); // used for soft interrupt only
    pinMode(spec_eos_,  INPUT);
    pinMode(spec_gain_, OUTPUT);
    pinMode(spec_st_,   OUTPUT);
    pinMode(spec_clk_,  OUTPUT);

    if (ext_trg_ != NO_PIN)
    {
        pinMode(ext_trg_,  OUTPUT);
        pinResetFast(ext_trg_);
    }

    if (ext_trg_ls_ != NO_PIN)
    {
        pinMode(ext_trg_ls_, OUTPUT);
        pinResetFast(ext_trg_ls_);
    }

    // setup hardware and fixed pins
    STM32_Pin_Info* PIN_MAP = HAL_Pin_Map();
    // TRG - input
    specPinTRG_Info = &PIN_MAP[spec_trg_];
    specPinTRG      = PIN_MAP[spec_trg_].gpio_pin;

    // reset everything
    pinResetFast(adc_cnv_);
    pinResetFast(SPI_MOSI);
    pinResetFast(SPI_SCK);
    pinResetFast(spec_st_);
    pinResetFast(spec_clk_);

    // Attach update interrupt for TIM7 and TRG pin
    // HAL version of this would be
    //   attachSystemInterrupt(SysInterrupt_TIM7_IRQ, spectroClockInterrupt);
    //   attachInterrupt(spec_trg_, spectroTRGInterrupt, RAISING, 1);
    const unsigned TIM7Index = 71;
    uint8_t trgPinSource = specPinTRG_Info->gpio_pin_source;
    uint8_t trgISRIndex = GPIO_IRQn[trgPinSource] + 0x10;
    uint32_t* isrs = (uint32_t*)(SCB->VTOR);

    // disable interrupts
    int is = __get_PRIMASK();
    __disable_irq();

    // store the system interrupt if TRG pin ISR is shared across several pins
    if (GPIO_IRQn[trgPinSource] == EXTI9_5_IRQn ||
        GPIO_IRQn[trgPinSource] == EXTI15_10_IRQn)
        sysIrqHandler = (EXT_IRQ_Handler)isrs[trgISRIndex];

    // override TIM7 and TRG pin interrupts
    isrs[TIM7Index]   = (uint32_t)spectroClockInterrupt;
    isrs[trgISRIndex] = (uint32_t)spectroTRGInterrupt;

    // enable interrupts
    if ((is & 1) == 0) {
        __enable_irq();
    }

    // set internal pins
    specPinCLK  = spec_clk_;
    specPinST   = spec_st_;
    adcPinCNV   = adc_cnv_;
    extPinTRG   = NO_PIN;
    extPinLIGHT = NO_PIN;

    return success;
}

// This routine to initiate and read spectrometer measurement data
void C12666MA::readSpectrometer(double measTimeUs,
                                bool doExtTriggering,
                                bool doLightTriggering)
{
    // no action if timer is on or in measurement
    if (timerOn)
        return;

    // number of reading cycles to do
    uint32_t readCycles = 1;
    if (measTimeUs > 0)
    {
        readCycles = measTimeUs / getIntTime(T_USEC);
        if (readCycles == 0)
            readCycles = 1;

        if (readCycles<<1 > UINT16_MAX)
            readCycles = UINT16_MAX>>1;
    }

    // set read cycles counter
    specReadCycleCounter = readCycles;

    // initialise variables
    specData = data;
    specDataCounter = dataCounts;

    // init counts and data
    for (int i=0; i<SPEC_PIXELS; i++)
    {
        data[i] = 0UL;
        dataCounts[i] = 0;
    }

    // initialise trigger pins if triggering is enabled
    if (doExtTriggering)
        extPinTRG   = ext_trg_;
    if (doLightTriggering)
        extPinLIGHT = ext_trg_ls_;

    // init ADC
    startADC(adc_cnv_);

    // initiate the timer
    startSpecTimer(extTrgMeasDelayTicks_, doExtTriggering);

    // loop until stop
    while (specState != SPEC_STOP)
        ;

    // stop the timer and cleanup
    stopSpecTimer();
    endADC();

    // reset trigger pins
    extPinTRG   = NO_PIN;
    extPinLIGHT = NO_PIN;
}

// Sensor constraints from Hamamatsu C12666MA spec sheet
bool C12666MA::getSensorConstraint(sensor_constraint_t sensConstraint, void* data)
{
    switch (sensConstraint) {
        case MAX_PIXELS:
            *((int*)data) = SPEC_PIXELS;
            return true;

        case MIN_WAVELENGTH:
            *((int*)data) = 340;
            return true;

        case MAX_WAVELENGTH:
            *((int*)data) = 780;
            return true;

        case MIN_SAT_VOLTAGE:
            *((float*)data) = 1.4;
            return true;

        case MAX_SAT_VOLTAGE:
            *((float*)data) = 2.7;
            return true;

        case MIN_SAT_VOLTAGE_HIGH_GAIN:
            *((float*)data) = 2.3;
            return true;

        case MAX_SAT_VOLTAGE_HIGH_GAIN:
            *((float*)data) = 4.0;
            return true;

        case MIN_INT_TIME_US:
            *((double*)data) = ticksToUsec((double)READ_TICKS);
            return true;

        case MAX_INT_TIME_US:
            *((double*)data) = MAX_INTEG_TIME_US;
            return true;

        case MAX_BIAS_INT_TIME_US:
            *((double*)data) = 500000; // 500 mSec
            return true;
    }

    return false;
}

// Set spectrometer gain to low or high
void C12666MA::setHighGain(bool highGain, bool saveState)
{
    // no action in measurement and called externally with saving state
    if (measuringData_ && saveState)
        return;

    if (highGain)
    {
        specState_ |= GAIN_BIT_MASK;
        pinSetFast(spec_gain_);
    }
    else
    {
        specState_ &= ~GAIN_BIT_MASK;
        pinResetFast(spec_gain_);
    }

    if (saveState && baseEEPROM_>=0)
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_STATE], specState_);

    // delay to stabilise the changes
    delay(100);
}

// Hardware specific - calibrates/measures gain factor or sets it explicitly.
// If supplied gainFactor < 0 then perform automatic measurement, else set
// the gain factor explicitly
// This should be performed after measuring saturation, bias and linearisation
bool C12666MA::calibrateGain(double gainFactor, TProgressFun progress)
{
    bool success = true;

    // no action if in measurement
    if (measuringData_)
        return false;

    if (gainFactor < 0) // measure automatically
    {
        // save current state
        uint8_t savedSpecState_ = specState_;
        double  savedIntTimeUs  = getIntTime(T_USEC);

        // reset all blacks since we are going to tune measurement
        resetBlackLevels();

        // set high gain and use ADC to cover saturation voltage
        setHighGain(true, false);
        setAdcRefInternal(ADC_AUTO);

        // delay to stabilise the changes
        delay(50);

        // allocate data
        float* measData   = new float[2*rangePixels_];
        float* hgMeasData = measData + rangePixels_;

        // try to make shortest reading
        setIntTimeInternal(0.0, false);
        readSpectrometer(-1, false, false);
        float stddev = 0.0;
        float maxMeasuredVoltage = processMeasurement(hgMeasData, false, &stddev);

        // should not overshoot saturation at minimum integration time
        success = getSatVoltage() > maxMeasuredVoltage;

        // get the limits
        double maxIntTimeUs = 0.0;
        getSensorConstraint(MAX_INT_TIME_US, &maxIntTimeUs);

        // target voltage is least 75% of saturation level
        float targetVoltage = getSatVoltage()*0.75;

        // min useful signal limit - stddev at least 200 ADC units
        float stddevMin = 200*getAdcRefVoltage()/ADC_MAX_VALUE;

        bool stillGoing = maxMeasuredVoltage < targetVoltage;
        while (stillGoing && success)
        {
            double curIntTimeUs = getIntTime(T_USEC);

            // calculate new integration time
            if (stddevMin > stddev)
                // too little exposure to apply ratio - double it
                curIntTimeUs *= 2;
            else
                curIntTimeUs *= targetVoltage/maxMeasuredVoltage;

            if (curIntTimeUs > maxIntTimeUs)
                curIntTimeUs = maxIntTimeUs;

            setIntTimeInternal(curIntTimeUs, false);

            // do new reading
            readSpectrometer(-1, false, false);
            maxMeasuredVoltage = processMeasurement(hgMeasData, false, &stddev);

            // check exit conditions
            if (maxMeasuredVoltage >= targetVoltage)
                stillGoing = false;
            else if (curIntTimeUs >= maxIntTimeUs && maxMeasuredVoltage < targetVoltage)
                stillGoing = false;
        }

        // check success of the operation
        success = success && maxMeasuredVoltage >= targetVoltage*0.99;
        if (success)
        {
            // now do accumulative reading for 0.5 sec or int.time x 4 - whichever is longer
            readSpectrometer(std::max(getIntTime(T_USEC)*4.1, 500000.0), false, false);
            processMeasurement(hgMeasData);

            // repeat accumulative reading for no gain
            setHighGain(false, false);
            setAdcRefInternal(ADC_AUTO);
            readSpectrometer(std::max(getIntTime(T_USEC)*4.1, 500000.0), false, false);
            processMeasurement(measData);

            // go through the selected pixel range and average the gain factor for
            // pixels > 70% of target voltage from linearised data
            gainFactor_ = 0.0;
            float lowerBound = targetVoltage*0.7;
            int count = 0;
            for (int i=0; i<rangePixels_; ++i)
                if (hgMeasData[i]>=lowerBound)
                    gainFactor_ +=
                        (getLinearisedMeas(i, hgMeasData, true) /
                         getLinearisedMeas(i, measData, false) - gainFactor_) / (++count);
        }

        // deallocate
        delete[] measData;

        // restore state
        specState_ = savedSpecState_;
        setAdcRefInternal(getAdcReference());
        setHighGain(getHighGain(), false);
        setIntTimeInternal(savedIntTimeUs, false);

        measuringData_ = false;
    }
    else if (gainFactor >= 1.0 && gainFactor < 100.0)
        gainFactor_ = gainFactor;
    else
        success = false;

    if (success && baseEEPROM_>=0)
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_GAIN_FACTOR], gainFactor_);

    return success;
}

// Return whether high gain is enabled for given spectrometer state if spectrometer
// supports gain. By default gain is not supported
bool C12666MA::getHighGain(bool lastMeas)
{
    return (lastMeas ? lastMeasSpecState_ : specState_) & GAIN_BIT_MASK;
}

// Hardware specific sets integration time in microseconds for a single
// measurement cycle.
//
// NOTE: if not within sensor allowed boundaries then it will be set to
//       closest minimum or maximum allowed value
// NOTE: internally time is measured in clock cycles so real integration
//       time will be aligned to the clock cycle boundaries
void C12666MA::setIntTimeInternal(double timeUs, bool saveState)
{
    // no action if timer is on or in measurement
    if (timerOn)
        return;

    double intTimeTicks = uSecToTicks(timeUs);

    // set minumum and maximum integration times
    double minIntTimeTicks = READ_TICKS;
    double maxIntTimeTicks = uSecToTicks((double)MAX_INTEG_TIME_US);

    if (intTimeTicks < minIntTimeTicks)
        intTimeTicks = minIntTimeTicks;

    if (intTimeTicks > maxIntTimeTicks)
        intTimeTicks = maxIntTimeTicks;

    // now determine integration ticks
    INTEG_TICKS = intTimeTicks-READ_TICKS;

    // even up INTEG_TICKS
    ++INTEG_TICKS;
    INTEG_TICKS &= ~1;

    if (saveState && baseEEPROM_>=0)
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_INTEGRATION_TIME], INTEG_TICKS);
}

// Retrieve currently set integration time
double C12666MA::getIntTime(time_units_t units)
{
    return ticksToUsec((double)(INTEG_TICKS+READ_TICKS))*usecToTimeUnits[units];
}

// Hardware specific sets the external trigger to measurement delay time.
// This defines time interval in uSec that offsets external trigger from
// the measurement. I.e. external trigger is raised and after this delay
// the integration and measurement starts.
//
// Specifying -1 as  delay will disable the external trigger
void C12666MA::setExtTrgMeasDelayInternal(double delayUs, bool saveState)
{
    // no action if timer is on or in measurement
    if (timerOn)
        return;

    if (delayUs < 0)
        extTrgMeasDelayTicks_ = 0;
    else
    {
        extTrgMeasDelayTicks_ = uSecToTicks(delayUs);

        // cannot be less than extr trigger high holding cycles
        if (extTrgMeasDelayTicks_ < EXT_TRG_HIGH_TICKS)
            extTrgMeasDelayTicks_ = EXT_TRG_HIGH_TICKS;

        ++extTrgMeasDelayTicks_;
        extTrgMeasDelayTicks_ &= ~1;
    }

    if (saveState && baseEEPROM_>=0)
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_TRG_MEAS_DELAY], extTrgMeasDelayTicks_);
}

// Return trigger measurement delay in specified units
double C12666MA::getExtTrgMeasDelay(time_units_t units)
{
    return extTrgMeasDelayTicks_
                ? ticksToUsec((double)extTrgMeasDelayTicks_)*usecToTimeUnits[units]
                : -1;
}
