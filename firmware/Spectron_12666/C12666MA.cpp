/*
 *  C12666MA.cpp - Hamamatsu C12666MA driver for Spectron board.
 *                 This is quite generic in handling spectrometer
 *                 read cycles. The AD7980 16 bit ADC is used to
 *                 read spectrometer output. All ADC interfaces
 *                 are tuned to run as fast as possible on Photon
 *                 hardware (STM32F205) at the price of portability.
 *
 *  Copyright 2015-2019 Alexey Danilchenko, Iliah Borg
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

// standard SPI pins
#define SPI_MOSI    A5
#define SPI_MISO    A4
#define SPI_SCK     A3

// EEPROM addresses
#define EEPROM_CALIBRATION_COEF_1     EEPROM_C12666_BASE_ADDR
#define EEPROM_CALIBRATION_COEF_2     EEPROM_C12666_BASE_ADDR+8
#define EEPROM_CALIBRATION_COEF_3     EEPROM_C12666_BASE_ADDR+16
#define EEPROM_CALIBRATION_COEF_4     EEPROM_C12666_BASE_ADDR+24
#define EEPROM_CALIBRATION_COEF_5     EEPROM_C12666_BASE_ADDR+32
#define EEPROM_CALIBRATION_COEF_6     EEPROM_C12666_BASE_ADDR+40
#define EEPROM_GAIN_ADDR              EEPROM_C12666_BASE_ADDR+48
#define EEPROM_ADC_REF_ADDR           EEPROM_C12666_BASE_ADDR+52
#define EEPROM_MEASURE_TYPE_ADDR      EEPROM_C12666_BASE_ADDR+56
#define EEPROM_INTEGRATION_TIME       EEPROM_C12666_BASE_ADDR+60
#define EEPROM_TRG_MEAS_DELAY         EEPROM_C12666_BASE_ADDR+64
#define EEPROM_SAT_VOLTAGE_HIGH_GAIN  EEPROM_C12666_BASE_ADDR+68
#define EEPROM_SAT_VOLTAGE_NO_GAIN    EEPROM_C12666_BASE_ADDR+72
#define EEPROM_MIN_BLACK_VOLTAGE      EEPROM_C12666_BASE_ADDR+76
#define EEPROM_SPEC_RANGE_MIN         EEPROM_C12666_BASE_ADDR+80
#define EEPROM_SPEC_RANGE_MAX         EEPROM_C12666_BASE_ADDR+84
#define EEPROM_NORM_COEF_ARRAY        EEPROM_C12666_BASE_ADDR+88

// Saturation voltage limits from Hamamatsu C12666MA spec sheet
#define MIN_SAT_VOLTAGE_HIGH_GAIN  2.3
#define MAX_SAT_VOLTAGE_HIGH_GAIN  4.0
#define MIN_SAT_VOLTAGE_NO_GAIN    1.4
#define MAX_SAT_VOLTAGE_NO_GAIN    2.7

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
#ifdef ADC_AVG_4
    // 4 ADC averaging reads - min integration time 18.51 ms
    #define SPEC_CLK_TICK_TIMER  85
#else
    // 2 ADC averaging reads - min integration time 11.3 ms
    #define SPEC_CLK_TICK_TIMER  50
#endif

// Macro to convert ticks to uSec
#define ticksToUsec(x) ((x)*SPEC_CLK_TICK_TIMER/TIMER_US_FACTOR)
#define uSecToTicks(x) ((x)*TIMER_US_FACTOR/SPEC_CLK_TICK_TIMER)

// Max ADC conversion value - 16 bit
#define ADC_MAX_VALUE   UINT16_MAX

// C12666MA:
//      Integration time = (INTEG_TICKS + READ_TICKS)/2/frequency
//      Integration time limits from datasheet: 0.01 sec to 10 sec
//      Each read takes 4 CLK cycles; 6 CLK cycles min. after the last read
//           for 256 pixels train is 4*256+6=1030 CLK cycles
//      Numbers below are in ticks: 2 ticks (h/l and l/h) per clock cycle
//      even numbers only!
#define TICKS_PER_PIXEL      8              // Sensor spec - ticks per single pixel readout
#define MIN_INTEG_TIME_US    1000UL         // 1ms minimum integration time to ensure Integration state will always be used
#define MAX_INTEG_TIME_US    10000000UL     // 10s maximum integration time
#define DEF_LEAD_TICKS       64             // anything greater than 38 seems OK up to 200KHz clock, room temperature
#define TRAIL_TICKS          12             // anything greater than 2 seems OK up to 200KHz clock, room temperature
#define READ_TICKS           (SPEC_PIXELS*TICKS_PER_PIXEL + TRAIL_TICKS)
#define EXT_TRG_HIGH_TICKS   uSecToTicks(1000)  // duration of ext TRG pin high signal - 1mSec

// Integration ticks set to minimum by default - integration time is
// formed by INTEG_TICKS + READ_TICKS
static uint32_t INTEG_TICKS = uSecToTicks(MIN_INTEG_TIME_US);

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

// ADC reference voltages
static const float adcVoltages[] = { 2.5, 3.0, 4.096, 5.0 };

// spectrometer states and trigger variables
static volatile bool         measuringData_ = false;
static volatile bool         timerOn = false;
static volatile spec_state_t specState;
static volatile bool         specDataReady = false;    // trigger for ADC conversion
static uint8_t               specCLK = LOW;            // current clock pin state
static uint8_t               specST  = LOW;            // current ST pin state
static uint32_t              specCounter = 0;          // counter
static uint32_t              extTRGCounter = 0;        // ext trigger counter
static uint32_t* volatile    specData = 0;             // pointer to current data for ADC reads
static uint8_t* volatile     specDataCounter = 0;      // pointer to current data for ADC reads counter

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
uint8_t  dataCounts[SPEC_PIXELS];

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

// initialise AD (AD7980) and setup SPI
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
inline void readADC(uint32_t* data, uint8_t* dataCounter) __attribute__((always_inline));

// function to read 16 bit value from ADC7980
inline void readADC(uint32_t* data, uint8_t* dataCounter)
{
    // 1st read
    // initiate conversion and wait for max conversion time
    pinSetFast(adcPinCNV);
    System.ticksDelay(adcConvTimeTicks);
    pinResetFast(adcPinCNV);

    // SPI enable
    SPI_BASE->CR1 |= SPI_CR1_SPE;

    // Wait for SPI data reception
    while (SPI_BASE->SR & SPI_I2S_FLAG_RXNE == 0) ;

    // Read SPI received data
    *data = SPI_BASE->DR;
    ++(*dataCounter);

    // disable
    SPI_BASE->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);

    // 2nd read
    // initiate conversion and wait for max conversion time
    if (!specDataReady)
        return;
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
        *data += SPI_BASE->DR;
        ++(*dataCounter);
    }

    // disable
    SPI_BASE->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);

#ifdef ADC_AVG_4
    // 3rd read
    // initiate conversion and wait for max conversion time
    if (!specDataReady)
        return;
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
        *data += SPI_BASE->DR;
        ++(*dataCounter);
    }

    // disable
    SPI_BASE->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);

    // 4th read
    // initiate conversion and wait for max conversion time
    if (!specDataReady)
        return;
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
        *data += SPI_BASE->DR;
        ++(*dataCounter);
    }

    // disable
    SPI_BASE->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);
#endif
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
                }
                specST  = specRead[specCounter] & ST_BIT;
                break;

            case SPEC_RESET2:
                ++specCounter;
                if (specCounter == READ_TICKS) {
                    specCounter = 0;
                    specState = SPEC_INTEGRATION;
                    // enable external light if defined
                    if (extPinLIGHT != NO_PIN)
                        pinSetFast(extPinLIGHT);
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
                    specCounter = 0;
                    specState = SPEC_TRAIL;
                    specData = 0;
                    specDataCounter = 0;
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
void startSpecTimer(uint32_t extTrgMeasDelay, bool doExtTriggering)
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
    if (doExtTriggering && extPinTRG != NO_PIN && extTrgMeasDelay > 0)
    {
        uint32_t delayTimeTicks = uSecToTicks(extTrgMeasDelay);

        // making it even
        delayTimeTicks &= ~1;

        // calculate LEAD_TICKS
        if (delayTimeTicks > DEF_LEAD_TICKS + READ_TICKS + READ_TICKS)
            LEAD_TICKS = delayTimeTicks - READ_TICKS - READ_TICKS;
        else
            LEAD_TICKS = DEF_LEAD_TICKS;

        // set trigger cycles
        extTRGCounter = EXT_TRG_HIGH_TICKS + LEAD_TICKS + READ_TICKS + READ_TICKS - delayTimeTicks;
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
C12666MA::C12666MA(uint8_t spec_gain, uint8_t spec_eos, uint8_t spec_trg,
                   uint8_t spec_clk, uint8_t spec_st, uint8_t adc_ref_sel1,
                   uint8_t adc_ref_sel2, uint8_t adc_cnv, uint8_t ext_trg,
                   uint8_t ext_trg_ls, const double *defaultCalibration)
{
    spec_gain_ = spec_gain;
    spec_eos_ = spec_eos;
    spec_trg_ = spec_trg;
    spec_clk_ = spec_clk;
    spec_st_ = spec_st;
    ext_trg_ = ext_trg;
    ext_trg_ls_ = ext_trg_ls;
    adc_ref_sel1_ = adc_ref_sel1;
    adc_ref_sel2_ = adc_ref_sel2;
    adc_cnv_ = adc_cnv;

    calibration_[0] = calibration_[1] = calibration_[2] = 0;
    calibration_[3] = calibration_[4] = calibration_[5] = 0;

    rangeStartIdx_ = 0;
    rangePixels_ = SPEC_PIXELS;
    meas_ = blackLevels_ = normCoef_ = 0;

    measuringData_ = false;
    applyBandPassCorrection_ = true;

    timerOn = false;
    specState = SPEC_STOP;
    specDataReady = false;

    // read saved data and set defaults
    EEPROM.get(EEPROM_MIN_BLACK_VOLTAGE, minBlackLevelVoltage_);
    if (isnan(minBlackLevelVoltage_)
        || minBlackLevelVoltage_ < 0.0
        || minBlackLevelVoltage_ > adcVoltages[ADC_2_5V])
        // EEPROM was empty
        minBlackLevelVoltage_ = 0.0;

    EEPROM.get(EEPROM_MEASURE_TYPE_ADDR, measurementType_);
    if (measurementType_ != MEASURE_RELATIVE &&
        measurementType_ != MEASURE_VOLTAGE &&
        measurementType_ != MEASURE_ABSOLUTE)
        // EEPROM was empty
        measurementType_ = MEASURE_RELATIVE;

    EEPROM.get(EEPROM_GAIN_ADDR, gain_);
    if (gain_ != NO_GAIN && gain_ != HIGH_GAIN)
        // EEPROM was empty
        gain_ = NO_GAIN;

    EEPROM.get(EEPROM_ADC_REF_ADDR, adcRef_);
    if (adcRef_ != ADC_2_5V   && adcRef_ != ADC_3V  &&
        adcRef_ != ADC_4_096V && adcRef_ != ADC_5V)
        // EEPROM was empty
        adcRef_ = ADC_5V;

    uint32_t trgMeasDelayUs = 0;
    extTrgMeasDelayUs_ = 0;
    EEPROM.get(EEPROM_TRG_MEAS_DELAY, trgMeasDelayUs);
    if (trgMeasDelayUs == 0 ||
        (trgMeasDelayUs >= ticksToUsec(EXT_TRG_HIGH_TICKS)
         && trgMeasDelayUs < 10000000))  // 10 sec as top limit
        extTrgMeasDelayUs_ = trgMeasDelayUs;

    uint32_t intTimeTicks = 0;
    EEPROM.get(EEPROM_INTEGRATION_TIME, intTimeTicks);
    if (ticksToUsec(intTimeTicks) >= MIN_INTEG_TIME_US
        && ticksToUsec(intTimeTicks) <= MAX_INTEG_TIME_US)
        INTEG_TICKS = intTimeTicks;
    else
        setIntTime(100 _mSEC, false);

    float satVoltage = 0.0;
    EEPROM.get(EEPROM_SAT_VOLTAGE_HIGH_GAIN, satVoltage);
    // check the high gain saturation voltage against Hamamatsu spec limits
    if (isnan(satVoltage)
        || satVoltage < MIN_SAT_VOLTAGE_HIGH_GAIN
        || satVoltage > MAX_SAT_VOLTAGE_HIGH_GAIN)
        satVoltageHighGain_ = MIN_SAT_VOLTAGE_HIGH_GAIN;
    else
        satVoltageHighGain_ = satVoltage;

    EEPROM.get(EEPROM_SAT_VOLTAGE_NO_GAIN, satVoltage);
    // check the no gain saturation voltage against Hamamatsu spec limits
    if (isnan(satVoltage)
        || satVoltage < MIN_SAT_VOLTAGE_NO_GAIN
        || satVoltage > MAX_SAT_VOLTAGE_NO_GAIN)
        satVoltageNoGain_ = MIN_SAT_VOLTAGE_NO_GAIN;
    else
        satVoltageNoGain_ = satVoltage;

    EEPROM.get(EEPROM_CALIBRATION_COEF_1, calibration_[0]);
    if (!isnan(calibration_[0])
        && calibration_[0] > 100
        && calibration_[0] < 500)  // first coeff should be around 300
    {
        EEPROM.get(EEPROM_CALIBRATION_COEF_2, calibration_[1]);
        EEPROM.get(EEPROM_CALIBRATION_COEF_3, calibration_[2]);
        EEPROM.get(EEPROM_CALIBRATION_COEF_4, calibration_[3]);
        EEPROM.get(EEPROM_CALIBRATION_COEF_5, calibration_[4]);
        EEPROM.get(EEPROM_CALIBRATION_COEF_6, calibration_[5]);
    }
    else if (defaultCalibration)
    {
        setWavelengthCalibrationInternal(defaultCalibration);
    }

    // initialise ranges
    int minWavelength, maxWavelength;
    getSensorRangeInternal(minWavelength, maxWavelength);
    setSensorRangeInternal(minWavelength, maxWavelength);

    // Initialize arrays - black with calibrated minimum
    for (int i=0; i<rangePixels_; i++)
    {
        blackLevels_[i] = minBlackLevelVoltage_;
        meas_[i] = 0.0;

        // read spectral response normalisation
        EEPROM.get(EEPROM_NORM_COEF_ARRAY+(i+rangeStartIdx_)*sizeof(float),
                   normCoef_[i]);
        if (isnan(normCoef_[i])
            || normCoef_[i]<0.00001
            || normCoef_[i]>10000)
            normCoef_[i] = 1.0;
    }

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
    bool success = true;

    // Setup pins
    pinMode(adc_ref_sel1_, OUTPUT);
    pinMode(adc_ref_sel2_, OUTPUT);
    pinMode(adc_cnv_,      OUTPUT);
    pinMode(SPI_MOSI,      OUTPUT);
    pinMode(SPI_MISO,      INPUT);
    pinMode(SPI_SCK,       OUTPUT);
    pinMode(spec_trg_,     INPUT_PULLDOWN); // used for soft interrupt only
    pinMode(spec_eos_,     INPUT);
    pinMode(spec_gain_,    OUTPUT);
    pinMode(spec_st_,      OUTPUT);
    pinMode(spec_clk_,     OUTPUT);

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

    // set defaults
    setGainInternal(gain_);
    setAdcRefInternal(adcRef_);

    // set internal pins
    specPinCLK  = spec_clk_;
    specPinST   = spec_st_;
    adcPinCNV   = adc_cnv_;
    extPinTRG   = NO_PIN;
    extPinLIGHT = NO_PIN;

    return success;
}

// Reset all stored values to default
void C12666MA::resetToDefaults(const double *defaultCalibration)
{
    minBlackLevelVoltage_ = 0.0;
    EEPROM.put(EEPROM_MIN_BLACK_VOLTAGE, minBlackLevelVoltage_);

    measurementType_ = MEASURE_RELATIVE;
    EEPROM.put(EEPROM_MEASURE_TYPE_ADDR, measurementType_);

    gain_ = NO_GAIN;
    EEPROM.put(EEPROM_GAIN_ADDR, gain_);

    adcRef_ = ADC_5V;
    EEPROM.put(EEPROM_ADC_REF_ADDR, adcRef_);

    extTrgMeasDelayUs_ = 0;
    EEPROM.put(EEPROM_TRG_MEAS_DELAY, extTrgMeasDelayUs_);

    setIntTime(100 _mSEC, false);

    satVoltageHighGain_ = MIN_SAT_VOLTAGE_HIGH_GAIN;
    EEPROM.put(EEPROM_SAT_VOLTAGE_HIGH_GAIN, satVoltageHighGain_);

    satVoltageNoGain_ = MIN_SAT_VOLTAGE_NO_GAIN;
    EEPROM.put(EEPROM_SAT_VOLTAGE_NO_GAIN, satVoltageNoGain_);

    setWavelengthCalibrationInternal(defaultCalibration);

    setSensorRange(-1, -1);
    calibrateSpectralResponse(0);
}

// Obtains sensor range from saved EEPROM
void C12666MA::getSensorRangeInternal(int& minWavelength, int& maxWavelength)
{
    EEPROM.get(EEPROM_SPEC_RANGE_MIN, minWavelength);
    EEPROM.get(EEPROM_SPEC_RANGE_MAX, maxWavelength);
    if (minWavelength != -1 && minWavelength < 100)
        minWavelength = -1;
    if (maxWavelength != -1 && maxWavelength > 1000)
        maxWavelength = -1;

    if (maxWavelength>0 && maxWavelength>0 && maxWavelength<=minWavelength)
        minWavelength = maxWavelength = -1;
}

// Sets sensor spectral range - this will set internal indexes, pixel numbers
// and array reallocations according to the new range. This method needs
// valid wavelength calibration coefficients to work.
void C12666MA::setSensorRangeInternal(int& minWavelength, int& maxWavelength)
{
    int rangeEndIdx = rangeStartIdx_ + rangePixels_;
    int rangeStartIdx = rangeStartIdx_;

    if (minWavelength>0 && maxWavelength>0 && maxWavelength<=minWavelength)
        return;

    // reset the range start for wavelength calculations
    rangeStartIdx_ = 0;

    if (getWavelength(0) <= 0)
    {
        // wavlength calibration is not set - use full range
        minWavelength = maxWavelength = -1;
        rangeStartIdx = 0;
        rangeEndIdx = SPEC_PIXELS;
    }
    else
    {
        // update lower bound
        if (minWavelength != 0)
        {
            if (minWavelength < 0)
                minWavelength = 340;    // value from Hamamatsu spec
            // search for lower bound index
            rangeStartIdx = 0;
            for (int i=0; i<SPEC_PIXELS; ++i)
                if (minWavelength <= (int)getWavelength(i))
                {
                    rangeStartIdx = i;
                    break;
                }
            if (!rangeStartIdx)
                // no valid one was found or exceeds the range
                minWavelength = getWavelength(0);
            // take one more pixel to enclose the range
            if (rangeStartIdx)
                --rangeStartIdx;
        }

        // update upper bound
        if (maxWavelength != 0)
        {
            if (maxWavelength < 0)
                maxWavelength = 780;    // value from Hamamatsu spec
            // search for lower bound index
            rangeEndIdx = SPEC_PIXELS;
            for (int i=SPEC_PIXELS; i>rangeStartIdx; --i)
                if (maxWavelength >= (int)getWavelength(i-1))
                {
                    rangeEndIdx = i;
                    break;
                }
            if (rangeEndIdx==SPEC_PIXELS)
                // no valid one was found or exceeds the range
                maxWavelength = getWavelength(SPEC_PIXELS-1);
            // take one more pixel to enclose the range
            if (rangeEndIdx < SPEC_PIXELS)
                ++rangeEndIdx;
        }
    }

    // setup arrays
    if (rangePixels_ < rangeEndIdx - rangeStartIdx)
    {
        // deallocate existing arrays
        delete[] meas_;
        meas_ = blackLevels_ = normCoef_ = 0;
    }

    rangeStartIdx_ = rangeStartIdx;
    rangePixels_ = rangeEndIdx - rangeStartIdx;

    if (!meas_)
    {
        meas_ = new float[3*rangePixels_];
        blackLevels_ = meas_ + rangePixels_;
        normCoef_ = blackLevels_ + rangePixels_;
    }
}

// Sets the spectrometer sensor range in nanometers. If the range is
// wider than current one, then this will reset spectral response
// normalisation. It also resets measured value and black levels.
//
// Specifying either value as 0 will not update that value.
//
// Specifying either value as -1 will reset to the spectrometer default
// for that value.
//
// Generally, the spectral range should be chosen at the beginning for
// the specific application, sensor calibrated with it and then left alone.
void C12666MA::setSensorRange(int minWavelength, int maxWavelength)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    // prevent measurement whilst resetting
    measuringData_ = true;

    int savedStartIdx = rangeStartIdx_;
    int savedRangePixels = rangePixels_;

    // update sensor data
    setSensorRangeInternal(minWavelength, maxWavelength);

    // store in EEPROM
    EEPROM.put(EEPROM_SPEC_RANGE_MIN, minWavelength);
    EEPROM.put(EEPROM_SPEC_RANGE_MAX, maxWavelength);

    // check if we need to reset data
    if (savedStartIdx != rangeStartIdx_ || savedRangePixels != rangePixels_)
    {
        // reset data
        for (int i=0; i<rangePixels_; i++)
        {
            normCoef_[i] = 1.0;
            blackLevels_[i] = minBlackLevelVoltage_;
            meas_[i] = 0.0;
        }

        lastMeasADCRef_ = adcRef_;
        lastMeasGain_ = gain_;

        for (int i=0; i<rangePixels_; i++)
            // write spectral response normalisation
            EEPROM.put(EEPROM_NORM_COEF_ARRAY+(i+rangeStartIdx_)*sizeof(float),
                       normCoef_[i]);
    }

    measuringData_ = false;
}

// Sets the wavelength calibration coeffiecients only and preserves them in EEPROM.
// Usually provided by Hamamatsu but can be overriden by user calculated ones.
//
// Return true if the calibration coefficients are changed
bool C12666MA::setWavelengthCalibrationInternal(const double* wavelengthCal)
{
    if (!wavelengthCal)
        return false;

    bool changed = calibration_[0] != wavelengthCal[0] ||
                   calibration_[1] != wavelengthCal[1] ||
                   calibration_[2] != wavelengthCal[2] ||
                   calibration_[3] != wavelengthCal[3] ||
                   calibration_[4] != wavelengthCal[4] ||
                   calibration_[5] != wavelengthCal[5];

    calibration_[0] = wavelengthCal[0];
    calibration_[1] = wavelengthCal[1];
    calibration_[2] = wavelengthCal[2];
    calibration_[3] = wavelengthCal[3];
    calibration_[4] = wavelengthCal[4];
    calibration_[5] = wavelengthCal[5];

    if (changed)
    {
        EEPROM.put(EEPROM_CALIBRATION_COEF_1, calibration_[0]);
        EEPROM.put(EEPROM_CALIBRATION_COEF_2, calibration_[1]);
        EEPROM.put(EEPROM_CALIBRATION_COEF_3, calibration_[2]);
        EEPROM.put(EEPROM_CALIBRATION_COEF_4, calibration_[3]);
        EEPROM.put(EEPROM_CALIBRATION_COEF_5, calibration_[4]);
        EEPROM.put(EEPROM_CALIBRATION_COEF_6, calibration_[5]);
    }

    return changed;
}

// Sets the wavelength calibration coeffiecients. Usually provided
// by Hamamatsu but can be overriden by user calculated ones.
//
// Setting this will reset spectral response normalisation.
void C12666MA::setWavelengthCalibration(const double* wavelengthCal)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    // prevent measurement whilst resetting
    measuringData_ = true;

    if (setWavelengthCalibrationInternal(wavelengthCal))
    {
        int minWavelength, maxWavelength;
        getSensorRangeInternal(minWavelength, maxWavelength);
        setSensorRangeInternal(minWavelength, maxWavelength);

        // reset data
        for (int i=0; i<rangePixels_; i++)
        {
            normCoef_[i] = 1.0;
            blackLevels_[i] = minBlackLevelVoltage_;
            meas_[i] = 0.0;
        }

        lastMeasADCRef_ = adcRef_;
        lastMeasGain_ = gain_;

        for (int i=0; i<rangePixels_; i++)
            // write spectral response normalisation
            EEPROM.put(EEPROM_NORM_COEF_ARRAY+(i+rangeStartIdx_)*sizeof(float),
                       normCoef_[i]);
    }

    measuringData_ = false;
}

// Sets the external trigger to measurement delay time. This defines time interval
// in uSec that offsets external trigger from the measurement. I.e. external trigger
// is raised and after this delay the integration and measurement starts.
//
// Specifying negative delay disables external triggering
void C12666MA::setExtTrgMeasDelay(int32_t extTrgMeasDelayUs, bool storeInEeprom)
{
    // no action if timer is on or in measurement
    if (timerOn)
        return;

    uint32_t delayTimeTicks = (extTrgMeasDelayUs * TIMER_US_FACTOR) / SPEC_CLK_TICK_TIMER;

    // cannot be less than extr trigger high holding cycles
    if (delayTimeTicks < EXT_TRG_HIGH_TICKS)
        delayTimeTicks = EXT_TRG_HIGH_TICKS;

    delayTimeTicks &= ~1;

    extTrgMeasDelayUs_ = extTrgMeasDelayUs < 0 ? 0 : ticksToUsec(delayTimeTicks);

    if (storeInEeprom)
        EEPROM.put(EEPROM_TRG_MEAS_DELAY, extTrgMeasDelayUs_);
}

// Set the integration (or sample collection) time, in microseconds
void C12666MA::setIntTime(uint32_t timeUs, bool storeInEeprom)
{
    // no action if timer is on or in measurement
    if (timerOn)
        return;

    uint32_t intTime = timeUs*TIMER_US_FACTOR;

    // only support 4 or 2 averaging ADC read

    // set minumum integration times for several averaging ADC reads (with extra cycles added)
    uint32_t minIntTime = (SPEC_CLK_TICK_TIMER * READ_TICKS) + (MIN_INTEG_TIME_US * TIMER_US_FACTOR);

    if (intTime < minIntTime)
        intTime = minIntTime;

    if (intTime > MAX_INTEG_TIME_US*TIMER_US_FACTOR)
        intTime = MAX_INTEG_TIME_US*TIMER_US_FACTOR;

    // now determine integration ticks
    INTEG_TICKS = (intTime/SPEC_CLK_TICK_TIMER) - READ_TICKS + 1;

    // this should never execute but better code defensively
    if (INTEG_TICKS == 0)
        INTEG_TICKS = uSecToTicks(MIN_INTEG_TIME_US);

    // even up INTEG_TICKS
    if (INTEG_TICKS & 1)
        INTEG_TICKS++;

    if (storeInEeprom)
        EEPROM.put(EEPROM_INTEGRATION_TIME, INTEG_TICKS);
}

// Retrieve currently set integration time
uint32_t C12666MA::getIntTime()
{
    return ticksToUsec(INTEG_TICKS+READ_TICKS);
}

// Convert aggregated readouts to voltage measurement floating point data
// and return the max value
float C12666MA::processMeasurement(float* measurement)
{
    // Initialize arrays
    float maxVal = 0.0;
    float adcRefVoltage = adcVoltages[adcRef_];
    for (int i=0; i<rangePixels_; i++)
    {
        measurement[i] = 0.0;

        if (dataCounts[i])
            measurement[i] =
                    ((float)data[i+rangeStartIdx_]*adcRefVoltage) /
                    ((float)dataCounts[i+rangeStartIdx_]*ADC_MAX_VALUE);

        if (measurement[i] > maxVal)
            maxVal = measurement[i];
    }

    // store ADC ref for this measurement
    if (measurement == meas_)
    {
        lastMeasADCRef_ = adcRef_;
        lastMeasGain_ = gain_;
    }

    return maxVal;
}

// Take spectrometer reading in automatic mode. This does not require
// integration time. Also as a result of measurement it will set the
// integration time measured as well as gain and ADC voltage reference
// (the latter only in some modes). The automatic measurement is tuned to
// use sensor setup to maximise the output ADC resolution/range.
//
// The automatic measurement can be tuned to achieve different results.
// The behavior of it is controlled controlled by autoType parameter and
// has the following modes:
//
//    AUTO_FOR_SET_REF   - Aims to maximise ADC reading within currently
//                         set reference voltage. I.e. achieving maximum
//                         resolution within selected reference voltage or
//                         saturation limit (whichever is smaller). Only
//                         gain and integration are changed in this mode.
//
//    AUTO_ALL_MIN_INTEG - Aims to maximise ADC reading across all ADC
//                         reference voltages whilst achieving minimum
//                         integration time. This essentially attempts
//                         to achieve maximum resolution on smallest
//                         reference voltage (to get shortest integration).
//
//    AUTO_ALL_MAX_RANGE - Aims to maximise ADC reading across all ADC
//                         reference voltages to maximise sensor output.
//                         This method attempts to achieve maximum use of
//                         the sensor output range and attempts to achieve
//                         maximum reading close to sensor saturation.
//
// NOTE: it is essential to set/measure sensor saturation levels for this
//       function to work!
//
// NOTE2: Because it changes exposure time, this mode will reset black level
//        measurements to minimum calibrated by default. This can be omitted
//        if specified. It generally is a good idea to recapture black levels
//        with established exposure parameters after this call to make
//        measurement more precise.
//
void C12666MA::takeAutoMeasurement(auto_measure_t autoType,
                                   bool doBlackReset,
                                   bool doExtTriggering)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    measuringData_ = true;

    // reset blacks to calibrated minimum
    if (doBlackReset)
        resetBlackLevels();

    // High gain is preferable since it allows more optimal filling of
    // ADC reference range. So start with high gain.
    setGainInternal(HIGH_GAIN);
    float satVoltage = satVoltageHighGain_;

    // change ADC ref if enabled
    if (autoType != AUTO_FOR_SET_REF)
    {
        if (satVoltage > adcVoltages[ADC_3V])
            setAdcRefInternal(ADC_4_096V);
        else if (satVoltage > adcVoltages[ADC_2_5V])
            setAdcRefInternal(ADC_3V);
        else
            setAdcRefInternal(ADC_2_5V);
    }

    // delay to stabilise the changes
    delay(50);

    // try to make shortest reading
    INTEG_TICKS = uSecToTicks(MIN_INTEG_TIME_US);
    readSpectrometer(0, false, doExtTriggering);
    float maxMeasuredVoltage = processMeasurement(meas_);

    // check for saturated data
    if (maxMeasuredVoltage > satVoltageHighGain_)
    {
        // too much - set to no gain
        setGainInternal(NO_GAIN);
        satVoltage = satVoltageNoGain_;

        // change ADC ref if enabled
        if (autoType != AUTO_FOR_SET_REF)
        {
            if (satVoltage > adcVoltages[ADC_3V])
                setAdcRefInternal(ADC_4_096V);
            else if (satVoltage > adcVoltages[ADC_2_5V])
                setAdcRefInternal(ADC_3V);
            else
                setAdcRefInternal(ADC_2_5V);
        }

        // delay to stabilise the changes
        delay(50);

        // repeat shortest measurement
        readSpectrometer(0, false, doExtTriggering);
        maxMeasuredVoltage = processMeasurement(meas_);
    }

    // Initial setup now done and we have the shortest measurement in a selected
    // gain and encompassing saturation limits (if allowed). Check that it is
    // less then maximum and attempt to maximise it
    if (maxMeasuredVoltage < satVoltage && autoType == AUTO_ALL_MIN_INTEG)
    {
        // setup optimal ADC reference to optimise within (lowest
        // to achieve shortest integration time)
        if (maxMeasuredVoltage > adcVoltages[ADC_3V])
            setAdcRefInternal(ADC_4_096V);
        else if (maxMeasuredVoltage > adcVoltages[ADC_2_5V])
            setAdcRefInternal(ADC_3V);
        else
            setAdcRefInternal(ADC_2_5V);

        // delay to stabilise the changes
        delay(50);
    }

    // correct saturation voltage
    if (satVoltage > adcVoltages[adcRef_])
        satVoltage = adcVoltages[adcRef_];

    // The measurement tolerance - reaching this will stop auto measurement
    // (by default within 2.5% from saturation point). This should also help
    // keeping bandpass correction inside 0..1 range
    float satVoltageLower = satVoltage*0.975;

    // Make sure saturation voltage is just below the absolute max
    satVoltage *= 0.99;

    // now go up the integration within the range until we maximise the exposure
    uint32_t intTicksStep = INTEG_TICKS;
    bool stillGoing = maxMeasuredVoltage < satVoltageLower;
    while (stillGoing)
    {
        // for integrations larger than a second keep watchdog happy
        // and Particle connection keep alive
        if (getIntTime() > 1000000)
        {
            ApplicationWatchdog::checkin();
            if (Particle.connected())
                Particle.process();
        }

        // determine increase or decrease of the integration
        if (maxMeasuredVoltage < satVoltage)
        {
            // go up
            intTicksStep = ((satVoltage-maxMeasuredVoltage)*(INTEG_TICKS+READ_TICKS))/maxMeasuredVoltage;
            INTEG_TICKS += intTicksStep;
        }
        else
        {
            // last incerase was too much - half the steps and go down
            intTicksStep >>= 1;
            INTEG_TICKS = INTEG_TICKS < intTicksStep ? 0 : INTEG_TICKS - intTicksStep;
        }

        // check the limits
        if (INTEG_TICKS < uSecToTicks(MIN_INTEG_TIME_US))
            INTEG_TICKS = uSecToTicks(MIN_INTEG_TIME_US);
        else if (INTEG_TICKS > uSecToTicks(MAX_INTEG_TIME_US))
            INTEG_TICKS = uSecToTicks(MAX_INTEG_TIME_US);

        // do new reading
        readSpectrometer(0, false, doExtTriggering);
        maxMeasuredVoltage = processMeasurement(meas_);

        // check exit conditions
        if (maxMeasuredVoltage >= satVoltageLower && maxMeasuredVoltage < satVoltage)
            stillGoing = false;
        else if (INTEG_TICKS == uSecToTicks(MAX_INTEG_TIME_US) && maxMeasuredVoltage < satVoltage)
            stillGoing = false;
        else if (INTEG_TICKS == uSecToTicks(MIN_INTEG_TIME_US) && maxMeasuredVoltage > satVoltage)
            stillGoing = false;
        else if (intTicksStep <= 2)
        {
            stillGoing = false;
            if (maxMeasuredVoltage > satVoltage)
            {
                // revert last iteration if tipped over
                INTEG_TICKS -= intTicksStep<<1;
                readSpectrometer(0, false, doExtTriggering);
            }
        }
    }

    measuringData_ = false;

    // save data that was established in EEPROM
    EEPROM.put(EEPROM_INTEGRATION_TIME, INTEG_TICKS);
    EEPROM.put(EEPROM_GAIN_ADDR, gain_);
    if (autoType != AUTO_FOR_SET_REF)
        EEPROM.put(EEPROM_ADC_REF_ADDR, adcRef_);
}

// Calculates and returns Tungsten emissivity at given wavelength and
// temperature
//
// For more details refer to R. M. Pon and J. P. Hessler
//     "Spectral emissivity of tungsten: analytic expressions for the
//      340nm to 2.6 um spectral region"
//
// Tunstean Eemissivity analytical expression is calculated for
// T=(Temp-2200K)/1000 in kK and wavelength L in micrometers as follows:
//
//    emT(T,L) = a0+a1*T+(b0+b1*T+b2*T*T)*(L-l0)+(c0+c1*T)*(L-l0)*(L-l0)
//
// given the following specification:
//
//    L,nm    l0     a0        a1       b0       b1      b2      c0      c1
//   300-420  380  0.47245  -0.0155  -0.0086  -0.0229  0.0000  -2.860   0.000
//   420-480  450  0.46361  -0.0172  -0.1304   0.0000  0.0000   0.520   0.000
//   480-580  530  0.45549  -0.0173  -0.1150   0.0000  0.0000  -0.500   0.000
//   580-640  610  0.44297  -0.0177  -0.1482   0.0000  0.0000   0.723   0.000
//   640-760  700  0.43151  -0.0207  -0.1441  -0.0551  0.0000  -0.278  -0.190
//   760-940  850  0.40610  -0.0259  -0.1889   0.0087  0.0290  -0.126   0.246
//
double emvTungst(double wvL, double tempK)
{
    // implement this form for reduced calculations
    double T = (tempK-2200.0)/1000;
    double emT = 0.33; // for anything > 940nm
    if (wvL < 420.0)
    {
        wvL = (wvL-380.0)/1000;
        emT = 0.47245-0.0155*T-(0.0086+0.0229*T)*wvL-2.86*wvL*wvL;
    }
    else if (wvL < 480.0)
    {
        wvL = (wvL-450.0)/1000;
        emT = 0.46361-0.0172*T-0.1304*wvL+0.52*wvL*wvL;
    }
    else if (wvL < 580.0)
    {
        wvL = (wvL-530.0)/1000;
        emT = 0.45549-0.0173*T-0.115*wvL-0.5*wvL*wvL;
    }
    else if (wvL < 640.0)
    {
        wvL = (wvL-610.0)/1000;
        emT = 0.44297-0.0177*T-0.1482*wvL+0.723*wvL*wvL;
    }
    else if (wvL < 760.0)
    {
        wvL = (wvL-700.0)/1000;
        emT = 0.43151-0.0207*T-(0.1441+0.0551*T)*wvL-(0.278+0.19*T)*wvL*wvL;
    }
    else if (wvL < 940.0)
    {
        wvL = (wvL-850.0)/1000;
        emT = 0.4061-0.0259*T+(0.0087*T+0.029*T*T-0.1889)*wvL+(0.246*T-0.126)*wvL*wvL;
    }

    return emT;
}

// This method calibrates sensor relative spectral response.
//
// It expects the sensor to be exposed to stabilised tungsten light source
// of the specified temperature, with black levels captured, measures
// sensor response for selected parameters, calculates expected theoretical
// response (relative against largest wavelength) for Planckian blackbody
// corrected for tungsten source, and then calculates corrections for
// measured sensor response (with blacks subtracted).
//
// Procedure for calibration:
// 1) run the tungsten light source on stabilised power supply for at
//    least 20 mins, measuring its temperature (using voltage/current
//    measurement, lamp resistance against lamp resistance at room
//    temperature) - see O. Harang, M. J. Kosch "Absolute Optical
//    Calibrations Using a Simple Tungsten Bulb:Theory" for details
// 2) do automatic measurement to capture spectrometer measurement and
//    parameters at minumal ADC voltage and high gain for higher
//    dynamic range
// 3) capture black levels with exposure parameters established by (2)
// 4) call this method to do the calibration specifying calculated lamp
//    temperature at (1) and potentially new measurement (or use the one
//    done at (2) - default)
//
// NOTE: This call can invalidate current measurement results
//
void C12666MA::calibrateSpectralResponse(float lampTempK, bool useCurrentMeasurement)
{
    const double AIR_REFRACTION = 1.00028;  // standard air refraction

    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    measuringData_ = true;

    // reset coesfficients
    if (lampTempK <= 0.0)
        for (int i=0; i<rangePixels_; i++)
            normCoef_[i] = 1.0;
    else
    {
        if (!useCurrentMeasurement)
        {
            // no external triggering and measuring with existing parameters
            readSpectrometer(0, false, false);
            processMeasurement(meas_);
        }

        // process data
        int maxIdx = 0;
        float maxVal = 0.0;

        // first pass - calculate max wavelength
        for (int i=0; i<rangePixels_; i++)
        {
            float measVal = meas_[i] > blackLevels_[i]
                                ? meas_[i]-blackLevels_[i]
                                : 0.0;
            if (measVal > maxVal)
            {
                maxVal = measVal;
                maxIdx = i;
            }
        }

        if (maxVal <= 0)
        {
            measuringData_ = false;
            return;
        }

        // second pass scale measurement relative to max and calculate normalisation
        double normWv = getWavelength(maxIdx);  // normalised to measured max
        double hckTA = 6.62606957293*2.99792458/1.38064881313*1000000./lampTempK/AIR_REFRACTION;
        double normVal = normWv*normWv*normWv*normWv*normWv*(exp(hckTA/normWv)-1.0);
        double normEmvTungst = emvTungst(normWv, lampTempK);
        float maxNorm = 0.0;
        for (int i=0; i<rangePixels_; i++)
        {
            double measuredRelVal = meas_[i] > blackLevels_[i]+(1.0/ADC_MAX_VALUE)
                                      ? (meas_[i]-blackLevels_[i])/maxVal
                                      : 0.0;
            double curWv = getWavelength(i);
            double calcRelVal = normVal*emvTungst(curWv, lampTempK) /
                    (normEmvTungst*curWv*curWv*curWv*curWv*curWv*(exp(hckTA/curWv)-1.0));
            normCoef_[i] = measuredRelVal>0 ? calcRelVal/measuredRelVal : 0;
            if (maxNorm < normCoef_[i])
                maxNorm = normCoef_[i];
        }

        // third pass - normalise calibration coefficients against maximum
        for (int i=0; i<rangePixels_; i++)
            normCoef_[i] = normCoef_[i] ? normCoef_[i]/maxNorm : 1.0;
    }

    measuringData_ = false;

    for (int i=0; i<rangePixels_; i++)
        // write spectral response normalisation
        EEPROM.put(EEPROM_NORM_COEF_ARRAY+(i+rangeStartIdx_)*sizeof(float),
                   normCoef_[i]);
}

// Take single measurement
void C12666MA::takeMeasurement(uint32_t timeUs, bool doExtTriggering)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    measuringData_ = true;

    // read main measurement data
    readSpectrometer(timeUs, doExtTriggering, doExtTriggering);
    processMeasurement(meas_);

    measuringData_ = false;
}

// Take single black level measurement
void C12666MA::takeBlackMeasurement(uint32_t timeUs)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    measuringData_ = true;

    // read black if needed
    readSpectrometer(timeUs, false, false);
    processMeasurement(blackLevels_);

    measuringData_ = false;
}

// Reset black levels to 0
void C12666MA::resetBlackLevels(float resetVoltage)
{
    // Initialize arrays
    if (resetVoltage < 0.0)
        resetVoltage = minBlackLevelVoltage_;
    for (int i=0; i<rangePixels_; i++)
        blackLevels_[i] = resetVoltage;
}

// This routine to initiate and read spectrometer measurement data
void C12666MA::readSpectrometer(uint32_t timeUs,
                                bool doExtTriggering,
                                bool doLightTriggering)
{
    // no action if timer is on or in measurement
    if (timerOn)
        return;

    uint32_t savedIntegration = INTEG_TICKS;

    // only use supplied time if non zero
    if (timeUs > 0)
        setIntTime(timeUs, false);

    // initialise variables
    specData = data;
    specDataCounter = dataCounts;

    // init stats and data
    for (int i=0; i<SPEC_PIXELS; i++)
    {
        // zero data
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
    startSpecTimer(extTrgMeasDelayUs_, doExtTriggering);

    // loop until stop
    while (specState != SPEC_STOP)
        ;

    // stop the timer and cleanup
    stopSpecTimer();
    endADC();

    // restore integration if needed
    if (timeUs > 0)
        INTEG_TICKS = savedIntegration;

    // reset trigger pins
    extPinTRG   = NO_PIN;
    extPinLIGHT = NO_PIN;
}

// Enable/disable Stearns and Stearns (1988) bandpass correction
void C12666MA::enableBandpassCorrection(bool enable)
{
    applyBandPassCorrection_ = enable;
}

// Set the saturation voltages. These are used to in auto integration mode
// of measurement. Passing values outside of ranges from Hamamatsu spec
// will reset approprite value to a default ones.
void C12666MA::setSaturationVoltages(float satVoltageHighGain,
                                     float satVoltageNoGain,
                                     bool storeInEeprom)
{
    // check the high gain saturation voltage against Hamamatsu spec limits
    if (satVoltageHighGain > 0.0)
    {
        if (satVoltageHighGain >= MIN_SAT_VOLTAGE_HIGH_GAIN
            && satVoltageHighGain <= MAX_SAT_VOLTAGE_HIGH_GAIN)
            satVoltageHighGain_ = satVoltageHighGain;
        else
            satVoltageHighGain_ = MIN_SAT_VOLTAGE_HIGH_GAIN;

        if (storeInEeprom)
            EEPROM.put(EEPROM_SAT_VOLTAGE_HIGH_GAIN, satVoltageHighGain_);
    }

    // check the no gain saturation voltage against Hamamatsu spec limits
    if (satVoltageNoGain > 0.0)
    {
        if (satVoltageNoGain >= MIN_SAT_VOLTAGE_NO_GAIN
            && satVoltageNoGain <= MAX_SAT_VOLTAGE_NO_GAIN)
            satVoltageNoGain_ = satVoltageNoGain;
        else
            satVoltageNoGain_ = MIN_SAT_VOLTAGE_NO_GAIN;

        if (storeInEeprom)
            EEPROM.put(EEPROM_SAT_VOLTAGE_NO_GAIN, satVoltageNoGain_);
    }
}

// Get average max within 5% off the measured maximum
float C12666MA::getAveragedMax(float maxVal, float* measurement)
{
    // calculate average max
    float avgMax = 0;
    int count = 0;
    for (int i=0; i<rangePixels_; i++)
        if (measurement[i] > maxVal*0.95)
        {
            avgMax += measurement[i];
            ++count;
        }

    return count ? avgMax/count : maxVal;
}

// Starting from minimal exposure - find the one where 1/3 of the sensor
// readouts is saturated.
bool C12666MA::findSaturatedExposure()
{
    INTEG_TICKS = uSecToTicks(MIN_INTEG_TIME_US);

    // now go up the integration until we maximise the exposure
    while (INTEG_TICKS < uSecToTicks(MAX_INTEG_TIME_US))
    {
        readSpectrometer(0, false, false);

        // get maximum from raw measurement
        uint32_t maxVal = 0;
        for (int i=0; i<SPEC_PIXELS; i++)
        {
            uint32_t val = dataCounts[i] ? data[i]/dataCounts[i] : 0;
            if (maxVal<val)
                maxVal = val;
        }
        int maxCount = 0;
        // count all values within 2% range from maximum
        if (maxVal)
        {
            maxVal = maxVal * 98 / 100;
            for (int i=0; i<SPEC_PIXELS; i++)
            {
                uint32_t val = dataCounts[i] ? data[i]/dataCounts[i] : 0;
                if (val > maxVal)
                    ++maxCount;
            }
        }
        // we reached saturation when count of
        // max values reaches 1/3 of all pixels
        if (maxCount >= SPEC_PIXELS/3)
            break;

        // double exposure and continue
        INTEG_TICKS <<= 1;
    }

    return INTEG_TICKS < uSecToTicks(MAX_INTEG_TIME_US);
}

// Automatic measurement of the saturation voltages. These are used in
// auto integration mode of measurement.
//
// Automatic setting works by exposing sensor to bright light and then
// calling this method to work out saturation voltages.
void C12666MA::measureSaturationVoltages()
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    measuringData_ = true;

    // save current reference, gain mode and integration time
    gain_t    savedGain     = getGain();
    adc_ref_t savedAdcRef   = getAdcReference();
    uint32_t  savedIntTicks = INTEG_TICKS;

    float hgSatVoltage = satVoltageHighGain_;
    float ngSatVoltage = satVoltageNoGain_;

    // set the 4.096V reference - that's the highest C12666MA will go
    setAdcRefInternal(ADC_4_096V);

    // set high gain first
    setGainInternal(HIGH_GAIN);

    // delay to stabilise the changes
    delay(50);

    // find high gain saturated voltage
    if (findSaturatedExposure())
        hgSatVoltage = getAveragedMax(processMeasurement(meas_), meas_);

    // set no gain next
    setGainInternal(NO_GAIN);

    // delay to stabilise the changes
    delay(50);

    // find no gain saturated voltage
    if (findSaturatedExposure())
        ngSatVoltage = getAveragedMax(processMeasurement(meas_), meas_);


    if (hgSatVoltage != satVoltageHighGain_ || ngSatVoltage != satVoltageNoGain_)
        setSaturationVoltages(hgSatVoltage, ngSatVoltage);

    // restore ADC reference and integration
    setAdcRefInternal(savedAdcRef);
    setGainInternal(savedGain);
    INTEG_TICKS = savedIntTicks;

    measuringData_ = false;
}

// Sets the minimal black level voltage. This is used for all measurements
// when no separate black levels were captured.
void C12666MA::setMinBlackVoltage(float minBlackVoltage)
{
    if (minBlackVoltage < 0.0 || minBlackVoltage > 2.0)
        // calibrated black levels have to be reasonable
        return;

    minBlackLevelVoltage_ = minBlackVoltage;

    // preserve the data
    EEPROM.put(EEPROM_MIN_BLACK_VOLTAGE, minBlackLevelVoltage_);
}

// Automatic measurement of the minimal black level voltage. This is used
// for all measurements when no separate black levels were captured.
//
// Measurement works by reading dark exposure of short integration
// time and recording average read value (except first point which
// always has skewed value).
//
// NOTE: this call invalidates current black levels
void C12666MA::measureMinBlackVoltage()
{
    measuringData_ = true;

    // save current reference and gain mode
    gain_t    savedGain   = getGain();
    adc_ref_t savedAdcRef = getAdcReference();

    // set the 2.5V reference - to make black measuremeent more precise
    setAdcRefInternal(ADC_2_5V);

    // set no gain
    setGainInternal(NO_GAIN);

    // delay to stabilise the changes
    delay(50);

    // do the measurement at 30 mSec to ensure short reading
    readSpectrometer(30000, false, false);
    float minBlackLevelVoltage = processMeasurement(blackLevels_);

    // restore ADC reference and gain
    setAdcRefInternal(savedAdcRef);
    setGainInternal(savedGain);

    // set the voltage
    setMinBlackVoltage(minBlackLevelVoltage);

    // reset all black levels to it
    resetBlackLevels();

    measuringData_ = false;
}

// Set the spectrometer measurement type. This allows to change the type
// of the measurement results and will affect all subsequent measurements.
// The measurement results could be stored:
//   (1) relatively - 0..1 values for currently selected ADC reference
//   (2) as voltage measurements irrespective to ADC parameters but dependent
//       on gain (these are absolute within the same gain settings)
//   (3) as absolute measurements, 0..1 range scaled to saturation voltage
void C12666MA::setMeasurementType(measure_t measurementType, bool storeInEeprom)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    measurementType_ = measurementType;

    if (storeInEeprom)
        EEPROM.put(EEPROM_MEASURE_TYPE_ADDR, measurementType_);
}

// Set spectrometer gain to low or high - internal function without
// reentrance checks or EEPROM updating
void C12666MA::setGainInternal(gain_t gain)
{
    gain_ = gain;

    if (gain_ == NO_GAIN)
        pinResetFast(spec_gain_);
    else
        pinSetFast(spec_gain_);
}

// Set spectrometer gain to low or high
void C12666MA::setGain(gain_t gain, bool storeInEeprom)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    setGainInternal(gain);

    if (storeInEeprom)
        EEPROM.put(EEPROM_GAIN_ADDR, gain_);

    // delay to stabilise the changes
    delay(200);
}

// Set ADC reference voltage to one of the specified values. This is
// internal function without reentrance checks or EEPROM updating
void C12666MA::setAdcRefInternal(adc_ref_t adcRef)
{
    adcRef_ = adcRef;

    if (adcRef_ & 1)
        pinSetFast(adc_ref_sel1_);
    else
        pinResetFast(adc_ref_sel1_);

    if (adcRef_ & 2)
        pinSetFast(adc_ref_sel2_);
    else
        pinResetFast(adc_ref_sel2_);
}

// Set ADC reference voltage to one of the specified values. Defines maximum
// analogue signal voltage for conversion. This generally should be used
// together with setting the spectrometer gain (high gain will mean higher
// reference voltage) but it is decoupled for flexibility of ADC control.
void C12666MA::setAdcReference(adc_ref_t ref, bool storeInEeprom)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    // physically setup the ADC reference
    setAdcRefInternal(ref);

    if (storeInEeprom)
        EEPROM.put(EEPROM_ADC_REF_ADDR, adcRef_);

    // delay to stabilise the changes
    delay(200);
}

// Get measured data for specified pixel
// Note: Applying bandpass correction can go out of range
double C12666MA::getMeasurement(uint16_t pixelIdx, bool normalise)
{
    double data = meas_[pixelIdx];

    if (applyBandPassCorrection_)
    {
        // Stearns and Stearns (1988) bandpass correction
        if (pixelIdx == 0)
            data = 1.083*data - 0.083*meas_[pixelIdx+1];
        else if (pixelIdx == rangePixels_-1)
            data = 1.083*data - 0.083*meas_[pixelIdx-1];
        else
            data = 1.166*data - 0.083*meas_[pixelIdx-1]
                              - 0.083*meas_[pixelIdx+1];
    }

    double adcRefVoltage = adcVoltages[lastMeasADCRef_];
    double satVoltage = (lastMeasGain_ == HIGH_GAIN)
                                ? satVoltageHighGain_
                                : satVoltageNoGain_;
    bool saturated = data > satVoltage;
    if (data > blackLevels_[pixelIdx])
    {
        data -= blackLevels_[pixelIdx];
        if (measurementType_ == MEASURE_ABSOLUTE)
            data /= satVoltage;
        else if (measurementType_ == MEASURE_RELATIVE)
            data /= adcRefVoltage;

        // apply spectral response corrections
        if (normalise && !saturated)
            data *= normCoef_[pixelIdx];
    }
    else
        data = 0;

    return data;
}

// get the wavelength for specified pixel
double C12666MA::getWavelength(uint16_t pixelNumber)
{
    // pixelNumber in formula start with 1
    double p = pixelNumber+rangeStartIdx_+1;
    return calibration_[0]
           + (p*calibration_[1])
           + (p*p*calibration_[2])
           + (p*p*p*calibration_[3])
           + (p*p*p*p*calibration_[4])
           + (p*p*p*p*p*calibration_[5]);
}
