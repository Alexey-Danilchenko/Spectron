/*
 *  C12666MA.cpp - Hamamatsu C12666MA driver for Spectron board.
 *                 This is quite generic in handling spectrometer
 *                 read cycles. The AD7980 16 bit ADC is used to
 *                 read spectrometer output. All ADC interfaces
 *                 are tuned to run as fast as possible on Photon
 *                 hardware (STM32F205) at the price of portability.
 *
 *  Copyright 2015-2017 Alexey Danilchenko, Iliah Borg
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

// standard SPI pins
#define SPI_MOSI    A5
#define SPI_MISO    A4
#define SPI_SCK     A3

// EEPROM addresses
#define EEPROM_GAIN_ADDR         0
#define EEPROM_ADC_REF_ADDR      4
#define EEPROM_BLACK_MODE_ADDR   8

#define EEPROM_FREE_ADDR         12  // free address for application usage

//
// State flow with no dark reading:
//     Lead -> Reset -> Reset2 -> Integration -> Read -> Trail -> Stop
// triggering external action happens at the beginning of Integration
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

// Max ADC conversion value - 16 bit
#define ADC_MAX_VALUE   UINT16_MAX

// C12666MA:
//      Integration time = (INTEG_TICKS + READ_TICKS)/2/frequency
//      Integration time limits from datasheet: 0.01 sec to 10 sec
//      Each read takes 4 CLK cycles; 6 CLK cycles min. after the last read
//           for 256 pixels train is 4*256+6=1030 CLK cycles
//      Numbers below are in ticks: 2 ticks (h/l and l/h) per clock cycle
//      even numbers only!
#define TICKS_PER_PIXEL    8              // Sensor spec - ticks per single pixel readout
#define MIN_INTEG_TIME_US  1000UL         // 1ms minimum integration time to ensure Integration state will always be used
#define MAX_INTEG_TIME_US  10000000UL     // 10s maximum integration time
#define DEF_LEAD_TICKS     64             // anything greater than 38 seems OK up to 200KHz clock, room temperature
#define TRAIL_TICKS        12             // anything greater than 2 seems OK up to 200KHz clock, room temperature
#define READ_TICKS         (SPEC_PIXELS*TICKS_PER_PIXEL + TRAIL_TICKS)
#define EXT_TRG_CYCLES     50             // duration of ext TRG pin high signal

// Integration ticks - change as needed, integration time is formed by INTEG_TICKS + READ_TICKS
static uint32_t INTEG_TICKS = 1000;

// External trigger ticks from the start of the spectro state mechine
static uint32_t EXT_TRG_TICKS = EXT_TRG_CYCLES + DEF_LEAD_TICKS + READ_TICKS + READ_TICKS;

// Lead ticks - this can change if EXT_TRG_TICKS may need to get larger
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
static volatile bool         measuringData_ = false;
static volatile bool         timerOn = false;
static volatile spec_state_t specState;
static volatile bool         specDataReady = false;    // trigger for ADC conversion
static uint8_t               specCLK = LOW;            // current clock pin state
static uint8_t               specST  = LOW;            // current ST pin state
static uint32_t              specCounter = 0;          // counter
static uint32_t              extTrgCounter = 0;        // ext trigger counter
static uint32_t* volatile    specData = 0;             // pointer to current data for ADC reads
static uint8_t* volatile     specDataCounter = 0;      // pointer to current data for ADC reads counter

// spectrometer pins used by timer
uint8_t adcPinCNV   = NO_PIN;
uint8_t specPinCLK  = NO_PIN;
uint8_t specPinST   = NO_PIN;
uint8_t extPinTRG   = NO_PIN;

// spectrometer trigger pin - hardware access
uint16_t specPinTRG = 0;    STM32_Pin_Info* specPinTRG_Info = 0;

// pin set/read macros
#define specTrgSoftInterrupt() EXTI->SWIER = specPinTRG

// set pin fast
#define pinSetVal(pin,val) if (val) pinSetFast(pin); else pinResetFast(pin)

// C12666MA default calibration - taken from my sensor 15F00163
const double DEF_CALIBRATION[] =
    {323.3668711, 2.384682045, -5.995865297E-4, -8.602293347E-6, 1.840343099E-8, -1.424592223E-11};

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
                }
                break;

            case SPEC_STOP:
            default:
                specCLK = LOW;
                break;
        }

        // process trigger
        if (extTrgCounter)
        {
            if (extTrgCounter == EXT_TRG_CYCLES)
                pinSetFast(extPinTRG);
            else if (extTrgCounter == 1)
                pinResetFast(extPinTRG);

            --extTrgCounter;
        }
    }
}

// start active timer
void startSpecTimer()
{
    TIM_TimeBaseInitTypeDef timerInit = {0};
    NVIC_InitTypeDef nvicInit = {0};

    // set in timer guard
    if (timerOn)
        return;

    timerOn = true;

    // init state
    specCounter = 0;
    extTrgCounter = EXT_TRG_TICKS;
    specState = SPEC_LEAD;
    specDataReady = false;

    // initial and next values of CLK and ST
    pinResetFast(specPinCLK);  // CLK initially low
    pinSetFast(specPinST);     // ST  initially high
    pinResetFast(extPinTRG);   // Ext trigger initially low
    specST  = HIGH;
    specCLK = CLK_HIGH;

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
    pinResetFast(extPinTRG);

    timerOn = false;
}

// Constructor
C12666MA::C12666MA(uint8_t spec_trg, uint8_t spec_eos, uint8_t spec_gain, uint8_t spec_clk, uint8_t spec_st,
                   uint8_t ext_trg, uint8_t adc_ref_sel1, uint8_t adc_ref_sel2, uint8_t adc_cnv,
                   const double *calibration)
{
    spec_trg_ = spec_trg;
    spec_eos_ = spec_eos;
    spec_gain_ = spec_gain;
    spec_clk_ = spec_clk;
    spec_st_ = spec_st;
    ext_trg_ = ext_trg;
    adc_ref_sel1_ = adc_ref_sel1;
    adc_ref_sel2_ = adc_ref_sel2;
    adc_cnv_ = adc_cnv;
    calibration_ = (calibration == 0 ? DEF_CALIBRATION : calibration);
    measuringData_ = false;
    applyBandPassCorrection_ = true;

    timerOn = false;
    specState = SPEC_STOP;
    specDataReady = false;

    // Initialize arrays
    for (int i=0; i<SPEC_PIXELS; i++)
    {
        blackLevels_[i] = 0.0;
        data_[i] = 0.0;
    }

    // read saved data and set defaults
    EEPROM.get(EEPROM_GAIN_ADDR, gain_);
    if (gain_ == 0xFFFFFFFF)
        // EEPROM was empty
        gain_ = NO_GAIN;

    EEPROM.get(EEPROM_ADC_REF_ADDR, adcRef_);
    if (adcRef_ == 0xFFFFFFFF)
        // EEPROM was empty
        adcRef_ = ADC_5V;

    EEPROM.get(EEPROM_BLACK_MODE_ADDR, blackMode_);
    if (blackMode_ == 0xFFFFFFFF)
        // EEPROM was empty
        blackMode_ = MANUAL_BLACK;

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
    pinMode(ext_trg_,      OUTPUT);

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
    pinResetFast(ext_trg_);

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
    setBlackMode((black_t)blackMode_, false);
    setGain((gain_t)gain_, false);
    setAdcReference((adc_ref_t)adcRef_, false);

    // set internal pins
    specPinCLK  = spec_clk_;
    specPinST   = spec_st_;
    adcPinCNV   = adc_cnv_;
    extPinTRG   = ext_trg_;

    // register particle variables
    success = success && Particle.variable("spGain",      gain_);
    success = success && Particle.variable("spADCRef",    adcRef_);
    success = success && Particle.variable("spBlackMode", blackMode_);

    return success;
}

// Sets the external trigger to measurement delay time. This defines time interval
// in uSec that offsets external trigger from the measurement. I.e. external trigger
// is raised and after this delay the integration and measurement starts.
void C12666MA::setExtTrgMeasDelay(uint32_t extTrgMeasDelayUs)
{
    uint32_t delayTimeTicks = (extTrgMeasDelayUs * TIMER_US_FACTOR) / SPEC_CLK_TICK_TIMER;

    if (delayTimeTicks & 1)
        --delayTimeTicks;

    // work out if delayfits into current lead + reset + reset2 periods
    if (delayTimeTicks > DEF_LEAD_TICKS + READ_TICKS + READ_TICKS)
        LEAD_TICKS = delayTimeTicks - READ_TICKS - READ_TICKS;
    else
        LEAD_TICKS = DEF_LEAD_TICKS;

    // set trigger cycles
    EXT_TRG_TICKS = EXT_TRG_CYCLES + LEAD_TICKS + READ_TICKS + READ_TICKS - delayTimeTicks;
}

// Sets the trigger pin to specifid one. This could be used to trigger different things
// with different type of measurements
void C12666MA::setExtTriggerPin(uint8_t ext_trg)
{
    if (measuringData_ || ext_trg == NO_PIN)
        return;

    ext_trg_ = ext_trg;
    extPinTRG = ext_trg_;
    pinMode(ext_trg_, OUTPUT);
    pinResetFast(ext_trg_);
}

// Set the integration (or sample collection) time, in microseconds
void C12666MA::setIntTime(uint32_t timeUs)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
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
        INTEG_TICKS = MIN_INTEG_TIME_US*TIMER_US_FACTOR / SPEC_CLK_TICK_TIMER;

    // even up INTEG_TICKS
    if (INTEG_TICKS & 1)
        INTEG_TICKS++;
}

// convert aggregated readouts to measurement floating point data
void processMeasurement(float* measurement)
{
    // Initialize arrays
    for (int i=0; i<SPEC_PIXELS; i++)
    {
        measurement[i] = 0.0;

        if (dataCounts[i])
            measurement[i] = (float)data[i]/(float)(dataCounts[i]*ADC_MAX_VALUE);
    }
}

// Take single measurement
void C12666MA::takeMeasurement()
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    measuringData_ = true;

    // read leading black if needed
    if (blackMode_ == LEADING_BLACK)
    {
        readSpectrometer();
        processMeasurement(blackLevels_);
    }

    // read main measurement data
    readSpectrometer();
    processMeasurement(data_);

    // read leading black if needed
    if (blackMode_ == FOLLOWUP_BLACK)
    {
        readSpectrometer();
        processMeasurement(blackLevels_);
    }

    measuringData_ = false;
}

// Take single black level measurement
void C12666MA::takeBlackMeasurement()
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    measuringData_ = true;

    // read black if needed
    readSpectrometer();
    processMeasurement(blackLevels_);

    measuringData_ = false;
}

// This routine to initiate and read spectrometer measurement data
void C12666MA::readSpectrometer()
{
    // no action if timer is on or in measurement
    if (timerOn)
        return;

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

    // init ADC
    startADC(adc_cnv_);

    // initiate the timer
    startSpecTimer();

    // loop until stop
    while (specState != SPEC_STOP)
        ;

    // stop the timer and cleanup
    stopSpecTimer();
    endADC();
}

// Enable/disable Stearns and Stearns (1988) bandpass correction
void C12666MA::enableBandpassCorrection(bool enable)
{
    if (timerOn || measuringData_)
        return;

    applyBandPassCorrection_ = enable;
}

// Set the mode to do black measurement. Could be set to one of the
// three modes:
//     MANUAL_BLACK   - no auto black measurement at all. Suitable for
//                      "always lit" spectrometer measurement in which
//                      case black measurement could be taken manually.
//     FOLLOWUP_BLACK - measures black following the main measurement at
//                      the same integration time
//     LEADING_BLACK  - measures black prior to the main measurement at
//                      the same integration time
//
// The last two are automatic ways to measure black suitable for measurement
// type where spectrometer illumination could be controlled so the dark
// output could be measured either before or after illumination.
void C12666MA::setBlackMode(black_t blackMode, bool storeInEeprom)
{
    if (timerOn || measuringData_)
        return;

    blackMode_ = blackMode;

    // store the position and counter in EEPROM
    if (storeInEeprom)
        EEPROM.put(EEPROM_BLACK_MODE_ADDR, blackMode_);

    // wipe out blacks
    for (int i=0; i<SPEC_PIXELS; i++)
        blackLevels_[i] = 0.0;
}

// Set spectrometer gain to low or high
void C12666MA::setGain(gain_t gain, bool storeInEeprom)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    gain_ = gain;
    if (gain == NO_GAIN)
        pinResetFast(spec_gain_);
    else
        pinSetFast(spec_gain_);

    // store the position and counter in EEPROM
    if (storeInEeprom)
        EEPROM.put(EEPROM_GAIN_ADDR, gain_);

    // delay to stabilise the changes
    delay(200);
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

    adcRef_ = ref;
    if (ref & 1)
        pinSetFast(adc_ref_sel1_);
    else
        pinResetFast(adc_ref_sel1_);

    if (ref & 2)
        pinSetFast(adc_ref_sel2_);
    else
        pinResetFast(adc_ref_sel2_);

    // store the position and counter in EEPROM
    if (storeInEeprom)
        EEPROM.put(EEPROM_ADC_REF_ADDR, adcRef_);

    // delay to stabilise the changes
    delay(200);
}

// auxiliary function to get data at specified pixel
inline double getData(uint16_t pixelIdx, bool subtractBlack, float* data, float* black)
{
    if (!subtractBlack) 
        return data[pixelIdx]; 
 
    return (data[pixelIdx] < black[pixelIdx]) 
            ? 0 
            : data[pixelIdx] - black[pixelIdx]; 
}

// Get measured data for specified pixel (with or without black level adjustment)
double C12666MA::getMeasurement(uint16_t pixelIdx, bool subtractBlack)
{
    double data = getData(pixelIdx, subtractBlack, data_, blackLevels_); 
    
    if (applyBandPassCorrection_)
    {
        // Stearns and Stearns (1988) bandpass correction
        if (pixelIdx == 0)
            data = 1.083*data - 0.083*getData(pixelIdx+1, subtractBlack, data_, blackLevels_);
        else if (pixelIdx == SPEC_PIXELS-1)
            data = 1.083*data - 0.083*getData(pixelIdx-1, subtractBlack, data_, blackLevels_);
        else
            data = 1.166*data - 0.083*getData(pixelIdx-1, subtractBlack, data_, blackLevels_)
                              - 0.083*getData(pixelIdx+1, subtractBlack, data_, blackLevels_);
    }
    
    return data;
}

// Get the read black value for specified pixel
double C12666MA::getBlackMeasurement(uint16_t pixelIdx)
{
    double black = blackLevels_[pixelIdx]; 
    
    if (applyBandPassCorrection_)
    {
        // Stearns and Stearns (1988) bandpass correction
        if (pixelIdx == 0)
            black = 1.083*black - 0.083*blackLevels_[pixelIdx+1];
        else if (pixelIdx == SPEC_PIXELS-1)
            black = 1.083*black - 0.083*blackLevels_[pixelIdx-1];
        else
            black = 1.166*black - 0.083*blackLevels_[pixelIdx-1]
                                - 0.083*blackLevels_[pixelIdx+1];
    }
    
    return black;
}

// get the wavelength for specified pixel
double C12666MA::getWavelength(uint16_t pixelNumber)
{
    // pixelNumber in formula start with 1
    double p = pixelNumber+1;
    return calibration_[0]
           + (p*calibration_[1])
           + (p*p*calibration_[2])
           + (p*p*p*calibration_[3])
           + (p*p*p*p*calibration_[4])
           + (p*p*p*p*p*calibration_[5]);
}
