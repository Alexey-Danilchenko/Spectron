/*
 *  C12880MA.cpp - Hamamatsu C12880MA driver for Spectron board.
 *                 This is quite generic in handling spectrometer
 *                 read cycles. The AD7980 16 bit ADC is used to
 *                 read spectrometer output. All ADC interfaces
 *                 are tuned to run as fast as possible on Photon
 *                 hardware (STM32F205) at the price of portability.
 *
 *  Copyright 2017-2018 Alexey Danilchenko, Iliah Borg
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

#include "C12880MA.h"

// standard SPI pins
#define SPI_MOSI    A5
#define SPI_MISO    A4
#define SPI_SCK     A3

// EEPROM addresses
#define EEPROM_ADC_REF_ADDR          EEPROM_C12880_BASE_ADDR
#define EEPROM_MEASURE_TYPE_ADDR     EEPROM_C12880_BASE_ADDR+4
#define EEPROM_INTEGRATION_TIME      EEPROM_C12880_BASE_ADDR+8
#define EEPROM_TRG_MEAS_DELAY        EEPROM_C12880_BASE_ADDR+12
#define EEPROM_SAT_VOLTAGE           EEPROM_C12880_BASE_ADDR+16
#define EEPROM_CALIBRATION_COEF_1    EEPROM_C12880_BASE_ADDR+20
#define EEPROM_CALIBRATION_COEF_2    EEPROM_C12880_BASE_ADDR+28
#define EEPROM_CALIBRATION_COEF_3    EEPROM_C12880_BASE_ADDR+36
#define EEPROM_CALIBRATION_COEF_4    EEPROM_C12880_BASE_ADDR+44
#define EEPROM_CALIBRATION_COEF_5    EEPROM_C12880_BASE_ADDR+52
#define EEPROM_CALIBRATION_COEF_6    EEPROM_C12880_BASE_ADDR+60

// Saturation voltage limits from Hamamatsu C12880A spec sheet
#define MIN_SAT_VOLTAGE  4.1
#define MAX_SAT_VOLTAGE  5.2

//
// State flow with single read integration cycle:
//    Ext.Trigger -> Lead -> Integration -> Read -> Trail -> Stop
//
// External triggering action is optional so without it starting state
// is Lead.
//
enum spec_state_t {
    SPEC_EXT_TRIG,
    SPEC_LEAD,
    SPEC_INTEGRATION,
    SPEC_READ,
    SPEC_TRAIL,
    SPEC_STOP
};

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//  The following values are calculated and working on Photon only
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// Timer prescaler - this is what CPU counter clock frequency is divided by to get the frequency
//      generally prescaler is calculated as SYSCORECLOCK (60000000 for Photon for base timers)
//      divided by the frequency of the timer counter. For example:
//
//      TIMER_PRESCALER = (SYSCORECLOCK / 1000000) - 1 to get TIM counter clock = 1MHz
//
#define TIMER_PRESCALER    5        // for basic timers at 60MHz it gives 100ns timer unit counter
#define TIMER_US_FACTOR   10        // conversion factor to/from timer units and microseconds

// This determines minimal clock tick duration for spectrometer,
// in above timer units (100ns units). It should be larger than
// ADC conversion times for AD7980.
#define SPEC_CLK_200KHZ      25   // this is Hamamatsu spec minimum - leaves only enough time for one ADC read
#define SPEC_CLK_156KHZ      32   // this is minimum at which there is enough time for 2 averaging ADC reads

// Current selection - use one of the above as needed
#define SPEC_CLK_TICK_TIMER  SPEC_CLK_156KHZ

// Macro to convert ticks to uSec and uSec to ticks
#define ticksToUsec(x) ((x)*SPEC_CLK_TICK_TIMER/TIMER_US_FACTOR)
#define uSecToTicks(x) ((x)*TIMER_US_FACTOR/SPEC_CLK_TICK_TIMER)

// Max ADC conversion value - 16 bit
#define ADC_MAX_VALUE   UINT16_MAX

// C12880MA:
//      Integration time = INTEG_TICKS/2/frequency
//      Integration time limits from datasheet: (6 + 48)/frequency to 10 sec
//      Each read takes 1 CLK cycle; 4 CLK cycles min. after the last read;
//      and 87 CLK cycles before TRG indicates valid data
//           for 288 pixels train is 87+1*288+4=379 CLK cycles
//      Numbers below are in ticks: 2 ticks (h/l and l/h) per clock cycle
//      even numbers only!
#define TICKS_PER_PIXEL      2            // Sensor spec - ticks per single pixel readout
#define MIN_INTEG_TIME_TICKS 108          // 6+48 clock periods - see C12880 datasheet
#define MAX_INTEG_TIME_US    1000000UL    // 1s maximum integration time
#define LEAD_TICKS           64           // this includes INTEG_START_TICKS - anything greater than 38 seems OK
#define TRAIL_TICKS          16           // anything greater than 2 seems OK
#define ST_LEAD_TICKS        7            // ticks after ST goes high (on falling CLK) when integration really starts
#define READ_TICKS           ((87 + SPEC_PIXELS)*TICKS_PER_PIXEL)
#define TRG_CYCLES           (SPEC_PIXELS + 88)     // spec pixels + 88 cycles after ST goes low
#define EXT_TRG_HIGH_TICKS   uSecToTicks(1000)      // duration of ext TRG pin high signal - 1mSec

// Integration ticks - change as needed
static uint32_t INTEG_TICKS = MIN_INTEG_TIME_TICKS;

// Ext trigger ticks - by default trigger at EXT_TRG_HIGH_TICKS before the end of LEAD state
static uint32_t EXT_TRG_TICKS = EXT_TRG_HIGH_TICKS+2;

// ADC conversion delay as per AD7980 spec sheet - CS mode-3 wire without Busy ind
static const uint32_t adcConvTimeTicks  = (71*System.ticksPerMicrosecond())/100;  // 710ns

// ADC reference voltages
static const float adcVoltages[] = { 2.5, 3.0, 4.096, 5.0 };

// spectrometer states and trigger variables
static volatile bool         timerOn = false;
static volatile spec_state_t specState;
static uint32_t              specCounter = 0;          // tick counter
static uint32_t              specCLK = 0;              // current clock pin state
static uint32_t              specST = 0;               // current ST pin state
static volatile uint32_t     extTrigger = 0;           // current trigger pin state
static volatile uint32_t     specTRGCounter = 0;       // spec TRG cycles counter
static volatile uint32_t     extTRGCounter = 0;        // ext TRG cycles counter
static volatile uint16_t     specReadCycleCounter = 0; // reading cycles counter
static uint32_t*             specData = 0;             // pointer to current data for ADC reads
static uint16_t*             specDataCounter = 0;      // pointer to current data for ADC reads counter

// spectrometer pins used by timer - direct hardware access, the fastest way
// input pins
uint16_t specPinTRG  = 0; __IO uint32_t* specPinTRG_IN = 0;  STM32_Pin_Info* specPinTRG_Info = 0;
// output pins - low, high, toggle masks and bit set/reset register
uint32_t specPinCLK_L  = 0; uint32_t specPinCLK_H  = 0; uint32_t specPinCLK_TM  = 0; __IO uint32_t* specPinCLK_BR = 0;
uint32_t specPinST_L   = 0; uint32_t specPinST_H   = 0; uint32_t specPinST_TM   = 0; __IO uint32_t* specPinST_BR = 0;
uint32_t adcPinCNV_L   = 0; uint32_t adcPinCNV_H   = 0; uint32_t adcPinCNV_TM   = 0; __IO uint32_t* adcPinCNV_BR = 0;
// external registering device (camera) and external light source triggers - output
uint32_t extPinTrig_L  = 0;  uint32_t extPinTrig_H  = 0;  uint32_t extPinTrig_TM  = 0;  __IO uint32_t* extPinTrig_BR  = 0;
uint32_t extPinLight_L = 0;  uint32_t extPinLight_H = 0;  uint32_t extPinLight_TM = 0;  __IO uint32_t* extPinLight_BR = 0;

// pin set/read macros
#define pinHigh(pin)          (*pin##_BR) = pin##_H
#define pinLow(pin)           (*pin##_BR) = pin##_L
#define pinSet(pin,val)       (*pin##_BR) = val
#define pinValToggle(val,pin) val ^= pin##_TM
#define pinRead(pin)          ((*pin##_IN) & pin)
#define pinDefined(pin)       (pin##_BR) != 0

// Static internal sensor readings arrays - these hold
// aggregated sensor measurements and measurement counts
static uint32_t data[SPEC_PIXELS];
static uint16_t dataCounts[SPEC_PIXELS];


// ------------------------------
// Hardware specific routines
// ------------------------------
#include "gpio_hal.h"
#include "pinmap_hal.h"
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
#define SPI_BASE ((SPI_TypeDef *) SPI1_BASE)

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
    pinLow(adcPinCNV);
}

// force inlining
inline void readADC(uint32_t* data, uint16_t* dataCounts) __attribute__((always_inline));

// Function to perform reading ADC7980.
// This does up to two accumulated reads of the ADC
// where the state of the TRG pin is checked following
// the second ADC conversion completion
inline void readADC(uint32_t* data, uint16_t* dataCounts)
{
    // 1st read
    // initiate conversion and wait for max conversion time
    pinHigh(adcPinCNV);
    System.ticksDelay(adcConvTimeTicks);
    pinLow(adcPinCNV);

    // SPI enable
    SPI_BASE->CR1 |= SPI_CR1_SPE;

    // Wait for SPI data reception
    while (SPI_BASE->SR & SPI_I2S_FLAG_RXNE == 0) ;

    // Read SPI received data
    *data += SPI_BASE->DR;

    // disable
    SPI_BASE->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);

    // 2nd read
    // initiate conversion and wait for max conversion time
    if (pinRead(specPinTRG))
    {
        pinHigh(adcPinCNV);
        System.ticksDelay(adcConvTimeTicks);
        pinLow(adcPinCNV);

        // SPI enable
        SPI_BASE->CR1 |= SPI_CR1_SPE;

        // Wait for SPI data reception
        while (SPI_BASE->SR & SPI_I2S_FLAG_RXNE == 0) ;

        // Read SPI received data
        *data += SPI_BASE->DR;

        // disable
        SPI_BASE->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);

        // increase count for the second read
        ++(*dataCounts);
    }

    // increase count for the first read
    ++(*dataCounts);
}

// deinitialise ADC SPI
inline void endADC()
{
    // Enable SPI1 reset state
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);
    // Release SPI1 from reset state
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);
}


// --------------------------------------------------
//   Timer and spectrometer clock handling routines
// --------------------------------------------------
// TRG pin handling interrupt
void spectroTRGInterrupt(void)
{
    if ((EXTI->PR & specPinTRG) && (EXTI->IMR & specPinTRG))
    {
        EXTI->PR = specPinTRG;

        // spec trigger counting is on
        if (specTRGCounter) {
            --specTRGCounter;

            // we are on a reading phase
            if (specTRGCounter < SPEC_PIXELS)
                readADC(specData++, specDataCounter++);
        }
    }

    // call system interrupt
    if (sysIrqHandler)
        sysIrqHandler();
}

// Spectrometer timer interrupt call. A single cycle is controlled by a
// state machine:
//    Ext.Trigger -> Lead -> Integration -> Read -> Trail -> Stop
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

        // write CLK,ST and ext trigger immediately
        pinSet(specPinCLK, specCLK);
        pinSet(specPinST,  specST);

        // toggle CLK
        pinValToggle(specCLK, specPinCLK);

        // state machine
        switch (specState) {
            case SPEC_EXT_TRIG:
                --specCounter;
                if (specCounter == 0) {
                    // triggering done - move to next step
                    specState = SPEC_LEAD;
                    specCounter = LEAD_TICKS;
                    // enable external light if defined
                    if (pinDefined(extPinLight))
                        pinHigh(extPinLight);
                } else if (extTRGCounter) {
                    --extTRGCounter;
                    if (extTRGCounter == 0)
                        pinLow(extPinTrig);
                }
                break;

            case SPEC_LEAD:
                --specCounter;
                if (specCounter == ST_LEAD_TICKS) {
                    // raise ST - some lead ticks are non-integrating
                    specST = specPinST_H;
                } else if (specCounter == 0) {
                    specCounter = INTEG_TICKS;
                    specState = SPEC_INTEGRATION;
                }
                break;

            case SPEC_INTEGRATION:
                --specCounter;
                if (specCounter==1)
                    // bring ST down - initiate integration stop
                    specST = specPinST_L;
                else if (specCounter==0) {
                    // start TRG count
                    specTRGCounter = TRG_CYCLES;
                    specCounter = READ_TICKS;
                    specState = SPEC_READ;
                }
                break;

            case SPEC_READ:
                --specCounter;
                if (specCounter==0) {
                    specCounter = TRAIL_TICKS;
                    specState = SPEC_TRAIL;
                }
                break;

            case SPEC_TRAIL:
                --specCounter;
                if (specCounter==0) {
                    --specReadCycleCounter;
                    if (specReadCycleCounter > 0) {
                        // initialise data variables and start another cycle
                        specData = data;
                        specDataCounter = dataCounts;
                        specCounter = LEAD_TICKS;
                        specState = SPEC_LEAD;
                    } else {
                        specCLK = specPinCLK_L;
                        specState = SPEC_STOP;
                        // disable external light if defined
                        if (pinDefined(extPinLight))
                            pinLow(extPinLight);
                    }
                }
                break;

            case SPEC_STOP:
            default:
                specCLK = specPinCLK_L;
                break;
        }
    }
}

// start active timer
void startSpecTimer(bool doExtTriggering)
{
    TIM_TimeBaseInitTypeDef timerInit = {0};
    NVIC_InitTypeDef nvicInit = {0};

    // set in timer guard
    if (timerOn)
        return;

    timerOn = true;

    // init state
    specTRGCounter = 0;
    if (doExtTriggering && pinDefined(extPinTrig) && EXT_TRG_TICKS > 0)
    {
        specCounter   = EXT_TRG_TICKS;
        extTRGCounter = EXT_TRG_HIGH_TICKS;
        specState     = SPEC_EXT_TRIG;
        pinHigh(extPinTrig);
    }
    else
    {
        specCounter = LEAD_TICKS;
        specState   = SPEC_LEAD;
    }

    // set all spec pins low
    pinLow(specPinCLK);
    pinLow(specPinST);

    // initial values of CLK and ST
    specCLK = specPinCLK_H;    // CLK initially high
    specST = specPinST_L;      // ST initially low

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

    // enable TRG pin interrupt
    EXTI->IMR  |= specPinTRG;    // enable interrupt
    EXTI->RTSR |= specPinTRG;    // set raising edge

    // enable timer IRQ
    nvicInit.NVIC_IRQChannel                   = GPIO_IRQn[specPinTRG_Info->gpio_pin_source];
    nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
    nvicInit.NVIC_IRQChannelSubPriority        = 0;
    nvicInit.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvicInit);

    // if we are not starting with external trigger - switch on light
    if (!doExtTriggering && pinDefined(extPinLight))
        pinHigh(extPinLight);
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
    EXTI->PR = specPinTRG;       // clear pending
    EXTI->IMR  &= ~specPinTRG;   // mask interrupt
    EXTI->RTSR &= ~specPinTRG;   // clear raising edge

    // disable NVIC IRQ line if it is not shared
    if (sysIrqHandler == 0)
    {
        nvicInit.NVIC_IRQChannel    = GPIO_IRQn[specPinTRG_Info->gpio_pin_source];
        nvicInit.NVIC_IRQChannelCmd = DISABLE;
        NVIC_Init(&nvicInit);
    }

    // reset pins
    pinLow(specPinST);
    specST = specPinST_L;
    pinLow(specPinCLK);
    specCLK = specPinCLK_L;

    timerOn = false;
}


// ---------------------------------------
//   C12880MA class and related routines
// ---------------------------------------

// Constructor
C12880MA::C12880MA(uint8_t spec_eos, uint8_t spec_trg, uint8_t spec_clk, uint8_t spec_st,
                   uint8_t adc_ref_sel1, uint8_t adc_ref_sel2, uint8_t adc_cnv,
                   uint8_t ext_trg, uint8_t ext_trg_ls, const double *defaultCalibration)
{
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

    measuringData_ = false;
    applyBandPassCorrection_ = true;

    timerOn = false;
    specState = SPEC_STOP;

    // Initialize arrays
    for (int i=0; i<SPEC_PIXELS; i++)
    {
        blackLevels_[i] = 0.0;
        data_[i] = 0.0;
    }

    // read saved data and set defaults
    EEPROM.get(EEPROM_MEASURE_TYPE_ADDR, measurementType_);
    if (measurementType_ != MEASURE_RELATIVE &&
        measurementType_ != MEASURE_VOLTAGE &&
        measurementType_ != MEASURE_ABSOLUTE)
        // EEPROM was empty
        measurementType_ = MEASURE_RELATIVE;

    // read saved data and set defaults
    EEPROM.get(EEPROM_ADC_REF_ADDR, adcRef_);
    if (adcRef_ != ADC_2_5V   && adcRef_ != ADC_3V  &&
        adcRef_ != ADC_4_096V && adcRef_ != ADC_5V)
        // EEPROM was empty
        adcRef_ = ADC_5V;

    uint32_t trgMeasDelayTicks = 0;
    EEPROM.get(EEPROM_TRG_MEAS_DELAY, trgMeasDelayTicks);
    if (trgMeasDelayTicks == 0 ||
        (trgMeasDelayTicks > EXT_TRG_HIGH_TICKS
         && ticksToUsec(trgMeasDelayTicks) < 10000000))  // 10 sec as top limit
        EXT_TRG_TICKS = trgMeasDelayTicks;

    uint32_t intTimeTicks = 0;
    EEPROM.get(EEPROM_INTEGRATION_TIME, intTimeTicks);
    if (intTimeTicks >= MIN_INTEG_TIME_TICKS
        && ticksToUsec(intTimeTicks) < MAX_INTEG_TIME_US)
        INTEG_TICKS = intTimeTicks;
    else
        setIntTime(500 _uSEC, false);

    float satVoltage = 0.0;
    EEPROM.get(EEPROM_SAT_VOLTAGE, satVoltage);
    // check the saturation voltage against Hamamatsu spec limits
    if (satVoltage >= MIN_SAT_VOLTAGE && satVoltage <= MAX_SAT_VOLTAGE)
        satVoltage_ = satVoltage;
    else
        satVoltage_ = MIN_SAT_VOLTAGE;

    EEPROM.get(EEPROM_CALIBRATION_COEF_1, calibration_[0]);
    if (calibration_[0] > 100 && calibration_[0] < 500)  // first coeff should be around 300
    {
        EEPROM.get(EEPROM_CALIBRATION_COEF_2, calibration_[1]);
        EEPROM.get(EEPROM_CALIBRATION_COEF_3, calibration_[2]);
        EEPROM.get(EEPROM_CALIBRATION_COEF_4, calibration_[3]);
        EEPROM.get(EEPROM_CALIBRATION_COEF_5, calibration_[4]);
        EEPROM.get(EEPROM_CALIBRATION_COEF_6, calibration_[5]);
    }
    else if (defaultCalibration)
    {
        setWavelengthCalibration(defaultCalibration);
    }
}

// Destructor
C12880MA::~C12880MA()
{
}

// Setup methods
bool C12880MA::begin()
{
    bool success = true;

    // Setup pins
    pinMode(adc_ref_sel1_, OUTPUT);
    pinMode(adc_ref_sel2_, OUTPUT);
    pinMode(adc_cnv_,      OUTPUT);
    pinMode(SPI_MOSI,      OUTPUT);
    pinMode(SPI_MISO,      INPUT);
    pinMode(SPI_SCK,       OUTPUT);
    pinMode(spec_eos_,     INPUT);
    pinMode(spec_trg_,     INPUT);
    pinMode(spec_st_,      OUTPUT);
    pinMode(spec_clk_,     OUTPUT);

    if (ext_trg_ != NO_PIN)
        pinMode(ext_trg_,  OUTPUT);
    if (ext_trg_ls_ != NO_PIN)
        pinMode(ext_trg_ls_, OUTPUT);

    // setup hardware and fixed pins
    STM32_Pin_Info* PIN_MAP = HAL_Pin_Map();
    // TRG - input
    specPinTRG_Info = &PIN_MAP[spec_trg_];
    specPinTRG      = PIN_MAP[spec_trg_].gpio_pin;
    specPinTRG_IN   = &(PIN_MAP[spec_trg_].gpio_peripheral->IDR);
    // CLK - outpout
    specPinCLK_H  = PIN_MAP[spec_clk_].gpio_pin;
    specPinCLK_L  = specPinCLK_H << 16;
    specPinCLK_TM = specPinCLK_H | specPinCLK_L;
    specPinCLK_BR = (uint32_t*)&(PIN_MAP[spec_clk_].gpio_peripheral->BSRRL);
    // ST - output
    specPinST_H  = PIN_MAP[spec_st_].gpio_pin;
    specPinST_L  = specPinST_H << 16;
    specPinST_TM = specPinST_H | specPinST_L;
    specPinST_BR = (uint32_t*)&(PIN_MAP[spec_st_].gpio_peripheral->BSRRL);
    // ADC CNV - output
    adcPinCNV_H  = PIN_MAP[adc_cnv_].gpio_pin;
    adcPinCNV_L  = adcPinCNV_H << 16;
    adcPinCNV_TM = adcPinCNV_H | adcPinCNV_L;
    adcPinCNV_BR = (uint32_t*)&(PIN_MAP[adc_cnv_].gpio_peripheral->BSRRL);
    // external device (camera) trigger - output
    if (ext_trg_ != NO_PIN)
    {
        extPinTrig_H  = PIN_MAP[ext_trg_].gpio_pin;
        extPinTrig_L  = extPinTrig_H << 16;
        extPinTrig_TM = extPinTrig_H | extPinTrig_L;
        extPinTrig_BR = (uint32_t*)&(PIN_MAP[ext_trg_].gpio_peripheral->BSRRL);
    }
    // external light trigger - output
    if (ext_trg_ls_ != NO_PIN)
    {
        extPinLight_H  = PIN_MAP[ext_trg_ls_].gpio_pin;
        extPinLight_L  = extPinLight_H << 16;
        extPinLight_TM = extPinLight_H | extPinLight_L;
        extPinLight_BR = (uint32_t*)&(PIN_MAP[ext_trg_ls_].gpio_peripheral->BSRRL);
    }

    // reset everything
    pinResetFast(adc_cnv_);
    pinResetFast(SPI_MOSI);
    pinResetFast(SPI_SCK);
    pinResetFast(spec_st_);
    pinResetFast(spec_clk_);
    if (ext_trg_ != NO_PIN)
        pinResetFast(ext_trg_);
    if (ext_trg_ls_ != NO_PIN)
        pinResetFast(ext_trg_ls_);

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
    setAdcRefInternal(adcRef_);

    return success;
}

// Sets the wavelength calibration coeffiecients. This is comma separated
// list of 6 values that represent polinomial coefficients. Usually provided
// by Hamamatsu but can be overriden by user calculated ones.
void C12880MA::setWavelengthCalibration(const double* wavelengthCal, bool storeInEeprom)
{
    if (!wavelengthCal)
        return;

    calibration_[0] = wavelengthCal[0];
    calibration_[1] = wavelengthCal[1];
    calibration_[2] = wavelengthCal[2];
    calibration_[3] = wavelengthCal[3];
    calibration_[4] = wavelengthCal[4];
    calibration_[5] = wavelengthCal[5];

    if (storeInEeprom)
    {
        EEPROM.put(EEPROM_CALIBRATION_COEF_1, calibration_[0]);
        EEPROM.put(EEPROM_CALIBRATION_COEF_2, calibration_[1]);
        EEPROM.put(EEPROM_CALIBRATION_COEF_3, calibration_[2]);
        EEPROM.put(EEPROM_CALIBRATION_COEF_4, calibration_[3]);
        EEPROM.put(EEPROM_CALIBRATION_COEF_5, calibration_[4]);
        EEPROM.put(EEPROM_CALIBRATION_COEF_6, calibration_[5]);
    }
}

// Sets the external trigger to measurement delay time. This defines time interval
// in uSec that offsets external trigger from the measurement. I.e. external trigger
// is raised and after this delay the integration and measurement starts.
//
// Specifying negative delay disables external triggering
void C12880MA::setExtTrgMeasDelay(int32_t extTrgMeasDelayUs, bool storeInEeprom)
{
    uint32_t delayTimeTicks = (extTrgMeasDelayUs * TIMER_US_FACTOR) / SPEC_CLK_TICK_TIMER;

    // cannot be less than extr trigger high holding cycles
    if (delayTimeTicks < EXT_TRG_HIGH_TICKS)
        delayTimeTicks = EXT_TRG_HIGH_TICKS;

    if (delayTimeTicks & 1)
        --delayTimeTicks;

    EXT_TRG_TICKS = extTrgMeasDelayUs < 0 ? 0 : delayTimeTicks+2;

    if (storeInEeprom)
        EEPROM.put(EEPROM_TRG_MEAS_DELAY, EXT_TRG_TICKS);
}

// Retrieve currently set ext trigger delay
int32_t C12880MA::getExtTrgMeasDelay()
{
    return EXT_TRG_TICKS ? ticksToUsec(EXT_TRG_TICKS-2) : -1;
}

// Set the integration (or sample collection) time, in microseconds
void C12880MA::setIntTime(uint32_t timeUs, bool storeInEeprom)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    uint32_t intTime = timeUs*TIMER_US_FACTOR;

    if (intTime < MIN_INTEG_TIME_TICKS*SPEC_CLK_TICK_TIMER)
        intTime = MIN_INTEG_TIME_TICKS*SPEC_CLK_TICK_TIMER;

    if (intTime > MAX_INTEG_TIME_US*TIMER_US_FACTOR)
        intTime = MAX_INTEG_TIME_US*TIMER_US_FACTOR;

    // at 200 kHz minimal clock we can only do max 2x ADC averaging reads
    // now determine integration ticks
    INTEG_TICKS = intTime/SPEC_CLK_TICK_TIMER;

    // this should never execute but better code defensively
    if (INTEG_TICKS < MIN_INTEG_TIME_TICKS)
        INTEG_TICKS = MIN_INTEG_TIME_TICKS;

    // even up INTEG_TICKS
    if (INTEG_TICKS & 1)
        INTEG_TICKS++;

    if (storeInEeprom)
        EEPROM.put(EEPROM_INTEGRATION_TIME, INTEG_TICKS);
}

// Retrieve currently set integration time
uint32_t C12880MA::getIntTime()
{
    return ticksToUsec(INTEG_TICKS);
}

// Convert aggregated readouts to measurement floating point data
// and return the max value
float C12880MA::processMeasurement(float* measurement, measure_t measurementType)
{
    // Initialize arrays
    float maxVal = 0.0;
    float adcRefVoltage = adcVoltages[adcRef_];
    for (int i=0; i<SPEC_PIXELS; i++)
    {
        measurement[i] = 0.0;

        if (dataCounts[i])
        {
            if (measurementType == MEASURE_VOLTAGE)
                measurement[i] =
                    ((float)data[i]*adcRefVoltage)/(float)(dataCounts[i]*ADC_MAX_VALUE);
            else if (measurementType == MEASURE_ABSOLUTE)
                measurement[i] =
                    ((float)data[i]*adcRefVoltage)/(satVoltage_*dataCounts[i]*ADC_MAX_VALUE);
            else
                measurement[i] =
                    (float)data[i]/(float)(dataCounts[i]*ADC_MAX_VALUE);
        }

        if (measurement[i] > maxVal)
            maxVal = measurement[i];
    }

    return maxVal;
}

// Take spectrometer reading in automatic mode. This does not require
// integration time. Also as a result of measurement it will set the
// integration time measured and ADC voltage reference (the latter only
// in some modes). The automatic measurement is tuned to use sensor setup
// to maximise the output ADC resolution/range.
//
// The automatic measurement can be tuned to achieve different results.
// The behavior of it is controlled controlled by autoType parameter and
// has the following modes:
//
//    AUTO_FOR_SET_REF   - Aims to maximise ADC reading within currently
//                         set reference voltage. I.e. achieving maximum
//                         resolution within selected reference voltage or
//                         saturation limit (whichever is smaller). Only
//                         integration is changed in this mode.
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
// NOTE2: Because it changes parameters, this mode will reset black level
//        measurements. Therefore if black subtraction is used, the black
//        levels need to be re-measured after this call with the same set
//        parameters (i.e. by simply calling takeBlackMeasurement()).
//
void C12880MA::takeAutoMeasurement(auto_measure_t autoType,
                                   bool doExtTriggering)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    measuringData_ = true;

    // reset blacks
    resetBlackLevels();

    float satVoltage = satVoltage_;

    // change ADC ref if enabled
    if (autoType != AUTO_FOR_SET_REF)
    {
        // set the reference voltage to include saturation
        setAdcRefInternal(ADC_5V);

        // delay to stabilise the changes
        delay(20);
    }

    // try to make shortest reading
    INTEG_TICKS = MIN_INTEG_TIME_TICKS;
    readSpectrometer(0, false, doExtTriggering);
    float maxMeasuredVoltage = processMeasurement(data_, MEASURE_VOLTAGE);

    // Initial setup now done and we have the shortest measurement in encompassing
    // saturation limits (if allowed). Check that it is less then maximum
    // (saturation or reference) and attempt to maximise it
    if (maxMeasuredVoltage < satVoltage && autoType == AUTO_ALL_MIN_INTEG)
    {
        // Setup optimal ADC reference to optimise within (lowest
        // to achieve shortest integration time) - it's already set to 5V
        if (maxMeasuredVoltage < adcVoltages[ADC_4_096V]
            && maxMeasuredVoltage > adcVoltages[ADC_3V])
            setAdcRefInternal(ADC_4_096V);
        else if (maxMeasuredVoltage > adcVoltages[ADC_2_5V])
            setAdcRefInternal(ADC_3V);
        else
            setAdcRefInternal(ADC_2_5V);

        // delay to stabilise the changes
        delay(20);
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
        // determine increase or decrease of the integration
        if (maxMeasuredVoltage < satVoltage)
        {
            // go up
            intTicksStep = ((satVoltage-maxMeasuredVoltage)*INTEG_TICKS)/maxMeasuredVoltage;
            INTEG_TICKS += intTicksStep;
        }
        else
        {
            // last incerase was too much - half the steps and go down
            intTicksStep >>= 1;
            INTEG_TICKS = INTEG_TICKS < intTicksStep ? 0 : INTEG_TICKS - intTicksStep;
        }

        // check the limits
        if (INTEG_TICKS < MIN_INTEG_TIME_TICKS)
            INTEG_TICKS = MIN_INTEG_TIME_TICKS;
        else if (INTEG_TICKS > uSecToTicks(MAX_INTEG_TIME_US))
            INTEG_TICKS = uSecToTicks(MAX_INTEG_TIME_US);

        // do new reading
        readSpectrometer(0, false, doExtTriggering);
        maxMeasuredVoltage = processMeasurement(data_, MEASURE_VOLTAGE);

        // check exit conditions
        if (maxMeasuredVoltage >= satVoltageLower && maxMeasuredVoltage < satVoltage)
            stillGoing = false;
        else if (INTEG_TICKS == MIN_INTEG_TIME_TICKS && maxMeasuredVoltage < satVoltage)
            stillGoing = false;
        else if (INTEG_TICKS == MIN_INTEG_TIME_TICKS && maxMeasuredVoltage > satVoltage)
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

    // reprocess measurement data with set measurement type
    maxMeasuredVoltage = processMeasurement(data_, measurementType_);
    measuringData_ = false;

    // save data that was established in EEPROM
    EEPROM.put(EEPROM_INTEGRATION_TIME, INTEG_TICKS);
    if (autoType != AUTO_FOR_SET_REF)
        EEPROM.put(EEPROM_ADC_REF_ADDR, adcRef_);
}

// Take single measurement
void C12880MA::takeMeasurement(uint32_t timeUs, bool doExtTriggering)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    measuringData_ = true;

    // read main measurement data
    readSpectrometer(timeUs, doExtTriggering, doExtTriggering);
    processMeasurement(data_, measurementType_);

    measuringData_ = false;
}

// Take single black level measurement
void C12880MA::takeBlackMeasurement(uint32_t timeUs)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    measuringData_ = true;

    // read black if needed
    readSpectrometer(timeUs, false, false);
    processMeasurement(blackLevels_, measurementType_);

    measuringData_ = false;
}

// Reset black levels to 0
void C12880MA::resetBlackLevels()
{
    // Initialize arrays
    for (int i=0; i<SPEC_PIXELS; i++)
        blackLevels_[i] = 0.0;
}

// This routine to initiate and read spectrometer measurement data
void C12880MA::readSpectrometer(uint32_t timeUs,
                                bool doExtTriggering,
                                bool doLightTriggering)
{
    // no action if timer is on or in measurement
    if (timerOn)
        return;

    // number of reading cycles to do
    uint32_t readCycles =
        (timeUs * TIMER_US_FACTOR) /
            ((INTEG_TICKS+LEAD_TICKS+READ_TICKS+TRAIL_TICKS) * SPEC_CLK_TICK_TIMER);

    if (readCycles < 1)
        readCycles = 1;
    if (readCycles<<1 > UINT16_MAX)
        readCycles = UINT16_MAX>>1;

    // set read cycles counter
    specReadCycleCounter = readCycles;

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

    // initialise light trigger pin if triggering is enabled
    if (ext_trg_ls_ != NO_PIN && doLightTriggering)
    {
        STM32_Pin_Info* PIN_MAP = HAL_Pin_Map();
        extPinLight_BR = (uint32_t*)&(PIN_MAP[ext_trg_ls_].gpio_peripheral->BSRRL);
    }
    else
        extPinLight_BR = 0;

    // init ADC
    startADC(adc_cnv_);

    // initiate the timer
    startSpecTimer(doExtTriggering);

    // loop until stop
    while (specState != SPEC_STOP)
        ;

    // stop the timer and cleanup
    stopSpecTimer();
    endADC();
}

// Enable/disable Stearns and Stearns (1988) bandpass correction
void C12880MA::enableBandpassCorrection(bool enable)
{
    applyBandPassCorrection_ = enable;
}

// Set the saturation voltage. This is used to in auto integration
// mode of measurement. Passing values outside of range from Hamamatsu
// spec will reset approprite value to the default one.
void C12880MA::setSaturationVoltage(float satVoltage, bool storeInEeprom)
{
    // check the saturation voltage against Hamamatsu spec limits
    if (satVoltage >= MIN_SAT_VOLTAGE && satVoltage <= MAX_SAT_VOLTAGE)
        satVoltage_ = satVoltage;
    else
        satVoltage_ = MIN_SAT_VOLTAGE;

    if (storeInEeprom)
        EEPROM.put(EEPROM_SAT_VOLTAGE, satVoltage_);
}

// get average max within 5% off the measured maximum
float getAveragedMax(float maxVal, float* measurement)
{
    // calculate average max
    float avgMax = 0;
    int count = 0;
    for (int i=0; i<SPEC_PIXELS; i++)
        if (measurement[i] > maxVal*0.95)
        {
            avgMax += measurement[i];
            ++count;
        }

    return count ? avgMax/count : maxVal;
}

// Automatic measurement of the saturation voltage. This is used in
// auto integration mode of measurement.
//
// Automatic setting works by exposing sensor to bright light and then
// calling this method to work out saturation voltages.
void C12880MA::measureSaturationVoltage()
{
    measuringData_ = true;

    // save current reference and gain mode and set
    adc_ref_t savedAdcRef = getAdcReference();

    // set the 5V reference - that's the highest C12880MA will go
    setAdcRefInternal(ADC_5V);

    // delay to stabilise the changes
    delay(50);

    // do the measurement at 1 sec to ensure sensor saturation
    readSpectrometer(1000000, false, false);
    float maxVoltage = processMeasurement(data_, MEASURE_VOLTAGE);
    float satVoltage = getAveragedMax(maxVoltage, data_);

    // restore ADC reference and gain
    setAdcRefInternal(savedAdcRef);

    measuringData_ = false;

    // set the data
    setSaturationVoltage(satVoltage);
}

// Set the spectrometer measurement type. This allows to change the type
// of the measurement results and will affect all subsequent measurements.
// The measurement results could be stored:
//   (1) relatively - 0..1 values for currently selected ADC reference
//   (2) as voltage measurements irrespective to ADC parameters but dependent
//       on gain (these are absolute within the same gain settings)
//   (3) as absolute measurements, 0..1 range scaled to saturation voltage
void C12880MA::setMeasurementType(measure_t measurementType, bool storeInEeprom)
{
    // no action if timer is on or in measurement
    if (timerOn || measuringData_)
        return;

    // reset blacks since they no longer valid
    if (measurementType_ != measurementType)
        resetBlackLevels();

    measurementType_ = measurementType;

    if (storeInEeprom)
        EEPROM.put(EEPROM_MEASURE_TYPE_ADDR, measurementType_);
}

// Set ADC reference voltage to one of the specified values. This is
// internal function without reentrance checks or EEPROM updating
void C12880MA::setAdcRefInternal(adc_ref_t adcRef)
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
// analogue signal voltage for conversion.
void C12880MA::setAdcReference(adc_ref_t ref, bool storeInEeprom)
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
//
// Note: Applying bandpass correction can go out of 0..1 range
double C12880MA::getMeasurement(uint16_t pixelIdx, bool subtractBlack)
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
double C12880MA::getBlackMeasurement(uint16_t pixelIdx)
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
double C12880MA::getWavelength(uint16_t pixelNumber)
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
