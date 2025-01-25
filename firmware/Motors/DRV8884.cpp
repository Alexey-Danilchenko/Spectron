/*
 *  DRV8884.cpp - Texas Instruments DRV8884 driver for Motor board.
 *                This controls stepper motor and rotary encoder.
 *                Rotary encoder bit is supposed to be used as
 *                confirmaion mechanism for stepper motor position.
 *                At this point only basic readings of rotary
 *                encoder are developed - no confirmation code is
 *                implemented.
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

#include "DRV8884.h"

// EEPROM addresses used by this class
#define EEPROM_CUR_POS_ADDR         EEPROM_DRV8884_BASE_ADDR
#define EEPROM_MIN_POS_ADDR         EEPROM_DRV8884_BASE_ADDR+4
#define EEPROM_MAX_POS_ADDR         EEPROM_DRV8884_BASE_ADDR+8
#define EEPROM_ROTARY_ADDR          EEPROM_DRV8884_BASE_ADDR+12
#define EEPROM_DECAY_ADDR           EEPROM_DRV8884_BASE_ADDR+16
#define EEPROM_TIMER_PER_ADDR       EEPROM_DRV8884_BASE_ADDR+20
#define EEPROM_TRQ_ADDR             EEPROM_DRV8884_BASE_ADDR+24
#define EEPROM_STEP_ADDR            EEPROM_DRV8884_BASE_ADDR+28
#define EEPROM_STEPS_PER_POS_ADDR   EEPROM_DRV8884_BASE_ADDR+32

//
// Timer prescaler - this is what CPU counter clock frequency is divided by to get the frequency
//      generally prescaler is calculated as SYSCORECLOCK (60000000 for Photon for base timers)
//      divided by the frequency of the timer counter. For example:
//
//      TIMER_PRESCALER = (SYSCORECLOCK / 1000000) - 1 to get TIM counter clock = 1MHz
//
#define  TIMER_PRESCALER     599       // for basic timers at 60MHz it gives 10us timer unit counter
#define  TIMER_UNITS_PER_SEC 100000UL  // number of timer units per sec for the above prescaler
#define  MIN_TIMER_PERIOD    10        // min timer tick - in above 10us units it will trigger every 0.1ms
#define  MAX_TIMER_PERIOD    50000     // max timer tick - in above 10us units it will trigger every 500ms
uint32_t TIMER_PERIOD_CNT =  1000;     // in above 10us units it will trigger every 10ms

// State machine
enum state_t {
    STATE_RUN,
    STATE_STOP
};

// driver state variables
static volatile state_t drvState      = STATE_STOP;
static bool             drvCLK        = LOW;
static int32_t          drvPulsesCount  = 0;
static volatile bool    timerOn       = false;
#ifdef ROTARY_ENCODER_ENABLED
static volatile int32_t rotaryCounter = false;
#endif

// Driver pins
uint8_t drvPinCLK  = NO_PIN;

// Decay DAC values
static const int decayDAC[] = {
    0,      // DECAY_SLOW_MIXED
    373,    // DECAY_MIXED_30_PERCENT, 300mV for 3.3V, 12 bit DAC
    1241,   // DECAY_MIXED_60_PERCENT, 1V for 3.3V, 12 bit DAC
    3723    // DECAY_SLOW, 3V for 3.3V, 12 bit DAC
};

// Full step multipliers
static const int fullStepMultiplier[] = {
    1,      // STEP_FULL
    16,     // STEP_SIXTEENTH
    8,      // STEP_EIGHTS
    1,      // none - not used
    2,      // STEP_HALF
    4,      // STEP_QUARTER
    2       // STEP_HALF_NON_CIRCULAR
};

// ----------------------------------------------------
//   Timer clock and pins interrupt handling routines
// ----------------------------------------------------
#ifdef ROTARY_ENCODER_ENABLED
void rotaryUp(void)
{
    ++rotaryCounter;
}

void rotaryDown(void)
{
    --rotaryCounter;
}
#endif

void drvClockInterrupt(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

        digitalWriteFast(drvPinCLK, drvCLK);

        switch (drvState) {
            case STATE_RUN:
                --drvPulsesCount;
                if (drvPulsesCount <= 0) {
                    drvPulsesCount = 0;
                    drvState = STATE_STOP;
                    drvCLK = LOW;
                }
                else
                    drvCLK = (drvCLK == HIGH) ? LOW : HIGH;
                break;


            case STATE_STOP:
            default:
                break;
        }
    }
}

// start active timer
void startDrvTimer(int32_t pulses)
{
    TIM_TimeBaseInitTypeDef timerInit = {0};
    NVIC_InitTypeDef nvicInit = {0};

    // set in timer guard
    if (timerOn)
        return;

    timerOn = true;

    // init state
    drvPulsesCount = pulses;
    drvState = STATE_RUN;

    // next value of CLK
    drvCLK = HIGH;

    // enable TIM7 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    // enable timer IRQ
    nvicInit.NVIC_IRQChannel                   = TIM7_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 3;
    nvicInit.NVIC_IRQChannelSubPriority        = 0;
    nvicInit.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvicInit);

    // setup timer
    timerInit.TIM_Prescaler         = TIMER_PRESCALER;
    timerInit.TIM_CounterMode       = TIM_CounterMode_Up;
    timerInit.TIM_Period            = TIMER_PERIOD_CNT;
    timerInit.TIM_ClockDivision     = TIM_CKD_DIV1;
    timerInit.TIM_RepetitionCounter = 0;

    // enable timer
    TIM_TimeBaseInit(TIM7, &timerInit);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM7, ENABLE);
}

// stop active timer
void stopDrvTimer()
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

    // reset pin
    pinResetFast(drvPinCLK);

    timerOn = false;
}

// Constructor/destructor
DRV8884::DRV8884(uint8_t nfault, uint8_t decay, uint8_t trq, uint8_t m0, uint8_t m1,
                 uint8_t dir, uint8_t step, uint8_t enable, uint8_t nsleep, uint8_t pref
#ifdef ROTARY_ENCODER_ENABLED
                 , uint8_t up_clk, uint8_t down_clk
#endif
                 )
{
    pinNfault_  = nfault;
    pinDecay_   = decay;
    pinTRQ_     = trq;
    pinM0_      = m0;
    pinM1_      = m1;
    pinDir_     = dir;
    pinStep_    = step;
    pinEnable_  = enable;
    pinNsleep_  = nsleep;
    pinPREF_    = pref;
#ifdef ROTARY_ENCODER_ENABLED
    pinUpCLK_   = up_clk;
    pinDownCLK_ = down_clk;
#endif

    drvState    = STATE_STOP;
    drvCLK      = LOW;

    drvPinCLK     = pinStep_;
    isRunning_    = false;
    decayMode_    = DECAY_SLOW_MIXED;
    torqueMode_   = TORQUE_FULL;
    steppingMode_ = STEP_FULL;
    direction_    = DIR_FORWARD;

    // read position details from EPROM
    EEPROM.get(EEPROM_CUR_POS_ADDR, curPosFullSteps_);
    if (curPosFullSteps_ < 0 || curPosFullSteps_ == 0xFFFFFFFF)
        // initialise
        curPosFullSteps_ = 0;

    EEPROM.get(EEPROM_MIN_POS_ADDR, minPos_);
    if (minPos_ == 0xFFFFFFFF)
        // EEPROM was empty
        minPos_ = 0;

    EEPROM.get(EEPROM_MAX_POS_ADDR, maxPos_);
    if (maxPos_ == 0xFFFFFFFF)
        // EEPROM was empty
        maxPos_ = 0;

    EEPROM.get(EEPROM_DECAY_ADDR, decayMode_);
    if (decayMode_ < 0  || decayMode_ > 3)
        // initialise
        decayMode_ = DECAY_SLOW_MIXED;

    EEPROM.get(EEPROM_TIMER_PER_ADDR, stepsPerSec_);
    if (stepsPerSec_ <=0 || stepsPerSec_ > 0xFFFF)
        // initialise
        setRotationSpeed(50);
    else
        setRotationSpeed(stepsPerSec_, false);

    EEPROM.get(EEPROM_TRQ_ADDR, torqueMode_);
    if (torqueMode_ < 0 || torqueMode_ > 2)
        // initialise
        torqueMode_ = TORQUE_FULL;

    EEPROM.get(EEPROM_STEP_ADDR, steppingMode_);
    if (steppingMode_ < 0 || steppingMode_ > 6)
        // initialise
        steppingMode_ = STEP_FULL;

    EEPROM.get(EEPROM_STEPS_PER_POS_ADDR, fullStepsPerPos_);
    if (fullStepsPerPos_ <= 0 || fullStepsPerPos_ >= 0xFFFF)
        // initialise
        fullStepsPerPos_ = 1;

#ifdef ROTARY_ENCODER_ENABLED
    EEPROM.get(EEPROM_ROTARY_ADDR, rotaryCounter_);
    if (rotaryCounter_ < 0 || rotaryCounter_ == 0xFFFFFFFF)
        // initialise
        rotaryCounter_ = getCurPos();
#endif
}

DRV8884::~DRV8884()
{
}

// Setup methods and setters
void DRV8884::begin()
{
    pinMode(pinDecay_,  OUTPUT);
    pinMode(pinTRQ_,    OUTPUT);
    pinMode(pinM0_,     OUTPUT);
    pinMode(pinM1_,     OUTPUT);
    pinMode(pinDir_,    OUTPUT);
    pinMode(pinStep_,   OUTPUT);
    pinMode(pinEnable_, OUTPUT);
    pinMode(pinNsleep_, OUTPUT);

#ifdef DRV8884_PREF_DAC_CONTROL_ENABLED
    pinMode(pinPREF_,   OUTPUT);
#else
    pinMode(pinPREF_,   INPUT_PULLDOWN);  // this is to connect it via resistor to the ground
#endif
    pinMode(pinNfault_, INPUT_PULLUP);

#ifdef ROTARY_ENCODER_ENABLED
    pinMode(pinUpCLK_,  INPUT_PULLDOWN);
    pinMode(pinDownCLK_,INPUT_PULLDOWN);
#endif

    // reset everything
    pinResetFast(pinEnable_);
    pinResetFast(pinNsleep_);
    pinResetFast(pinDir_);
    pinResetFast(pinStep_);
#ifdef DRV8884_PREF_DAC_CONTROL_ENABLED
    prefDAC_ = PREF_DAC_FULL_CURRENT;
    analogWrite(pinPREF_,  prefDAC_);
#endif

    //attachInterrupt(NFAULT, error_blink, CHANGE);
    attachSystemInterrupt(SysInterrupt_TIM7_IRQ, drvClockInterrupt);

    // set defaults
    setDirection(DIR_FORWARD);
    analogWrite(pinDecay_, decayDAC[decayMode_]);
    setTorque((torque_t)torqueMode_, false);
    setSteppingMode((step_t)steppingMode_, false);
}

// Move number of positions in the current direction. By default this will
// not move past origin (min position). Specifying allowBeyondLimits
// will allow to ignore that (it should be used for calibration)
//
// Number of positions to move is fractional to allow finer control over of
// the positions to move. This allows using individual steps within
// single position and normally used for adjustments.
void DRV8884::movePositions(float positions, bool allowBeyondLimits)
{
    uint32_t steps = positions*fullStepsPerPos_;

    if (isRunning_ || timerOn || steps <= 0)
        return;

    isRunning_ = true;

    uint32_t minPosFullSteps = minPos_*fullStepsPerPos_;
    uint32_t maxPosFullSteps = maxPos_*fullStepsPerPos_;

    // check against the steps beyond limits
    if (!allowBeyondLimits)
        if (direction_ == DIR_REVERSE
            && curPosFullSteps_ < minPosFullSteps + steps)
            steps = curPosFullSteps_ - minPosFullSteps;
        else if (direction_ == DIR_FORWARD
                 && curPosFullSteps_ > maxPosFullSteps - steps)
            steps = maxPosFullSteps - curPosFullSteps_;

    // reset
    pinResetFast(pinStep_);

    // wake up the chip
    pinSetFast(pinEnable_);
    pinSetFast(pinNsleep_);

#ifdef ROTARY_ENCODER_ENABLED
    // set rotary
    rotaryCounter = rotaryCounter_;

    // attach rotary pin interrupts
    attachInterrupt(pinUpCLK_,   rotaryUp,   RISING, 3);
    attachInterrupt(pinDownCLK_, rotaryDown, RISING, 3);
#endif

    // start the timer
    int32_t driveSteps = steps*fullStepMultiplier[steppingMode_];
    startDrvTimer(driveSteps*2);

    // loop until state machine is running
    while (drvState != STATE_STOP)
        ;

    // stop the timer
    stopDrvTimer();

#ifdef ROTARY_ENCODER_ENABLED
    // detach pin interrupts
    detachInterrupt(pinUpCLK_);
    detachInterrupt(pinDownCLK_);

    rotaryCounter_ = rotaryCounter;
#endif

    // adjust the current position
    if (pinReadFast(pinNfault_) == HIGH)
    {
        if (direction_ == DIR_FORWARD)
            curPosFullSteps_ += steps;
        else
            curPosFullSteps_ -= steps;
    }

    // sleep
    pinResetFast(pinEnable_);
    pinResetFast(pinNsleep_);

    // store the position and counter in EEPROM
    EEPROM.put(EEPROM_CUR_POS_ADDR, curPosFullSteps_);
#ifdef ROTARY_ENCODER_ENABLED
    EEPROM.put(EEPROM_ROTARY_ADDR, rotaryCounter_);
#endif

    // reset guard
    isRunning_ = false;
}

// Sets number of full motor steps per one position unit. The postions
// are used to move motor and are not dependent to a selected step mode
// or size.
void DRV8884::setStepsPerPosition(int stepsPerPosition, bool storeInEeprom)
{
    if (isRunning_ || timerOn || stepsPerPosition <= 0)
        return;

    // maintain position
    double curPos = getCurPos();
    fullStepsPerPos_ = stepsPerPosition;
    curPosFullSteps_ = curPos * fullStepsPerPos_;

    // store them in EEPROM
    if (storeInEeprom)
    {
        EEPROM.put(EEPROM_STEPS_PER_POS_ADDR, fullStepsPerPos_);
        EEPROM.put(EEPROM_CUR_POS_ADDR, curPosFullSteps_);
    }
}

// Set position limits - this is performed once when calibration is done
// and stored in EEPROM together with position
void DRV8884::setLimits(int minPos, int maxPos)
{
    if (isRunning_ || timerOn || minPos >= maxPos)
        return;

    minPos_ = minPos;
    maxPos_ = maxPos;

    // store them in EEPROM
    EEPROM.put(EEPROM_MIN_POS_ADDR, minPos_);
    EEPROM.put(EEPROM_MAX_POS_ADDR, maxPos_);
}

#ifdef DRV8884_PREF_DAC_CONTROL_ENABLED
// Set the current limit via DAC controlled PREF (see DRV8884 spec sheet)
void DRV8884::setPREF(pref_t prefDAC)
{
    if (isRunning_ || timerOn ||
        prefDAC < PREF_DAC_FULL_CURRENT ||
        prefDAC > PREF_DAC_ZERO_CURRENT)
        return;

    prefDAC_ = prefDAC;
    analogWrite(pinPREF_, prefDAC_);

    // delay to stabilise the changes
    delay(100);
}
#endif

// Set decay mode
void DRV8884::setDecayMode(decay_t decay)
{
    if (isRunning_ || timerOn || decay > 3 || decay < 0)
        return;

    decayMode_ = decay;
    analogWrite(pinDecay_, decayDAC[decayMode_]);

    // store them in EEPROM
    EEPROM.put(EEPROM_DECAY_ADDR, decayMode_);

    // delay to stabilise the changes
    delay(100);
}

// Set decay mode
void DRV8884::setTorque(torque_t torque, bool storeInEeprom)
{
    if (isRunning_ || timerOn)
        return;

    torqueMode_ = torque;
    if (torqueMode_ == TORQUE_75_PERCENT)
        // TRQ in High Z state
        pinMode(pinTRQ_, INPUT);
    else {
        pinMode(pinTRQ_, OUTPUT);
        if (torqueMode_ == TORQUE_FULL)
            pinResetFast(pinTRQ_);
        else
            pinSetFast(pinTRQ_);
    }

    // store them in EEPROM
    if (storeInEeprom)
        EEPROM.put(EEPROM_TRQ_ADDR, torqueMode_);
}

// Set stepping mode
void DRV8884::setSteppingMode(step_t stepMode, bool storeInEeprom)
{
    if (isRunning_ || timerOn)
        return;

    steppingMode_ = stepMode;

    if (steppingMode_ & 2)
        // M0 in High Z state
        pinMode(pinM0_, INPUT);
    else
    {
        pinMode(pinM0_, OUTPUT);
        if (steppingMode_ & 1)
            pinSetFast(pinM0_);
        else
            pinResetFast(pinM0_);
    }

    if (steppingMode_ & 4)
        pinSetFast(pinM1_);
    else
        pinResetFast(pinM1_);

    // store them in EEPROM
    if (storeInEeprom)
        EEPROM.put(EEPROM_STEP_ADDR, steppingMode_);
}

// Set direction
void DRV8884::setDirection(dir_t direction)
{
    if (isRunning_ || timerOn)
        return;

    direction_ = direction;
    if (direction_ == DIR_FORWARD)
        pinResetFast(pinDir_);
    else
        pinSetFast(pinDir_);

    // delay to stabilise the changes
    delay(100);
}

 // Set rotation speed
bool DRV8884::setRotationSpeed(int stepsPerSec, bool storeInEeprom)
{
    if (isRunning_ || timerOn)
        return false;

    int timerPeriod = TIMER_UNITS_PER_SEC / (stepsPerSec * 2);

    if (timerPeriod < MIN_TIMER_PERIOD || timerPeriod > MAX_TIMER_PERIOD)
        return false;

    stepsPerSec_ = stepsPerSec;
    TIMER_PERIOD_CNT = timerPeriod;

    // store them in EEPROM
    if (storeInEeprom)
        EEPROM.put(EEPROM_TIMER_PER_ADDR, stepsPerSec_);

    return true;
}

// Reset postion to origin (set the cur pos to specified one)
void DRV8884::resetPosition(float curPos)
{
    if (isRunning_ || timerOn || curPos < 0)
        return;

    curPosFullSteps_ = curPos*fullStepsPerPos_;

    // store the position in EEPROM
    EEPROM.put(EEPROM_CUR_POS_ADDR, curPosFullSteps_);

#ifdef ROTARY_ENCODER_ENABLED
    rotaryCounter_ = 0;

    // store the counter in EEPROM
    EEPROM.put(EEPROM_ROTARY_ADDR,  rotaryCounter_);
#endif
}
