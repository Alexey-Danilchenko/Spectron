/*
 *  DRV8884.h - Texas Instruments DRV8884 driver for Motor board.
 *
 *  Copyright 2017 Alexey Danilchenko, Iliah Borg
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

#if !defined(_DRV8884_H_)
#define _DRV8884_H_

#include "application.h"

// It is possible to connect PREF pin directly to Photon
// without resistor if no fine control over output current
// is needed. In this case full current is enabled by
// configuring PREF pin as INPUT_PULLDOWN - this enables
// max DRV8884 curret output (1A). If your board does this
// disable (comment out) the definition below.
#define DRV8884_PREF_DAC_CONTROL_ENABLED

// No pin assigned
#define NO_PIN (TOTAL_PINS+1)

// Direction (see DRV8884 spec sheet)
enum dir_t {
    DIR_FORWARD = 0,
    DIR_REVERSE = 1
};

// DRV8844 decay types (see DRV8884 spec sheet)
enum decay_t {
    DECAY_SLOW_MIXED       = 0,
    DECAY_MIXED_30_PERCENT = 1,  // 300mV
    DECAY_MIXED_60_PERCENT = 2,  // 1V
    DECAY_SLOW             = 3   // 3V
};

// DRV8844 step modes (see DRV8884 spec sheet)
enum step_t {
    STEP_FULL              = 0,
    STEP_SIXTEENTH         = 1,
    STEP_EIGHTS            = 2,
    STEP_HALF              = 4,
    STEP_QUARTER           = 5,
    STEP_HALF_NON_CIRCULAR = 6
};

// DRV8844 torque modes (see DRV8884 spec sheet)
enum torque_t {
    TORQUE_FULL       = 0,
    TORQUE_50_PERCENT = 1,
    TORQUE_75_PERCENT = 2
};

#ifdef DRV8884_PREF_DAC_CONTROL_ENABLED
// PREF current limiting - DAC controlled at 0..1.232V (see DRV8884 spec sheet)
enum pref_t {
    PREF_DAC_FULL_CURRENT = 0,   // for full torque this will correspond to 1A
    PREF_DAC_ZERO_CURRENT = 1526 // min current - value for 1.232V on 3.3V, 12 bit DAC
};
#endif

// Spectron motor board class
//
//    This class controls DRV8884 and LS7083 quadrature encoder.
//    This class uses TIM7 timer interrupt it affects timer
//    configuration when used
//
class DRV8884 {
private:
    // Pin definitions
    uint8_t pinNfault_, pinDecay_, pinTRQ_, pinM0_, pinM1_, pinDir_, pinStep_;
    uint8_t pinEnable_, pinNsleep_, pinPREF_;

    // These are rotary encoder pins for counting ups and downs. Not used
    // for currently but could be used to double check on precision of
    // stepper movements
    uint8_t pinUpCLK_, pinDownCLK_;

    // Variables
    bool     isRunning_;       // Is the motor currently running
    int      minPos_;          // Minimum allowed position - lower limit
    int      maxPos_;          // Maximum allowed position - upper limit
    int      curPos_;          // Current position
    int      fullStepsPerPos_; // Full steps per one position
    int      rotaryCounter_;   // Current position
    int      decayMode_;       // Current decay mode
    int      torqueMode_;      // Current torque mode
    int      steppingMode_;    // Current stepping mode
    int      direction_;       // Current direction
    int      stepsPerSec_;     // Current rotation speed 

#ifdef DRV8884_PREF_DAC_CONTROL_ENABLED
    int      prefDAC_;      // Current PREF setting
#endif

public:

    // Constructor/destructor
    // Parameters:
    //     nfault       - DRV8884 fault pin
    //     decay        - DRV8884 decay control pin
    //     trq          - DRV8884 torque control pin
    //     m0, m1       - DRV8884 microstepping mode pins
    //     dir          - DRV8884 direction control pin
    //     step         - DRV8884 step clock pin
    //     enable       - DRV8884 motor enablement pin
    //     nsleep       - DRV8884 sleep pin
    //     pref         - DRV8884 current limit control pin
    //     up_clk       - LS7083 up clock pin
    //     down_clk     - LS7083 down clock pin
    DRV8884(uint8_t nfault, uint8_t decay, uint8_t trq, uint8_t m0, uint8_t m1,
            uint8_t dir, uint8_t step, uint8_t enable, uint8_t nsleep, uint8_t pref,
            uint8_t up_clk, uint8_t down_clk);
    ~DRV8884();

    // Setup methods and setters
    void begin();

    // Move number of positions in the current direction. By default this will
    // not move past origin (min position). Specifying allowBeyondLimits
    // will allow to ignore that (it should be used for calibration)
    void movePositions(uint32_t positions, bool allowBeyondLimits = false);

    // Reset postion (set the cur pos to specified value)
    void resetPosition(int curPos);

    // Various setters

    // Sets number of full motor steps per one position unit. The postions
    // are used to move motor and are not dependent to a selected step mode 
    // or size.
    void setStepsPerPosition(int stepsPerPosition, bool storeInEeprom = true);

    // Set position limits - this is performed once when calibration is done
    // and stored in EEPROM together with position
    void setLimits(int minPos, int maxPos);

    // Set decay mode
    void setDecayMode(decay_t decay);

    // Set decay mode
    void setTorque(torque_t torque, bool storeInEeprom = true);

    // Set stepping mode
    void setSteppingMode(step_t stepMode, bool storeInEeprom = true);

    // Set direction
    void setDirection(dir_t direction);
    
    // Set rotation speed
    bool setRotationSpeed(int stepsPerSec, bool storeInEeprom = true);

#ifdef DRV8884_PREF_DAC_CONTROL_ENABLED
    // Get/Set the current limit via DAC controlled PREF (see DRV8884 spec sheet)
    void    setPREF(pref_t prefDAC);
    pref_t  getPREF() { return prefDAC_; }
#endif

    // Getters
    int      getCurPos()        { return curPos_; }
    int      getMaxPos()        { return maxPos_; }
    int      getMinPos()        { return minPos_; }
    decay_t  getDecayMode()     { return (decay_t)decayMode_; }
    torque_t getTorque()        { return (torque_t)torqueMode_; }
    step_t   getSteppingMode()  { return (step_t)steppingMode_; }
    dir_t    getDirection()     { return (dir_t)direction_; }
    int      getRotationSpeed() { return stepsPerSec_; }
    bool     isRunning()        { return isRunning_; }
};
#endif
