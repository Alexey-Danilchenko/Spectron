/*
 *  motor_api.h - Implementation of API calls to Spectron Motor boards
 *
 *  Copyright 2017-2018 Alexey Danilchenko
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

#ifndef MOTOR_API_H
#define MOTOR_API_H

#include <QVector>
#include "particle_api.h"

typedef QVector<double> TDoubleVec;

//
// Class that provides access to Spectron Motor board over Particle cloud.
// 
// This uses ParticleAPI classes and expects them to be connected 
// and logged in.
//
class SpecMotorDevice: public ParticleDevice
{
public:
    // DRV8844 decay types (see DRV8884 spec sheet)
    enum TDecayType {
        DECAY_SLOW_MIXED       = 0,
        DECAY_MIXED_30_PERCENT = 1,  // 300mV
        DECAY_MIXED_60_PERCENT = 2,  // 1V
        DECAY_SLOW             = 3   // 3V
    };

    // DRV8844 step modes (see DRV8884 spec sheet)
    enum TStepMode {
        STEP_FULL              = 0,
        STEP_SIXTEENTH         = 1,
        STEP_EIGHTS            = 2,
        STEP_HALF              = 4,
        STEP_QUARTER           = 5,
        STEP_HALF_NON_CIRCULAR = 6
    };

    // DRV8844 torque modes (see DRV8884 spec sheet)
    enum TTorque {
        TORQUE_FULL       = 0,
        TORQUE_50_PERCENT = 1,
        TORQUE_75_PERCENT = 2
    };

    // constructors/destructors
    SpecMotorDevice();
    ~SpecMotorDevice();

    // copy operator from another Particle device
    SpecMotorDevice& operator=(ParticleDevice& device);

    // refreshes he class from remote location
    bool refresh();
    
    // actions
    bool moveToWavelength(double wavelength);
    bool moveSteps(double wavelengthSteps);
    bool moveToStart();
    bool moveToEnd();

    // various setters for motors
    bool setStepsPerWavelength(int stepsPerWavelength);
    bool setSpectralRange(int minWavelength, int maxWavelength);
    bool resetCurrentPos(int newWavelengthPos);
    bool setDecay(TDecayType decay);
    bool setSteppingMode(TStepMode stepMode);
    bool setRotationSpeed(int stepsPerSec);
    bool setTorqueMode(TTorque torque);
    
    // getters
    TDecayType getDecay()              { return m_decayType; }
    TStepMode  getSteppingMode()       { return m_steppingMode; }
    TTorque    getTorqueMode()         { return m_torque; }
    int        getStepsPerWavelength() { return m_stepsPerWavelength; }
    int        getMinWavelength()      { return m_minWavelength; }
    int        getMaxWavelength()      { return m_maxWavelength; }
    double     getCurrentPos()         { return m_curWavelength; }
    int        getRotationSpeed()      { return m_stepsPerSec; }
    

private:

    // members
    TDecayType m_decayType;
    TStepMode  m_steppingMode;
    TTorque    m_torque;
    int        m_stepsPerWavelength;
    int        m_minWavelength;
    int        m_maxWavelength;
    double     m_curWavelength;
    int        m_stepsPerSec;
};

#endif // MOTOR_API_H
