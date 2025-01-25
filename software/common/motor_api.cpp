/*
 *  motor_api.cpp - Implementation of API calls to Spectron Motor boards
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


#include "motor_api.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QString>
#include <QByteArray>

// --------------------------------------
//  Spectron Motor Device implementation
// --------------------------------------

// constructors/destructors
SpecMotorDevice::SpecMotorDevice()
    : ParticleDevice(), m_decayType(DECAY_SLOW_MIXED), m_steppingMode(STEP_FULL),
      m_torque(TORQUE_FULL), m_stepsPerWavelength(1), m_minWavelength(380),
      m_maxWavelength(720), m_curWavelength(380), m_stepsPerSec(200)
{
}

SpecMotorDevice::~SpecMotorDevice()
{
}

 // copy operator from another Particle device
SpecMotorDevice& SpecMotorDevice::operator=(ParticleDevice& device)
{
    ParticleDevice::operator=(device);
    return *this;
}

bool SpecMotorDevice::refresh()
{
    ParticleDevice::refresh();

    // refresh local variables
    if (!isConnected())
        return false;

    m_curWavelength = getVariableValue("drvCurPos").toDouble();
    m_stepsPerWavelength = getVariableValue("drvFStpPrPos").toInt();
    m_minWavelength = getVariableValue("drvMinPos").toInt();
    m_maxWavelength = getVariableValue("drvMaxPos").toInt();
    m_stepsPerSec = getVariableValue("drvStepsSec").toInt();
    m_steppingMode = (TStepMode)getVariableValue("drvStepMode").toInt();
    m_decayType = (TDecayType)getVariableValue("drvDecayMod").toInt();
    m_torque = (TTorque)getVariableValue("drvTrqMode").toInt();

	return true;
}

// actions
bool SpecMotorDevice::moveToWavelength(double wavelength)
{
    QString param;
    param.setNum(wavelength);
    if (callFunction("drvMoveToPos", param) == -1)
        return false;

    m_curWavelength = getVariableValue("drvCurPos").toDouble();

    return true;
}

bool SpecMotorDevice::moveSteps(double wavelengthSteps)
{
    QString param;
    param.setNum(wavelengthSteps);
    if (wavelengthSteps>0)
        param.insert(0,"+");

    if (callFunction("drvMoveToPos", param) == -1)
        return false;

    m_curWavelength = getVariableValue("drvCurPos").toDouble();

    return true;
}

bool SpecMotorDevice::moveToStart()
{
    if (callFunction("drvMoveToPos", "START") == -1)
        return false;

    m_curWavelength = getVariableValue("drvCurPos").toDouble();

    return true;
}

bool SpecMotorDevice::moveToEnd()
{
    if (callFunction("drvMoveToPos", "END") == -1)
        return false;

    m_curWavelength = getVariableValue("drvCurPos").toDouble();

    return true;
}

// various setters for motors
bool SpecMotorDevice::setStepsPerWavelength(int stepsPerWavelength)
{
    if (stepsPerWavelength <= 0)
        return true;

    QString param;
    param.setNum(stepsPerWavelength);
    if (callFunction("drvSetStPPos", param) == -1)
        return false;

    m_stepsPerWavelength = getVariableValue("drvFStpPrPos").toInt();
    m_curWavelength = getVariableValue("drvCurPos").toDouble();

    return true;
}

bool SpecMotorDevice::setSpectralRange(int minWavelength, int maxWavelength)
{
    if (minWavelength < 340 || maxWavelength > 850 || minWavelength > maxWavelength)
        return true;

    QString param = QString("%1,%2").arg(minWavelength).arg(maxWavelength);
    if (callFunction("drvSetLmts", param) == -1)
        return false;

    m_minWavelength = getVariableValue("drvMinPos").toInt();
    m_maxWavelength = getVariableValue("drvMaxPos").toInt();

    return true;
}

bool SpecMotorDevice::resetCurrentPos(int newWavelengthPos)
{
    if (newWavelengthPos <= 0)
        return true;

    QString param;
    param.setNum(newWavelengthPos);
    if (callFunction("drvResetPos", param) == -1)
        return false;

    m_curWavelength = getVariableValue("drvCurPos").toDouble();

    return true;
}

bool SpecMotorDevice::setDecay(TDecayType decay)
{
    QString param;
    param.setNum(decay);
    if (callFunction("drvSetDecay", param) == -1)
        return false;

    m_decayType = (TDecayType)getVariableValue("drvDecayMod").toInt();

    return true;
}

bool SpecMotorDevice::setSteppingMode(TStepMode stepMode)
{
    QString param;
    param.setNum(stepMode);
    if (callFunction("drvSetStpMd", param) == -1)
        return false;

    m_steppingMode = (TStepMode)getVariableValue("drvStepMode").toInt();

    return true;
}

bool SpecMotorDevice::setRotationSpeed(int stepsPerSec)
{
    if (stepsPerSec <= 0)
        return true;

    QString param;
    param.setNum(stepsPerSec);
    if (callFunction("drvSetRotSpd", param) == -1)
        return false;

    m_stepsPerSec = getVariableValue("drvStepsSec").toInt();

    return true;
}

bool SpecMotorDevice::setTorqueMode(TTorque torque)
{
    QString param;
    param.setNum(torque);
    if (callFunction("drvSetTrqMod", param) == -1)
        return false;

    m_torque = (TTorque)getVariableValue("drvTrqMode").toInt();

    return true;
}
