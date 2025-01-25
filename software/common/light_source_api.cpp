/*
 *  light_source_api.cpp - Implementation of API calls to Spectron Light Source boards
 *
 *  Copyright 2018 Alexey Danilchenko
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


#include "light_source_api.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QString>
#include <QByteArray>

// ---------------------------------------------
//  Spectron Light Source Device implementation
// ---------------------------------------------

// constructors/destructors
SpecLightSourceDevice::SpecLightSourceDevice()
    : ParticleDevice(), m_brightness(1)
{
}

SpecLightSourceDevice::SpecLightSourceDevice(const ParticleDevice& device)
    : ParticleDevice(device), m_brightness(1)
{
}

SpecLightSourceDevice::~SpecLightSourceDevice()
{
}

 // copy operator from another Particle device
SpecLightSourceDevice& SpecLightSourceDevice::operator=(ParticleDevice& device)
{
    ParticleDevice::operator=(device);
    return *this;
}

bool SpecLightSourceDevice::setBrightness(double brightness)
{
    m_brightness = brightness;

    if (m_brightness < 0)
        m_brightness = 0;
    else if (m_brightness > 1)
        m_brightness = 1;

	return true;
}


// -------------------------------------------------
//  Spectron LED Light Source Device implementation
// -------------------------------------------------

// constructors/destructors
SpecLEDDevice::SpecLEDDevice()
    : SpecLightSourceDevice()
{
}

SpecLEDDevice::SpecLEDDevice(const ParticleDevice& device)
    : SpecLightSourceDevice(device)
{
}

SpecLEDDevice::~SpecLEDDevice()
{
}


// copy operator from another Particle device
SpecLEDDevice& SpecLEDDevice::operator=(ParticleDevice& device)
{
    SpecLightSourceDevice::operator=(device);
    return *this;
}

// refreshes he class from remote location
bool SpecLEDDevice::refresh()
{
    ParticleDevice::refresh();

    // refresh local variables
    if (!isConnected())
        return false;

    setBrightness(m_brightness);

	return true;
}

bool SpecLEDDevice::trigger(int trigTimeMs)
{
    QString param;
    param.setNum(trigTimeMs);
    return callFunction("ledTrigger", param) != -1;
}

bool SpecLEDDevice::trigger(bool enable)
{
    return callFunction("ledTrigger", 
                        enable ? QString("ON") 
                               : QString("OFF")) != -1;
}

bool SpecLEDDevice::setBrightness(double brightness)
{
    SpecLightSourceDevice::setBrightness(brightness);
    
    QString param;
    param.setNum((int)4095*m_brightness);
    return callFunction("ledSetBrtns", param) != -1;
}

// ---------------------------------------------------
//  Spectron Xenon Light Source Device implementation
//  
//  This device should be tuned to optimal output and 
//  setup. Not many of the set functions are called 
//  here therefore. E.q. my lamp and power supply 
//  reach maximum output at 900V, 200Hz
// ---------------------------------------------------

// constructors/destructors
SpecXenonDevice::SpecXenonDevice()
    : SpecLightSourceDevice()
{
}

SpecXenonDevice::SpecXenonDevice(const ParticleDevice& device)
    : SpecLightSourceDevice(device)
{
}

SpecXenonDevice::~SpecXenonDevice()
{
}

// copy operator from another Particle device
SpecXenonDevice& SpecXenonDevice::operator=(ParticleDevice& device)
{
    SpecLightSourceDevice::operator=(device);
    return *this;
}

// refreshes he class from remote location
bool SpecXenonDevice::refresh()
{
    ParticleDevice::refresh();

    // refresh local variables
    if (!isConnected())
        return false;

    setBrightness(m_brightness);

	return true;
}

bool SpecXenonDevice::trigger(int trigTimeMs)
{
    QString param;
    param.setNum(trigTimeMs);
    return callFunction("XLTrigger", param) != -1;
}

bool SpecXenonDevice::trigger(bool enable)
{
    // trigger only at rated power in continuous mode
    return callFunction("XLTriggerRtd", enable ? QString("ON")
                                               : QString("OFF")) != -1;
}

bool SpecXenonDevice::setBrightness(double brightness)
{
    SpecLightSourceDevice::setBrightness(brightness);

    QString param;
    param.setNum((int)1000*m_brightness);
    return callFunction("XLSetBrghtns", param) != -1;
}
