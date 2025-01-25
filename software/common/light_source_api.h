/*
 *  light_source_api.h - Implementation of API calls to Spectron Light Source boards
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

#ifndef LIGHT_SRC_API_H
#define LIGHT_SRC_API_H

#include <QVector>
#include "particle_api.h"

typedef QVector<double> TDoubleVec;

//
// Base class that provides access to Spectron Light Source board over Particle cloud.
//
// This uses ParticleAPI classes and expects them to be connected
// and logged in.
//
class SpecLightSourceDevice: public ParticleDevice
{
public:
    // constructors/destructors
    SpecLightSourceDevice();
    SpecLightSourceDevice(const ParticleDevice& device);
    ~SpecLightSourceDevice();

    // copy operator from another Particle device
    SpecLightSourceDevice& operator=(ParticleDevice& device);

    // refreshes he class from remote location
    virtual bool refresh() = 0;

    // actions
    virtual bool trigger(int trigTimeMs) = 0;   // timed operation
    virtual bool trigger(bool enable) = 0;      // continuous operation

    // various setters
    virtual bool setBrightness(double brightness);  // brightness in 0...1 range

    // getters
    double getBrightness() { return m_brightness; };

protected:
    // members
    double  m_brightness;
};

//
// LED light source class that provides access to LED Light Source board 
// over Particle cloud.
//
// This uses ParticleAPI classes and expects them to be connected
// and logged in.
//
class SpecLEDDevice: public SpecLightSourceDevice
{
public:
    // constructors/destructors
    SpecLEDDevice();
    SpecLEDDevice(const ParticleDevice& device);
    ~SpecLEDDevice();

    // copy operator from another Particle device
    SpecLEDDevice& operator=(ParticleDevice& device);

    // refreshes he class from remote location
    virtual bool refresh();

    // actions
    bool trigger(int trigTimeMs);   // timed operation
    bool trigger(bool enable);      // continuous operation

    // various setters
    bool setBrightness(double brightness);  // brightness in 0...1 range

    // getters
};


//
// Xenon light source class that provides access to Xenon Light Source board 
// over Particle cloud.
//
// This uses ParticleAPI classes and expects them to be connected
// and logged in.
//
class SpecXenonDevice: public SpecLightSourceDevice
{
public:
    // constructors/destructors
    SpecXenonDevice();
    SpecXenonDevice(const ParticleDevice& device);
    ~SpecXenonDevice();

    // copy operator from another Particle device
    SpecXenonDevice& operator=(ParticleDevice& device);

    // refreshes he class from remote location
    virtual bool refresh();

    // actions
    bool trigger(int trigTimeMs);   // timed operation
    bool trigger(bool enable);      // continuous operation

    // various setters
    bool setBrightness(double brightness);  // brightness in 0...1 range

    // getters
};


#endif // LIGHT_SRC_API_H
