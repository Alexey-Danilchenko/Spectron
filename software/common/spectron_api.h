/*
 *  spectron_api.h - Implementation of API calls to Spectron boards with
 *                   Hamamatsu Micro Spectrometer sensors
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

#ifndef SPECTRON_API_H
#define SPECTRON_API_H

#include <QVector>
#include "particle_api.h"

typedef QVector<double> TDoubleVec;

//
// Class that provides access to Spectron board over Particle cloud.
//
// This uses ParticleAPI classes and expects them to be connected
// and logged in.
//
class SpectronDevice: public ParticleDevice
{
public:
    enum TAdcRef {
        ADC_2_5V   = 0,
        ADC_3V     = 1,
        ADC_4_096V = 2,
        ADC_5V     = 3
    };

    enum TGain {
        NO_GAIN   = 0,
        HIGH_GAIN = 1
    };

    enum TMeasType {
        MEASURE_RELATIVE = 0, // Measurement results are relative, scaled to 0..1 range
                              // within currently selected ADC reference voltage (bandpass
                              // correction can push maximum above 1)
        MEASURE_VOLTAGE  = 1, // Measurement results are in voltage and are absolute within
                              // the same gain, scaled to 0..sensor saturation range
        MEASURE_ABSOLUTE = 2 // Measurement results are absolute, scaled to 0..1 range
                              // within sensor saturation range (bandpass correction can
                              // push maximum above 1)
    };

    enum TAutoType {
        AUTO_FOR_SET_REF   = 0,  // Maximises range for currently set ADC reference voltage
        AUTO_ALL_MIN_INTEG = 1,  // Maximises range for all ADC that achieves mimimal integration
        AUTO_ALL_MAX_RANGE = 2   // Maximises range for sensor saturation limit
    };

    // constructors/destructors
    SpectronDevice();
    ~SpectronDevice();

    // copy operator from another Particle device
    SpectronDevice& operator=(ParticleDevice& device);

    // refreshes he class from remote location
    bool refresh();

    // various actions on a sensor board
    bool setTriggerMeasurementDelay(int delayUs);
    bool setGain(TGain gain);
    bool setADCReference(TAdcRef adcRef);
    bool setMeasureType(TMeasType measType);
    bool setIntegrationTime(int integrationTimeUs);
    bool measure(int integrationTime, bool doExtTrigger = false);
    bool measureBlack(int integrationTime, bool refreshLastMeasurement = false);
    bool measureSaturation();
    bool measureAuto(TAutoType autoType);

    // getters
    double getWavelength(int pixelNum);
    double getLastMeasurement(int pixelNum);

    int          totalPixels()       { return m_totalPixels; }
    bool         supportsGain()      { return m_supportsGain; }
    TGain        getGain()           { return m_gain; }
    TAdcRef      getADCReference()   { return m_adcRef; }
    TMeasType    getMeasureType()    { return m_measType; }
    int          getIntegTime()      { return m_integTime; }
    int          getExtTrgDelay()    { return m_extTrgDelay; }
    double       getSatVoltage()     { return m_satVoltage[m_gain]; }

private:

    // members
    double          m_specCalibration[6];
    double          m_satVoltage[2];
    TAdcRef         m_adcRef;
    TGain           m_gain;
    TMeasType       m_measType;
    int             m_integTime;
    int             m_extTrgDelay;
    TDoubleVec      m_lastMeasurement;
    bool            m_supportsGain;
    int             m_totalPixels;
};

#endif // SPECTRON_API_H
