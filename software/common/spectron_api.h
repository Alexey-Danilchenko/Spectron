/*
 *  spectron_api.h - Implementation of API calls to Spectron boards with
 *                   Hamamatsu Micro Spectrometer sensors
 *
 *  Copyright 2017-2019 Alexey Danilchenko
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

    enum TDataType {
        ET_MEASUREMENT   = 0,    // results of the last measurement
        ET_BLACK_LEVELS  = 1,    // black level voltages
        ET_NORMALISATION = 2     // normalisation coefficients
    };

    enum TRangeType {
        RT_EXPLICIT = 0,    // range defined explicitly
        RT_DEFAULT  = 1,    // default range for this spectrometer type
        RT_MAX      = 2     // maximum range for this spectrometer
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
    bool setMinBlack(double blackLevelVoltage = -1.0);
    bool calibrateSpectralResponse(double lampTempK, bool useLastMeasurement = true);
    bool measure(int integrationTime=0, bool doExtTrigger = false);
    bool measureBlack(int integrationTime = 0, bool refreshLastMeasurement = false);
    bool measureSaturation();
    bool measureAuto(TAutoType autoType);
    bool getSpectrometerData(TDataType dataType);
    bool setSpectralRange(TRangeType rangeType, int minWavelength=-1, int maxWavelength=-1);
    bool resetToDefaults();
    void setSpectralRespCorrection(bool enable) {  m_applySpectralCorrection = enable; }

    // getters
    double getMinWavelength();
    double getMaxWavelength();
    double getWavelength(int pixelNum);
    double getLastMeasurement(int pixelNum);

    int          totalPixels()              { return m_totalPixels; }
    bool         supportsGain()             { return m_supportsGain; }
    TGain        getGain()                  { return m_gain; }
    TAdcRef      getADCReference()          { return m_adcRef; }
    TMeasType    getMeasureType()           { return m_measType; }
    int          getIntegTime()             { return m_integTime; }
    int          getExtTrgDelay()           { return m_extTrgDelay; }
    double       getSatVoltage()            { return m_satVoltage[m_gain]; }
    double       getSatVoltage(TGain gain)  { return m_satVoltage[gain]; }
    double       getMinBlackVoltage()       { return m_minVlackVoltage; }
    double       getMaxLastMeasuredValue()  { return m_maxLastMeasuredValue; }
    bool         applySpectralCorrections() { return m_applySpectralCorrection; }

private:
    // private functions
    void getData();

    // members
    double          m_specCalibration[6];
    double          m_satVoltage[2];
    double          m_minVlackVoltage;
    TAdcRef         m_adcRef;
    TGain           m_gain;
    TMeasType       m_measType;
    int             m_integTime;
    int             m_extTrgDelay;
    TDoubleVec      m_lastMeasurement;
    double          m_maxLastMeasuredValue;
    bool            m_supportsGain;
    int             m_totalPixels;
    int             m_pixelOffsetIdx;
    bool            m_applySpectralCorrection;
};

#endif // SPECTRON_API_H
