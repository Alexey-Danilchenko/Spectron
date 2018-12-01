/*
 *  spectron_api.cpp - Implementation of API calls to Spectron boards with
 *                     Hamamatsu Micro Spectrometer sensors
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


#include "spectron_api.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QString>
#include <QThread>
#include <QByteArray>

// --------------------------------------
//     Spectron Device implementation
// --------------------------------------

static bool translateFloatArray(const QByteArray& buf, TDoubleVec& dblVec, int totalPixels)
{
    // table from '+' to 'z'
    static const uint8_t b64Dec[] = {
        62,  255, 63,  255, 255, 52,  53, 54, 55, 56, 57, 58, 59, 60, 61, 255,
        255, 0,   255, 255, 255, 255, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
        10,  11,  12,  13,  14,  15,  16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
        255, 255, 255, 255, 63,  255, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
        36,  37,  38,  39,  40,  41,  42, 43, 44, 45, 46, 47, 48, 49, 50, 51
    };

    float fData = 0;
    byte *data = (uint8_t*)&fData;
    int value = 0, bits = -8;
    int inCnt = 0, outCnt = 0;

    dblVec.clear();

    while (inCnt < buf.size())
    {
        char ch = buf.at(inCnt++);
        if (ch < '+' || ch > 'z')
            break;
        ch -= '+';
        if (b64Dec[ch] >= 64)
            break;
        value = (value << 6) + b64Dec[ch];
        bits += 6;
        if (bits >= 0)
        {
            data[outCnt++&3] = (value >> bits) & 0xFF;
            if ((outCnt&3) == 0)
            {
                dblVec.append((double)fData);
                data[0]=data[1]=data[2]=data[3]=0;
                if (dblVec.size() >= totalPixels)
                    break;
            }
            bits -= 8;
        }
    }

    return true;
}

// constructors/destructors
SpectronDevice::SpectronDevice()
    : ParticleDevice(), m_supportsGain(false),
      m_adcRef(ADC_2_5V), m_gain(NO_GAIN), m_totalPixels(256),
      m_measType(MEASURE_RELATIVE), m_integTime(0), m_extTrgDelay(0)
{
    for (int i=0; i<6; i++)
        m_specCalibration[i] = 0.0;
    m_satVoltage[0] = m_satVoltage[1] = 5.0;
}


SpectronDevice::~SpectronDevice()
{
}

 // copy operator from another Particle device
SpectronDevice& SpectronDevice::operator=(ParticleDevice& device)
{
    ParticleDevice::operator=(device);
    return *this;
}

bool SpectronDevice::refresh()
{
    ParticleDevice::refresh();

    // refresh local variables
    if (!isConnected())
        return false;

    m_adcRef = (TAdcRef)getVariableValue("spADCRef").toInt();
    if (hasVariable("spGain"))
    {
        m_gain = (TGain)getVariableValue("spGain").toInt();
        m_satVoltage[NO_GAIN] = getVariableValue("spNGSatVolt").toDouble();
        m_satVoltage[HIGH_GAIN] = getVariableValue("spHGSatVolt").toDouble();
        m_supportsGain = true;
    }
    else
    {
        m_gain = NO_GAIN;
        m_satVoltage[NO_GAIN] = getVariableValue("spSatVoltage").toDouble();
        m_supportsGain = false;
    }
    m_totalPixels = getVariableValue("spNumPixels").toInt();
    m_integTime = getVariableValue("spIntegTime").toInt();
    m_extTrgDelay = getVariableValue("spTrgMeasDl").toInt();
    m_measType = (TMeasType)getVariableValue("spMeasType").toInt();

    // translate calibration array if have not read it yet
    if (m_specCalibration[0] == 0.0)
    {
        QString calArr = QString("[%1]").arg(getVariableValue("spCal").toString());
        QJsonParseError parseError;
        QJsonDocument jsonDoc = QJsonDocument::fromJson(calArr.toUtf8(), &parseError);
        if (parseError.error == QJsonParseError::NoError
            && jsonDoc.isArray())
        {
            QJsonArray cal = jsonDoc.array();
            if (cal.size() == 6)
                for (int i=0; i<6; i++)
                    m_specCalibration[i] = cal.at(i).toDouble();
        }
    }
	return true;
}

bool SpectronDevice::setTriggerMeasurementDelay(int delayUs)
{
    if (delayUs < 0)
        return true;

    QString param;
    m_extTrgDelay = delayUs;
    param.setNum(delayUs);
    return callFunction("spSetTrMesDl", param) != -1;
}

bool SpectronDevice::setGain(TGain gain)
{
    if (hasFunction("spSetGain"))
    {
        m_gain = gain;
        QString param;
        param.setNum(gain);
        return callFunction("spSetGain", param) != -1;
    }
    return false;
}

bool SpectronDevice::setADCReference(TAdcRef adcRef)
{
    m_adcRef = adcRef;
    QString param;
    param.setNum(adcRef);
    return callFunction("spSetADCRef", param) != -1;
}

bool SpectronDevice::setMeasureType(TMeasType measType)
{
    m_measType = measType;
    QString param;
    param.setNum(m_measType);
    return callFunction("spSetMeasTyp", param) != -1;
}

bool SpectronDevice::setIntegrationTime(int integrationTimeUs)
{
    if (integrationTimeUs <= 0)
        return true;

    QString param;
    param.setNum(integrationTimeUs);
    if (callFunction("spSetIntTime", param) == -1)
        return false;
    m_integTime = getVariableValue("spIntegTime").toInt();

    return true;
}

bool SpectronDevice::measure(int measTimeUs, bool doExtTrigger)
{
    bool success = false;

    QString param;
    param.setNum(measTimeUs);
    if (doExtTrigger)
        param.append(",TRG");
    if (callFunction("spMeasure", param) != -1)
    {
        QByteArray lastMeas = getVariableValue("spLastMeas1").toString().toUtf8();
        lastMeas.append(getVariableValue("spLastMeas2").toString().toUtf8());
        lastMeas.append(getVariableValue("spLastMeas3").toString().toUtf8());
        success = translateFloatArray(lastMeas, m_lastMeasurement, m_totalPixels);
    }

    return success;
}

bool SpectronDevice::measureBlack(int measTimeUs, bool refreshLastMeasurement)
{
    bool success = false;

    QString param;
    if (measTimeUs > 0)
        param.setNum(measTimeUs);

    // call
    if (success = callFunction("spMeasureBlk", param) != -1
        && refreshLastMeasurement)
    {
        QByteArray lastMeas = getVariableValue("spLastMeas1").toString().toUtf8();
        lastMeas.append(getVariableValue("spLastMeas2").toString().toUtf8());
        lastMeas.append(getVariableValue("spLastMeas3").toString().toUtf8());
        success = translateFloatArray(lastMeas, m_lastMeasurement, m_totalPixels);
    }

    return success;
}

// automatic saturation measurement
bool SpectronDevice::measureSaturation()
{
    return callFunction("spSetSatVolt", "AUTO") != -1;
}

// automatic measurement - determines the best integration time
bool SpectronDevice::measureAuto(TAutoType autoType)
{
    bool success = false;
    QString param = "AUTO_FOR_SET_REF";

    if (autoType == AUTO_ALL_MIN_INTEG)
        param = "AUTO_ALL_MIN_INTEG";
    else if (autoType == AUTO_ALL_MAX_RANGE)
        param = "AUTO_ALL_MAX_RANGE";

    if (callFunction("spMeasure", param) != -1)
    {
        QByteArray lastMeas = getVariableValue("spLastMeas1").toString().toUtf8();
        lastMeas.append(getVariableValue("spLastMeas2").toString().toUtf8());
        lastMeas.append(getVariableValue("spLastMeas3").toString().toUtf8());
        success = translateFloatArray(lastMeas, m_lastMeasurement, m_totalPixels);

        // refresh variables
        m_adcRef = (TAdcRef)getVariableValue("spADCRef").toInt();
        if (m_supportsGain)
            m_gain = (TGain)getVariableValue("spGain").toInt();
        m_integTime = getVariableValue("spIntegTime").toInt();
    }

    return success;
}

// get bandpass corrected (or not) measurement result
double SpectronDevice::getLastMeasurement(int pixelNum)
{
    if (pixelNum >= m_lastMeasurement.size()
        || m_lastMeasurement.size() == 0)
        return 0.0;

    return m_lastMeasurement.at(pixelNum);
}

// get the wavelength for specified pixel
double SpectronDevice::getWavelength(int pixelNum)
{
    // pixelNumber in formula start with 1
    double p = pixelNum+1;
    return m_specCalibration[0]
           + (p*m_specCalibration[1])
           + (p*p*m_specCalibration[2])
           + (p*p*p*m_specCalibration[3])
           + (p*p*p*p*m_specCalibration[4])
           + (p*p*p*p*p*m_specCalibration[5]);
}

