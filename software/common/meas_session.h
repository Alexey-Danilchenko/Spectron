/*
    meas_session.h - Class holding, saving and retrieving measurement 
                     session details. This contains all captured 
                     spectra, valid or invalid, position stopped at,
                     capturing device IDs (sensors). It is also 
                     responsible for saving and restoring the data 
                     to/from file (JSON format).
    
    Copyright 2017-2019 Alexey Danilchenko

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3, or (at your option)
    any later version with ADDITION (see below).

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, 51 Franklin Street - Fifth Floor, Boston,
    MA 02110-1301, USA.
*/
#ifndef MEAS_SESSION_H
#define MEAS_SESSION_H

#include <QDateTime> 
#include <QString>
#include <QVector>

#include "spectron_api.h"
#include "motor_api.h"
#include "light_source_api.h"

// Channels in measured raw file
enum EChannel 
{
    C_RED    = 0,
    C_GREEN  = 1,
    C_BLUE   = 2,

    C_MAX_CHANNELS
};

//
// Struct holding single measurement data
//
struct TSingleMeas 
{
    // data
    TDoubleVec  m_measSpectrum;
    bool        m_discardedMeas;
    QString     m_rawFileName;
    double      m_avgRawValues[C_MAX_CHANNELS];
    QDateTime   m_captureTime;
    
    // constructors
    TSingleMeas() : m_discardedMeas(false), 
                    m_rawFileName(""), 
                    m_captureTime(QDateTime::currentDateTimeUtc())
    { 
        for (int i=0; i<C_MAX_CHANNELS; i++) 
            m_avgRawValues[i] = 0.0;

    }

    // copy, assign, move - use defaults
    TSingleMeas(const TSingleMeas&) = default;
    TSingleMeas& operator=(const TSingleMeas&) = default;
    TSingleMeas(TSingleMeas&&) = default;
    TSingleMeas& operator=(TSingleMeas&&) = default;
    
    // sort by capture time
    bool operator <(const TSingleMeas& rhs)
    {
        return m_captureTime < rhs.m_captureTime;
    }
};

//
// Class holding measurement session data
//
class CMeasSession 
{
public:
    // type definitions
    using TMeasSet = QSet<TSingleMeas>;   // set of single measurements for the same wavelength

    // Constructors
    CMeasSession();
    CMeasSession(QString fileName);
    ~CMeasSession();
    
    // Configured device control
    auto& getSpectronDevice()    { return m_specDevice; } 
    auto& getLightSourceDevice() { return m_lightSrcDevice; } 
    auto& getMotorDevice()       { return m_motorDevice; } 
    
    void setSpectronDevice(ParticleDevice& device)    { m_specDevice = device; }
    void setLightSourceDevice(ParticleDevice& device) { m_lightSrcDevice = device; }
    void setMotorDevice(ParticleDevice& device)       { m_motorDevice = device; }

private:
    // type definitions
    using TMeasMap = QMap<int, TMeasSet>; // array of the measurements for wavelengths
    
    // members
    bool                        m_unsavedData;       // has unsaved changes
    QString                     m_fileName;          // associated file name to save data to
    TMeasMap                    m_measMap;           // measurements map for the specified range
    int                         m_startNm;           // start of the spectral range under measurement
    int                         m_endNm;             // end of the spectral range under measurement
    int                         m_stepNm;            // step in nm
    int                         m_exposureTimeMs;    // exposure time
    SpectronDevice              m_specDevice;        // spectrometer device used to capture measurements
    SpecLightSourceDevice       m_lightSrcDevice;    // light source device used to capture measurements
    SpecMotorDevice             m_motorDevice;       // motor device used to control and capture measurements
};

#endif // MEAS_SESSION_H
