/*
 *  particle_api.h - Implementation of subset of Particle API in QT
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

#ifndef PARTICLE_API_H
#define PARTICLE_API_H

#include <QObject>
#include <QMap>
#include <QSet>
#include <QList>
#include <QJsonValue>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QNetworkAccessManager>

// typdefs for easier handling of sized structures
typedef unsigned char byte;
typedef short              int16;
typedef unsigned short     uint16;
typedef int                int32;
typedef unsigned int       uint32;
typedef long long          int64;
typedef unsigned long long uint64;

class ParticleDevice;

typedef QList<ParticleDevice> TParticleDeviceList;

//
// Main class that provides more or less generic interface to Particle API.
//
// This exists in only one instance and holds the single instance of
// QNetworkAccessManager. The ParticleDevice class is declared as a friend
// to access protected get(), post() and put(). All communications are
// synchronous for simplicity and block for configurable timeout.
//
// Prior to get list of devices, login must be performed to obtain access
// token for the API.
//
// This does implement only a subset of Particle API:
//   - logging in
//   - obtaining device list and individual device classes
//   - calling functions on a specified device
//   - reading variables on a specified device
//
class ParticleAPI
{
public:
    friend class ParticleDevice;

    static ParticleAPI& instance();

    // login with given auth token
    bool login(const QString& authToken);
    // full login
    bool login(const QString &username, const QString &password, int expiresInSec = -1);
    // get list of matching devices
    bool getAllMatchingDevices(TParticleDeviceList &devices, QString startWith, bool connectedOnly = true);

    QString& getLastError() { return m_LastErrorStr; }
    QString& getAuthToken() { return m_authToken; }
    bool isLoggedIn() {return !m_authToken.isEmpty(); }
    ~ParticleAPI();

protected:
    bool get(const QUrl &relPath, QByteArray& resultData);
    bool post(const QUrl &relPath, const QUrlQuery &qryData, QByteArray& resultData, bool setAuthentication = false);
    bool put(const QUrl &relPath, const QUrlQuery &qryData, QByteArray& resultData);
    bool refreshConnectedDeviceList();

private:
    ParticleAPI();

    void setRawHeaders(QNetworkRequest *request, bool setAuthentication = false);
    bool syncSend(QNetworkReply *reply, QByteArray& resultData);

    // members
    QNetworkAccessManager *m_manager;
    QString m_authToken;
    QString m_authUser;
    QString m_authPassword;
    QString m_LastErrorStr;

    // constants
    static const QString c_particleApiUrl;
    static const QString c_particleApiPathLogin;
    static const QString c_particleApiVersion;
    static const QString c_particleApiDevices;
    static const QString c_particleApiPathUser;
    static const QString c_particleApiPathDevices;
    static const QString c_particleOAuthClient;
};

//
// A class that provides access to Particle device.
//
// This is instantiated for the selected device and obtained from ParticleAPI
// class. It could be initialised directly if the device ID is known and
// ParticleAPI is active (logged in).
//
// This does implement only a subset of Particle API:
//   - calling functions on this device
//   - reading variables on this device
//
class ParticleDevice
{
public:
    // internal type declarations
    enum TVarType
    {
        VAR_NONE   = 0,  // no variable
        VAR_STRING = 1,
        VAR_INT    = 2,
        VAR_DOUBLE = 3
    };
    typedef QMap<QString, TVarType> TVarMap;
    typedef QSet<QString>           TStringSet;
    typedef QList<QString>          TStringList;

    ParticleDevice();
    ParticleDevice(const QString& deviceID);
    ParticleDevice(const ParticleDevice& copy);
    virtual ~ParticleDevice();

    ParticleDevice& operator=(ParticleDevice& rhs) 
        { m_deviceID=rhs.m_deviceID; m_dataValid=false; m_connected=false; return *this; }

    bool isValid()             { return m_dataValid; }
    bool isConnected()         { return m_connected; }
    TStringList getVariables() { return m_variables.keys(); }
    TStringList getFunctions() { return m_functions.values(); }

    bool hasFunction(const QString& function) { return m_functions.contains(function); }
    bool hasVariable(const QString& variable) { return m_variables.contains(variable); }
    TVarType getVariableType(const QString& variable)
        { return m_variables.contains(variable) ? m_variables.value(variable): VAR_NONE; }

    // read variable value from device
    QJsonValue getVariableValue(const QString& variable);

    // call a function on a device
    int callFunction(const QString& function, const QString& arg);

    // actually retrieves the data and populates the class
    virtual bool refresh();

    QString& getLastResponse() { return m_lastResponse; }

private:
    // members
    ParticleAPI&  m_API;
    QString       m_deviceID;
    QString       m_deviceName;
    TStringSet    m_functions;
    TVarMap       m_variables;
    bool          m_connected;
    bool          m_dataValid;
    QString       m_lastResponse;
};

#endif // PARTICLE_API_H
