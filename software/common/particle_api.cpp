/*
 *  particle_api.cpp - Implementation of subset of Particle API in QT
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

#include "particle_api.h"
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QUrlQuery>
#include <QTimer>
#include <QEventLoop>

// --------------------------------------
//      ParticleAPI implementation
// --------------------------------------
const QString ParticleAPI::c_particleApiUrl         = "https://api.particle.io/";
const QString ParticleAPI::c_particleApiPathLogin   = "oauth/token";
const QString ParticleAPI::c_particleApiVersion     = "v1";
const QString ParticleAPI::c_particleApiDevices     = "devices";
const QString ParticleAPI::c_particleApiPathUser    = "user";
const QString ParticleAPI::c_particleApiPathDevices = "devices";
const QString ParticleAPI::c_particleOAuthClient    = "particle:particle";

ParticleAPI& ParticleAPI::instance()
{
    static ParticleAPI s_Instance;

    return s_Instance;
}

ParticleAPI::ParticleAPI()
{
    m_manager = new QNetworkAccessManager();
}

ParticleAPI::~ParticleAPI()
{
    delete m_manager;
}

void ParticleAPI::setRawHeaders(QNetworkRequest *request, bool setAuthentication)
{
    if (setAuthentication)
    {
        QByteArray authHeader = "Basic ";
        authHeader.append(c_particleOAuthClient.toUtf8().toBase64());
        request->setRawHeader("Authorization", authHeader);
    }
    // any other common headers - add here
    request->setHeader(QNetworkRequest::ContentTypeHeader,
                       "application/x-www-form-urlencoded");
}

// login with given auth token
bool ParticleAPI::login(const QString& authToken)
{
    m_authUser = "";
    m_authPassword = "";
    m_authToken = authToken;
    return true;
}

// full login
bool ParticleAPI::login(const QString &username, const QString &password, int expiresInSec)
{
    bool success = false;

    m_authUser = username;
    m_authPassword = password;

    QUrlQuery qryData;
    QString expiryStr;
    expiryStr.setNum(expiresInSec);

    qryData.addQueryItem("grant_type", "password");
    qryData.addQueryItem("username",   m_authUser);
    qryData.addQueryItem("password",   m_authPassword);
    if (expiresInSec >= 0)
        qryData.addQueryItem("expires_in", expiryStr);

    QByteArray resultData;
    QUrl loginPath = c_particleApiPathLogin;
    if (post(loginPath, qryData, resultData, true))
    {
        // process data
        QJsonParseError parseError;
        QJsonDocument reply = QJsonDocument::fromJson(resultData, &parseError);
        if (parseError.error == QJsonParseError::NoError
            && reply.isObject()
            && reply.object().contains("access_token"))
        {
            m_authToken = reply.object()["access_token"].toString().trimmed();
            success = true;
        }
        else
        {
            // error - invalid reply
        }
    }
    else
    {
        // report an error
        // cannot login
    }

    return success;
}

bool ParticleAPI::getAllMatchingDevices(TParticleDeviceList &devices, QString startsWith, bool connectedOnly)
{
    bool success = false;

    // cler resulting list
    devices.clear();

    QByteArray resultData;
    QUrl getDevicesPath = QString("%1/%2").arg(c_particleApiVersion).arg(c_particleApiDevices);
    if (get(getDevicesPath, resultData))
    {
        // process data
        QJsonParseError parseError;
        QJsonDocument reply = QJsonDocument::fromJson(resultData, &parseError);
        if (parseError.error == QJsonParseError::NoError
            && reply.isArray())
        {
            QJsonArray deviceArr = reply.array();
            QJsonArray::const_iterator devIter = deviceArr.constBegin();
            while (devIter != deviceArr.constEnd()
                   && devIter->isObject())
            {
                QJsonObject device = devIter->toObject();
                if ((startsWith.isEmpty() || device["name"].toString().startsWith(startsWith))
                    && (device["connected"].toBool() || !connectedOnly))
                {
                    // add the device
                    ParticleDevice newDevice(device["id"].toString());
                    devices.append(newDevice);
                }

                ++devIter;
            }
            success = true;
        }
        else
        {
            // error - invalid reply
        }
    }
    else
    {
        // report an error
        // cannot login
    }

    return success;
}

bool ParticleAPI::syncSend(QNetworkReply *reply, QByteArray& resultData)
{
    // timeout - 2 mins
    const qint64 networkTimeoutMs = 120000;

    bool success = true;

    QTimer timer;
    timer.setInterval(networkTimeoutMs);
	timer.setSingleShot(true);

    QEventLoop loop;
    QObject::connect(reply, SIGNAL(finished()), &loop, SLOT(quit()));
	QObject::connect(&timer, SIGNAL(timeout()), reply, SLOT(abort()));

	timer.start();
    loop.exec();
    timer.stop();

    if (reply->isFinished() && reply->error() == QNetworkReply::NoError)
    {
        resultData = reply->readAll();
    }
    else
    {
        success = false;
        // check for timeouts
        if (reply->error() == QNetworkReply::OperationCanceledError)
            m_LastErrorStr = "The connection to the remote server timed out";
        else
            m_LastErrorStr = reply->errorString();
    }

    reply->deleteLater();

    return success;
}

bool ParticleAPI::get(const QUrl &relPath, QByteArray& resultData)
{
    m_LastErrorStr.clear();

    QUrl url = QUrl(c_particleApiUrl).resolved(relPath);
    if (!m_authToken.isEmpty())
        url.setQuery(QString("access_token=%1").arg(m_authToken));

    QNetworkRequest request(url);
    setRawHeaders(&request);

    QNetworkReply *reply = m_manager->get(request);

    // we are only interested in synchronous calls
    return syncSend(reply, resultData);
}

bool ParticleAPI::post(const QUrl &relPath, const QUrlQuery &qryData, QByteArray& resultData, bool setAuthentication)
{
    m_LastErrorStr.clear();

    QUrl url = QUrl(c_particleApiUrl).resolved(relPath);
    if (!m_authToken.isEmpty())
        url.setQuery(QString("access_token=%1").arg(m_authToken));

    QNetworkRequest request(url);
    setRawHeaders(&request, setAuthentication);

    QNetworkReply *reply = m_manager->post(request, qryData.toString(QUrl::FullyEncoded).toUtf8());

    // we are only interested in synchronous calls
    return syncSend(reply, resultData);
}

bool ParticleAPI::put(const QUrl &relPath, const QUrlQuery &qryData, QByteArray& resultData)
{
    m_LastErrorStr.clear();

    QUrl url = QUrl(c_particleApiUrl).resolved(relPath);
    if (!m_authToken.isEmpty())
        url.setQuery(QString("access_token=%1").arg(m_authToken));

    QNetworkRequest request(url);
    setRawHeaders(&request);

    QNetworkReply *reply = m_manager->put(request, qryData.toString(QUrl::FullyEncoded).toUtf8());

    // we are only interested in synchronous calls
    return syncSend(reply, resultData);
}

// --------------------------------------
//     ParticleDevice implementation
// --------------------------------------
ParticleDevice::ParticleDevice()
    : m_API(ParticleAPI::instance()), m_deviceID(""), m_dataValid(false), m_connected(false)
{
    // remote connection happens when refresh is called - later
}

ParticleDevice::ParticleDevice(const QString& deviceID)
    : m_API(ParticleAPI::instance()), m_deviceID(deviceID), m_dataValid(false), m_connected(false)
{
    // remote connection happens when refresh is called - later
}

ParticleDevice::ParticleDevice(const ParticleDevice& copy)
    : m_API(ParticleAPI::instance()), m_deviceID(copy.m_deviceID), m_dataValid(false), m_connected(false)
{
}

ParticleDevice::~ParticleDevice()
{
}

// read variable value from device
QJsonValue ParticleDevice::getVariableValue(const QString& variable)
{
    QJsonValue result;

    if (!variable.isEmpty() 
        && (m_variables.contains(variable) || !m_dataValid))
    {
        QByteArray resultData;
        QUrl varPath = QString("%1/%2/%3/%4").arg(m_API.c_particleApiVersion)
                                             .arg(m_API.c_particleApiDevices)
                                             .arg(m_deviceID)
                                             .arg(variable);
        if (m_API.get(varPath, resultData))
        {
            m_lastResponse += "\n";
            m_lastResponse += resultData;
            // process data
            QJsonParseError parseError;
            QJsonDocument reply = QJsonDocument::fromJson(resultData, &parseError);
            if (parseError.error == QJsonParseError::NoError
                && reply.isObject())
            {
                QJsonObject varData = reply.object();

                result = varData["result"];
            }
            else
            {
                // error - invalid reply
            }
        }
        else
        {
            // report an error
        }
    }

    return result;
}

// call a function on a device
int ParticleDevice::callFunction(const QString& function, const QString& arg)
{
    int result = -1;

    if (!function.isEmpty() 
        && (m_functions.contains(function) || !m_dataValid))
    {
        QByteArray resultData;
        QUrlQuery  qryData;
        QUrl funcPath = QString("%1/%2/%3/%4").arg(m_API.c_particleApiVersion)
                                              .arg(m_API.c_particleApiDevices)
                                              .arg(m_deviceID)
                                              .arg(function);
        qryData.addQueryItem("arg", arg);
                                              
        if (m_API.post(funcPath, qryData, resultData))
        {
            m_lastResponse = resultData;
            // process data
            QJsonParseError parseError;
            QJsonDocument reply = QJsonDocument::fromJson(resultData, &parseError);
            if (parseError.error == QJsonParseError::NoError
                && reply.isObject())
            {
                QJsonObject fResult = reply.object();

                result = fResult["return_value"].toInt(-1);
            }
            else
            {
                // error - invalid reply
            }
        }
        else
        {
            // report an error
        }
    }

    return result;
}

// refresh device data, functions and variables
bool ParticleDevice::refresh()
{
    bool success = false;

    QByteArray resultData;
    QUrl getDevicePath = QString("%1/%2/%3").arg(m_API.c_particleApiVersion)
                                            .arg(m_API.c_particleApiDevices)
                                            .arg(m_deviceID);
    if (m_API.get(getDevicePath, resultData))
    {
        m_lastResponse = resultData;
    
        // process data
        QJsonParseError parseError;
        QJsonDocument reply = QJsonDocument::fromJson(resultData, &parseError);
        if (parseError.error == QJsonParseError::NoError
            && reply.isObject())
        {
            QJsonObject device = reply.object();

            m_deviceName = device["name"].toString();
            m_connected  = device["connected"].toBool();

            // repopulate variables and functions
            m_functions.clear();
            m_variables.clear();

            // read all functions
            if (device["functions"].isArray())
            {
                QJsonArray fArray = device["functions"].toArray();
                QJsonArray::const_iterator fIter = fArray.constBegin();
                while (fIter != fArray.constEnd())
                {
                    m_functions.insert(fIter->toString());
                    ++fIter;
                }
            }

            // read all variables
            if (device["variables"].isObject())
            {
                QJsonObject vObj = device["variables"].toObject();
                QJsonObject::const_iterator vIter = vObj.constBegin();
                while (vIter != vObj.constEnd())
                {
                    QString  varName = vIter.key();
                    QString  varTypeStr = vIter.value().toString();
                    TVarType varType = VAR_NONE;
                    if (varTypeStr == "int32")
                        varType = VAR_INT;
                    else if (varTypeStr == "double")
                        varType = VAR_DOUBLE;
                    else if (varTypeStr == "string")
                        varType = VAR_STRING;
                    m_variables.insert(varName, varType);
                    ++vIter;
                }
            }
            success = true;
            m_dataValid = true;
        }
        else
        {
            // error - invalid reply
        }
    }
    else
    {
        // report an error
    }

    return success;
}
