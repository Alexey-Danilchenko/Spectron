/*
    meas_session.cpp - Implementation for the class holding, saving 
                       and retrieving measurement session details. 
                       This contains all captured spectra, valid or 
                       invalid, position stopped at, capturing device 
                       IDs (sensors). It is also responsible for 
                       saving and restoring the data to/from file 
                       (JSON format).
    
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

#include "meas_session.h"
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonParseError>
