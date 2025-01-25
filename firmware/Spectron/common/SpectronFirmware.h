/*
 *  SpectronFirmware.h - Spectron board firmware main common file.
 *                       This carries out common firmware functions for
 *                       all supported spectrometer types and is expected
 *                       to be included in the main firmware file.
 *
 *  Copyright 2017-2020 Alexey Danilchenko, Iliah Borg
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

#if !defined(_SpectronFirmware_H_)
#define _SpectronFirmware_H_

#include "application.h"

#include "Spectrometer.h"

// Register firmware functions and variables in Particle cloud
bool specRegisterCloudFunctions(CSpectrometer& spec);

// Runs delayed long-running tasks. Some calibration functions can take
// long time so they scheduled rather then being run inplace.
void specRunDelayedTasks();

#endif
