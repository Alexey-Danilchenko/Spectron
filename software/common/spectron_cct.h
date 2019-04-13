/*
 *  spectron_cct.h - Declarations of various colour functions operating 
 *                   on spectral measurements from Hamamatsu sensors.
 *
 *  Copyright 2019 Alexey Danilchenko
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

// Inputs: Wavelength in nanometers
double xFunc_1931(double wavelength);
double yFunc_1931(double wavelength);
double zFunc_1931(double wavelength);

void calculateColourParam(SpectronDevice& spectron, double &CCT, double &x, double &y);
