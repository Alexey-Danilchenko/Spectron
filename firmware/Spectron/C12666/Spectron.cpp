/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Development/spectron/firmware/Spectron/C12666/Spectron.ino"
/*
 *  Spectron.ino - Spectron board firmware main file.
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

void setup();
void loop(void);
#line 22 "c:/Development/spectron/firmware/Spectron/C12666/Spectron.ino"
SYSTEM_MODE(MANUAL);

// Specify EEPROM base address for C12666 state saved - should
// be before included header
#define EEPROM_C12666_BASE_ADDR  0

#include "C12666MA.h"

#include "SpectronFirmware.h"

#define ADC_REF_SEL_1  A1
#define ADC_REF_SEL_2  A0
#define ADC_CNV        DAC
#define SPI_SS_1       WKP
#define D_PWM          RX
#define SPI_SS_2       A2
#define SPI_MOSI       A5
#define SPI_MISO       A4
#define SPI_SCK        A3

// External measurement trigger pin - not used currently
#define TRG_IN         A2

// Light source could be triggered externally or from the spectral
// measurement via pin. If the latter is needed set the pin to
// the one used (TX is used for my board)
#define TRG_LIGHT_SRC  NO_PIN

// Pin for external registering device triggering - camera
// for my board
#define TRG_CAMERA     D2

// Spectrometer pins
#define TRG_3V         D3
#define EOS_3V         D4
#define CLK_3V         D5
#define ST_3V          D6
#define GAIN_3V        D7

// Factory calibration data for my sensor 15F00163 from Hamamatsu
// specify the real one for your sensor here
const double FACTORY_CALIBRATION[] = {
   323.3668711,2.384682045,-5.995865297E-4,-8.602293347E-6,1.840343099E-8,-1.424592223E-11
};

// spectrometer object - can be only one per application
C12666MA spec(GAIN_3V,
              EOS_3V,
              TRG_3V,
              CLK_3V,
              ST_3V,
              ADC_REF_SEL_1,
              ADC_REF_SEL_2,
              ADC_CNV,
              TRG_CAMERA,
              TRG_LIGHT_SRC,
              FACTORY_CALIBRATION,
              EEPROM_C12666_BASE_ADDR);

// main firmware initialisation
void setup()
{
    pinMode(TRG_CAMERA, OUTPUT);
    pinMode(TRG_LIGHT_SRC,  OUTPUT);

    // initialise spectrometer
    spec.begin();

    // initialise Particle variables
    specRegisterCloudFunctions(spec);

    // connect
    if (!Particle.connected())
        Particle.connect();
    Particle.process();
}

// Main event loop
//    only use it for particle connection keep alive when spectrometer
//    measurement is not running
void loop(void)
{
    // call for Photon process for manual system mode
    if (!spec.isMeasuring()) {
        specRunDelayedTasks();
        if (Particle.connected())
            Particle.process();
        else
            Particle.connect();
    }    
}
