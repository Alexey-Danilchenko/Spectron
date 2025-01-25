/*
 *  C12880MA.h - Hamamatsu C12880MA driver for Spectron board.
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

#if !defined(_C12880MA_H_)
#define _C12880MA_H_

#include "application.h"
#include "Spectrometer.h"

// Spectrometer class
//
// This class uses TIM7 timer interrupt as well as SPI so it affects their
// configuration when used
//
class C12880MA: public CSpectrometer  {
private:
    // pin definitions - ADC assumes use of the standard SPI pins
    uint8_t adc_cnv_, spec_eos_, spec_trg_, spec_clk_, spec_st_, ext_trg_, ext_trg_ls_;

protected:
    // --------------------------------------------------------------------------
    //  Virtual methods - hardware specific implementations for Hamamatsu C12880
    // --------------------------------------------------------------------------

    // This is the main method that should be overriden by specific spectrometer
    // implementation. It should implement hardware specific initialisation, reading 
    // loop and termination.
    void readSpectrometer(double measTimeUs, 
                          bool doExtTriggering, 
                          bool doLightTriggering) override;

    // Hardware specific sensor constraints - sets data variable for a specific sensor
    // constraint (pointer is expected to point to a variable of correct datatype for 
    // that constraint)
    bool getSensorConstraint(sensor_constraint_t sensConstraint, void* data) override;

    // Hardware specific sets integration time in microseconds for a single
    // measurement cycle.
    //
    // NOTE: if not within sensor allowed boundaries then it will be set to
    //       closest minimum or maximum allowed value
    // NOTE: internally time is measured in clock cycles so real integration
    //       time will be aligned to the clock cycle boundaries
    void setIntTimeInternal(double timeUs, bool saveState = false) override;

    // Hardware specific sets the external trigger to measurement delay time.
    // This defines time interval in uSec that offsets external trigger from 
    // the measurement. I.e. external trigger is raised and after this delay 
    // the integration and measurement starts.
    //
    // Specifying -1 as  delay will disable the external trigger
    void setExtTrgMeasDelayInternal(double delayUs, bool saveState = false) override;

public:

    // Constructor/destructor
    // Parameters:
    //     spec_eos      - C12880MA EOS receving pin
    //     spec_trg      - C12880MA TRG controlling pin
    //     spec_clk      - C12880MA CLK clock pin
    //     spec_st       - C12880MA ST start pin
    //     adc_ref_sel1  - ADC voltage reference selection pin 1
    //     adc_ref_sel2  - ADC voltage reference selection pin 2
    //     adc_cnv       - ADC conversion pin (essentially SPI selection pin for ADC SPI intreface)
    //     ext_trg       - optional trigger pin for capture device (setting HIGH triggers external device)
    //     ext_trg_ls    - optional trigger pin for lightsource (setting HIGH triggers light source)
    //     wvCalibration - factory wavelength calibration factors (array of 6 doubles from Hamamatsu test sheet)
    //     baseEEPROM    - base address to srore the spectrometer settings in EEPROM (-1 if not used)
    C12880MA(uint8_t spec_eos, uint8_t spec_trg, uint8_t spec_clk, uint8_t spec_st,
             uint8_t adc_ref_sel1, uint8_t adc_ref_sel2, uint8_t adc_cnv,
             uint8_t ext_trg, uint8_t ext_trg_ls, 
             const double *wvCalibration, const int baseEEPROM = 0);
    ~C12880MA();

    // Initialise known pins. Specific hardware implementations typically will
    // have their own implementations of this initialising spectrometer specific
    // pins.
    bool begin();

    // --------------------------------------------------------------------------
    //  Virtual methods - hardware specific implementations for Hamamatsu C12666
    // --------------------------------------------------------------------------

    // Return trigger measurement delay in specified units
    double getExtTrgMeasDelay(time_units_t units = T_USEC) override;
    
    // Return integration time in specified units
    double getIntTime(time_units_t units = T_USEC) override;
};

#endif
