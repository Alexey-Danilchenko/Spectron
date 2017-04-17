/*
 *  C12880MA.h - Hamamatsu C12880MA driver for Spectron board.
 *
 *  Copyright 2017 Alexey Danilchenko, Iliah Borg
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

// No pin assigned
#define NO_PIN (TOTAL_PINS+1)

// Number of spectral channels
#define SPEC_PIXELS  288

// time multipliers for easier integration time parameter
#define _SEC  *1000000UL
#define _mSEC *1000UL
#define _uSEC 

// enums
enum adc_ref_t {
    ADC_2_5V = 0,
    ADC_3V   = 1,
    ADC_4V   = 2,
    ADC_5V   = 3
};

enum black_t {
    MANUAL_BLACK   = 0,
    FOLLOWUP_BLACK = 1,
    LEADING_BLACK  = 2
};

// Spectrometer class
//
//    This class uses TIM7 timer interrupt as well as SPI so it affects their
//    configuration when used
//
class C12880MA {
private:
    // pin definitions - ADC assumes use of the standard SPI pins
	uint8_t spec_eos_, spec_trg_, spec_clk_, spec_st_, ext_trg_;
    uint8_t adc_ref_sel1_, adc_ref_sel2_, adc_cnv_;

    // Variables
    const double *calibration_;  // Hamamatsu calibration constants to provide wavelenghts
    int          blackMode_;     // Current black measuring mode
    int          adcRef_;        // ADC reference voltage
    bool         measuringData_; // Is currently measuring
    bool         applyBandPassCorrection_; // Whether to apply bandpass correction

    float  data_[SPEC_PIXELS];         // Last measurement 0..1 scaled to max ADC value
    float  blackLevels_[SPEC_PIXELS];  // Last black measurement scaled as above

    // Low-level communication - this will initiate and read spectrometer measurement data
    void readSpectrometer();

public:

    // Constructor/destructor
    // Parameters:
    //     spec_eos     - C12880MA EOS receving pin
    //     spec_trg     - C12880MA TRG controlling pin
    //     spec_clk     - C12880MA CLK clock pin
    //     spec_st      - C12880MA ST start pin
    //     ext_trg      - optional trigger pin (setting HIGH triggers external light source, camera etc.)
    //     adc_ref_sel1 - ADC voltage reference selection pin 1
    //     adc_ref_sel2 - ADC voltage reference selection pin 2
    //     adc_cnv      - ADC conversion pin (essentially SPI selection pin for ADC SPI intreface)
    //     calibration  - optional wavelength calibration factors (6 floats from Hamamatsu test sheet)
    C12880MA(uint8_t spec_eos, uint8_t spec_trg, uint8_t spec_clk, uint8_t spec_st, uint8_t ext_trg,
             uint8_t adc_ref_sel1, uint8_t adc_ref_sel2, uint8_t adc_cnv,
             const double *calibration = 0);
    ~C12880MA();

    // Setup methods and setters
    bool begin();

    // Sets the external trigger to measurement delay time. This defines time interval 
    // in uSec that offsets external trigger from the measurement. I.e. external trigger 
    // is raised and after this delay the integration and measurement starts.
    void setExtTrgMeasDelay(uint32_t extTrgMeasDelayUs);

    // Sets the trigger pin to specifid one. This could be used to trigger different things
    // with different type of measurements
    void setExtTriggerPin(uint8_t ext_trg);

    // Set integration time in microseconds (if not within 0.02s...10s boundary
    // - it will be set to either minimum or maximum allowed value)
    void setIntTime(uint32_t timeUs);

    // Set ADC reference voltage to one of the specified values. Defines maximum
    // analogue signal voltage for ADC conversion.
    void setAdcReference(adc_ref_t ref, bool storeInEeprom = true);

    // Enable/disable Stearns and Stearns (1988) bandpass correction
    void enableBandpassCorrection(bool enable);

    // Set the mode to do black measurement. Could be set to one of the
    // three modes:
    //     MANUAL_BLACK   - no auto black measurement at all. Suitable for
    //                      "always lit" spectrometer measurement in which
    //                      case black measurement could be taken manually.
    //     FOLLOWUP_BLACK - measures black following the main measurement at
    //                      the same integration time
    //     LEADING_BLACK  - measures black prior to the main measurement at
    //                      the same integration time
    //
    // The last two are automatic ways to measure black suitable for measurement
    // type where spectrometer illumination could be controlled so the dark
    // output could be measured either before or after illumination.
    void setBlackMode(black_t blackMode, bool storeInEeprom = true);

    // Take spectrometer reading at specified integration time
    void takeMeasurement();

    // Take normal spectrometer reading at specified integration time and
    // stores it as black level. This allows to do manually controlled
    // measurement of the black level reference so it should be taken
    // with spectrometer in the non-illuminated conditions
    void takeBlackMeasurement();

    // Get measured data for specified pixel (with or without black level adjustment)
    double getMeasurement(uint16_t pixelIdx, bool subtractBlack = true);

    // Get the read black value for specified pixel
    double getBlackMeasurement(uint16_t pixelIdx);

    // Get the wavelength for specified pixel
    double getWavelength(uint16_t pixel);  // get the wavelength for specified pixel

    // Attribute getters
    adc_ref_t getAdcReference()    { return (adc_ref_t)adcRef_; }
    black_t getBlackMode()         { return (black_t)blackMode_; }
    bool isMeasuring()             { return measuringData_; }
    bool isBandpassCorrected()     { return applyBandPassCorrection_; }
};

#endif
