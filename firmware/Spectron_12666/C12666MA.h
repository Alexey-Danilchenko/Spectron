/*
 *  C12666MA.h - Hamamatsu C12666MA driver for Spectron board.
 *
 *  Copyright 2015-2018 Alexey Danilchenko, Iliah Borg
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

#if !defined(_C12666MA_H_)
#define _C12666MA_H_

#include "application.h"

// Configurational definitions

// Enable 4 averaging reads (slower) by uncommenting below
// #define ADC_AVG_4

// No pin assigned
#ifndef NO_PIN
#define NO_PIN (TOTAL_PINS+1)
#endif

// Number of spectral channels
#define SPEC_PIXELS  256

// time multipliers for easier integration time parameter
#define _SEC  *1000000UL
#define _mSEC *1000UL
#define _uSEC

// enums
enum measure_t {
    MEASURE_RELATIVE = 0, // Measurement results are relative, scaled to 0..1 range
                          // within currently selected ADC reference voltage (bandpass
                          // correction can push maximum above 1)
    MEASURE_VOLTAGE  = 1, // Measurement results are in voltage and are absolute within
                          // the same gain, scaled to 0..sensor saturation range
    MEASURE_ABSOLUTE = 2, // Measurement results are absolute, scaled to 0..1 range
                          // within sensor saturation range (bandpass correction can
                          // push maximum above 1)
};

enum adc_ref_t {
    ADC_2_5V   = 0,   // 2.5V reference
    ADC_3V     = 1,   // 3V reference
    ADC_4_096V = 2,   // 4.096V reference
    ADC_5V     = 3    // 5V reference
};

enum gain_t {
    NO_GAIN   = 0,
    HIGH_GAIN = 1
};

// Types of automatic measurement
enum auto_measure_t {
    AUTO_FOR_SET_REF,    // Maximises range for currently set ADC reference voltage
    AUTO_ALL_MIN_INTEG,  // Maximises range for all ADC that achieves mimimal integration
    AUTO_ALL_MAX_RANGE   // Maximises range for sensor saturation limit
};

// EEPROM base address and area size used by this class
// (base address should be defined externally)
#ifndef EEPROM_C12666_BASE_ADDR
#define EEPROM_C12666_BASE_ADDR  0
#endif
#define EEPROM_C12666_SIZE       76

// Spectrometer class
//
//    This class uses TIM7 timer interrupt as well as SPI so it affects their
//    configuration when used
//
class C12666MA {
private:
    // pin definitions - ADC assumes use of the standard SPI pins
    uint8_t spec_gain_, spec_eos_, spec_trg_, spec_clk_, spec_st_, ext_trg_, ext_trg_ls_;
    uint8_t adc_ref_sel1_, adc_ref_sel2_, adc_cnv_;

    // Variables
    double    calibration_[6];  // Hamamatsu calibration constants to provide wavelenghts
    gain_t    gain_;            // High/low gain
    adc_ref_t adcRef_;          // ADC reference voltage
    measure_t measurementType_; // Measurement result type - relative 0..1 value or voltage
    bool      measuringData_;   // Is currently measuring
    bool      applyBandPassCorrection_; // Whether to apply bandpass correction
    int       extTrgMeasDelayUs_;       // Measurement delay from triggering if enabled
    float     satVoltageHighGain_;      // Sensor saturation voltage with high gain
    float     satVoltageNoGain_;        // Sensor saturation voltage with no gain

    float     data_[SPEC_PIXELS];         // Last measurement 0..1 scaled according to measurementType_
    float     blackLevels_[SPEC_PIXELS];  // Last black measurement scaled as above

    // Low-level communication - this will initiate and read spectrometer measurement data
    void readSpectrometer(uint32_t timeUs, bool doExtTriggering, bool doLightTriggering);
    void setAdcRefInternal(adc_ref_t adcRef);
    void setGainInternal(gain_t gain);
    float processMeasurement(float* measurement, measure_t measurementType = MEASURE_RELATIVE);

public:

    // Constructor/destructor
    // Parameters:
    //     spec_gain    - C12666MA GAIN controlling pin
    //     spec_eos     - C12666MA EOS receving pin
    //     spec_trg     - C12666MA TRG controlling pin - used for software interrupt only
    //     spec_clk     - C12666MA CLK clock pin
    //     spec_st      - C12666MA ST start pin
    //     adc_ref_sel1 - ADC voltage reference selection pin 1
    //     adc_ref_sel2 - ADC voltage reference selection pin 2
    //     adc_cnv      - ADC conversion pin (essentially SPI selection pin for ADC SPI intreface)
    //     ext_trg      - optional trigger pin for capture device (setting HIGH triggers external device)
    //     ext_trg_ls   - optional trigger pin for lightsource (setting HIGH triggers light source)
    //     calibration  - optional wavelength calibration factors (array of 6 doubles from Hamamatsu test sheet)
    C12666MA(uint8_t spec_gain, uint8_t spec_eos, uint8_t spec_trg, uint8_t spec_clk, uint8_t spec_st,
             uint8_t adc_ref_sel1, uint8_t adc_ref_sel2, uint8_t adc_cnv,
             uint8_t ext_trg, uint8_t ext_trg_ls, const double *defaultCalibration);
    ~C12666MA();

    // Setup methods and setters
    bool begin();

    // Sets the wavelength calibration coeffiecients. This is array of 6
    // values that represent polinomial coefficients. Usually provided
    // by Hamamatsu but can be overriden by user calculated ones.
    void setWavelengthCalibration(const double* wavelengthCal, bool storeInEeprom = true);

    // Sets the external trigger to measurement delay time. This defines time interval
    // in uSec that offsets external trigger from the measurement. I.e. external trigger
    // is raised and after this delay the integration and measurement starts.
    //
    // Specifying -1 as  delay will disable the external trigger
    void setExtTrgMeasDelay(int32_t extTrgMeasDelayUs, bool storeInEeprom = true);

    // Set integration time in microseconds for a single measurement cycle.
    // The real measurement time can be more that this period in which case
    // several measurement cycles will be taken sequentially and averaged out.
    //
    // NOTE: if not within 0.02s...1s boundary then it will be set to either
    // minimum or maximum allowed value)
    void setIntTime(uint32_t timeUs, bool storeInEeprom = true);

    // Set the spectrometer measurement type. This allows to change the type
    // of the measurement results and will affect all subsequent measurements.
    // The measurement results could be stored:
    //   (1) relatively - 0..1 values for currently selected ADC reference
    //   (2) as voltage measurements irrespective to ADC parameters but dependent
    //       on gain (these are absolute within the same gain settings)
    //   (3) as absolute measurements, 0..1 range scaled to saturation voltage
    void setMeasurementType(measure_t measurementType, bool storeInEeprom = true);

    // Set spectrometer gain to low or high
    void setGain(gain_t gain, bool storeInEeprom = true);

    // Set ADC reference voltage to one of the specified values. Defines maximum
    // analogue signal voltage for ADC conversion. This generally should be used
    // together with setting the spectrometer gain (high gain will mean higher
    // reference voltage) but it is decoupled for flexibility of ADC control.
    void setAdcReference(adc_ref_t ref, bool storeInEeprom = true);

    // Enable/disable Stearns and Stearns (1988) bandpass correction
    void enableBandpassCorrection(bool enable);

    // Set the saturation voltages. These are used to in auto integration mode
    // of measurement. Passing values outside of ranges from Hamamatsu spec 
    // will reset approprite value to a default ones.
    void setSaturationVoltages(float satVoltageHighGain,
                               float satVoltageNoGain,
                               bool storeInEeprom = true);

    // Automatic measurement of the saturation voltages. These are used to in
    // auto integration mode of measurement.
    //
    // Automatic setting works by exposing sensor to bright light and then
    // calling this method to work out saturation voltages.
    void measureSaturationVoltages();

    // Take spectrometer reading in automatic mode. This does not require
    // integration time. Also as a result of measurement it will set the
    // integration time measured as well as gain and ADC voltage reference
    // (the latter only in some modes). The automatic measurement is tuned to
    // use sensor setup to maximise the output ADC resolution/range.
    //
    // The automatic measurement can be tuned to achieve different results.
    // The behavior of it is controlled controlled by autoType parameter and
    // has the following modes:
    //
    //    AUTO_FOR_SET_REF   - Aims to maximise ADC reading within currently
    //                         set reference voltage. I.e. achieving maximum
    //                         resolution within selected reference voltage or
    //                         saturation limit (whichever is smaller). Only
    //                         gain and integration are changed in this mode.
    //
    //    AUTO_ALL_MIN_INTEG - Aims to maximise ADC reading across all ADC
    //                         reference voltages whilst achieving minimum
    //                         integration time. This essentially attempts
    //                         to achieve maximum resolution on smallest
    //                         reference voltage (to get shortest integration).
    //
    //    AUTO_ALL_MAX_RANGE - Aims to maximise ADC reading across all ADC
    //                         reference voltages to maximise sensor output.
    //                         This method attempts to achieve maximum use of
    //                         the sensor output range and attempts to achieve
    //                         maximum reading close to sensor saturation.
    //
    // NOTE: it is essential to set/measure sensor saturation levels for this
    //       function to work!
    //
    // NOTE2: Because it changes parameters, this mode will reset black level
    //        measurements. Therefore if black subtraction is used, the black
    //        levels need to be re-measured after this call with the same set
    //        parameters (i.e. by simply calling takeBlackMeasurement()).
    //
    void takeAutoMeasurement(auto_measure_t autoType = AUTO_FOR_SET_REF,
                             bool doExtTriggering = false);

    // Take spectrometer reading at specified time. If specified time is larger
    // than integration time take several measurement cycles at integration time
    // to fit the specified time period. If specified time is 0 then take
    // measurement at set integration time.
    void takeMeasurement(uint32_t timeUs = 0, bool doExtTriggering = false);

    // Take normal spectrometer reading at specified integration time and
    // stores it as black level. The timeUs parameter is the same as described
    // above in takeMeasurement() function.
    //
    // This allows to do manually controlled measurement of the black level
    // reference so it should be taken with spectrometer in the non-illuminated
    // conditions
    void takeBlackMeasurement(uint32_t timeUs = 0);

    // Reset black levels to 0
    void resetBlackLevels();

    // Get measured data for specified pixel (with or without black level adjustment)
    double getMeasurement(uint16_t pixelIdx, bool subtractBlack = true);

    // Get the read black value for specified pixel
    double getBlackMeasurement(uint16_t pixelIdx);

    // Get the wavelength for specified pixel
    double getWavelength(uint16_t pixel);  // get the wavelength for specified pixel

    // Attribute getters
    const double* getWavelengthCalibration() { return calibration_; }
    gain_t getGain()                         { return gain_; }
    adc_ref_t getAdcReference()              { return adcRef_; }
    measure_t getMeasurementType()           { return measurementType_; }
    float getHighGainSatVoltage()            { return satVoltageHighGain_; }
    float getNoGainSatVoltage()              { return satVoltageNoGain_; }
    bool isMeasuring()                       { return measuringData_; }
    bool isBandpassCorrected()               { return applyBandPassCorrection_; }
    int32_t getExtTrgMeasDelay()             { return extTrgMeasDelayUs_; }
    uint32_t getIntTime();             // returns currently set integration time in uSec
};

#endif
