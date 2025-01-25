/*
 *  Spectrometer.h - Hamamatsu spectrometer common driver class
 *                   for Spectron board.
 *
 *  Copyright 2015-now Alexey Danilchenko, Iliah Borg
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
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, 51 Franklin Street - Fifth Floor, Boston,
 *  MA 02110-1301, USA.
 */

#if !defined(_Spectrometer_H_)
#define _Spectrometer_H_

#include "application.h"

// Max ADC conversion value - 16 bit
#define ADC_MAX_VALUE   UINT16_MAX

// maximum pixels in supported spectrometers
#define MAX_SPEC_PIXELS 288

// No pin assigned
#ifndef NO_PIN
#define NO_PIN (TOTAL_PINS+1)
#endif

// enums
enum time_units_t {
    T_USEC   = 0,   // time in microseconds
    T_MS,           // time in milliseconds
    T_SEC,          // time in seconds

    MAX_TIME_UNITS     // last element to deduce max number
};

enum adc_ref_t {
    ADC_AUTO    = -1, // closest ADC voltage larger than saturation limit
    ADC_2_5V    = 0,  // 2.5V reference - lowest limit imposed by ADC
    ADC_3V,           // 3V reference
    ADC_4_096V,       // 4.096V reference
    ADC_5V,           // 5V reference
    ADC_MAX_VOLTAGES  // max ADC voltages
};

// Types of automatic measurement
enum auto_measure_t {
    AUTO_FOR_SET_REF_GAIN = 0,  // Maximises range for currently set ADC reference voltage and gain
    AUTO_FOR_SET_REF      = 1,  // Maximises range for currently set ADC reference voltage
    AUTO_ALL_MIN_INTEG    = 2,  // Maximises range for all ADC that achieves mimimal integration
    AUTO_ALL_MAX_RANGE    = 3   // Maximises range for sensor saturation limit
};

// Spectrometer class
//
//    This class utilises a range of spectrometer pixels from Hamamatsu sensor
//    defined by first physical sensor pixel index and number of pixels.
//    This is done to cater for the useful range - spectrometers come with a
//    sensor covering sometimes substantially broader range than specification
//    and in those extended areas signal is not very reliable and exhibits
//    substantial errors. Limiting the range allows to achieve better
//    performance and normalisation overall.
//
//    For colour science applications, ranges 380-730nm should be sufficient.
//
//    For any other applications, sensor range 340-780nm from specification
//    is a good starting point.
//
//    Ranges could be set via API call, but extending them will invalidate
//    spectral response normalisation.
//
class CSpectrometer {
private:
    // Spectrometer common pin definitions
    uint8_t adc_ref_sel1_, adc_ref_sel2_;

protected:
    // Variables
    const double*  defaultCalibration_;// Hamamatsu provided calibration constants to generate wavelenghts
    double         calibration_[6];    // Stored calibration constants to generate wavelenghts
    int            rangeStartIdx_;     // Index of the first spectrometer pixel in a spectrometer range
    int            rangePixels_;       // Total pixels in a measured spectral range
    float          *satVoltage_;       // Sensor saturation voltage (for both gains)
    double         (*linearCoeff_)[5]; // Linearisation coeff for each of the gains
    float          avgBias_;           // Average bias (stored from calibrations)
    float          *bias_;             // Calibrated pixel level bias voltage (may drift with temperature)
    float          *blackLevels_;      // Black levels voltage (without bias) captured for the last measurement
    float          *normCoef_;         // Normalisations coefficients for spectrometer per pixel
    float          *meas_;             // Last measured voltage per pixel
    double         lastMeasScale_;     // Scale for last measurement to scale for gain and 1 sec integration time
    bool           biasInvalid_;       // Whether bias needs recalculating
    bool           measuringData_;     // Is currently measuring
    uint8_t        specState_;         // ADC reference voltage, gain etc
    uint8_t        lastMeasSpecState_; // Last measurement spec state
    double         adcVoltages_[ADC_MAX_VOLTAGES]; // ADC reference voltages - calibratable

    // Spectrometer state EEPROM offset idxs
    enum eeprom_idx_t {
        EEPROM_CALIBRATION_COEF = 0,   // 6 doubles
        EEPROM_STATE,                  // 1 byte
        EEPROM_INTEGRATION_TIME,       // uint32
        EEPROM_TRG_MEAS_DELAY,         // uint32
        EEPROM_SAT_VOLTAGES,           // 1 or 2 floats
        EEPROM_AVG_BIAS,               // float
        EEPROM_GAIN_FACTOR,            // 0 or 1 float
        EEPROM_SPEC_RANGE_MIN,         // uint32
        EEPROM_SPEC_RANGE_MAX,         // uint32
        EEPROM_LINEARISATION_COEF,     // 5 or 10 doubles
        EEPROM_ADC_VOLTAGES,           // 4 doubles
        EEPROM_NORM_COEF_ARRAY,        // floats - up to number of pixels in a sensor

        EEPROM_MAX_SETTINGS
    };

    // Sensor constraints enumeration
    enum sensor_constraint_t {
        MAX_PIXELS = 0,
        MIN_WAVELENGTH,
        MAX_WAVELENGTH,
        MIN_SAT_VOLTAGE,
        MAX_SAT_VOLTAGE,
        MIN_SAT_VOLTAGE_HIGH_GAIN,
        MAX_SAT_VOLTAGE_HIGH_GAIN,
        MIN_INT_TIME_US,
        MAX_INT_TIME_US,
        MAX_BIAS_INT_TIME_US
    };

    // EEPROM base address to save/restore the spectrometer state
    const int baseEEPROM_;
    // Offsets in EEPROM for this spectrometer type
    const int *offsEEPROM_;

    // Low-level internal variables and routines
    uint32_t* rawMeas_;       // points to hardware specific raw measurement buffer
    uint16_t* rawMeasCounts_; // points to hardware specific raw measurement count buffer

    // ------------------------------------------------------------------------
    //  Virtual methods - to be defined in hardware specific implementations
    // ------------------------------------------------------------------------

    // This is the main method that should be overriden by specific spectrometer
    // implementation. It should implement hardware specific initialisation, reading
    // loop and termination.
    virtual void readSpectrometer(double readTimeUs,
                                  bool   doExtTriggering,
                                  bool   doLightTriggering) = 0;

    // Hardware specific sensor constraints - sets data variable for a specific sensor
    // constraint (pointer is expected to point to a variable of correct datatype for
    // that constraint)
    virtual bool getSensorConstraint(sensor_constraint_t sensConstraint, void* data) = 0;

    // Hardware specific sets integration time in microseconds for a single
    // measurement cycle.
    //
    // NOTE: if not within sensor allowed boundaries then it will be set to
    //       closest minimum or maximum allowed value
    // NOTE: internally time is measured in clock cycles so real integration
    //       time will be aligned to the clock cycle boundaries
    virtual void setIntTimeInternal(double timeUs, bool saveState = false) = 0;

    // Hardware specific sets the external trigger to measurement delay time.
    // This defines time interval in uSec that offsets external trigger from
    // the measurement. I.e. external trigger is raised and after this delay
    // the integration and measurement starts.
    //
    // Specifying -1 as  delay will disable the external trigger
    virtual void setExtTrgMeasDelayInternal(double delayUs,  bool saveState = false) = 0;

    // Progress callback pointer for long running measurements
    using TProgressFunc = void (*)(int,bool);

    // Low-level internal variables and routines
    void setAdcRefInternal(int adcRef);
    void setSensorRangeInternal(int& minWavelength, int& maxWavelength);
    void getSensorRangeInternal(int& minWavelength, int &maxWavelength);
    float getRawMeasValue(uint16_t pixelIdx, bool removeBias = false);
    float processMeasurement(float* measurement, bool avgPrevious = false, float* stddev = 0);
    bool setWavelengthCalibrationInternal(const double* wavelengthCal);
    float findSaturatedLimit(float minV, float maxV, TProgressFunc progress = 0);
    void setMeasScale();
    double getLinearisedMeas(uint16_t pixelIdx, float* measurement, bool highGain);

public:
    // Constructor/destructor
    // Parameters:
    //     adc_ref_sel1       - ADC voltage reference selection pin 1
    //     adc_ref_sel2       - ADC voltage reference selection pin 2
    //     defaultCalibration - factory wavelength calibration factors (array of 6
    //                          doubles from Hamamatsu test sheet)
    //     baseEEPROM     - base address to srore the spectrometer settings in EEPROM
    //                      (-1 if not used)
    CSpectrometer(uint8_t adc_ref_sel1, uint8_t adc_ref_sel2,
                  const double *defaultCalibration, const int baseEEPROM = 0);
    virtual ~CSpectrometer();

    // ------------------------------------------------------------------------
    //  Virtual methods - to be defined in hardware specific implementations
    // ------------------------------------------------------------------------

    // Whether spectrometers supports gain and high gain setting
    // By default gain is not supported
    virtual bool supportsGain() { return false; }
    virtual void setHighGain(bool highGain, bool saveState = false) {}

    // Hardware specific - calibrates/measures gain factor or sets it directly.
    // If supplied gainFactor < 0 then perform automatic measurement, else set
    // the gain factor explicitly
    virtual bool calibrateGain(double gainFactor = -1, TProgressFunc progress = 0)
        { return false; }

    // Virtual getters

    // Return whether high gain is enabled for given spectrometer state if spectrometer
    // supports gain. By default gain is not supported
    virtual bool getHighGain(bool lastMeas = false) { return false; }

    // Return gain factor for the current gain setting
    // No gain factor by default
    virtual float getGainFactor() { return 1.0; }

    // Return trigger measurement delay in specified units
    virtual double getExtTrgMeasDelay(time_units_t units = T_USEC) = 0;

    // Return integration time in specified units
    virtual double getIntTime(time_units_t units = T_USEC) = 0;

    // ------------------------------------------------------------------------
    //    Common functions for all implementations
    // ------------------------------------------------------------------------

    // Initialise spectrometer state from EEPROM if available and known pins.
    // Specific hardware implementations typically will have their own implementations
    // of this initialising spectrometer specific pins but calling this is a must in
    // all inherited classes.
    bool begin();

    // Reset all stored values to default
    void resetToDefaults();

    // Sets the spectrometer sensor range in nanometers. If the range is
    // wider than current one, then this will reset spectral response
    // normalisation.
    //
    // Specifying either value as 0 will not update that value.
    //
    // Specifying either value as -1 will reset to the spectrometer physical
    // limit for that value.
    //
    // Generally, the spectral range should be chosen at the beginning for
    // the specific application, sensor calibrated with it and then left alone.
    void setSensorRange(int minWavelength, int maxWavelength);

    // Sets the wavelength calibration coeffiecients. This is array of 6
    // values that represent polinomial coefficients. Usually provided
    // by Hamamatsu but can be overriden by user calculated ones.
    //
    // Setting this will reset spectral response normalisation.
    bool setWavelengthCalibration(const double* wavelengthCal);

    // Set ADC reference voltage to one of the specified values. Defines maximum
    // analogue signal voltage for ADC conversion. This generally should be used
    // together with setting the spectrometer gain (high gain will mean higher
    // reference voltage) but it is decoupled for flexibility of ADC control.
    void setAdcReference(adc_ref_t ref, bool saveState = false);

    // Set integration time in chosen units for a single measurement cycle.
    // The real measurement time can be more that this period in which case
    // several measurement cycles will be taken sequentially and averaged out.
    //
    // NOTE: if not within sensor allowed boundaries then it will be set to
    //       closest minimum or maximum allowed value
    void setIntTime(double time, time_units_t units = T_USEC, bool saveState = false);

    // Sets the external trigger to measurement delay time. This defines time
    // interval in specified units that offsets external trigger from
    // the measurement. I.e. external trigger is raised and after this delay
    // the integration and measurement starts.
    //
    // Specifying -1 as  delay will disable the external trigger
    void setExtTrgMeasDelay(double delayTime, time_units_t units = T_USEC, bool saveState = false);

    // Sets the saturation voltage for current gain. These are used to in auto
    // integration mode of measurement. Passing values outside of ranges from
    // Hamamatsu spec will reset approprite value to a default ones.
    void setSaturationVoltage(float satVoltage, bool highGain);
    void setSaturationVoltage(float satVoltage)
        { setSaturationVoltage(satVoltage, getHighGain()); }

    // Automatic measurement of the saturation voltage(s). These are used in
    // auto integration mode of measurement.
    //
    // Automatic setting works by exposing sensor to bright light and then
    // calling this method to work out saturation voltages.
    //
    // If progress callback is supplied it will be called during calibration runs
    // periodically to refresh progression of the calibration.
    bool measureSaturation(TProgressFunc progress = 0);

    // Measurement of the bias.
    //
    // Measurement works by reading a series of dark exposure of short integrations
    // series and interpolating at each point with rational polynomial by integration
    // time. Bias is taken as an interpolated value at integration time of 0.
    //
    // Each calibration updates the the average bias that is used after startup
    // before first calibration run. If avgMinOnly is set then average bias is
    // only updated if new average is less than previously recorded one, othwerise
    // it is updated regardless.
    //
    // If progress callback is supplied it will be called during calibration runs
    // periodically to refresh progression of the calibration.
    bool measureBias(bool avgMinOnly = true, TProgressFunc progress = 0);

    // Linearise sensor response.
    //
    // Linearisation is based on the MDPI article by M.Nehir, C.Frank, S.Asmann and
    // E.P.Achterberg "Improving Optical Measurements: Non-Linearity Compensation of
    // Compact Charge-Coupled Device (CCD) Spectrometers" available here
    //     https://www.mdpi.com/1424-8220/19/12/2833/pdf
    // and a few other sources.
    //
    // General approach is as follows (sensor saturation should be already calibrated):
    //     1) measure/calibrate bias prior to linearisation (dark sensor)
    //     2) use high stability constant current LED source to light spectrometer via
    //        integrating sphere or diffusing or ND filter such that standard deviation
    //        at short integration time is between 30 and 100 ADC units
    //     3) at shortest integration time with useful signal as above select pixel with
    //        peak value and use it for linearsation (this prevents effects of cross talk
    //        or blooming)
    //     4) determine longest integration time such that this selected pixel reads near
    //        its saturation limit
    //     3) obtain a series of measurements for the selectes pixel intensity between
    //        shortest and longest integration times equally spaced and take away bias
    //     4) find linear gradient k for exposure represented as meas=k*time by
    //        linear regression and calculate expected values of measurements
    //        against measured values
    //     5) calculate linearisation 5th degree polynomial correcting response of the
    //        above to the calculated expected measured values
    //
    // If progress callback is supplied it will be called during calibration runs
    // periodically to refresh progression of the calibration.
    //
    bool linearise(TProgressFunc progress = 0);

    // Sets linearisation polynomial coefficients explicitly for given high gain.
    // If linearisation array is null then resets current linearisation to unity.
    bool setLinearisation(const double* linearisation, bool highGain);
    bool setLinearisation(const double* linearisation)
        { return setLinearisation(linearisation, getHighGain()); }

    // This method calibrates sensor relative spectral response.
    //
    // It should only be invoked after calibrating saturation, bias and linerisation.
    //
    // It expects the sensor to be exposed to stabilised tungsten light source
    // of the specified temperature, with black levels captured, measures
    // sensor response for selected parameters, calculates expected theoretical
    // response (relative against largest wavelength) for Planckian blackbody
    // corrected for tungsten source, and then calculates corrections for
    // measured sensor response (with blacks subtracted).
    //
    // Normalisation and calibration is done relative to the specified wavelength
    // or measured maximim if 0 was specified.
    //
    // Procedure for calibration:
    // 1) run the tungsten light source on stabilised power supply for at
    //    least 20 mins, measuring its temperature (using voltage/current
    //    measurement, lamp resistance against lamp resistance at room
    //    temperature) - see O. Harang, M. J. Kosch "Absolute Optical
    //    Calibrations Using a Simple Tungsten Bulb:Theory" for details
    // 2) do automatic measurement to capture spectrometer measurement and
    //    parameters at minumal ADC voltage and high gain for higher
    //    dynamic range
    // 3) capture black levels with exposure parameters established by (2)
    // 4) call this method to do the calibration specifying calculated lamp
    //    temperature at (1) and potentially new measurement (or use the one
    //    done at (2) - default)
    //
    // NOTE: This call can invalidate current measurement results
    //
    void calibrateSpectralResponse(float lampTempK,
                                   bool useCurrentMeasurement = true,
                                   uint32_t normWvlNm = 0);

    // This method calibrates ADC voltages.
    //
    // This will attempt to set more precise ADC voltages using the given ADC
    // reference as a base. The other voltages will be calculated relative to
    // the given one. This calibration needs a stable LED light directed at
    // spectrometer.
    //
    // It should be calibrated for accurate bias and linearisation but after
    // saturation calibration. Since linearisation is performed for the closest
    // larger than saturation voltage, it is generally a good idea to use that
    // ADC voltage as a base.
    //
    void calibrateADCVoltages(adc_ref_t baseADC);

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
    //    AUTO_FOR_SET_REF_GAIN - Aims to maximise ADC reading within currently
    //                            set reference voltage and gain. I.e. achieving
    //                            maximum resolution within selected gain and
    //                            reference voltage or saturation limit (whichever
    //                            is smaller). Only integration is changed in this
    //                            mode.
    //
    //    AUTO_FOR_SET_REF      - Aims to maximise ADC reading within currently
    //                            set reference voltage. I.e. achieving maximum
    //                            resolution within selected reference voltage or
    //                            saturation limit (whichever is smaller). Only
    //                            gain and integration are changed in this mode.
    //
    //    AUTO_ALL_MIN_INTEG    - Aims to maximise ADC reading across all ADC
    //                            reference voltages whilst achieving minimum
    //                            integration time. This essentially attempts
    //                            to achieve maximum resolution on smallest
    //                            reference voltage (to get shortest integration).
    //
    //    AUTO_ALL_MAX_RANGE    - Aims to maximise ADC reading across all ADC
    //                            reference voltages to maximise sensor output.
    //                            This method attempts to achieve maximum use of
    //                            the sensor output range and attempts to achieve
    //                            maximum reading close to sensor saturation.
    //
    // NOTE: it is essential to set/measure sensor saturation levels for this
    //       function to work!
    //
    // NOTE2: Because it changes exposure time, this mode will reset black level
    //        measurements to minimum calibrated by default. This can be omitted
    //        if specified. It generally is a good idea to recapture black levels
    //        with established exposure parameters after this call to make
    //        measurement more precise.
    //
    void takeAutoMeasurement(auto_measure_t autoType = AUTO_FOR_SET_REF,
                             double measTime = -1,
                             time_units_t units = T_USEC,
                             bool doBlackReset = true,
                             bool doExtTriggering = false,
                             bool saveState = false);

    // Take spectrometer reading for time specified (in specified units). If supplied
    // time is larger than integration time then take several measurement cycles at
    // integration time to fit the specified time period. If specified time is 0 then
    // take measurement at set integration time.
    void takeMeasurement(double time = 0,
                         time_units_t units = T_USEC,
                         bool doExtTriggering = false);

    // Take normal spectrometer reading at specified integration time and
    // stores it as black level. The time and units parameters are the same as
    // described above in takeMeasurement() function.
    //
    // This allows to do manually controlled measurement of the black level
    // reference so it should be taken with spectrometer in the non-illuminated
    // conditions.
    //
    // If avgPrevious is set, then average the measurement with ethe previous one
    // Averaging like that only makes sense at the same gain and integration time.
    void takeBlackMeasurement(double time = 0,
                              time_units_t units = T_USEC,
                              bool doExtTriggering = false,
                              bool avgPrevious = false);

    // Reset black levels to given level. If negative value is specified the
    // levels are reset to calibrated minimum black (default behavior).
    void resetBlackLevels(float resetVoltage = -1.0);

    // Get measured data for specified pixel (normalised or as is)
    double getMeasurement(uint16_t pixelIdx,
                          bool rawMeas = false,
                          bool applyBandpassCorrection = true);

    // Get the read black voltage for specified pixel
    float getBlackLevel(uint16_t pixelIdx) { return blackLevels_[pixelIdx]<bias_[pixelIdx]
                                                    ? 0.0
                                                    : blackLevels_[pixelIdx]-bias_[pixelIdx]; }

    // Get the read bias voltage for specified pixel (dark voltage at zero integration time)
    float getBiasVoltage(uint16_t pixelIdx) { return bias_[pixelIdx]; }

    // Get the normalisation coefficients
    float getNormalisationCoef(uint16_t pixelIdx) { return normCoef_[pixelIdx]; }

    // Get the wavelength for specified pixel in nanometers
    double getWavelength(uint16_t pixel);

    // Get ADC reference
    adc_ref_t getAdcReference(bool lastMeas = false);
    adc_ref_t getAdcReference(double voltage);
    double getAdcRefVoltage(bool lastMeas = false);
    double getAdcRefVoltage(adc_ref_t ref);

    // Attribute getters
    const double* getWavelengthCalibration()    { return calibration_; }
    const double* getLinearCoefs()              { return linearCoeff_[getHighGain()]; }
    const double* getLinearCoefs(bool highGain) { return linearCoeff_[highGain]; }
    int getTotalPixels()                        { return rangePixels_; }
    int getStartPixelIdx()                      { return rangeStartIdx_; }
    float getSatVoltage()                       { return satVoltage_[getHighGain()]; }
    float getSatVoltage(bool highGain)          { return satVoltage_[highGain]; }
    float getAvgBias()                          { return avgBias_; }
    bool biasInvalid()                          { return biasInvalid_; }
    bool isMeasuring()                          { return measuringData_; }
};

#endif
