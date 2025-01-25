/*
 *  SpectronFirmware.cpp - Spectron board firmware functions main common file.
 *                         This carries out common firmware functions for
 *                         all supported spectrometer types and is expected
 *                         to be included in the main firmware file.
 *
 *  Copyright 2017-2021 Alexey Danilchenko, Iliah Borg
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

#include "SpectronFirmware.h"

STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

// Encoded string result size
#define ENC_RESULT_STR_SIZE (((MAX_SPEC_PIXELS)*sizeof(float)*4/3)+16)

#define BIAS_INVALID_MASK   0x1000

// Board type identifier
static String BOARD_TYPE = "SPEC2_SPECTROMETER";

// Particle exposed variables
retained int    specPixels = MAX_SPEC_PIXELS;
retained int    specOffsetIdx = 0;
retained char   specCalibrationStr[128];
retained char   specSatVoltageStr[48];
retained char   specLinearisationStr[172];
retained double specAdcRefVoltage = 0.0;
retained int    specHighGain = false;
retained double specGainFactor = 0.0;
retained double specAvgBias = 0.0;
retained double specIntegTimeUs = 0.0;
retained double specExtTrigDelayUs = 0.0;
retained char   specEncData[ENC_RESULT_STR_SIZE]; // Base64 encoded floats

// Delayed tasks types
enum delayed_task_t {
    DT_NONE = 0, // no task is running
    DT_BIAS,
    DT_MEASURE,
    DT_MEASURE_BLACK,
    DT_MEASURE_AUTO
};

// Holds the currently running delayed task ID
struct delayed_task_data_t {
    delayed_task_t taskID;
    int            taskState;
    union {
        bool avgMinOnly;       // bias task
        struct {               // measuring task
            bool avgBlack;
            bool doExtTrg;
            double measureTimeUs;
            auto_measure_t autoType;
            bool saveState;
        };
    };
} delayedTask;

// maximum size for string variable data in Particle
const int maxVarSize = 620;

// re-entry prevention
static bool measuring = false;

// spectrometer object
static CSpectrometer* spec = 0;

// Data type for the data we need to get from spectrometer
enum spec_data_t {
    ET_MEASUREMENT     = 0,
    ET_RAW_MEASUREMENT = 1,
    ET_BIAS            = 2,
    ET_BLACK           = 3,
    ET_NORMALISATION   = 4
};

// Auxialiary helper methods

// Build up saturation voltage string
void buildSpecSatVoltageStr()
{
    if (spec->supportsGain())
        String::format("%.10G,%.10G",
                       spec->getSatVoltage(false),
                       spec->getSatVoltage(true)).toCharArray(specSatVoltageStr,
                                                              sizeof(specSatVoltageStr));
    else
        String::format("%.10G", spec->getSatVoltage()).toCharArray(specSatVoltageStr,
                                                                   sizeof(specSatVoltageStr));
}

// Build up linearisation string
void buildSpecLinearisationStr()
{
    if (spec->supportsGain())
    {
        const double* linCoefs   = spec->getLinearCoefs(false);
        const double* linCoefsHg = spec->getLinearCoefs(true);
        String::format("%.10G,%.10G,%.10G,%.10G,%.10G,%.10G,%.10G,%.10G,%.10G,%.10G",
                       linCoefs[0],
                       linCoefs[1],
                       linCoefs[2],
                       linCoefs[3],
                       linCoefs[4],
                       linCoefsHg[0],
                       linCoefsHg[1],
                       linCoefsHg[2],
                       linCoefsHg[3],
                       linCoefsHg[4]).toCharArray(specLinearisationStr,
                                                  sizeof(specLinearisationStr));
    }
    else
    {
        const double* linearCoefs = spec->getLinearCoefs();
        String::format("%.10G,%.10G,%.10G,%.10G,%.10G",
                       linearCoefs[0],
                       linearCoefs[1],
                       linearCoefs[2],
                       linearCoefs[3],
                       linearCoefs[4]).toCharArray(specLinearisationStr,
                                                   sizeof(specLinearisationStr));
    }
}

// Build up wavelength string
void buildSpecCalibrationStr()
{
    const double* calibration = spec->getWavelengthCalibration();
    String::format("%.10G,%.10G,%.10G,%.10G,%.10G,%.10G",
                   calibration[0],
                   calibration[1],
                   calibration[2],
                   calibration[3],
                   calibration[4],
                   calibration[5]).toCharArray(specCalibrationStr,
                                               sizeof(specCalibrationStr));
}

// Encode measurement result in Base64
void encodeMeasurement(spec_data_t encodeType)
{
    static const char* encB64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+-";

    // transfer measurement as Base64 data to series of string variables
    int bufSize = sizeof(specEncData);
    memset(specEncData, 0, bufSize);
    char* encData = specEncData;
    int charCount = 0;

    float floatVal = 0;
    uint8_t *data = (uint8_t*)&floatVal;
    int inCnt = 0, outCnt = 0;
    int value = 0, bits = -6;

    while (inCnt>>2 < spec->getTotalPixels())
    {
        // read the next float
        if ((inCnt&3) == 0)
            switch (encodeType)
            {
                case ET_MEASUREMENT:
                    floatVal = spec->getMeasurement(inCnt>>2, false);
                    break;
                case ET_RAW_MEASUREMENT:
                    floatVal = spec->getMeasurement(inCnt>>2, true, false);
                    break;
                case ET_BIAS:
                    floatVal = spec->getBiasVoltage(inCnt>>2);
                    break;
                case ET_BLACK:
                    floatVal = spec->getBlackLevel(inCnt>>2);
                    break;
                case ET_NORMALISATION:
                    floatVal = spec->getNormalisationCoef(inCnt>>2);
                    break;
            }

        value = (value<<8) + data[inCnt&3];
        ++inCnt;
        bits += 8;

        while (bits >= 0)
        {
            // advance string to next Particle variable if we reached
            // maximum for the current one
            if (charCount >= maxVarSize)
            {
                ++encData;
                charCount = 0;
                --bufSize;
            }

            *encData++ = encB64[(value>>bits)&0x3F];
            ++charCount;
            --bufSize;
            bits -= 6;
        }
    }

    if (bits > -6)
        *encData++ = encB64[((value<<8)>>(bits+8))&0x3F];

    while (outCnt & 3)
        *encData++ = '=';
}


// Cloud functions

// Gets the requested pixel array data into spLastMeasN variables. Format of
// the parameter string:
//    MEAS_RAW          - results of the measurement with no corrections applied
//    MEASUREMENT       - results of the measurement, linearised, black
//                        subtracted and normalised
//    BIAS              - calibrated bias voltages captured
//    BLACK_LEVELS      - black level voltages captured
//    NORMALISATION     - spectral response normalisation coefficients
//
int specGetData(String paramStr)
{
    if (!spec || measuring || spec->isMeasuring() || delayedTask.taskID != DT_NONE)
        return -1;

    // set measurement mode - preventing reentry
    measuring = true;

    // get data type
    paramStr.trim().toUpperCase();
    spec_data_t encType = ET_MEASUREMENT;
    if (paramStr == "BLACK_LEVELS")
        encType = ET_BLACK;
    else if (paramStr == "BIAS")
        encType = ET_BIAS;
    else if (paramStr == "NORMALISATION")
        encType = ET_NORMALISATION;
    else if (paramStr == "MEAS_RAW")
        encType = ET_RAW_MEASUREMENT;
    else if (paramStr != "MEASUREMENT")
        return -1;

    // encode data
    encodeMeasurement(encType);

    // reset measurement mode
    measuring = false;

    return 0;
}

// Resets all internal parameters to their defaults
int specResetToDefaults(String paramStr)
{
    if (!spec || spec->isMeasuring() || delayedTask.taskID != DT_NONE)
        return -1;

    spec->resetToDefaults();

    // initialise Particle variables
    specAdcRefVoltage   = spec->getAdcRefVoltage();
    specHighGain        = spec->getHighGain();
    specIntegTimeUs     = spec->getIntTime(T_USEC);
    specExtTrigDelayUs  = spec->getExtTrgMeasDelay(T_USEC);
    specPixels          = spec->getTotalPixels();
    specOffsetIdx       = spec->getStartPixelIdx();

    // build up strings
    buildSpecSatVoltageStr();
    buildSpecLinearisationStr();
    buildSpecCalibrationStr();

    return 0;
}

// Set the spectral sensor range. Limiting sensor range to the specification
// or narrower is generally useful - it leads to better calibration and more
// precise measurements. Format of the parameter string:
//
//    [<min>],[<max>] - The lower and upper wavelength bounds of the
//                      spectrometer range in nanometers. If skipped or 0
//                      that bound will not be amended.
//    DEFAULT         - Sets to sensor default range (according to specification)
//    MAX             - Sets to sensor maximum range (all available pixels)
//
int specSetRange(String paramStr)
{
    if (!spec || spec->isMeasuring() || paramStr.length() == 0 || delayedTask.taskID != DT_NONE)
        return -1;

    // all uppercase
    paramStr.trim().toUpperCase();

    // parse the string
    if (paramStr.equals("DEFAULT"))
        spec->setSensorRange(-1, -1);
    else if (paramStr.equals("MAX"))
        spec->setSensorRange(1, 20000);  // use very large range
    else
    {
        // range was explicitly supplied
        int minWavelength = paramStr.toInt();
        int maxWavelength = 0;

        int sepIdx = paramStr.indexOf(',');
        if (sepIdx > 0)
            maxWavelength = paramStr.substring(sepIdx+1).trim().toInt();
        spec->setSensorRange(minWavelength, maxWavelength);
    }

    // update Particle variables
    specPixels    = spec->getTotalPixels();
    specOffsetIdx = spec->getStartPixelIdx();

    return 0;
}

// Sets the integration time. Format of the parameter string:
//    <delay>[,SAVE] - sets the intgration time in usec
//
// If an optional SAVE is specified, the setting is persisted in sensor EEPROM
//
int specSetIntegrationTime(String paramStr)
{
    if (!spec || spec->isMeasuring() || delayedTask.taskID != DT_NONE)
        return -1;

    // all uppercase
    paramStr.trim().toUpperCase();

    // parse the string
    bool saveState = paramStr.endsWith(",SAVE");
    if (saveState)
    {
        paramStr.remove(paramStr.length()-5);
        paramStr.trim();
    }

    double intTimeUs = atof(paramStr.c_str());

    // use specified time if it is there
    if (intTimeUs <= 0.0)
        return -1;

    spec->setIntTime(intTimeUs, T_USEC, saveState);

    // update Particle variable
    specIntegTimeUs = spec->getIntTime(T_USEC);

    return 0;
}

// Sets external trigger to measurement delay or switches off
// external triggering altogether. Format of the parameter string:
//    OFF[,SAVE]     - disable external triggering
//    <delay>[,SAVE] - sets the delay in usec
//
// If an optional SAVE is specified, the setting is persisted in sensor EEPROM
//
int specSetTriggerMeasurementDelay(String paramStr)
{
    if (!spec || spec->isMeasuring() || paramStr.length() == 0 || delayedTask.taskID != DT_NONE)
        return -1;

    // all uppercase
    paramStr.trim().toUpperCase();

    // parse the string
    bool saveState = paramStr.endsWith(",SAVE");
    if (saveState)
    {
        paramStr.remove(paramStr.length()-5);
        paramStr.trim();
    }

    double trgMeasDelay = 0;
    if (paramStr.equals("OFF"))
        trgMeasDelay = -1;
    else
        trgMeasDelay = paramStr.toFloat();

    spec->setExtTrgMeasDelay(trgMeasDelay, T_USEC, saveState);

    // update Particle variable
    specExtTrigDelayUs = spec->getExtTrgMeasDelay();

    return 0;
}

// Sets the spectrometer state - ADC voltage or gain. Format of the parameter string:
//    HIGH_GAIN[,SAVE]  - if supported enables high gain
//    NO_GAIN[,SAVE]    - if supported disables high gain
//    ADC,2.5V[,SAVE]   - sets ADC voltage to 2.5V
//    ADC,3V[,SAVE]     - sets ADC voltage to 3V
//    ADC,4.096V[,SAVE] - sets ADC voltage to 4.096V
//    ADC,5V[,SAVE]     - sets ADC voltage to 5V
//    ADC,AUTO[,SAVE]   - sets closest ADC voltage larger than saturation limit
//                        for current gain
//
// If an optional SAVE is specified, the setting is persisted in sensor EEPROM
//
int specSetState(String paramStr)
{
    if (!spec || spec->isMeasuring() || delayedTask.taskID != DT_NONE)
        return -1;

    paramStr.trim().toUpperCase();

    bool saveState = paramStr.endsWith(",SAVE");
    if (saveState)
    {
        paramStr.remove(paramStr.length()-5);
        paramStr.trim();
    }

    if (paramStr.startsWith("ADC"))
    {
        adc_ref_t adcRef = ADC_5V;
        if (paramStr == "ADC,AUTO")
            adcRef = ADC_AUTO;
        else if (paramStr == "ADC,2.5V")
            adcRef = ADC_2_5V;
        else if (paramStr == "ADC,3V")
            adcRef = ADC_3V;
        else if (paramStr == "ADC,4.096V")
            adcRef = ADC_4_096V;

        spec->setAdcReference(adcRef, saveState);

        // update Particle variable
        specAdcRefVoltage = d[spec->getAdcReference()];
    }
    else if (spec->supportsGain() && paramStr.endsWith("GAIN"))
    {
        spec->setHighGain(paramStr == "HIGH_GAIN", saveState);

        // update Particle variable
        specHighGain = spec->getHighGain();
    }

    return 0;
}

// Run spectral measurement (black or normal). Format of the parameter string:
//    BLACK,RESET                               - reset black levels
//    BLACK,<time>[,AVG,TRG]                    - measure black levels for specified time
//                                                in usec
//    <time>[,TRG]                              - measurement time in usec followed by
//                                                optional external triggering (if
//                                                specified)
//    AUTO_FOR_SET_REF_GAIN[,<time>][,TRG,SAVE] - Automatic measurement with current ADC
//                                                voltage and gain and optional
//                                                triggering.
//    AUTO_FOR_SET_REF[,<time>][,TRG,SAVE]      - Automatic measurement with current ADC
//                                                voltage and optional triggering
//    AUTO_ALL_MIN_INTEG[,<time>][,TRG,SAVE]    - Automatic measurement for min
//                                                integration and optional triggering
//    AUTO_ALL_MAX_RANGE[,<time>][,TRG,SAVE]    - Automatic measurement for max range and
//                                                optional triggering
//
// If an optional SAVE is specified, the settings established during AUTO modes are
// persisted in sensor EEPROM.
//
int specMeasure(String paramStr)
{
    if (!spec || measuring || spec->isMeasuring() || delayedTask.taskID != DT_NONE)
        return -1;

    int result = 0;

    // set measurement mode - preventing reentry
    measuring = true;

    // get reading time
    paramStr.trim().toUpperCase();

    delayedTask.saveState = paramStr.endsWith(",SAVE");
    if (delayedTask.saveState)
    {
        paramStr.remove(paramStr.length()-5);
        paramStr.trim();
    }
    bool doExtTrg = paramStr.endsWith(",TRG");
    if (doExtTrg)
    {
        paramStr.remove(paramStr.length()-4);
        paramStr.trim();
    }

    if (paramStr.startsWith("BLACK,"))
    {
        if (paramStr.endsWith(",RESET"))
            spec->resetBlackLevels();
        else
        {
            double measTimeUs = atof(paramStr.substring(6).c_str());
            if (measTimeUs < 0)
                return -1;

            bool doAvg = paramStr.endsWith(",AVG");

            if (measTimeUs > 12000000.0)     // anything >12s is delayed
            {
                delayedTask.taskID        = DT_MEASURE_BLACK;
                delayedTask.taskState     = 1;
                delayedTask.measureTimeUs = measTimeUs;
                delayedTask.doExtTrg      = doExtTrg;
                delayedTask.avgBlack      = doAvg;
                result = delayedTask.taskID;
            }
            else
            {
                spec->takeBlackMeasurement(measTimeUs, T_USEC, doExtTrg, doAvg);
                if (spec->biasInvalid())
                    result = BIAS_INVALID_MASK;
            }
        }
    }
    else
    {
        // do the measurement
        if (paramStr.startsWith("AUTO"))
        {
            delayedTask.taskID        = DT_MEASURE_AUTO;
            delayedTask.taskState     = 1;
            delayedTask.doExtTrg      = doExtTrg;
            delayedTask.measureTimeUs = -1.0;
            delayedTask.autoType      = AUTO_FOR_SET_REF;
            result = delayedTask.taskID;

            // parse parameters
            if (paramStr.startsWith("AUTO_ALL_MIN_INTEG"))
                delayedTask.autoType = AUTO_ALL_MIN_INTEG;
            else if (paramStr.startsWith("AUTO_ALL_MAX_RANGE"))
                delayedTask.autoType = AUTO_ALL_MAX_RANGE;
            else if (paramStr.startsWith("AUTO_FOR_SET_REF_GAIN"))
                delayedTask.autoType = AUTO_FOR_SET_REF_GAIN;

            int idx = paramStr.indexOf(',');
            if (idx > 0)
                delayedTask.measureTimeUs = atof(paramStr.substring(idx+1).c_str());
        }
        else
        {
            double measTimeUs = atof(paramStr.c_str());
            if (measTimeUs < 0)
            {
                measuring = false;
                return -1;
            }

            // take measurement
            if (measTimeUs > 12000000.0)     // anything >12s is delayed
            {
                delayedTask.taskID        = DT_MEASURE;
                delayedTask.taskState     = 1;
                delayedTask.measureTimeUs = measTimeUs;
                delayedTask.doExtTrg      = doExtTrg;
                result = delayedTask.taskID;
            }
            else
            {
                spec->takeMeasurement(measTimeUs, T_USEC, doExtTrg);
                if (spec->biasInvalid())
                    result = BIAS_INVALID_MASK;
            }
        }

        // transfer measurement as Base64 data to series of string variables
        if (delayedTask.taskID == DT_NONE)
            encodeMeasurement(ET_MEASUREMENT);
    }

    // reset measurement mode
    measuring = false;

    return result;
}

// Calibrate various parameters of spectrometer. Generic format of the parameter string:
//
//    <Calibration Type>,[ calibration specific parameters]
//
// The following calibration types are supported:
//
//    WAVELENGTH - sets wavelength calibration coefficients.
//
//              WAVELENGTH calibration specific parameters format:
//                <A0>,<A1>,<A2>,<A3>,<A4>,<A5> - coefficients values
//
//    RESPONSE - calibrates spectral response with the following procedure:
//               1) run the tungsten light source on stabilised power supply for at
//                  least 20 mins, measuring its temperature (using voltage/current
//                  measurement, lamp resistance against lamp resistance at room
//                  temperature) - see O. Harang, M. J. Kosch "Absolute Optical
//                  Calibrations Using a Simple Tungsten Bulb:Theory" for details
//               2) do automatic measurement to capture spectrometer measurement and
//                  parameters at minumal ADC voltage and high gain for higher
//                  dynamic range
//               3) capture black levels with exposure parameters established by (2)
//               4) run normal measurement with exposure parameters established by (2)
//                  running measurement for 5-10 seconds (averaging out)
//               5) call this method to do the calibration specifying calculated lamp
//                  temperature at (1) and use the measurement done at (4) - default)
//
//              RESPONSE calibration specific parameters format:
//                  <TempK>[,MEASURE]  - TempK - lamp temperature in K,
//                                       second parameter only specified to perform
//                                       measuring instead of using already existing one
//                  RESET              - reset calibration to unity
//
//    SATURATION - sets sensor saturation (calibrated externally).
//
//              SATURATION calibration specific parameters format:
//                 <sat.voltage>[,<high gain sat.voltage>]
//                      - explicit saturation voltage (and high gain saturation if
//                        sensor supports gain)
//
//    BIAS       - measures sensor bias. This needs to run before any reasonable
//                 spectometer usage and after it attained working temperature. The
//                 calibration works by performing a series of dark reading of varying
//                 integration times, interpolating each point response across
//                 integration time domain by rational function and taking interpolated
//                 value at zero integration time. Sensor needs to be dark for this
//                 calibration.
//
//              BIAS calibration specific parameters format:
//                  MIN_AVG - store average bias minimum only (i.e. only if average
//                            less than stored one).
//
//    LINEARISATION - sets sensor linearisation curves (calibrated externally).
//
//              LINEARISATION calibration specific parameters format:
//                  RESET - reset calibration to unity
//                  <A0>,<A1>,<A2>,<A3>,<A4>[,<A0>,<A1>,<A2>,<A3>,<A4>]
//                      - linearisation coefficients for no gain, followed by the same
//                        coefficients for high gain if spectrometer supports gain
//
//    GAIN       - sets sensor gain factor calibrated externally (if gain is supported
//                 by the hardware).
//
//              GAIN calibration specific parameters format:
//                  RESET   - reset gain factor to unity
//                  <value> - set gain factor to supplied value explicitly if > 1.0
//
int specCalibrate(String paramStr)
{
    if (!spec || measuring || spec->isMeasuring() ||
        paramStr.length() == 0 || delayedTask.taskID  != DT_NONE)
        return -1;

    // all uppercase
    int result = 0;
    paramStr.trim().toUpperCase();

    // set measurement mode - preventing reentry
    measuring = true;

    // parse the calibration type
    if (paramStr.startsWith("WAVELENGTH,"))
    {
        if (paramStr.endsWith(",RESET"))
            spec->setWavelengthCalibration(0);
        else
        {
            double calibration[6] = { 0.0,0.0,0.0,0.0,0.0,0.0 };

            // parse the calibration coefficient string
            int sepIdx = 11;
            for (int i=0; i<6; ++i)
            {
                calibration[i] = atof(paramStr.substring(sepIdx).c_str());
                if (calibration[i] == 0.0)
                {
                    result = -1;
                    break;
                }

                sepIdx = paramStr.indexOf(',', sepIdx+1);
                if (sepIdx<=0 && i!=5)
                {
                    result = -1;
                    break;
                }
                ++sepIdx;
            }

            // save calibration
            result = spec->setWavelengthCalibration(calibration) - 1;
        }

        // update Particle variables
        if (result >= 0)
        {
            specPixels    = spec->getTotalPixels();
            specOffsetIdx = spec->getStartPixelIdx();
            buildSpecCalibrationStr();
        }
    }
    else if (paramStr.startsWith("RESPONSE,"))
    {
        // Spectral response calibration
        if (paramStr.endsWith(",RESET"))
            spec->calibrateSpectralResponse(0.0);
        else
        {
            bool useCurrentMeasurement = !paramStr.endsWith(",MEASURE");
            float tempK = paramStr.substring(9).toFloat();
            spec->calibrateSpectralResponse(tempK, useCurrentMeasurement);
        }
    }
    else if (paramStr.startsWith("SATURATION,"))
    {
        // explicit setting of the saturation voltages
        double satVoltage = atof(paramStr.substring(11).c_str());
        if (satVoltage > 0.0)
            spec->setSaturationVoltage(satVoltage, false);

        if (spec->supportsGain())
        {
            // parse the separate high gain if supported
            int sepIdx = paramStr.indexOf(',', 11);
            if (sepIdx > 0)
            {
                satVoltage = atof(paramStr.substring(sepIdx+1).c_str());
                if (satVoltage > 0.0)
                    spec->setSaturationVoltage(satVoltage, true);
            }
        }
        buildSpecSatVoltageStr();
    }
    else if (paramStr.startsWith("BIAS"))
    {
        // delay the bias measurements - they can take a while
        delayedTask.taskID     = DT_BIAS;
        delayedTask.taskState  = 1;
        delayedTask.avgMinOnly = paramStr.endsWith(",MIN_AVG");
        result = delayedTask.taskID;
    }
    else if (paramStr.startsWith("LINEARISATION,"))
    {
        if (paramStr.endsWith(",RESET"))
        {
            spec->setLinearisation(0, false);
            if (spec->supportsGain())
                spec->setLinearisation(0, true);
            buildSpecLinearisationStr();
        }
        else
        {
            double linear[10] = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };

            // parse the calibration coefficient string
            int sepIdx = 14;
            const int count = spec->supportsGain() ? 10 : 5;
            for (int i=0; i<count; ++i)
            {
                linear[i] = atof(paramStr.substring(sepIdx).c_str());
                if (linear[i] == 0.0)
                {
                    result = -1;
                    break;
                }

                sepIdx = paramStr.indexOf(',', sepIdx+1);
                if (sepIdx<=0 && i!=count-1)
                {
                    result = -1;
                    break;
                }
                ++sepIdx;
            }
            // set parsed linearisation
            if (result >= 0)
                result = spec->setLinearisation(linear, false) - 1;
            if (spec->supportsGain() && result >= 0)
                result = spec->setLinearisation(linear+5, true) - 1;
            buildSpecLinearisationStr();
        }
    }
    else if (paramStr.startsWith("GAIN,") && spec->supportsGain())
    {
        if (paramStr.endsWith(",RESET"))
            spec->calibrateGain(1.0);    // only reset - takes no time
        else
        {
            double gainFactor = atof(paramStr.substring(5).c_str());
            if (gainFactor > 0)
                result = spec->calibrateGain(gainFactor) - 1;
            else
                result = -1;
        }
    }
    else
        result = -1;

    // reset measurement mode
    measuring = false;

    return result;
}

// Callback for delayed task run
void delayedTaskProgress(int percentageDone, bool isRelative)
{
    static unsigned long lastPublishedTimeMs = 0;

    if (percentageDone<=0)
        percentageDone = 1;

    if (!isRelative)
        delayedTask.taskState = percentageDone;
    else
        delayedTask.taskState += percentageDone;

    if (Particle.disconnected())
        Particle.connect();
    Particle.process();

    unsigned long curTimeMs = millis();
    if ((curTimeMs - lastPublishedTimeMs) >= 1000)  // limit events to 1 per sec
    {
        lastPublishedTimeMs = curTimeMs;
        Particle.publish("delayedTask",
                         String::format("progress:%d", delayedTask.taskState),
                         PRIVATE);
    }
}

// Runs delayed long-running tasks. Some calibration functions can take
// long time so they scheduled rather then being run inplace.
void specRunDelayedTasks()
{
    if (delayedTask.taskID == DT_NONE || !spec || spec->isMeasuring())
        return;

    switch (delayedTask.taskID)
    {
        case DT_BIAS:
            // sets the state to -1 when unsuccessful or 0 when succesful
            delayedTask.taskState = spec->measureBias(delayedTask.avgMinOnly,
                                                      &delayedTaskProgress) - 1;
            if (delayedTask.taskState == 0)
                specAvgBias = spec->getAvgBias();
            break;
        case DT_MEASURE:
            // measurement is always successful
            delayedTaskProgress(5, false);
            spec->takeMeasurement(delayedTask.measureTimeUs, T_USEC,
                                  delayedTask.doExtTrg);
            delayedTaskProgress(95, false);
            encodeMeasurement(ET_MEASUREMENT);
            delayedTask.taskState = spec->biasInvalid() ? BIAS_INVALID_MASK : 0;
            break;
        case DT_MEASURE_AUTO:
            // measurement is always successful
            delayedTaskProgress(5, false);
            // take auto measurement
            spec->takeAutoMeasurement(delayedTask.autoType,
                                      delayedTask.measureTimeUs,
                                      T_USEC,
                                      true,     // always reset blacks
                                      delayedTask.doExtTrg,
                                      delayedTask.saveState);
            // update Particle variables
            specAdcRefVoltage = spec->getAdcRefVoltage();
            specHighGain      = spec->getHighGain();
            specIntegTimeUs   = spec->getIntTime(T_USEC);
            delayedTaskProgress(95, false);
            encodeMeasurement(ET_MEASUREMENT);
            delayedTask.taskState = spec->biasInvalid() ? BIAS_INVALID_MASK : 0;
            break;
        case DT_MEASURE_BLACK:
            // dark measurement is always successful
            delayedTaskProgress(5, false);
            spec->takeBlackMeasurement(delayedTask.measureTimeUs,
                                       T_USEC,
                                       delayedTask.doExtTrg,
                                       delayedTask.avgBlack);
            delayedTask.taskState = spec->biasInvalid() ? BIAS_INVALID_MASK : 0;
            break;
    }

    delayedTask.taskID = DT_NONE;

    // publish completion event - ensure we are connected and ensure it is delivered
    if (Particle.disconnected())
        Particle.connect();
    waitFor(Particle.connected, 60000);

    Particle.publish("delayedTask",
                     String::format("complete:%d", delayedTask.taskState),
                     PRIVATE | WITH_ACK);
}

// Registers firmware functions and variables in Particle cloud. It should
// be called from firmware setup()
bool specRegisterCloudFunctions(CSpectrometer& spec_)
{
    // disable publishing vitals - they are useless for this project and override
    // delayed tasks events when long measurements are run
    Particle.publishVitals(0);

    // initialisation success
    static bool initSuccess = true;

    // initialise variables
    spec = &spec_;
    memset(specEncData, 0, sizeof(specEncData));

    // initialise Particle variables
    specAdcRefVoltage  = spec->getAdcRefVoltage();
    specHighGain       = spec->getHighGain();
    specIntegTimeUs    = spec->getIntTime(T_USEC);
    specExtTrigDelayUs = spec->getExtTrgMeasDelay(T_USEC);
    specAvgBias        = spec->getAvgBias();
    specPixels         = spec->getTotalPixels();
    specOffsetIdx      = spec->getStartPixelIdx();
    specGainFactor     = spec->getGainFactor();

    // no delayed tasks
    delayedTask.taskID    = DT_NONE;
    delayedTask.taskState = 0;

    // build up strings
    buildSpecSatVoltageStr();
    buildSpecLinearisationStr();
    buildSpecCalibrationStr();

    // register Particle variables
    initSuccess = initSuccess && Particle.variable("BOARD_TYPE",          BOARD_TYPE);
    initSuccess = initSuccess && Particle.variable("spNumPixels",         specPixels);
    initSuccess = initSuccess && Particle.variable("spADCRefVoltage",     specAdcRefVoltage);
    if (spec->supportsGain())
    {
        initSuccess = initSuccess && Particle.variable("spHighGain",      specHighGain);
        initSuccess = initSuccess && Particle.variable("spGainFactor",    specGainFactor);
    }
    initSuccess = initSuccess && Particle.variable("spIntegTimeUsec",     specIntegTimeUs);
    initSuccess = initSuccess && Particle.variable("spTrgMeasDelayUsec",  specExtTrigDelayUs);
    initSuccess = initSuccess && Particle.variable("spWavelenCalibration",specCalibrationStr);
    initSuccess = initSuccess && Particle.variable("spLinearisation",     specLinearisationStr);
    initSuccess = initSuccess && Particle.variable("spSatVoltage",        specSatVoltageStr);
    initSuccess = initSuccess && Particle.variable("spAverageBias",       specAvgBias);
    initSuccess = initSuccess && Particle.variable("spPixelOffsetIdx",    specOffsetIdx);
    initSuccess = initSuccess && Particle.variable("delayedTaskState",    delayedTask.taskState);

    char* encData = specEncData;
    int count = 1;
    while ((size_t)(encData-specEncData) < sizeof(specEncData))
    {
        String varName = "spData";
        varName += String(count++);
        initSuccess = initSuccess && Particle.variable(varName, encData);
        encData += maxVarSize+1;
    }

    // register functions
    initSuccess = initSuccess && Particle.function("spSetState",            specSetState);
    initSuccess = initSuccess && Particle.function("spGetData",             specGetData);
    initSuccess = initSuccess && Particle.function("spMeasure",             specMeasure);
    initSuccess = initSuccess && Particle.function("spSetIntegrationTime",  specSetIntegrationTime);
    initSuccess = initSuccess && Particle.function("spSetTrigMeasureDelay", specSetTriggerMeasurementDelay);
    initSuccess = initSuccess && Particle.function("spCalibrate",           specCalibrate);
    initSuccess = initSuccess && Particle.function("spSetSpectralRange",    specSetRange);
    initSuccess = initSuccess && Particle.function("spResetToDefaults",     specResetToDefaults);

    return initSuccess;
}
