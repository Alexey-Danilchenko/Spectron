/*
 *  Spectron.ino - Spectron board firmware main file.
 *
 *  Copyright 2017-2019 Alexey Danilchenko, Iliah Borg
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

SYSTEM_MODE(MANUAL);

// Specify EEPROM base address for C12666 state saved - should
// be before included header
#define EEPROM_C12666_BASE_ADDR  0

#include "C12666MA.h"

#define ADC_REF_SEL_1  A1
#define ADC_REF_SEL_2  A0
#define ADC_CNV        DAC
#define SPI_SS_1       WKP
#define D_PWM          RX
#define SPI_SS_2       A2
#define SPI_MOSI       A5
#define SPI_MISO       A4
#define SPI_SCK        A3
#define SDA            D0
#define SCL            D1

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

#define LCD_DC         SDA
#define LCD_RESET      SCL
#define LCD_SD_CS      SPI_SS_2
#define LCD_CS         SPI_SS_1
#define LCD_MOSI       SPI_MOSI
#define LCD_MISO       SPI_MISO
#define LCD_SCK        SPI_SCK
#define LCD_BACKLIGHT  D_PWM

#define ENC_RESULT_STR_SIZE (((SPEC_PIXELS)*sizeof(float)*4/3)+16)

// Board type identifier
static String BOARD_TYPE = "SPEC2_SPECTROMETER";

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
              FACTORY_CALIBRATION);

// Particle exposed variables
int        specPixels = SPEC_PIXELS;
int        specOffsetIdx;
char       specCalibrationStr[128];
int        specAdcRef;
int        specGain;
int        specMeasureType;
double     highGainSatVoltage;
double     noGainSatVoltage;
double     specMinBlackVoltage;
uint32_t   specIntegTime;
int32_t    specExtTrigDelay;
char       specEncData[ENC_RESULT_STR_SIZE]; // Base64 encoded floats

// maximum size for string variable data in Particle
const int  maxVarSize = 620;

// initialisation success
static bool initSuccess = true;

// re-entry prevention
static bool measuring = false;

// Auxiliary functions
enum encode_t {
    ET_MEASUREMENT   = 0,
    ET_BLACK_LEVELS  = 1,
    ET_NORMALISATION = 3
};

// Encode measurement result in Base64
void encodeMeasurement(encode_t encodeType, bool normalise=true)
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

    while (inCnt>>2 < SPEC_PIXELS)
    {
        // read the next float
        if ((inCnt&3) == 0)
            switch (encodeType)
            {
                case ET_MEASUREMENT:
                    floatVal = spec.getMeasurement(inCnt>>2, normalise);
                    break;
                case ET_BLACK_LEVELS:
                    floatVal = spec.getBlackLevelVoltage(inCnt>>2);
                    break;
                case ET_NORMALISATION:
                    floatVal = spec.getNormalisationCoef(inCnt>>2);
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
//    BLACK_LEVELS      - black level voltages captured
//    NORMALISATION     - spectral response normalisation coefficients
//    MEASUREMENT       - results of the measurement with current
//                        black levels only applied
//    MEAS_NORMALISED   - results of the measurement with current
//                        black levels and normalisation applied
//
int specGetData(String paramStr)
{
    if (measuring || spec.isMeasuring())
        return -1;

    // set measurement mode - preventing reentry
    measuring = true;

    // get data type
    bool normalise = true;
    paramStr.trim().toUpperCase();
    encode_t encType = ET_MEASUREMENT;
    if (paramStr == "BLACK_LEVELS")
        encType = ET_BLACK_LEVELS;
    else if (paramStr == "NORMALISATION")
        encType = ET_NORMALISATION;
    else if (paramStr == "MEASUREMENT")
        normalise = false;
    else if (paramStr != "MEAS_NORMALISED")
        return -1;

    // encode data
    encodeMeasurement(encType, normalise);

    // reset measurement mode
    measuring = false;

    return 0;
}

// Sets wavelength calibration coefficients. Format of parameter string:
//
//    <K0>,<K1>,<K2>,<K3>,<K4>,<K5> - coefficients values
//
int specSetWavelengthCalibration(String paramStr)
{
    double calibration[6] = { 0.0,0.0,0.0,0.0,0.0,0.0 };

    // all uppercase
    paramStr.trim().toUpperCase();

    if (spec.isMeasuring() || paramStr.length() == 0)
        return -1;

    // parse the calibration coefficient string
    int sepIdx = 0;
    for (int i=0; i<6; i++)
    {
        calibration[i] = atof(paramStr.substring(sepIdx).c_str());
        if (calibration[i] == 0.0)
            return -1;

        sepIdx = paramStr.indexOf(',', sepIdx+1);
        if (sepIdx<=0 && i!=5)
            return -1;
        ++sepIdx;
    }

    // save calibration
    spec.setWavelengthCalibration(calibration);

    // update Particle variables
    specPixels    = spec.getTotalPixels();
    specOffsetIdx = spec.getStartPixelIdx();
    String::format("%.10G,%.10G,%.10G,%.10G,%.10G,%.10G",
                   calibration[0],
                   calibration[1],
                   calibration[2],
                   calibration[3],
                   calibration[4],
                   calibration[5]).toCharArray(specCalibrationStr,
                                               sizeof(specCalibrationStr));

    return 0;
}

// Resets all internal parameters to their defaults
int specResetToDefaults(String paramStr)
{
    if (spec.isMeasuring())
        return -1;

    spec.resetToDefaults(FACTORY_CALIBRATION);

    // initialise Particle variables
    specAdcRef          = spec.getAdcReference();
    specGain            = spec.getGain();
    specMeasureType     = spec.getMeasurementType();
    specIntegTime       = spec.getIntTime();
    specExtTrigDelay    = spec.getExtTrgMeasDelay();
    highGainSatVoltage  = spec.getHighGainSatVoltage();
    noGainSatVoltage    = spec.getNoGainSatVoltage();
    specMinBlackVoltage = spec.getMinBlackVoltage();
    specPixels          = spec.getTotalPixels();
    specOffsetIdx       = spec.getStartPixelIdx();

    // build
    const double* calibration = spec.getWavelengthCalibration();
    String::format("%.10G,%.10G,%.10G,%.10G,%.10G,%.10G",
                   calibration[0],
                   calibration[1],
                   calibration[2],
                   calibration[3],
                   calibration[4],
                   calibration[5]).toCharArray(specCalibrationStr,
                                               sizeof(specCalibrationStr));
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
    // all uppercase
    paramStr.trim().toUpperCase();

    if (spec.isMeasuring() || paramStr.length() == 0)
        return -1;

    // parse the string
    if (paramStr.equals("DEFAULT"))
        spec.setSensorRange(-1, -1);
    else if (paramStr.equals("MAX"))
        spec.setSensorRange(1, 20000);  // use very large range
    else
    {
        // range was explicitly supplied
        int minWavelength = paramStr.toInt();
        int maxWavelength = 0;

        int sepIdx = paramStr.indexOf(',');
        if (sepIdx > 0)
            maxWavelength = paramStr.substring(sepIdx+1).trim().toInt();
        spec.setSensorRange(minWavelength, maxWavelength);
    }

    // update Particle variables
    specPixels    = spec.getTotalPixels();
    specOffsetIdx = spec.getStartPixelIdx();

    return 0;
}

// Sets external trigger to measurement delay or switches off
// external triggering altogether. Format of the parameter string:
//    OFF          - disable external triggering
//    <delay>      - sets the delay from triggering to start of the
//                   integration
int specSetTriggerMeasurementDelay(String paramStr)
{
    // all uppercase
    paramStr.trim().toUpperCase();

    if (spec.isMeasuring() || paramStr.length() == 0)
        return -1;

    // parse the string
    int32_t trgMeasDelayUs = 0;
    if (paramStr.equals("OFF"))
        trgMeasDelayUs = -1;
    else
        trgMeasDelayUs = paramStr.toInt();

    spec.setExtTrgMeasDelay(trgMeasDelayUs);

    // update Particle variable
    specExtTrigDelay = spec.getExtTrgMeasDelay();

    return 0;
}

int specSetGain(String paramStr)
{
    if (spec.isMeasuring())
        return -1;

    int gain = paramStr.toInt();
    if (gain != NO_GAIN &&
        gain != HIGH_GAIN)
        return -1;

    spec.setGain((gain_t)gain);

    // update Particle variable
    specGain = spec.getGain();

    return 0;
}

int specSetADCRef(String paramStr)
{
    if (spec.isMeasuring())
        return -1;

    int adcRef = paramStr.toInt();
    if (adcRef != ADC_2_5V &&
        adcRef != ADC_3V &&
        adcRef != ADC_4_096V &&
        adcRef != ADC_5V)
        return -1;

    spec.setAdcReference((adc_ref_t)adcRef);

    // update Particle variable
    specAdcRef = spec.getAdcReference();

    return adcRef==spec.getAdcReference() ? 0 : -1;
}

int specSetMeasurementType(String paramStr)
{
    if (spec.isMeasuring())
        return -1;

    int measType = paramStr.toInt();
    if (measType != MEASURE_RELATIVE &&
        measType != MEASURE_VOLTAGE &&
        measType != MEASURE_ABSOLUTE)
        return -1;

    spec.setMeasurementType((measure_t)measType);

    // update Particle variable
    specMeasureType = spec.getMeasurementType();

    return 0;
}

int specSetIntegrationTime(String intTimeStr)
{
    if (spec.isMeasuring())
        return -1;

    int32_t intTime = intTimeStr.toInt();

    // use specified time if it is there
    if (intTime <= 0)
        return -1;

    spec.setIntTime(intTime);

    // update Particle variable
    specIntegTime = spec.getIntTime();

    return 0;
}

// Run spectral measurement. Format of the parameter string:
//    <time>[,TRG]             - measurement time in uSec followed by optional
//                               external triggering (if specified)
//    AUTO[,TRG]               - Automatic measurement with current ADC voltage
//                               and optional triggering
//    AUTO_ALL_MIN_INTEG[,TRG] - Automatic measurement for min integration and
//                               optional triggering
//    AUTO_ALL_MAX_RANGE[,TRG] - Automatic measurement for max range and
//                               optional triggering
int specMeasure(String paramStr)
{
    if (measuring || spec.isMeasuring())
        return -1;

    // set measurement mode - preventing reentry
    measuring = true;

    // get reading time
    paramStr.trim().toUpperCase();
    bool doExtTrg = paramStr.endsWith(",TRG");
    bool isAuto = paramStr.startsWith("AUTO");

    // do the measurement
    if (isAuto)
    {
        auto_measure_t autoType = AUTO_FOR_SET_REF;
        if (paramStr.startsWith("AUTO_ALL_MIN_INTEG"))
            autoType = AUTO_ALL_MIN_INTEG;
        else if (paramStr.startsWith("AUTO_ALL_MAX_RANGE"))
            autoType = AUTO_ALL_MAX_RANGE;

        // take auto measurement
        spec.takeAutoMeasurement(autoType, doExtTrg);

        // update Particle variable
        specAdcRef    = spec.getAdcReference();
        specGain      = spec.getGain();
        specIntegTime = spec.getIntTime();
    }
    else
    {
        int32_t measTimeUs = paramStr.toInt();
        if (measTimeUs < 0)
        {
            measuring = false;
            return -1;
        }

        // take measurement
        spec.takeMeasurement(measTimeUs, doExtTrg);
    }

    // transfer measurement as Base64 data to series of string variables
    encodeMeasurement(ET_MEASUREMENT);

    // reset measurement mode
    measuring = false;

    return 0;
}

// Run black measurement. Format of the parameter string:
//    <time>      - measurement time in uSec (if 0 uses last one)
//    RESET       - resets black levels
int specMeasureBlack(String paramStr)
{
    paramStr.trim().toUpperCase();

    if (measuring || spec.isMeasuring())
        return -1;

    // set measurement mode - preventing reentry
    measuring = true;

    // parse the time
    if (paramStr.equals("RESET"))
        spec.resetBlackLevels();
    else
    {
    int32_t measTimeUs = paramStr.toInt();
    if (measTimeUs < 0)
        return -1;

    spec.takeBlackMeasurement(measTimeUs);
    }

    // reset measurement mode
    measuring = false;

    return 0;
}

// Set the saturation voltages. These are used to in auto integration
// mode of measurement. Format of the parameter string:
//    <hgV>,<ngV>  - high gain voltage (hgV) and no gain voltage (ngV), either
//                   is optional if only one needs setting
//    AUTO         - measure the saturation voltages automatically
//
// For automatic setting expose sensor to bright light and then call this
// method with AUTO to work out saturation voltages.
int specSetSaturationVoltages(String paramStr)
{
    // all uppercase
    paramStr.trim().toUpperCase();

    if (spec.isMeasuring() || paramStr.length() == 0)
        return -1;

    // parse the string
    if (paramStr.equals("AUTO"))
        spec.measureSaturationVoltages();
    else
    {
        float hgVoltage = -1;
        float ngVoltage = -1;

        // parse the string
        int sepIdx = paramStr.indexOf(',');
        if (sepIdx < 0)
            return -1;

        String hgVoltageStr = paramStr.substring(0, sepIdx).trim();
        String ngVoltageStr = paramStr.substring(sepIdx+1).trim();

        // parse high gain voltage
        if (hgVoltageStr.length() > 0)
        {
            hgVoltage = atof(hgVoltageStr);
            if (hgVoltage <= 0.0)
                return -1;
        }

        // parse no gain voltage
        if (ngVoltageStr.length() > 0)
        {
            ngVoltage = atof(ngVoltageStr);
            if (ngVoltage <= 0.0)
                return -1;
        }

        // set the voltages
        spec.setSaturationVoltages(hgVoltage, ngVoltage);
    }

    // update Particle variable
    highGainSatVoltage = spec.getHighGainSatVoltage();
    noGainSatVoltage   = spec.getNoGainSatVoltage();

    return 0;
}

// Set the minimum black level voltage either to spacified value or automatic.
//    <blackLevelVoltage>  - value of min black level voltage
//    AUTO                 - measure min black level voltage automatically
//
int specSetMinBlack(String paramStr)
{
    // all uppercase
    paramStr.trim().toUpperCase();

    if (spec.isMeasuring() || paramStr.length() == 0)
        return -1;

    // parse the string
    if (paramStr.equals("AUTO"))
        spec.measureMinBlackVoltage();
    else
        spec.setMinBlackVoltage(paramStr.toFloat());

    // update variable
    specMinBlackVoltage = spec.getMinBlackVoltage();

    return 0;
}

// Calibrate spectral response. Format of the parameter string:
//    <TempK>[,MEASURE]  - TempK - lamp temperature in K,
//                         second parameter only specified to perform measuring
//                         instead of using already existing one
//    RESET              - reset calibration to unity
//
// Procedure for calibration:
//
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
//    done at (2))
//
int specCalibrateSpectralResponse(String paramStr)
{
    // all uppercase
    paramStr.trim().toUpperCase();

    if (measuring || spec.isMeasuring() || paramStr.length() == 0)
        return -1;

    // set measurement mode - preventing reentry
    measuring = true;

    // parse the string
    bool useCurrentMeasurement = !paramStr.endsWith(",MEASURE");
    if (paramStr.equals("RESET"))
        spec.calibrateSpectralResponse(0.0);
    else
    {
        float tempK = paramStr.toFloat();

        // set the voltages
        spec.calibrateSpectralResponse(tempK, useCurrentMeasurement);
    }

    // update measurement results
    encodeMeasurement(ET_NORMALISATION);

    // reset measurement mode
    measuring = false;

    return 0;
}

// main firmware initialisation
void setup()
{
    // initialise variables
    memset(specEncData, 0, sizeof(specEncData));

    pinMode(TRG_CAMERA, OUTPUT);
    pinMode(TRG_LIGHT_SRC,  OUTPUT);

    // initialise spectrometer
    initSuccess = spec.begin();

    // initialise Particle variables
    specAdcRef          = spec.getAdcReference();
    specGain            = spec.getGain();
    specMeasureType     = spec.getMeasurementType();
    specIntegTime       = spec.getIntTime();
    specExtTrigDelay    = spec.getExtTrgMeasDelay();
    highGainSatVoltage  = spec.getHighGainSatVoltage();
    noGainSatVoltage    = spec.getNoGainSatVoltage();
    specMinBlackVoltage = spec.getMinBlackVoltage();
    specPixels          = spec.getTotalPixels();
    specOffsetIdx       = spec.getStartPixelIdx();

    // build
    const double* calibration = spec.getWavelengthCalibration();
    String::format("%.10G,%.10G,%.10G,%.10G,%.10G,%.10G",
                   calibration[0],
                   calibration[1],
                   calibration[2],
                   calibration[3],
                   calibration[4],
                   calibration[5]).toCharArray(specCalibrationStr,
                                               sizeof(specCalibrationStr));

    // register Particle variables
    initSuccess = initSuccess && Particle.variable("BOARD_TYPE",          BOARD_TYPE);
    initSuccess = initSuccess && Particle.variable("spNumPixels",         specPixels);
    initSuccess = initSuccess && Particle.variable("spADCRef",            specAdcRef);
    initSuccess = initSuccess && Particle.variable("spGain",              specGain);
    initSuccess = initSuccess && Particle.variable("spMeasurementType",   specMeasureType);
    initSuccess = initSuccess && Particle.variable("spIntegrationTime",   specIntegTime);
    initSuccess = initSuccess && Particle.variable("spTrigMeasureDelay",  specExtTrigDelay);
    initSuccess = initSuccess && Particle.variable("spWavelenCalibration",specCalibrationStr);
    initSuccess = initSuccess && Particle.variable("spHighGainSatVoltage",highGainSatVoltage);
    initSuccess = initSuccess && Particle.variable("spNoGainSatVoltage",  noGainSatVoltage);
    initSuccess = initSuccess && Particle.variable("spMinBlackVoltage",   specMinBlackVoltage);
    initSuccess = initSuccess && Particle.variable("spPixelOffsetIdx",    specOffsetIdx);

    char* encData = specEncData;
    int count = 1;
    while (encData-specEncData < sizeof(specEncData))
    {
        String varName = "spData";
        varName += String(count++);
        initSuccess = initSuccess && Particle.variable(varName, encData);
        encData += maxVarSize+1;
    }

    // register functions
    initSuccess = initSuccess && Particle.function("spGetData",               specGetData);
    initSuccess = initSuccess && Particle.function("spMeasure",               specMeasure);
    initSuccess = initSuccess && Particle.function("spMeasureBlack",          specMeasureBlack);
    initSuccess = initSuccess && Particle.function("spSetIntegrationTime",    specSetIntegrationTime);
    initSuccess = initSuccess && Particle.function("spSetTrigMeasureDelay",   specSetTriggerMeasurementDelay);
    initSuccess = initSuccess && Particle.function("spSetADCRef",             specSetADCRef);
    initSuccess = initSuccess && Particle.function("spSetGain",               specSetGain);
    initSuccess = initSuccess && Particle.function("spSetMeasurementType",    specSetMeasurementType);
    initSuccess = initSuccess && Particle.function("spSetWavelenCalibration", specSetWavelengthCalibration);
    initSuccess = initSuccess && Particle.function("spSetSaturationVoltage",  specSetSaturationVoltages);
    initSuccess = initSuccess && Particle.function("spSetMinBlackVoltage",    specSetMinBlack);
    initSuccess = initSuccess && Particle.function("spCalibrateSpectralResp", specCalibrateSpectralResponse);
    initSuccess = initSuccess && Particle.function("spSetSpectralRange",      specSetRange);
    initSuccess = initSuccess && Particle.function("spResetToDefaults",       specResetToDefaults);

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
    if (!spec.isMeasuring())
        if (Particle.connected())
            Particle.process();
        else
            Particle.connect();
}
