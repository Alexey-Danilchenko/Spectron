/*
 *  Spectron.ino - Spectron board firmware main file.
 *
 *  Copyright 2017-2018 Alexey Danilchenko, Iliah Borg
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

// Specify EEPROM base address for C12880 state saved - should
// be before included header
#define EEPROM_C12880_BASE_ADDR  0

#include "C12880MA.h"

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

// Factory calibration data for my sensor 16H00851 from Hamamatsu
// specify the real one for your sensor here
const double FACTORY_CALIBRATION[] = {
    305.0440912,2.715822498,-1.072966469E-03,-8.897283237E-06,1.519598265E-08,-4.899202027E-12
};

// spectrometer object - can be only one per application
C12880MA spec(EOS_3V,
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
const int  specPixels = SPEC_PIXELS;
char       specCalibrationStr[128];
int        specAdcRef;
int        specMeasureType;
double     specSatVoltage;
uint32_t   specIntegTime;
int32_t    specExtTrigDelay;
char       lastMeas[ENC_RESULT_STR_SIZE]; // Base64 encoded floats

// maximum size for string variable data in Particle
const int  maxVarSize = 620;

// initialisation success
static bool initSuccess = true;

// Cloud functions
// Sets wavelength calibration coefficients one at a time. When the last one
// is set, they are committed to the device and persisted. Thus to set all of
// them calls to this function needs to be performed sequentially for each
// coefficient.
//
// The committing of already set coefficients could be forced earlier with
// the parameter. This will allow to change subset of the coefficients.
//
// Format of parameter string:
//    <N>,<ValueN> - sets coefficient N (0..5) value to non zero ValueN and
//                   persists if there are no more coefficients left unset
//
//    <N>,<ValueN>,SAVE - as above but commit and persist already set
//                         coefficients
//
// This complications only exist because of limitation of the Particle API
// parameter string length
//
int specSetWavelengthCalibration(String paramStr)
{
    static double calibration[6] = { 0.0,0.0,0.0,0.0,0.0,0.0 };

    // all uppercase
    paramStr.trim().toUpperCase();

    if (spec.isMeasuring() || paramStr.length() == 0)
        return -1;

    // parse the calibration coefficient string
    int sepIdx = paramStr.indexOf(',');
    if (sepIdx <= 0 || sepIdx+1 == paramStr.length())
        return -1;

    int coefN = paramStr.toInt();
    double valueN = atof(paramStr.c_str()+sepIdx+1);

    if (coefN < 0 || coefN > 5 || valueN == 0.0)
        return -1;

    bool doCommit = paramStr.endsWith(",SAVE");

    calibration[coefN] = valueN;

    // validate if we have finished collecting all 6 values
    if (!doCommit)
    {
        bool doCommit = true;
        for (int i=0; i<6 && doCommit; i++)
            doCommit = calibration[i] != 0.0;

    }
    else
    {
        // fill in the 0
        const double* savedCalibration = spec.getWavelengthCalibration();
        for (int i=0; i<6; i++)
            if (calibration[i] == 0.0)
                calibration[i] = savedCalibration[i];
    }

    // commit the set data
    if (doCommit)
    {
        spec.setWavelengthCalibration(calibration);

        // build Particle variable
        String::format("%.10G,%.10G,%.10G,%.10G,%.10G,%.10G",
                       calibration[0],
                       calibration[1],
                       calibration[2],
                       calibration[3],
                       calibration[4],
                       calibration[5]).toCharArray(specCalibrationStr,
                                                   sizeof(specCalibrationStr));

        // reset the data for the future setting session
        for (int i=0; i<6; i++)
            calibration[i] = 0.0;
    }

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

// Reading time in uSec as a parameter
int specMeasure(String paramStr)
{
    static const char* encB64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+-";
    static bool measuring = false;

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
    int bufSize = sizeof(lastMeas);
    memset(lastMeas, 0, bufSize);
    char* meas = lastMeas;
    int charCount = 0;

    float measValue = 0;
    uint8_t *data = (uint8_t*)&measValue;
    int inCnt = 0, outCnt = 0;
    int value = 0, bits = -6;

    while (inCnt>>2 < SPEC_PIXELS)
    {
        // read the next float
        if ((inCnt&3) == 0)
            measValue = spec.getMeasurement(inCnt>>2);

        value = (value<<8) + data[inCnt&3];
        ++inCnt;
        bits += 8;

        while (bits >= 0)
        {
            // advance string to next Particle variable if we reached
            // maximum for the current one
            if (charCount >= maxVarSize)
            {
                ++meas;
                charCount = 0;
                --bufSize;
            }

            *meas++ = encB64[(value>>bits)&0x3F];
            ++charCount;
            --bufSize;
            bits -= 6;
        }
    }

    if (bits > -6)
        *meas++ = encB64[((value<<8)>>(bits+8))&0x3F];

    while (outCnt & 3)
        *meas++ = '=';

    // reset measurement mode
    measuring = false;

    return 0;
}

// Reading time in uSec as a parameter
int specMeasureBlack(String paramStr)
{
    paramStr.trim().toUpperCase();

    if (spec.isMeasuring() || paramStr.length() == 0)
        return -1;

    // parse the time
    int32_t measTimeUs = paramStr.toInt();
    if (measTimeUs < 0)
        return -1;

    spec.takeBlackMeasurement(measTimeUs);

    return 0;
}

int specResetBlack(String notUsedStr)
{
    if (spec.isMeasuring())
        return -1;

    spec.resetBlackLevels();

    return 0;
}

// Set the saturation voltages. These are used to in auto integration
// mode of measurement. Format of the parameter string:
//    <satVoltage> - explicit value for saturation voltage
//    AUTO         - measure the saturation voltages automatically
//
// For automatic setting expose sensor to bright light and then call this
// method with AUTO to work out saturation voltages.
int specSetSaturationVoltage(String paramStr)
{
    // all uppercase
    paramStr.trim().toUpperCase();

    if (spec.isMeasuring() || paramStr.length() == 0)
        return -1;

    // parse the string
    if (paramStr.equals("AUTO"))
        spec.measureSaturationVoltage();
    else
    {
        float satVoltage = atof(paramStr);

        if (satVoltage <= 0.0)
            return -1;

        // set the voltages
        spec.setSaturationVoltage(satVoltage);
    }

    // update Particle variable
    specSatVoltage = spec.getSatVoltage();

    return 0;
}

// main firmware initialisation
void setup()
{
    // initialise variables
    memset(lastMeas, 0, sizeof(lastMeas));

    pinMode(TRG_CAMERA, OUTPUT);
    pinMode(TRG_LIGHT_SRC,  OUTPUT);

    // initialise spectrometer
    initSuccess = spec.begin();

    // initialise Particle variables
    specAdcRef         = spec.getAdcReference();
    specMeasureType    = spec.getMeasurementType();
    specIntegTime      = spec.getIntTime();
    specExtTrigDelay   = spec.getExtTrgMeasDelay();
    specSatVoltage     = spec.getSatVoltage();

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
    initSuccess = initSuccess && Particle.variable("BOARD_TYPE",   BOARD_TYPE);
    initSuccess = initSuccess && Particle.variable("spNumPixels",  specPixels);
    initSuccess = initSuccess && Particle.variable("spADCRef",     specAdcRef);
    initSuccess = initSuccess && Particle.variable("spMeasType",   specMeasureType);
    initSuccess = initSuccess && Particle.variable("spIntegTime",  specIntegTime);
    initSuccess = initSuccess && Particle.variable("spTrgMeasDl",  specExtTrigDelay);
    initSuccess = initSuccess && Particle.variable("spCal",        specCalibrationStr);
    initSuccess = initSuccess && Particle.variable("spSatVoltage", specSatVoltage);

    char* meas = lastMeas;
    int count = 1;
    while (meas-lastMeas < sizeof(lastMeas))
    {
        String varName = "spLastMeas";
        varName += String(count++);
        initSuccess = initSuccess && Particle.variable(varName, meas);
        meas += maxVarSize+1;
    }

    // register functions
    initSuccess = initSuccess && Particle.function("spMeasure",    specMeasure);
    initSuccess = initSuccess && Particle.function("spMeasureBlk", specMeasureBlack);
    initSuccess = initSuccess && Particle.function("spSetIntTime", specSetIntegrationTime);
    initSuccess = initSuccess && Particle.function("spSetTrMesDl", specSetTriggerMeasurementDelay);
    initSuccess = initSuccess && Particle.function("spSetADCRef",  specSetADCRef);
    initSuccess = initSuccess && Particle.function("spSetMeasTyp", specSetMeasurementType);
    initSuccess = initSuccess && Particle.function("spSetWvlnCal", specSetWavelengthCalibration);
    initSuccess = initSuccess && Particle.function("spResetBlk",   specResetBlack);
    initSuccess = initSuccess && Particle.function("spSetSatVolt", specSetSaturationVoltage);

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
