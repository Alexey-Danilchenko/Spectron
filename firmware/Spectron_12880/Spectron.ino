/*
 *  Spectron.ino - Spectron board firmware main file.
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

SYSTEM_MODE(MANUAL);

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

#define TRG_IN         A2   // extrenal trigger button
#define TRG_FLASH      TX
#define TRG_CAMERA     D2

#define TRG_3V         D3
#define EOS_3V         D4
#define CLK_3V         D5
#define ST_3V          D6
#define GAIN_3V        D7

#define LCD_DC        SDA
#define LCD_RESET     SCL
#define LCD_SD_CS     SPI_SS_2
#define LCD_CS        SPI_SS_1
#define LCD_MOSI      SPI_MOSI
#define LCD_MISO      SPI_MISO
#define LCD_SCK       SPI_SCK
#define LCD_BACKLIGHT D_PWM

#define INACTIVE_DELAY_MS 120000

#define STR_EXPAND(t1,t2,t3,t4,t5,t6) #t1","#t2","#t3","#t4","#t5","#t6
#define STR(tok) STR_EXPAND(tok)

// enum for trigger
enum trigger_t {
    TRIGGER_CAMERA = 0,
    TRIGGER_FLASH  = 1
};

// Calibration data for my sensor 16H00851
// specify the real one for your sensor here
#define CALIBRATION_16H00851   305.0440912,2.715822498,-1.072966469E-03,-8.897283237E-06,1.519598265E-08,-4.899202027E-12

// calibration array for spectrometer object
const double CALIBRATION[] = { CALIBRATION_16H00851 };

// spectrometer object - can be only one per application
C12880MA spec(EOS_3V,
              TRG_3V,
              CLK_3V,
              ST_3V,
              TRG_CAMERA,
              ADC_REF_SEL_1,
              ADC_REF_SEL_2,
              ADC_CNV,
              CALIBRATION);

// Particle exposed variables
const char specCalibration[] = STR(CALIBRATION_16H00851);
const int  specPixels = SPEC_PIXELS;
trigger_t  specTriggerType = TRIGGER_CAMERA;
char       lastMeas[2480];

// maximum size for string variable data in Particle
const int  maxVarSize = 620;

// initialisation success
static bool initSuccess = true;

// cloud functions
int specSetTriggerMeasurementDelay(String trgMeasDelayStr)
{
    if (spec.isMeasuring())
        return -1;

    int32_t trgMeasDelayUs = trgMeasDelayStr.toInt();
    
    if (trgMeasDelayUs < 0)
        return -1;
        
    spec.setExtTrgMeasDelay(trgMeasDelayUs);

    return 0;
}

int specSetTriggerType(String trgTypeStr)
{
    if (spec.isMeasuring())
        return -1;

    int trgType = trgTypeStr.toInt();
    if (trgType == TRIGGER_CAMERA)
        spec.setExtTriggerPin(TRG_CAMERA);
    else if (trgType == TRIGGER_FLASH)
        spec.setExtTriggerPin(TRG_FLASH);
    else
        return -1;

    return 0;
}

int specSetBlackMode(String blackModeStr)
{
    if (spec.isMeasuring())
        return -1;

    int blackMode = blackModeStr.toInt();
    if (blackMode != MANUAL_BLACK &&
        blackMode != FOLLOWUP_BLACK &&
        blackMode != LEADING_BLACK)
        return -1;
    spec.setBlackMode((black_t)blackMode);

    return 0;
}

int specSetADCRef(String adcRefStr)
{
    if (spec.isMeasuring())
        return -1;

    int adcRef = adcRefStr.toInt();
    if (adcRef != ADC_2_5V &&
        adcRef != ADC_3V &&
        adcRef != ADC_4V &&
        adcRef != ADC_5V)
        return -1;
    spec.setAdcReference((adc_ref_t)adcRef);

    return adcRef==spec.getAdcReference() ? 0 : -1;
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
    
    return 0;
}

int specMeasure(String intTimeStr)
{
    static const char *hexDigits = "0123456789ABCDEF";
    static bool measuring = false;

    if (measuring || spec.isMeasuring())
        return -1;

    // set measurement mode
    measuring = true;

    // set integration time if not empty
    if (intTimeStr.length() > 0)
    {
        int32_t intTime = intTimeStr.toInt();

        // use specified time if it is there
        if (intTime > 0)
            spec.setIntTime(intTime);
    }

    // take measurement
    spec.takeMeasurement();

    // transfer measurement data to series of string
    // variables
    int bufSize = sizeof(lastMeas);
    memset(lastMeas, 0, bufSize);
    char* meas = lastMeas;
    int charCount = 0;
    for (int i=0; i<SPEC_PIXELS; i++)
    {
        // Here the array of floats are printed as
        // 32 bit hex numbers - the assumption is that
        // floats are in IEEE 754 (binary32) format
        float measValue = spec.getMeasurement(i);
        uint32_t &data = *((uint32_t *)&measValue);
        for (int nCnt=28; nCnt>=0; nCnt-=4)
        {
            char nibble = hexDigits[(data >> nCnt) & 0xF];

            // check if we reached the end of the overall buffer
            if (bufSize < 2)
                break;

            // advance string to next Particle variable if we reached
            // maximum for the current one
            if (charCount >= maxVarSize)
            {
                ++meas;
                charCount = 0;
                --bufSize;
            }

            *meas++ = nibble;
            ++charCount;
            --bufSize;
        }
    }

    // reset measurement mode
    measuring = false;

    return 0;
}

int specMeasureBlack(String intTimeStr)
{
    if (spec.isMeasuring())
        return -1;

    // set integration time if not empty
    if (intTimeStr.length() > 0)
    {
        int32_t intTime = intTimeStr.toInt();

        // use specified time if it is there
        if (intTime > 0)
            spec.setIntTime(intTime);
    }

    // take measurement
    spec.takeBlackMeasurement();

    return 0;
}

// main firmware initialisation
void setup()
{
    // initialise variables
    memset(lastMeas, 0, sizeof(lastMeas));

    pinMode(TRG_CAMERA, OUTPUT);
    pinMode(TRG_FLASH,  OUTPUT);

    // initialise spectrometer
    initSuccess = spec.begin();
    spec.setIntTime(1 _mSEC);   // 0.001s to start with

    // register Particle variables
    initSuccess = initSuccess && Particle.variable("spCal", specCalibration);
    initSuccess = initSuccess && Particle.variable("spNumPixels", specPixels);
    initSuccess = initSuccess && Particle.variable("spTrgType", specTriggerType);
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
    initSuccess = initSuccess && Particle.function("spSetTrMesDl", specSetTriggerMeasurementDelay);
    initSuccess = initSuccess && Particle.function("spSetTrgType", specSetTriggerType);
    initSuccess = initSuccess && Particle.function("spSetBlckMod", specSetBlackMode);
    initSuccess = initSuccess && Particle.function("spSetADCRef",  specSetADCRef);
    initSuccess = initSuccess && Particle.function("spSetIntTime", specSetIntegrationTime);
    initSuccess = initSuccess && Particle.function("spMeasure",    specMeasure);
    initSuccess = initSuccess && Particle.function("spMeasureBlk", specMeasureBlack);

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
