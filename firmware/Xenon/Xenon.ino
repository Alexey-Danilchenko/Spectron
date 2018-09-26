/*
 *  Xenon.ino - Spectron Xenon light source control main file.
 *
 *  Copyright 2018 Alexey Danilchenko, Iliah Borg
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

SYSTEM_THREAD(ENABLED);

#include "math.h"

// Pin definitions
#define ENABLE_REF       D6

#define TRIGGER_PWM      D3

#define DAC_SHUTDOWN     D2
#define DAC_LDAC         A0
#define DAC_CLEAR        A1
#define DAC_SPI_SS       A2

// A few parameters for the circuit
#define MAX_XL_BRIGHTNESS     1000
#define MAX_XL_TIMED_EXPOSURE 120000
#define MAX_DAQ_CODE          4095

// --------------------------------------------------------
// Xenon lamp parameters
// These need some adjusting for each particular lamp type.
//  
// Parameters as set here were tested for Excelitas FX-1163
// lamp mounted in metal cooling jacket.
// --------------------------------------------------------
#define MIN_XL_VOLTAGE              400     // Min lamp voltage
#define MAX_XL_VOLTAGE              1000    // Max lamp voltage
#define XL_MAIN_CAP_UF              0.5     // Lamp charge capacitor value for energy calculations
#define XL_RATED_POWER_WT           20      // Lamp rated power from the specs
#define XL_MAX_POWER_WT             40      // Allowed/acceptable overshoot for lamp power: if lamp is
                                            //    in the cooling jacket it can sustain more power through it
#define XL_TRG_PULSE_WIDTH_US       10      // Trigger pulse width in microseconds
#define XL_MAX_FLASH_RATE_HZ        300     // Lamp max trigger rate per second from specs
#define XL_MIN_FLASH_RATE_HZ        10      // Minimal trigger rate to prevent flickering
#define XL_MIN_PREF_FLASH_RATE_HZ   50      // Minimal trigger rate acceptable for given voltage. This is
                                            // only used for calculation of the values when setting brightness

// Xenon lamp functions
#define XLampFlashEnegy(voltage)   ((XL_MAIN_CAP_UF * voltage * voltage)/2000000)
#define XLampFlashVoltage(energy)  (sqrt(((energy) * 2000000) / XL_MAIN_CAP_UF))

// EEPROM addresses
#define EEPROM_XL_BRIGHTNESS      0
#define EEPROM_XL_EXPOSURE_TIME   4
#define EEPROM_XL_VOLTAGE         8
#define EEPROM_XL_TRG_RATE        12
#define EEPROM_XL_MAX_POWER       16

#define EEPROM_FREE_ADDR          20  // free address for application usage

// DAC Commands
#define CMD_NOP                   0
#define CMD_LOAD_INP              (0b0010 << 12)
#define CMD_LOAD_INP_AND_DAC      (0b0100 << 12)
#define CMD_LOAD_DAC_FROM_INP     (0b0110 << 12)
#define CMD_ENTER_SHUTDOWN        (0b1000 << 12)
#define CMD_EXIT_SHUTDOWN         (0b1100 << 12)

// forward declaration
void expFinished();

// Board type identifier
static String BOARD_TYPE = "SPEC2_XENON";

// Variables
static Timer expTimer_(2000, expFinished);           // Lamp exposure timer
static bool xlampIsOn_ = false;                      // current lamp state
static int32_t xlampMaxPower_ = XL_RATED_POWER_WT;   // lamp max power
static int32_t xlampBrightness_ = MAX_XL_BRIGHTNESS; // lamp voltage (defines flash brightness)
static int32_t xlampVoltage_ = 0;                    // lamp voltage (defines flash brightness)
static int32_t xlampTrgRate_ = 0;                    // lamp triggering rate
static int32_t exposureTime_ = 2000;                 // lamp exposure time in ms, 0 - manual on/off

// Calculate and set Xenon lamp parameters for given brightness
// This will favour higher voltage over frequency until XL_MIN_PREF_FLASH_RATE_HZ
// rate. Below that rate voltage will start to go down.
void setXLampParamsForBrightness()
{
    // calculate lamp energy relative to brightness
    double xlampPwr = (double)xlampMaxPower_ * xlampBrightness_ / MAX_XL_BRIGHTNESS;

    // loop through voltages from the highest to the lowest until
    // we have acceptable rates
    int32_t voltage = MAX_XL_VOLTAGE;
    int32_t flashRate = XL_MIN_FLASH_RATE_HZ;
    while (voltage > MIN_XL_VOLTAGE)
    {
        flashRate = xlampPwr / XLampFlashEnegy(voltage);
        if (flashRate > XL_MIN_PREF_FLASH_RATE_HZ)
            break;  // found it
        voltage -= 50;  // step down by 50V
    }

    // check for less then minimal voltage
    if (voltage < MIN_XL_VOLTAGE)
    {
        voltage = MIN_XL_VOLTAGE;
        flashRate = xlampPwr / XLampFlashEnegy(voltage);
    }

    // check for less then minimal rate
    if (flashRate < XL_MIN_FLASH_RATE_HZ)
    {
        flashRate = XL_MIN_FLASH_RATE_HZ;

        // adjust brightness
        xlampPwr = flashRate * XLampFlashEnegy(voltage);
        xlampBrightness_ = xlampPwr * MAX_XL_BRIGHTNESS / xlampMaxPower_;
    }

    // set lamp parameters in one go
    xlampVoltage_ = voltage;
    xlampTrgRate_ = flashRate;
}

// sets flash rate for given voltage and updates brightness
void setXLampParamsForVoltage()
{
    double flashPwr = xlampTrgRate_ * XLampFlashEnegy(xlampVoltage_);

    if (flashPwr > xlampMaxPower_)
    {
        // exceeded max lamp power - adjust flash rate
        xlampTrgRate_ = xlampMaxPower_ / XLampFlashEnegy(xlampVoltage_);
        if (xlampTrgRate_ > XL_MAX_FLASH_RATE_HZ)
            xlampTrgRate_ = XL_MAX_FLASH_RATE_HZ;
        else if (xlampTrgRate_ < XL_MIN_FLASH_RATE_HZ)
            xlampTrgRate_ = XL_MIN_FLASH_RATE_HZ;
        flashPwr = xlampTrgRate_ * XLampFlashEnegy(xlampVoltage_);
    }

    // recalculate the brightness
    xlampBrightness_ = flashPwr * MAX_XL_BRIGHTNESS / xlampMaxPower_;
}

// sets voltage for given flash rate and updates brightness
void setXLampParamsForTrgRate()
{
    double flashPwr = xlampTrgRate_ * XLampFlashEnegy(xlampVoltage_);

    if (flashPwr > xlampMaxPower_)
    {
        // exceeded max lamp power - adjust voltage
        xlampVoltage_ = XLampFlashVoltage((double)xlampMaxPower_/xlampTrgRate_);
        if (xlampVoltage_ > MAX_XL_VOLTAGE)
            xlampVoltage_ = MAX_XL_VOLTAGE;
        else if (xlampVoltage_ < MIN_XL_VOLTAGE)
            xlampVoltage_ = MIN_XL_VOLTAGE;
        flashPwr = xlampTrgRate_ * XLampFlashEnegy(xlampVoltage_);
    }

    // recalculate the brightness
    xlampBrightness_ = flashPwr * MAX_XL_BRIGHTNESS / xlampMaxPower_;
}

// setup DAC
void setupDAC()
{
    if (!xlampIsOn_)
    {
        // setup DAC command
        int32_t dacCode = xlampVoltage_ * (MAX_DAQ_CODE+1) / MAX_XL_VOLTAGE;
        if (dacCode > MAX_DAQ_CODE)
            dacCode = MAX_DAQ_CODE;

        int16_t dacCmd = CMD_LOAD_INP_AND_DAC | (dacCode & 0xFFF);

        // initialise SPI transfer
        pinResetFast(DAC_SPI_SS);

        // send the data
        uint16_t tmpByte = 0;
        tmpByte = SPI.transfer((dacCmd>>8) & 0xFF);
        tmpByte = SPI.transfer(dacCmd & 0xFF);
        pinSetFast(DAC_SPI_SS);
    }
}

// early init
void initXLamp()
{
    pinMode(ENABLE_REF,   OUTPUT);
    pinMode(TRIGGER_PWM,  OUTPUT);
    pinMode(DAC_SHUTDOWN, OUTPUT);
    pinMode(DAC_LDAC,     OUTPUT);
    pinMode(DAC_CLEAR,    OUTPUT);
    pinMode(DAC_SPI_SS,   OUTPUT);

    pinResetFast(ENABLE_REF);
    pinResetFast(DAC_CLEAR);
    pinResetFast(DAC_SHUTDOWN);
    pinSetFast(DAC_LDAC);
    pinSetFast(DAC_SPI_SS);

    analogWriteResolution(TRIGGER_PWM, 16);
    analogWrite(TRIGGER_PWM, 0, 50);

    // read the data from EEPROM
    // read lamp voltage details from EPROM
    EEPROM.get(EEPROM_XL_BRIGHTNESS, xlampBrightness_);
    if (xlampBrightness_ < 0 || xlampBrightness_ > MAX_XL_BRIGHTNESS)
        // EEPROM was empty
        xlampBrightness_ = MAX_XL_BRIGHTNESS;

    // read exposureTime details from EPROM
    EEPROM.get(EEPROM_XL_EXPOSURE_TIME, exposureTime_);
    if (exposureTime_ < 0 || exposureTime_ > MAX_XL_TIMED_EXPOSURE)
        // EEPROM was empty - set to manual exposure time
        exposureTime_ = 0;

    // read xlampMaxPower details from EPROM
    EEPROM.get(EEPROM_XL_MAX_POWER, xlampMaxPower_);
    if (xlampMaxPower_ <= 0 || xlampMaxPower_ > XL_MAX_POWER_WT)
        // EEPROM was empty - set to manual exposure time
        xlampMaxPower_ = XL_RATED_POWER_WT;

    // get voltage and triggering rate if they are valid
    EEPROM.get(EEPROM_XL_VOLTAGE,  xlampVoltage_);
    EEPROM.get(EEPROM_XL_TRG_RATE, xlampTrgRate_);

    // recalculate Xenon lamp parameters if invalid
    if (xlampVoltage_ < MIN_XL_VOLTAGE || xlampVoltage_ > MAX_XL_VOLTAGE ||
        xlampTrgRate_ < XL_MIN_FLASH_RATE_HZ || xlampTrgRate_ > XL_MAX_FLASH_RATE_HZ)
        setXLampParamsForBrightness();
}

// early startup
STARTUP(initXLamp());

// switch on Xenon lamp
void xlampOn()
{
    if (!xlampIsOn_ && xlampBrightness_ > 0)
    {
        xlampIsOn_ = true;

        // wake up DAC
        pinSetFast(DAC_SHUTDOWN);

        // wait for DAC output to stabilise
        delayMicroseconds(350);

        // calculate and start the PWM
        analogWrite(TRIGGER_PWM,
                    (xlampTrgRate_ * XL_TRG_PULSE_WIDTH_US * 65535) / 1000000,
                    xlampTrgRate_);

        // start the timer if timed
        if (exposureTime_ > 0)
        {
            expTimer_.changePeriod(exposureTime_);
            expTimer_.start();
        }
    }
}

// switch off Xenon lamp
void xlampOff(bool calledFromISR)
{
    if (xlampIsOn_)
    {
        xlampIsOn_ = false;
        if (expTimer_.isActive())
            if (calledFromISR)
                expTimer_.stopFromISR();
            else
                expTimer_.stop();
        analogWrite(TRIGGER_PWM, 0, xlampTrgRate_);
        pinResetFast(DAC_SHUTDOWN);
    }
}

// timer callback - exposure finished
void expFinished()
{
    xlampOff(true);
}

// cloud functions
int xlampSetExposureTime(String exposureTimeStr)
{
    exposureTimeStr.trim().toUpperCase();

    if (xlampIsOn_ || exposureTimeStr.length() == 0)
        return -1;

    int32_t exposureTime = exposureTimeStr.toInt();

    if (exposureTime < 0 || exposureTime > MAX_XL_TIMED_EXPOSURE)
        return -1;

    if (exposureTime_ != exposureTime)
    {
        exposureTime_ = exposureTime;
        EEPROM.put(EEPROM_XL_EXPOSURE_TIME, exposureTime_);
    }

    return 0;
}

// Sets the lamp brightness level (0..1000). Format of the parameter string:
//    <brightness> - sets the lamp brightness to a specified value
//    MAX          - sets the lamp brightness to a maximum
//
// This is a convenience method that calculates and sets lamp voltage and
// triggering rate. It will favour higher voltage over triggering rate.
int xlampSetBrightness(String paramStr)
{
    int brightness = -1;

    // only allow change when lamp is not running
    // or running in untimed mode
    if (xlampIsOn_ && exposureTime_ > 0)
        return -1;

    // all uppercase
    paramStr.trim().toUpperCase();

    if (paramStr.length() == 0)
        return -1;

    // parse the brightness string
    if (paramStr.equals("MAX"))
        brightness = MAX_XL_BRIGHTNESS;
    else
        brightness = paramStr.toInt();

    if (brightness < 0 || brightness > MAX_XL_BRIGHTNESS)
        return -1;

    if (xlampBrightness_ != brightness)
    {
        xlampBrightness_ = brightness;

        bool xlampWasOn = xlampIsOn_;
        if (xlampIsOn_)
            xlampOff(false);

        // recalculate params and update the DAC
        setXLampParamsForBrightness();

        // write all parameters to EEPROM
        EEPROM.put(EEPROM_XL_BRIGHTNESS, xlampBrightness_);
        EEPROM.put(EEPROM_XL_VOLTAGE, xlampVoltage_);
        EEPROM.put(EEPROM_XL_TRG_RATE, xlampTrgRate_);

        // update DAC
        setupDAC();

        if (xlampWasOn)
            xlampOn();
    }

    return 0;
}

// Sets the lamp voltage directly. If necessary triggering rate is adjusted.
int xlampSetVoltage(String paramStr)
{
    int voltage = -1;

    // only allow change when lamp is not running
    // or running in untimed mode
    if (xlampIsOn_ && exposureTime_ > 0)
        return -1;

    // all uppercase
    paramStr.trim().toUpperCase();

    if (paramStr.length() == 0)
        return -1;

    // parse the brightness string
    if (paramStr.equals("MAX"))
        voltage = MAX_XL_VOLTAGE;
    else if (paramStr.equals("MIN"))
        voltage = MIN_XL_VOLTAGE;
    else
        voltage = paramStr.toInt();

    if (voltage < MIN_XL_VOLTAGE || voltage > MAX_XL_VOLTAGE)
        return -1;

    if (xlampVoltage_ != voltage)
    {
        xlampVoltage_ = voltage;

        bool xlampWasOn = xlampIsOn_;
        if (xlampIsOn_)
            xlampOff(false);

        // recalculate params and update the DAC
        setXLampParamsForVoltage();

        // write all parameters to EEPROM
        EEPROM.put(EEPROM_XL_BRIGHTNESS, xlampBrightness_);
        EEPROM.put(EEPROM_XL_VOLTAGE, xlampVoltage_);
        EEPROM.put(EEPROM_XL_TRG_RATE, xlampTrgRate_);

        // update DAC
        setupDAC();

        if (xlampWasOn)
            xlampOn();
    }

    return 0;
}

// Sets the lamp triggering rate directly. If necessary voltage is adjusted.
int xlampSetTriggerRate(String paramStr)
{
    int trgRate = -1;

    // only allow change when lamp is not running
    // or running in untimed mode
    if (xlampIsOn_ && exposureTime_ > 0)
        return -1;

    // all uppercase
    paramStr.trim().toUpperCase();

    if (paramStr.length() == 0)
        return -1;

    // parse the brightness string
    if (paramStr.equals("MAX"))
        trgRate = XL_MAX_FLASH_RATE_HZ;
    else if (paramStr.equals("MIN"))
        trgRate = XL_MIN_FLASH_RATE_HZ;
    else
        trgRate = paramStr.toInt();

    if (trgRate < XL_MIN_FLASH_RATE_HZ || trgRate > XL_MAX_FLASH_RATE_HZ)
        return -1;

    if (xlampTrgRate_ != trgRate)
    {
        xlampTrgRate_ = trgRate;

        bool xlampWasOn = xlampIsOn_;
        if (xlampIsOn_)
            xlampOff(false);

        // recalculate params and update the DAC
        setXLampParamsForTrgRate();

        // write all parameters to EEPROM
        EEPROM.put(EEPROM_XL_BRIGHTNESS, xlampBrightness_);
        EEPROM.put(EEPROM_XL_VOLTAGE, xlampVoltage_);
        EEPROM.put(EEPROM_XL_TRG_RATE, xlampTrgRate_);

        // update DAC
        setupDAC();

        if (xlampWasOn)
            xlampOn();
    }

    return 0;
}

// Sets the lamp max power (Watts) directly. Be very careful if it exceeds
// manufacturer's specified one
int xlampSetMaxLampPower(String paramStr)
{
    int xlampMaxPower = -1;

    // only allow change when lamp is not running
    // or running in untimed mode
    if (xlampIsOn_ && exposureTime_ > 0)
        return -1;

    // all uppercase
    paramStr.trim().toUpperCase();

    if (paramStr.length() == 0)
        return -1;

    // parse the brightness string
    if (paramStr.equals("MAX"))
        xlampMaxPower = XL_MAX_POWER_WT;
    else if (paramStr.equals("RATED") || paramStr.length()==0)
        xlampMaxPower = XL_RATED_POWER_WT;
    else
        xlampMaxPower = paramStr.toInt();

    if (xlampMaxPower <=0 || xlampMaxPower > XL_MAX_POWER_WT)
        return -1;

    if (xlampMaxPower_ != xlampMaxPower)
    {
        xlampMaxPower_ = xlampMaxPower;

        bool xlampWasOn = xlampIsOn_;
        if (xlampIsOn_)
            xlampOff(false);

        // recalculate params and update the DAC
        setXLampParamsForBrightness();

        // write all parameters to EEPROM
        EEPROM.put(EEPROM_XL_MAX_POWER, xlampMaxPower_);
        EEPROM.put(EEPROM_XL_BRIGHTNESS, xlampBrightness_);
        EEPROM.put(EEPROM_XL_VOLTAGE, xlampVoltage_);
        EEPROM.put(EEPROM_XL_TRG_RATE, xlampTrgRate_);

        // update DAC
        setupDAC();

        if (xlampWasOn)
            xlampOn();
    }

    return 0;
}

int xlampTrigger(String paramStr)
{
    paramStr.trim().toUpperCase();

    if (paramStr.length() == 0)
        return -1;

    // parse the string
    if (paramStr.equals("ON"))
        xlampOn();
    else if (paramStr.equals("OFF"))
        xlampOff(false);
    else
        return -1;

    return 0;
}

// main firmware initialisation
void setup()
{
    // setup remaining pins
    pinSetFast(DAC_CLEAR);  // remove !CLR signal
    pinSetFast(ENABLE_REF); // enable REF - power supply sequencing for DAC

    // initialise SPI
    SPI.begin(DAC_SPI_SS);
    SPI.setClockSpeed(1, MHZ);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    pinSetFast(DAC_SPI_SS);     // active low

    // set the DAC
    setupDAC();

    // register Particle functions
    bool initSuccess =           Particle.function("XLSetExpTime", xlampSetExposureTime);
    initSuccess = initSuccess && Particle.function("XLSetBrghtns", xlampSetBrightness);
    initSuccess = initSuccess && Particle.function("XLSetVoltage", xlampSetVoltage);
    initSuccess = initSuccess && Particle.function("XLSetTrgRate", xlampSetTriggerRate);
    initSuccess = initSuccess && Particle.function("XLSetMaxPowr", xlampSetMaxLampPower);
    initSuccess = initSuccess && Particle.function("XLTrigger",    xlampTrigger);

    // register Particle variables
    Particle.variable("BOARD_TYPE",   BOARD_TYPE);
    Particle.variable("XLMaxPower",   xlampMaxPower_);
    Particle.variable("XLExpTime",    exposureTime_);
    Particle.variable("XLBrightness", xlampBrightness_);
    Particle.variable("XLVoltage",    xlampVoltage_);
    Particle.variable("XLFlashRate",  xlampTrgRate_);
}

// Main event loop - nothing to do here
void loop(void)
{
}
