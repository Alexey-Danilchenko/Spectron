/*
 *  LED.ino - Spectron LED light source control main file.
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

#define ENABLE_1       A1
#define PWM_1          A4
#define OPENLED_1      D6
#define SHORTLED_1     D5

#define ENABLE_2       A0
#define PWM_2          D2
#define OPENLED_2      D3
#define SHORTLED_2     D4

#define FAN_PWM        WKP
#define EN_FAN         A3

#define ONE_WIRE_DATA  A2

#define MAX_BRIGHTNESS      4095
#define PWM_FREQUENCY       100

#define MAX_FAN_SPEED       4095
#define FAN_PWM_FREQUENCY   500

// EEPROM addresses
#define EEPROM_CH1_BRIGHTNESS       0
#define EEPROM_CH2_BRIGHTNESS       4

#define EEPROM_FREE_ADDR            8  // free address for application usage

// forward declaration
void expFinished();

// Board type identifier
static String BOARD_TYPE = "SPEC2_LED";

// Variables
static Timer expTimer(2000, expFinished);       // LED exposure timer
static bool ledIsOn_ = false;                   // current led state
static int32_t ch1Brightness_ = MAX_BRIGHTNESS; // channel 1 LED brightness
static int32_t ch2Brightness_ = 0;              // channel 1 LED brightness - off
static int32_t shortLED_ = 0;                   // short LED state
static int32_t openLED_ = 0;                    // open LED state

// early init
void initLED()
{
    pinMode(ENABLE_1,   OUTPUT);
    pinMode(PWM_1,      OUTPUT);
    pinMode(OPENLED_1,  INPUT_PULLUP);
    pinMode(SHORTLED_1, INPUT_PULLUP);

    pinMode(ENABLE_2,   OUTPUT);
    pinMode(PWM_2,      OUTPUT);
    pinMode(OPENLED_2,  INPUT_PULLUP);
    pinMode(SHORTLED_2, INPUT_PULLUP);

    pinMode(FAN_PWM,    OUTPUT);
    pinMode(EN_FAN,     OUTPUT);

    pinResetFast(ENABLE_1);
    pinResetFast(ENABLE_2);
    pinResetFast(EN_FAN);

    analogWriteResolution(FAN_PWM, 12);
    analogWrite(FAN_PWM, 0, FAN_PWM_FREQUENCY);

    analogWriteResolution(PWM_1, 12);
    analogWrite(PWM_1, 0, PWM_FREQUENCY);

    analogWriteResolution(PWM_2, 12);
    analogWrite(PWM_2, 0, PWM_FREQUENCY);

    // read the data from EEPROM
    // read ch1Brightness_ details from EPROM
    EEPROM.get(EEPROM_CH1_BRIGHTNESS, ch1Brightness_);
    if (ch1Brightness_ < 0 || ch1Brightness_ > MAX_BRIGHTNESS)
        // EEPROM was empty
        ch1Brightness_ = MAX_BRIGHTNESS;

    // read ch2Brightness_ details from EPROM
    EEPROM.get(EEPROM_CH2_BRIGHTNESS, ch2Brightness_);
    if (ch2Brightness_ < 0 || ch2Brightness_ > MAX_BRIGHTNESS)
        // EEPROM was empty
        ch2Brightness_ = 0;
}

// early startup
STARTUP(initLED());

// shortled interrupt
void shortLED()
{
    shortLED_ = pinReadFast(SHORTLED_1) == LOW || pinReadFast(SHORTLED_2) == LOW;
}

// shortled interrupt
void openLED()
{
    openLED_ = pinReadFast(OPENLED_1) == LOW || pinReadFast(OPENLED_2) == LOW;
}

// switch off LEDs
void ledOff(bool calledFromISR)
{
    if (ledIsOn_)
    {
        ledIsOn_ = false;
        if (expTimer.isActive())
            if (calledFromISR)
                expTimer.stopFromISR();
            else
                expTimer.stop();
        analogWrite(PWM_1, 0, PWM_FREQUENCY);
        analogWrite(PWM_2, 0, PWM_FREQUENCY);
        analogWrite(FAN_PWM, 0, FAN_PWM_FREQUENCY);
        pinResetFast(ENABLE_1);
        pinResetFast(ENABLE_2);
        pinResetFast(EN_FAN);
    }
}

// timer callback - exposure finished
void expFinished()
{
    ledOff(true);
}

// auxiliary function
int parseBrightnessParam(String paramStr, int defBrightness)
{
    if (paramStr.length() == 0)
        return defBrightness;

    if (paramStr.equals("MAX"))
        return MAX_BRIGHTNESS;

    return paramStr.toInt();
}

// Cloud functions

// switch LEDs on and off
int ledTrigger(String paramStr)
{
    paramStr.trim().toUpperCase();

    if (paramStr.length() == 0)
        return -1;

    // parse the string
    if (paramStr.equals("OFF"))
        ledOff(false);
    else if (!ledIsOn_ &&
             (ch1Brightness_ > 0 || ch2Brightness_ > 0))
    {
        int32_t exposureTime = paramStr.toInt();
        if (paramStr.equals("ON"))
            exposureTime = 0;
        else
        {
            exposureTime = paramStr.toInt();

            if (exposureTime < 0 || exposureTime > 1000000)
                return -1;
        }

        ledIsOn_ = true;
        analogWrite(FAN_PWM, MAX_FAN_SPEED, FAN_PWM_FREQUENCY);
        pinSetFast(EN_FAN);

        if (ch1Brightness_ > 0)
        {
            pinSetFast(ENABLE_1);
            analogWrite(PWM_1, ch1Brightness_, PWM_FREQUENCY);
        }
        if (ch2Brightness_ > 0)
        {
            pinSetFast(ENABLE_2);
            analogWrite(PWM_2, ch2Brightness_, PWM_FREQUENCY);
        }
        if (exposureTime > 0)
        {
            expTimer.changePeriod(exposureTime);
            expTimer.start();
        }
    }

    return 0;
}

//
// Sets the same brightness level (0..4095). Format of the parameter string:
//    <brightness> - sets the same brightness for channel 1 and 2
//    <br1>,<br2>  - sets br1 brightness for channel 1 and br2 for channel2
//                   <br1> and <br2> can be omitted to set only one channel,
//                   for example: ",500" will set only channel 2 brightness to 500
//
int ledSetBrightness(String paramStr)
{
    // all uppercase
    paramStr.trim().toUpperCase();

    if (paramStr.length() == 0)
        return -1;

    // parse the string
    int br1 = -1;
    int br2 = -1;
    int sepIdx = paramStr.indexOf(',');
    if (sepIdx < 0)
    {
        br1 = br2 = parseBrightnessParam(paramStr, -1); // no default
    }
    else
    {
        br1 = parseBrightnessParam(paramStr.substring(0, sepIdx).trim(),
                                   ch1Brightness_);
        br2 = parseBrightnessParam(paramStr.substring(sepIdx+1).trim(),
                                   ch2Brightness_);
    }

    if (br1 < 0 || br1 > MAX_BRIGHTNESS)
        return -1;

    if (br2 < 0 || br2 > MAX_BRIGHTNESS)
        return -1;

    if (ch1Brightness_ != br1)
    {
        ch1Brightness_ = br1;
        EEPROM.put(EEPROM_CH1_BRIGHTNESS, ch1Brightness_);

        if (ledIsOn_)
        {
            analogWrite(PWM_1, ch1Brightness_, PWM_FREQUENCY);

            if (ch1Brightness_ > 0)
                pinSetFast(ENABLE_1);
            else
                pinResetFast(ENABLE_1);
        }
    }

    if (ch2Brightness_ != br2)
    {
        ch2Brightness_ = br2;
        EEPROM.put(EEPROM_CH2_BRIGHTNESS, ch2Brightness_);

        if (ledIsOn_)
        {
            analogWrite(PWM_2, ch2Brightness_, PWM_FREQUENCY);

            if (ch2Brightness_ > 0)
                pinSetFast(ENABLE_2);
            else
                pinResetFast(ENABLE_2);
        }
    }

    return 0;
}

// main firmware initialisation
void setup()
{
    // register interrupts
    attachInterrupt(SHORTLED_1,  shortLED, CHANGE);
    attachInterrupt(SHORTLED_2,  shortLED, CHANGE);
    attachInterrupt(OPENLED_1,   openLED,  CHANGE);
    attachInterrupt(OPENLED_2,   openLED,  CHANGE);

    // register Particle functions
    bool initSuccess =           Particle.function("ledSetBrtns",  ledSetBrightness);
    initSuccess = initSuccess && Particle.function("ledTrigger",   ledTrigger);

    // register Particle variables
    Particle.variable("BOARD_TYPE",   BOARD_TYPE);
    Particle.variable("ledCh1Brtnes", ch1Brightness_);
    Particle.variable("ledCh2Brtnes", ch2Brightness_);
    Particle.variable("ledShort",     shortLED_);
    Particle.variable("ledOpen",      openLED_);
}

// Main event loop - nothing to do here
void loop(void)
{
}
