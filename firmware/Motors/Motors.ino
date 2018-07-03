/*
 *  Motors.ino - Spectron firmware motor control main file.
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

SYSTEM_THREAD(ENABLED);

#include "DRV8884.h"

#define NFAULT      D2
#define DECAY       DAC
#define TRQ         RX
#define M1          WKP
#define M0          A5
#define DIR         A4
#define STEP        A3
#define ENABLE      A2
#define NSLEEP      A1
#define PREF        A0

#define DOWN_CLK    D0
#define UP_CLK      D1

// Motor board DRV8884 driver - can be only one per application
DRV8884 motor(NFAULT,
              DECAY,
              TRQ,
              M0,
              M1,
              DIR,
              STEP,
              ENABLE,
              NSLEEP,
              PREF,
              UP_CLK,
              DOWN_CLK);

// Board type identifier
static String BOARD_TYPE = "SPEC2_MOTOR";

// initialisation success
bool initSuccess = true;

// early init
void initDRV8884()
{
    pinMode(ENABLE, OUTPUT);
    pinMode(NSLEEP, OUTPUT);
    pinResetFast(ENABLE);
    pinResetFast(NSLEEP);
}

// early startup i
STARTUP(initDRV8884());

// cloud functions
int drvMoveToPosition(String paramStr)
{
    paramStr.trim().toUpperCase();

    if (motor.isRunning() || paramStr.length() == 0)
        return -1;

    int movePositions = 0;
    bool forced = false;

    // parse the string
    if (paramStr.equals("START"))
        movePositions = motor.getMinPos() - motor.getCurPos();
    else if (paramStr.equals("END"))
        movePositions = motor.getMaxPos() - motor.getCurPos();
    else
    {
        forced = paramStr.endsWith(",FORCE");
        movePositions = paramStr.toInt();
        if (paramStr.charAt(0) != '+' && paramStr.charAt(0) != '-')
            // we have absolute movement - convert to relative
            movePositions -= motor.getCurPos();
    }

    if (movePositions == 0)
        return -1;
        
    if (movePositions < 0)
        motor.setDirection(DIR_REVERSE);
    else
        motor.setDirection(DIR_FORWARD);

    motor.movePositions(abs(movePositions), forced);

    return 0;
}

int drvSetStepsPerPosition(String stepsPerPositionStr)
{
   if (motor.isRunning())
        return -1;
    
    int32_t stepsPerPosition = stepsPerPositionStr.toInt();
    motor.setStepsPerPosition(stepsPerPosition);

    return 0;
}

int drvSetLimits(String limitsStr)
{
    if (motor.isRunning())
        return -1;

    // parse the string
    int sepIdx = limitsStr.indexOf(',');
    if (sepIdx <= 0 || sepIdx+1 == limitsStr.length())
        return -1;

    int minPos = limitsStr.substring(0, sepIdx).toInt();
    int maxPos = limitsStr.substring(sepIdx+1).toInt();

    motor.setLimits(minPos, maxPos);

    return 0;
}

int drvResetPos(String newPosStr)
{
    if (motor.isRunning())
        return -1;
    
    int32_t newPos = newPosStr.toInt();
    motor.resetPosition(newPos);

    return 0;
}

int drvSetDecay(String decayStr)
{
    int32_t decay = decayStr.toInt();

    if (motor.isRunning() || decay>3 || decay<0)
        return -1;

    motor.setDecayMode((decay_t)decay);

    return 0;
}

int drvSetSteppingMode(String stepModeStr)
{
    int32_t stepMode = stepModeStr.toInt();

    if (motor.isRunning() || stepMode>6 || stepMode<0 || stepMode==3)
        return -1;

    motor.setSteppingMode((step_t)stepMode);

    return 0;
}

int drvSetRotationSpeed(String stepsPerSecStr)
{
    int32_t stepsPerSec = stepsPerSecStr.toInt();

    if (motor.isRunning())
        return -1;

    if (motor.setRotationSpeed(stepsPerSec))
        return 0;
        
    return -1;
}

int drvSetTorqueMode(String torqueModeStr)
{
    int32_t torqueMode = torqueModeStr.toInt();

    if (motor.isRunning() || torqueMode>2 || torqueMode<0)
        return -1;

    motor.setTorque((torque_t)torqueMode);

    return 0;
}

// main firmware initialisation
void setup()
{
    // initialise spectrometer
    motor.begin();

    // register Particle variable
    bool initSuccess = Particle.variable("BOARD_TYPE", BOARD_TYPE);

    // register Particle functions
    initSuccess = initSuccess && Particle.function("drvMoveToPos", drvMoveToPosition);
    initSuccess = initSuccess && Particle.function("drvSetStPPos", drvSetStepsPerPosition);
    initSuccess = initSuccess && Particle.function("drvSetLmts",   drvSetLimits);
    initSuccess = initSuccess && Particle.function("drvResetPos",  drvResetPos);
    initSuccess = initSuccess && Particle.function("drvSetDecay",  drvSetDecay);
    initSuccess = initSuccess && Particle.function("drvSetStpMd",  drvSetSteppingMode);
    initSuccess = initSuccess && Particle.function("drvSetRotSpd", drvSetRotationSpeed);
    initSuccess = initSuccess && Particle.function("drvSetTrqMod", drvSetTorqueMode);
}

// Main event loop - nothing to do here
void loop(void)
{
}
