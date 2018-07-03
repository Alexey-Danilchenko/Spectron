/*
 *  Spectron_LCD.ino - Demo application of the Spectron board capabilities  
 *                     with Adafruit ILI9340 TFT screen displaying measured 
 *                     spectrum. This is not actively maintained.
 *
 *  Copyright 2015-2017 Alexey Danilchenko
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

#include "C12666MA.h"
#include "Adafruit_ILI9340.h"

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

// calibration data for my sensor 15F00163
const float CALIBRATION_15F00163[] =
    {323.3668711, 2.384682045, -5.995865297E-4, -8.602293347E-6, 1.840343099E-8, -1.424592223E-11};

// spectrometer object - can be only one per application
C12666MA spec(TRG_3V,
              EOS_3V,
              GAIN_3V,
              CLK_3V,
              ST_3V,
              TRG_CAMERA,
              ADC_REF_SEL_1,
              ADC_REF_SEL_2,
              ADC_CNV,
              CALIBRATION_15F00163);

// TFT screen object
Adafruit_ILI9340 tft(LCD_SD_CS,
                     LCD_DC,
                     LCD_RESET);

// Wavelength indexes for screen 400,450,500,550,600,650,700,750
#define WV_LABELS      10
#define WV_LABEL_START 350
#define WV_LABEL_STEP  50
uint16_t wvLabelIndex[WV_LABELS];

// indicator if black was measured
bool blackMeasured = true;

// last active time
unsigned long lastActive = 0;

// draw the main screen
void drawScreen()
{
    tft.3Init();
    tft.fillScreen(ILI9340_BLACK);
    tft.setRotation(3);    // orientation landscape

    // draw header
    tft.setCursor(0, 0);
    tft.setTextColor(ILI9340_WHITE);
    tft.setTextSize(1);
    tft.print("Peak: ");
    tft.print(spec.getPeakWavelength(), 2);
    tft.print("nm    Max: ");
    tft.print(spec.getPeakValue());
    tft.print("    Black: ");
    tft.print(spec.getMaxBlack());

    // draw axis
    uint16_t g_x = 0, g_y = 10, g_w = 320, g_h = 220;
    uint16_t color = ILI9340_WHITE;
    tft.drawFastHLine(0, g_y+g_h, g_w, color);
    uint16_t lblWavelength = WV_LABEL_START;
    for (int i=0; i<WV_LABELS; i++)
    {
        if (wvLabelIndex[i] == SPEC_PIXELS)
            break;
        uint16_t l_x = g_x + (wvLabelIndex[i] * g_w / SPEC_PIXELS);
        tft.drawFastVLine(l_x, g_y+g_h-1, 3, color);
        tft.setCursor(l_x - 8, g_y+g_h+2);
        tft.print(lblWavelength);
        lblWavelength += WV_LABEL_STEP;
    }

    // draw the maxvoltage
    // draw spectrum
    if (spec.hasMeasuredData())
    {
        uint16_t x0 = g_x;
        uint16_t y0 = g_y + g_h - (((uint32_t)g_h*spec.getMeasurement(0) + 16383) /16384);
        for (int i=1; i<SPEC_PIXELS; i++)
        {
            uint16_t x1 = g_x + ((i * g_w + SPEC_PIXELS - 1) / SPEC_PIXELS);
            uint16_t y1 = g_y + g_h - (((uint32_t)g_h*spec.getMeasurement(i) + 16383) /16384);
            tft.drawLine(x0, y0, x1, y1, ILI9340_YELLOW);
            x0 = x1;
            y0 = y1;
        }
    }
    else if (!blackMeasured)
    {
        tft.setTextSize(2);
        tft.setCursor(g_x, g_y + (g_h>>1)- 32);
        tft.setTextColor(ILI9340_RED);
        tft.println("    Cover the sensor and");
        tft.println("  trigger black reference");
        tft.println("       measurement");
    }
}

// main firmware initialisation
void setup()
{
    pinMode(LCD_BACKLIGHT, OUTPUT);
    pinMode(LCD_CS,        OUTPUT);
    pinMode(LCD_SD_CS,     OUTPUT);
    pinMode(LCD_DC,        OUTPUT);
    pinMode(LCD_RESET,     OUTPUT);

    pinMode(TRG_IN,     INPUT_PULLUP);
    pinMode(TRG_CAMERA, OUTPUT);
    pinMode(TRG_FLASH,  OUTPUT);

    // reset everything
    pinResetFast(LCD_CS);
    pinResetFast(LCD_SD_CS);
    pinResetFast(LCD_DC);
    pinResetFast(LCD_RESET);


    // initialise spectrometer
    spec.begin();
    spec.setIntTime(500);   // 0.5s to start with
    spec.setAdcReference(ADC_4V);

    // get the wavelength label indexes
    uint16_t idx = 0;
    for (int i=0; i<WV_LABELS; i++)
    {
        uint16_t lblWavelength = WV_LABEL_START + (i * WV_LABEL_STEP);
        while (idx < SPEC_PIXELS &&
               lblWavelength > spec.getWavelength(idx))
               ++idx;
        if (idx < SPEC_PIXELS)
            wvLabelIndex[i] = idx;
        else
            wvLabelIndex[i] = SPEC_PIXELS;
    }

    // enable TFT backlight
    pinSetFast(LCD_BACKLIGHT);
    tft.begin();

    // paint the screen
    drawScreen();

    lastActive = millis();
}

// Main event loop:
//     - Wait for trigger button press
//     - Start spectometer measurement and trigger camera/flash
//     - Display the results on TFT
//     - Store results on SD card
void loop(void)
{
    // poll the trigger button
    if (pinReadFast(TRG_IN) == LOW)
    {
        // debounce check
        delay(100);
        if (pinReadFast(TRG_IN) == LOW)
        {
            pinSetFast(LCD_BACKLIGHT);
            lastActive = millis();
            
            // initiate measurement
            spec.takeMeasurement();

            // paint the screen
            drawScreen();
        }
    }

    // check for inactivity
    if ((millis() - lastActive) > INACTIVE_DELAY_MS)
        pinResetFast(LCD_BACKLIGHT);

    // call for Spark process for manual system mode
    if (Particle.connected())
        Particle.process();
    else
        Particle.connect();
 }
