/***************************************************
This is an Arduino Library for the Adafruit 2.2" SPI display.
This library works with the Adafruit 2.2" TFT Breakout w/SD card
----> http://www.adafruit.com/products/1480

Check out the links above for our tutorials and wiring diagrams
These displays use SPI to communicate, 4 or 5 pins are required to
interface (RST is optional)
Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
MIT license, all text above must be included in any redistribution
****************************************************/

#ifndef ADAFRUIT_ILI9340_H
#define ADAFRUIT_ILI9340_H

// Hack to get this to work in Spark IDE
#include "Adafruit_mfGFX.h"

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))

#define pinLO(_pin)    (pinResetFast(_pin))
#define pinHI(_pin)    (pinSetFast(_pin))
#define inline inline __attribute__((always_inline))

//typedef unsigned char prog_uchar;

#define ILI9340_TFTWIDTH  240
#define ILI9340_TFTHEIGHT 320

#define ILI9340_NOP     0x00
#define ILI9340_SWRESET 0x01
#define ILI9340_RDDID   0x04
#define ILI9340_RDDST   0x09

#define ILI9340_SLPIN   0x10
#define ILI9340_SLPOUT  0x11
#define ILI9340_PTLON   0x12
#define ILI9340_NORON   0x13

#define ILI9340_RDMODE  0x0A
#define ILI9340_RDMADCTL  0x0B
#define ILI9340_RDPIXFMT  0x0C
#define ILI9340_RDIMGFMT  0x0A
#define ILI9340_RDSELFDIAG  0x0F

#define ILI9340_INVOFF  0x20
#define ILI9340_INVON   0x21
#define ILI9340_GAMMASET 0x26
#define ILI9340_DISPOFF 0x28
#define ILI9340_DISPON  0x29

#define ILI9340_CASET   0x2A
#define ILI9340_PASET   0x2B
#define ILI9340_RAMWR   0x2C
#define ILI9340_RAMRD   0x2E

#define ILI9340_PTLAR   0x30
#define ILI9340_MADCTL  0x36


#define ILI9340_MADCTL_MY  0x80
#define ILI9340_MADCTL_MX  0x40
#define ILI9340_MADCTL_MV  0x20
#define ILI9340_MADCTL_ML  0x10
#define ILI9340_MADCTL_RGB 0x00
#define ILI9340_MADCTL_BGR 0x08
#define ILI9340_MADCTL_MH  0x04

#define ILI9340_PIXFMT  0x3A

#define ILI9340_FRMCTR1 0xB1
#define ILI9340_FRMCTR2 0xB2
#define ILI9340_FRMCTR3 0xB3
#define ILI9340_INVCTR  0xB4
#define ILI9340_DFUNCTR 0xB6

#define ILI9340_PWCTR1  0xC0
#define ILI9340_PWCTR2  0xC1
#define ILI9340_PWCTR3  0xC2
#define ILI9340_PWCTR4  0xC3
#define ILI9340_PWCTR5  0xC4
#define ILI9340_VMCTR1  0xC5
#define ILI9340_VMCTR2  0xC7

#define ILI9340_RDID1   0xDA
#define ILI9340_RDID2   0xDB
#define ILI9340_RDID3   0xDC
#define ILI9340_RDID4   0xDD

#define ILI9340_GMCTRP1 0xE0
#define ILI9340_GMCTRN1 0xE1

// Color definitions
#define    ILI9340_BLACK   0x0000
#define    ILI9340_BLUE    0x001F
#define    ILI9340_RED     0xF800
#define    ILI9340_GREEN   0x07E0
#define ILI9340_CYAN    0x07FF
#define ILI9340_MAGENTA 0xF81F
#define ILI9340_YELLOW  0xFFE0
#define ILI9340_WHITE   0xFFFF


class Adafruit_ILI9340 : public Adafruit_GFX {

public:
    Adafruit_ILI9340(uint8_t cs, uint8_t dc, uint8_t rst);

    void begin(void);
    void spiInit(void);
    void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void pushColor(uint16_t color);
    void fillScreen(uint16_t color);
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
    void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void setRotation(uint8_t r);
    void invertDisplay(boolean i);

    uint16_t Color565(uint8_t r, uint8_t g, uint8_t b);

    /* These are not for current use, 8-bit protocol only! */
    uint8_t  readdata(void), readcommand8(uint8_t);

    void spiwrite(uint8_t);
    void writecommand(uint8_t c);
    void writedata(uint8_t d);
    void commandList(uint8_t *addr);
    
    uint8_t  spiread(void);

private:

    uint8_t  tabcolor;

    volatile uint8_t *mosiport, *clkport, *dcport, *rsport, *csport;

    uint8_t  _cs, _dc, _rst, _mosi, _miso, _sclk;
};

#endif
