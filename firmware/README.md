# Firmware

This contains all the firmware (and extra example) running on the hardware boards used in this project.

All main board components (Hamamatsu [C12666MA](http://www.hamamatsu.com/jp/en/C12666MA.html), [C12880MA](http://www.hamamatsu.com/jp/en/C12880MA.html) and [TI DRV8884](http://www.ti.com/product/DRV8884)) are implemented as standalone driver objects. This makes it easier to some degree to re-use them on other Particle Photon based projects.

# Spectron 2 Board Fimwares

The main components of the boards are Hamamatsu [C12666MA](http://www.hamamatsu.com/jp/en/C12666MA.html) or [C12880MA](http://www.hamamatsu.com/jp/en/C12880MA.html) spectrometers. Whilst these spectrometer sensors share a lot of common features, operationally they are quite different. A separate firmwares therefore were produced for each spectrometer type.

The spectrometers are driven by hardware timer clock in both cases. The spectrometers readouts are performed via external 16bit SPI based ADC. The board also has four selectable precision voltage references (2.5V, 3.0V, 4.096V and 5V) to allow more precise ADC range fitting to the spectrometer output.

For the purpose of the measurements, the board provides two external trigger pins (only one can be selected and used at a time). The spectrometer driver makes use of the selected trigger pin at the time of the measurement (the trigger signal can be offset and issued prior to the measurement with configurable delay). For the goals of this project, these pins are labelled `TRG_FLASH` and `TRG_CAMERA` in the code for flash and digital camera triggering accordingly.

The spectrometer drivers also have configurable black subtraction mode that could be used to eliminate spectrometer offset voltage (that appears to be nonlinear). This can be configured in one of the following ways:

- to capture black frame output manually 
- to capture black frame output before the main measurement
- to capture black frame output after the main measurement

When black levels are captured the spectrometer readings are adjusted accordingly.

The selected reference voltage, gain (if supported) and black mode are stored in EEPROM and persist even when the board is powered off.

Calibration data provided by Hamamatsu for each sensor (used to determine wavelengths mapping for each sensor) needs to be provided to the driver to get accurate readings. If omitted, the calibration data from my own sensors is used as a fall-back.

All the readings are captured as floating point values in 0..1 range. By default, readings from the driver are also bandpass corrected (this is configurable) using Stearns and Stearns (1988) bandpass correction method.


Below are individual specifics of each firmware.

## Spectron 2 - С12666MA firmware

[This firmware](Spectron_12666) implements the functionality for the Spectron2 board with Hamamatsu [C12666MA](http://www.hamamatsu.com/jp/en/C12666MA.html) spectrometer connected (using appropriate sensor board). 

The spectrometer is driven by hardware clock with 100KHz (default) or 59KHz frequency. In both modes sensor readouts are oversampled to provide more stable readings. The slower clock is enabled by uncommenting `ADC_AVG_4` definition. The specifics of each clock frequency:

* 100KHz: integration times: 11.3msec - 10sec, oversampling with two averaging reads per spectrometer pixel

* 59KHz: integration times: 18.51msec - 10sec, oversampling with four averaging reads per spectrometer pixel

In addition to selectable voltage references, the firmware can enable/disable spectrometer supported gain. This may allow even tighter usage of ADC range.

The firmware is implemented partially outside of Particle Photon HAL - using direct hardware and ports access for performance critical parts (timer, soft pin interrupts, ADC readouts). The readouts are triggered by software pin interrupt - C12666MA (unlike C12880MA) lacks hardware support for signal ready trigger.

## Spectron 2 - С12880MA firmware

[This firmware](Spectron_12880) implements the functionality for the Spectron2 board with Hamamatsu [C12880MA](http://www.hamamatsu.com/jp/en/C12880MA.html) spectrometer connected (using appropriate sensor board). 

The spectrometer is driven by hardware clock with 156KHz (default) or 200KHz frequency. In 156KHz frequency mode sensor readouts are oversampled to provide more stable and smoother readings. The specifics of each clock frequency mode:

* 100KHz: enabled by setting `SPEC_CLK_TICK_TIMER` definition to `SPEC_CLK_156KHZ`, integration times: 345.6usec - 1sec, oversampling with two averaging reads per spectrometer pixel, produces much smoother output, works well even though it is below minimal Hamamatsu specified working frequency

* 200KHz: enabled by setting `SPEC_CLK_TICK_TIMER` definition to `SPEC_CLK_200KHZ`, integration times: 270usec - 1sec, no oversampling with single read per spectrometer pixel, minimal official working frequency for C12880MA according to Hamamatsu specification

The C12880MA does not support gain but is a lot more sensitive than C12666MA. Using selectable reference voltages will allow better use of ADC range.

The firmware is implemented substantially outside of Particle Photon HAL - using direct hardware and ports access for performance critical parts (GPIO pin access, timer, pin interrupts, ADC readouts). The readouts are triggered by C12880MA hardware TRG pin which allows more reliable read timings.

## Spectron 2 - LCD demo firmware

[This firmware](LCD) demonstrates usage of the Spectron 2 board with Hamamatsu C12666MA spectrometer and multipurpose interface connector with [Adafruit 2.2" 18-bit TFT LCD module](https://www.adafruit.com/product/1480) attached. The C12666MA driver should be taken from [C12666 firmware](Spectron_12666) (this avoid the duplication). The firmware will wait for trigger pin (button attached to it) to be risen high, start spectral measurement of predefined integration time and display the spectrum on the TFT screen afterwards.

This code was tested on early stages of the project development but is not actively maintained. Its main purpose if to provide a demonstration of Spectron 2 board capabilities.


# Motor Board Firmware

[This firmware](Motors) implements functionality for the Spectron 2 Motors board that is operating stepper motor and rotary encoder to control monochromator wavelength selection. The [TI DRV8884](http://www.ti.com/product/DRV8884) is used to drive stepper motor and [LS7083](http://www.omnipro.net/lsi-csi/LS7083-S) quadrature converter chip to interface with rotary encoder. 

The DRV8884 allows to fine tune stepper parameters to suit each and every application. All of these stepper parameters as well as position details are stored in EEPROM and persist even when the board is powered off. The expectations are that these particular boards will be tuned up (for example using Particle cloud functions via Particle console) for a specific application and all the board parameters will be set up and stored at that stage.

The firmware is designed to operate in terms of target device position and its lower and upper limits. The logical po
sition is mapped into physical steps for controlling stepper. All position parameters (current position, lower and upper position limits) are also persisted in the EEPROM. For this specific application, current position reflects wavelength selected on monochromator with upper and lower limits designating the spectral range that monochromator can control.


# LED Light Source

[This firmware](LED) implements functionality for Spectron 2 LED Lightsource board using [Yuji VTC 5600K wide spectrum LEDs](https://store.yujiintl.com/collections/vtc-series/products/vtc-series-high-cri-cob-led-135l-pack-5-pcs?variant=45426214099). 

The firmware allows controlling the output brigntness of two LED channels independently (each channel up to 4095 brightness levels) or simultaneously, enabling/disabling LED output as well as doing timed exposures.

It is used as a main light source for Spectron 2 project. 


# Pulsed Xenon Light Source

[This firmware](Xenon) implements functionality for Spectron 2 Xenon Lightsource board using [Excelitas FX-116x series lamps](http://www.excelitas.com/Pages/Product/1100-Series-Family.aspx) on [Excelitas/Perkin-Elmer trigger module FYD‐1150‐B](http://www.excelitas.com/Downloads/DTS_1100Series_Trigger_Modules.pdf) driven by [Excelitas/Perkin-Elmer PS-11xx power supply](http://www.excelitas.com/Downloads/DTS_1100Series_Power_Supplies.pdf). 

The firmware allows controlling the output brigntness (average output power) of the flash lamp, or varying voltage or trigger rate directly (within allowed lamp  specifications), enabling/disabling Xenon Lamp output as well as doing timed exposures.


