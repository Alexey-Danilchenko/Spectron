# Spectron

Spectron - an open source project for measuring and obtaining digital devices spectral sensitivity curves

Copyright (C) 2014-2018 [Alexey Danilchenko](mailto:alexey.danilchenko@gmail.com), [Iliah Borg](mailto:iliah.i.borg@gmail.com)

*This is a work in progress*

# Introduction

This is the second generation of the open source Spectron project.  Initially this was conceived with the goal of constructing an automated device with an aid to accurately measure spectral sensitivity curves for digital camera sensors. The project has a broader use though and can be used to measure spectral sensitivity curves of various light sensitive sensors (not just camera sensors) - for example spectral sensitivity curves of photodiodes.

The main goal however is still to aid measurement of spectral sensitivity of digital cameras sensors and this is what this project and documentation will be aimed towards. For more details on the application of that and the ultimate reasons _why_ please [read this](CameraSRF.md).

The second generation of Spectron board was produced with the more accurate spectral readings in mind, better analogue circuitry to drive Hamamatsu spectrometers and more stable measurement results.


# Concept

The spectral sensitivity measurements of a selected device are fairly simple in principle:

1. broadband light source of high intensity is passed via monochromator
2. monochromator output is fed to the integrating sphere 
3. integrating sphere output is simultaneously captured by target device and measured spectrally
4. steps 1-3 repeated through selected spectral range in discrete steps (5nm steps over 380-720nm range for example)
5. the results of measurements from target device are scaled accodringly to the spectra of the light measured at each step and target spectral sensitivity curves are produced

# Hardware

The hardware for the above concept consists of the light source (LED or pulsed  Xenon), Mini-Chrom MC1-02 monochromator, 2.5" integrating sphere and several distinct components. The Spectrometer component is a hardware driving Hamamatsu [C12666MA](http://www.hamamatsu.com/jp/en/C12666MA.html) or [C12880MA](http://www.hamamatsu.com/jp/en/C12880MA.html) spectrometer sensor to capture spectral readings from the integrating sphere and initiate synchronised readings from target device. The Stepper Motors component is a hardware for automating monochromator operation (esentially converting manually controlled monochromator to scanning monochromator, turning broadband light source into tunable source). The Light Source component is a hardware for controlling broadband light source that provides input for the monochromator.

With the precise intensity control of the LED light source, the whole system is essentially providing tunable monochromatic light source covering the entire visible range.

All hardware component boards use [Particle Photon controllers](https://docs.particle.io/datasheets/wi-fi/photon-datasheet/) at their core to drive the functionality. Overall application control is performed from Windows/Mac application running on a computer operating the boards via Particle or local cloud (the latter is more responsive).

Boards used for all individual components can be of course utilised for standalone projects - see corresponding sections for details.

For more details please refer to individual parts of the project below:

* [Firmware](firmware) - [Particle Photon](https://www.particle.io/products/hardware/photon-wifi-dev-kit) code running on the boards
* [Hardware](hardware) - boards, schematics (Eagle) and BOM 
* [Software](software) - Windows/MacOS X application controlling the overall functionality

# Open Source

This project is open source software and hardware and is released under:

* [GNU General Public License v3](https://www.gnu.org/licenses/gpl-3.0.en.html) for the software parts and 
* [CERN Open Hardware Licence]( http://www.ohwr.org/attachments/735/CERNOHLv1_1.txt) for hardware parts
