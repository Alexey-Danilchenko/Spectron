# Hardware

All the custom made circuits and boards were designed in [Cadsoft Eagle](http://www.autodesk.com/products/eagle/overview). All components for these were separated into standalone Eagle library for easier access. 

__Note:__ All the boards were tuned up for making with [OSH Park](https://oshpark.com/) - all relevant DRC and ERC rule checks were therefore applied an validated.

This project has the following several distinct and standalone components:

* [Spectrometer](Spectrometer) - board controlling spectrometer and performing spectral measurements
* [Stepper Motor Control](Motor) - board driving stepper motor for automating monochromator operation
* [Light Sources](Light_Source) - selection of controllable light sources providing input for monochromator
