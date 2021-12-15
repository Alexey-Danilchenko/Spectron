# Practical application of camera sensor spectral response function

The goal here is to obtain data that would allow us to compute an ad hoc (flexible, actual scene- and light-tailored, adjustable during the raw conversion process) colour transform between raw RGB and perceived colour, based directly on the spectral properties of the light in the scene and the sensor spectral sensitivity.

The popular approach to camera colour profiling is to shoot a target, and, knowing the colours on the target, to compute a transform between those colours and the corresponding values in the digital capture.

Problems with the profiling that is based on shooting a target include:
- minimizing illumination non-uniformity across the target (in terms of intensity / white balance)
- minimizing stray light
- minimizing (colour) reflexes from the surroundings
- minimizing flare and glare introduced by the lighting and the lens
- minimizing lens vignetting
- minimizing digital vignetting / angular dependencies introduced by the sensor
- optimizing exposure to avoid noisy and non-linear areas on the sensor characteristic curves
- minimizing the target reference data uncertainty
- a limited number of patches on the target, and the limited real-world-adequacy of those patches’ spectral distributions
- dependency of the resulting profile on the shooting light spectrum is very problematic, given the diversity of the light sources used for actual shooting
- limited quality and accuracy of chromatic adaptation in a 3D colour domain when attempting to adapt the profile to the light in an actual shot (especially when the actual light sources don't have smooth spectra, like it is with LEDs and fluorescent light sources, or, in general, when the light used to shoot the target and the light in the actual shot are far apart spectrally / CCT-wise)

In summary, profiling based on shooting a target has rather limited applicability as it depends on many factors, some of which are hard / costly to control, while some, mostly the chromatic adaptation, may not be solvable at all.

To overcome these limitations we will be using the basic physical property of the sensor - spectral sensitivity (it has, by the way, its own EXIF tag, 34852 / 0x8824, which we never saw to be used). Spectral sensitivity, measured once, can be used to create colour transforms without any target shots, on the fly, and those transforms can be tailored to the light in an actual shot.

## Terminology

**SPD** - _spectral power distribution_, a graph / histogram (often a table) of wavelengths and corresponding intensities (energy levels). SPD characterizes emitted or reflected light. SPD of the light falling on a sensor / receptor (human eye) causes sensor / receptor excitation.

Sensor excitation results in raw RGB response in the form of raw RGB data.

Eye excitation results in the sensation of colour, that is in human colour perception. Human response can be represented as CIE XYZ. (Actually, we can use the LMS responsivity spectra of human perception instead of XYZ.)

**SRF** - _spectral response function_ (aka spectral sensitivity). Here it is used to predict the sensor raw RGB response when the sensor is excited by a light of the given wavelength falling on it.

SRF for a sensor is much like an observer colour matching functions for human perception; in both cases responses are additive, that is the total response to an SPD falling on the sensor / receptor is the sum of responses to individual pairs 'wavelength - intensity'.

We will use **\<cam observer\>** for SRFs.

## Computation method

To compute a colour transform using an SRF, first we need to define some excitation and determine both the human response (XYZ) and the camera response (raw RGB) for this excitation.

We will be using \<cam observer\> over a simulated excitation SPD in order to obtain a set of raw RGB data. Such simulation offers at least 3 benefits:
- any synthetic calibration target can be used, including targets that are nearly impossible to create physically and shoot; all one needs is spectral measurements of objects - colours in Nature (leaves, flowers, ...), various skin tones, etc. - and now there in no target properties uncertainty or need in exposure optimization;
- since the light is virtual, each synthetic calibration target "patch" is under the same light SPD (no angular dependencies, no intensity variations, no white balance variations, no shading, no colour reflexes from the surroundings, no lens / digital vignetting, etc.);
- we can calculate light-in-the-scene tailored colour transforms "on the fly" by interactively adjusting the light SPD while rendering raw, or using measured light SPD to match the light in the actual photographed scene (instead of applying chromatic adaptation on top of a colour transform or interpolating between the colour transforms in a reduced, compared to spectra, 3D colorimetric space).


### Inputs
- simulated spectral excitation - some set of spectral data that excites the receptors (the sensor and the eye). It can be a product of some reference spectral reflection data and some light SPD, or SPDs of a multitude of coloured lights / filtered lights
- sensor SRF (camera observer)
- the human perception (CIE XYZ data) of that spectral data (to be determined using the standard colorimetry formulae)
- the camera perception (raw RGB) of that spectral data (predicted trough the sensor SRF / camera observer)

### End result
- a colour transform between raw RGB and XYZ colorimetric space, can be a 3x3 matrix, can be something more complex

### Let
- *\<scene light SPD\>* be a known / asserted light in a scene for which we want to calculate the colour transform
- *\<calibration target sd\>* be spectral reflection data of a real / synthetic calibration target
- *\<human observer\>* be the observer used to render \<calibration target sd\> to XYZ

To determine the raw RGB, we will be rendering the sensor excitation through the camera observer.

XYZ (human perception) is:

> *\<calibration target XYZ\>* = *\<scene light SPD\>* * *\<calibration target sd\>* * *\<human observer\>*

From the camera side, with *\<calibration target sd\>* in \[0 .. 1\] domain

> *\<simulated sensor excitation\>* = *\<calibration target sd\>* * *\<scene light SPD\>*

By definition

> *\<sensor excitation\>* * *\<cam observer\>* = *\<raw RGB\>*

Thus

> *\<simulated raw RGB\>* = *\<simulated sensor excitation\>* * *\<cam observer\>*

Having *\<calibration target XYZ\>* and *\<simulated raw RGB\>* we can proceed to calculate a colour transform.

