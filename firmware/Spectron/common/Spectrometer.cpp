/*
 *  Spectrometer.cpp - Hamamatsu spectrometer common driver class for
 *                     Spectron board. This is quite generic and does
 *                     implement high level driver with normalisation
 *                     black subtraction etc. Interfaces to low level
 *                     devices are implemented by low level drivers
 *                     performing readings and hardware interface.
 *
 *  Copyright 2015-2021 Alexey Danilchenko, Iliah Borg
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
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, 51 Franklin Street - Fifth Floor, Boston,
 *  MA 02110-1301, USA.
 */

#include "Spectrometer.h"
#include <algorithm>
#include <cmath>

// Mask to set
#define ADC_REF_MASK 0xF

// Gain indexes
#define NO_GAIN    0
#define HIGH_GAIN  1

// Spec state settings
#define CURRENT
#define LAST_MEAS  true

// Time to usec multipliers
static const double timeUnitsToUsec[MAX_TIME_UNITS] = { 1.0, 1000.0, 1000000.0 };

// ---------------------------
//  Auxiliary local functions
// ---------------------------

// Calculates and returns Tungsten emissivity at given wavelength and
// temperature
//
// For more details refer to R. M. Pon and J. P. Hessler
//     "Spectral emissivity of tungsten: analytic expressions for the
//      340nm to 2.6 um spectral region"
//
// Tungsten Emissivity analytical expression is calculated for
// T=(Temp-2200K)/1000 in kK and wavelength L in micrometers as follows:
//
//    emT(T,L) = a0+a1*T+(b0+b1*T+b2*T*T)*(L-l0)+(c0+c1*T)*(L-l0)*(L-l0)
//
// given the following specification:
//
//    L,nm    l0     a0        a1       b0       b1      b2      c0      c1
//   300-420  380  0.47245  -0.0155  -0.0086  -0.0229  0.0000  -2.860   0.000
//   420-480  450  0.46361  -0.0172  -0.1304   0.0000  0.0000   0.520   0.000
//   480-580  530  0.45549  -0.0173  -0.1150   0.0000  0.0000  -0.500   0.000
//   580-640  610  0.44297  -0.0177  -0.1482   0.0000  0.0000   0.723   0.000
//   640-760  700  0.43151  -0.0207  -0.1441  -0.0551  0.0000  -0.278  -0.190
//   760-940  850  0.40610  -0.0259  -0.1889   0.0087  0.0290  -0.126   0.246
//
double emvTungst(double wvL, double tempK)
{
    // implement this form for reduced calculations
    double T = (tempK-2200.0)/1000;
    double emT = 0.33; // for anything > 940nm
    if (wvL < 420.0)
    {
        wvL = (wvL-380.0)/1000;
        emT = 0.47245-0.0155*T-(0.0086+0.0229*T)*wvL-2.86*wvL*wvL;
    }
    else if (wvL < 480.0)
    {
        wvL = (wvL-450.0)/1000;
        emT = 0.46361-0.0172*T-0.1304*wvL+0.52*wvL*wvL;
    }
    else if (wvL < 580.0)
    {
        wvL = (wvL-530.0)/1000;
        emT = 0.45549-0.0173*T-0.115*wvL-0.5*wvL*wvL;
    }
    else if (wvL < 640.0)
    {
        wvL = (wvL-610.0)/1000;
        emT = 0.44297-0.0177*T-0.1482*wvL+0.723*wvL*wvL;
    }
    else if (wvL < 760.0)
    {
        wvL = (wvL-700.0)/1000;
        emT = 0.43151-0.0207*T-(0.1441+0.0551*T)*wvL-(0.278+0.19*T)*wvL*wvL;
    }
    else if (wvL < 940.0)
    {
        wvL = (wvL-850.0)/1000;
        emT = 0.4061-0.0259*T+(0.0087*T+0.029*T*T-0.1889)*wvL+(0.246*T-0.126)*wvL*wvL;
    }

    return emT;
}

// Performs a rational polynomial 2/2 curve fitting to a set of points using
// least-squares approach. This implementation is tuned for embedded devices.
//
// The rational function fitted here is:
//
//    y = (a0 + a1*x + a2*x^2) / (1 + b1*x + b2*x^2)
//
// Results retuned as a vector of doubles: a0,a1,a2,b1,b2.
// Results array change between invocations so need to be saved.
//
double* fitRational2(const float *x, const float *y, const int nSamples)
{
    // results array - will store {a0,a1,a2,b1,b2}
    static double result[5];

    // B = normal augmented matrix that stores the equations.
    double B[5][6] = {{0.0,0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0,0.0}};

    // populate distinct values
    for (int i = 0; i < nSamples; ++i)
    {
        double x2 = x[i]*x[i];
        double x3 = x2*x[i];
        double x4 = x3*x[i];
        double y2 = y[i]*y[i];

        B[0][1] += x[i];
        B[0][2] += x2;
        B[1][2] += x3;
        B[2][2] += x4;

        B[0][3] -= x[i]*y[i];
        B[0][4] -= x2*y[i];
        B[1][4] -= x3*y[i];
        B[2][4] -= x4*y[i];

        B[3][3] += x2*y2;
        B[3][4] += x3*y2;
        B[4][4] += x4*y2;

        B[0][5] += y[i];
        B[3][5] -= x[i]*y2;
    }

    // constants and copy values
    B[0][0] = nSamples;
    B[1][0] = B[0][1];
    B[2][0] = B[1][1] = B[0][2];
    B[2][1] = B[1][2];
    B[3][0] = B[0][3];

    B[4][0] = B[3][1] = B[1][3] = B[0][4];
    B[4][1] = B[3][2] = B[2][3] = B[1][4];
    B[4][2] = B[2][4];

    B[4][3] = B[3][4];

    B[1][5] = -B[0][3];
    B[2][5] = -B[0][4];
    B[4][5] = -B[3][3];

    // Pivotisation of the B matrix.
    for (int i = 0; i < 5; ++i)
        for (int k = i+1; k < 5; ++k)
            if (B[i][i] < B[k][i])
                for (int j = 0; j <= 5; ++j)
                {
                    double tmp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = tmp;
                }

    // Performs the Gaussian elimination.
    // (1) Make all elements below the pivot equals to zero
    //     or eliminate the variable.
    for (int i=0; i<4; ++i)
        for (int k =i+1; k<5; ++k)
        {
            double t = B[k][i] / B[i][i];
            for (int j=0; j<=5; ++j)
                B[k][j] -= t*B[i][j];         // (1)
        }

    // Back substitution.
    // (1) Set the variable as the rhs of last equation
    // (2) Subtract all lhs values except the target coefficient.
    // (3) Divide rhs by coefficient of variable being calculated.
    for (int i=4; i >= 0; --i)
    {
        result[i] = B[i][5];                        // (1)
        for (int j = 0; j<5; ++j)
            if (j != i)
                result[i] -= B[i][j] * result[j];   // (2)
        result[i] /= B[i][i];                       // (3)
    }

    return result;
}

/**
 * Performs a 5th degree polynomial curve fitting (polynomial with zero free value)
 * to a set of points using least-squares approach. This implementation is tuned for
 * embedded devices.
 *
 * The function fitted here is:
 *
 *    y = a1*x + a2*x^2 + a3*x^3 + a4*x^4 + a5*x^5
 *
 * Results stored in array of doubles: a1,a2,a3,a4,a5.
 * Results array change between invocations so need to be saved.
 */
void fitPolyZero5(const float *x, const float *y, const int nSamples, double* result)
{
    // B = normal augmented matrix that stores the equations.
    double B[5][6] = {{0.0,0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0,0.0}};

    // populate distinct values
    for (int i = 0; i < nSamples; ++i)
    {
        double xi = x[i];
        B[0][5] += xi*y[i];
        xi *= x[i];  // xi^2
        B[0][0] += xi;
        B[1][5] += xi*y[i];
        xi *= x[i];  // xi^3
        B[0][1] += xi;
        B[2][5] += xi*y[i];
        xi *= x[i];  // xi^4
        B[0][2] += xi;
        B[3][5] += xi*y[i];
        xi *= x[i];  // xi^5
        B[0][3] += xi;
        B[4][5] += xi*y[i];
        xi *= x[i];  // xi^6
        B[0][4] += xi;
        xi *= x[i];  // xi^7
        B[1][4] += xi;
        xi *= x[i];  // xi^8
        B[2][4] += xi;
        xi *= x[i];  // xi^9
        B[3][4] += xi;
        xi *= x[i];  // xi^10
        B[4][4] += xi;
    }

    // copy values
    B[1][0] = B[0][1];
    B[2][0] = B[1][1] = B[0][2];
    B[3][0] = B[2][1] = B[1][2] = B[0][3];
    B[4][0] = B[3][1] = B[2][2] = B[1][3] = B[0][4];
    B[4][1] = B[3][2] = B[2][3] = B[1][4];
    B[4][2] = B[3][3] = B[2][4];
    B[4][3] = B[3][4];

    // Pivotisation of the B matrix.
    for (int i = 0; i < 5; ++i)
        for (int k = i+1; k < 5; ++k)
            if (B[i][i] < B[k][i])
                for (int j = 0; j <= 5; ++j)
                {
                    double tmp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = tmp;
                }

    // Performs the Gaussian elimination.
    // (1) Make all elements below the pivot equals to zero
    //     or eliminate the variable.
    for (int i=0; i<4; ++i)
        for (int k =i+1; k<5; ++k)
        {
            double t = B[k][i] / B[i][i];
            for (int j=0; j<=5; ++j)
                B[k][j] -= t*B[i][j];         // (1)
        }

    // Back substitution.
    // (1) Set the variable as the rhs of last equation
    // (2) Subtract all lhs values except the target coefficient.
    // (3) Divide rhs by coefficient of variable being calculated.
    for (int i=4; i >= 0; --i)
    {
        result[i] = B[i][5];                        // (1)
        for (int j = 0; j<5; ++j)
            if (j != i)
                result[i] -= B[i][j] * result[j];   // (2)
        result[i] /= B[i][i];                       // (3)
    }
}

// Performs a linear regression without intercept term.
//
// The function fitted here is:
//
//    y = A*x
//
// Retuns A as a result
//
double fitLinearZero(const float *x, const float *y, const int nSamples)
{
    double xy = 0.0;
    double x2 = 0.0;

    // populate sums
    for (int i = 0; i < nSamples; ++i)
    {
        xy += x[i]*y[i];
        x2 += x[i]*x[i];
    }

    return x2!=0.0 ? xy/x2 : 0.0;
}

// ---------------------------
//       Class methods
// ---------------------------

// Constructor
CSpectrometer::CSpectrometer(uint8_t adc_ref_sel1, uint8_t adc_ref_sel2,
                             const double *defaultCalibration,
                             const int baseEEPROM_)
        : adc_ref_sel1_(adc_ref_sel1), adc_ref_sel2_(adc_ref_sel2),
          defaultCalibration_(defaultCalibration),
          calibration_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , rangeStartIdx_(0), rangePixels_(0),
          satVoltage_(0), linearCoeff_(0), avgBias_(0.0), bias_(0), blackLevels_(0),
          normCoef_(0), meas_(0), lastMeasScale_(1.0), biasInvalid_(false),
          measuringData_(false), specState_(ADC_5V), lastMeasSpecState_(ADC_5V),
          baseEEPROM_(baseEEPROM_), offsEEPROM_(0), rawMeas_(0), rawMeasCounts_(0),
          adcVoltages_{2.5, 3.0, 4.096, 5.0}
{
}

// Destructor
CSpectrometer::~CSpectrometer()
{
    delete[] meas_;
    free(satVoltage_);
}

// Setup methods
bool CSpectrometer::begin()
{
    // EEPROM indexes
    static const int EEPROM_OFFSETS[2][EEPROM_MAX_SETTINGS] =
    {
        { 0, 48, 49, 53, 57, 61, -1, 65, 69, 73, 113, 145 },
        { 0, 48, 49, 53, 57, 65, 69, 73, 77, 81, 161, 193 }
    };

    // EEPROM setup
    offsEEPROM_ = EEPROM_OFFSETS[supportsGain()];

    // defaults
    getSensorConstraint(MAX_PIXELS, &rangePixels_);

    // Allocate and initialise arrays for saturation and linearisation
    satVoltage_ = (float*)malloc((sizeof(*satVoltage_)
                                 +sizeof(*linearCoeff_))*(supportsGain()+1));
    linearCoeff_ = (double (*)[5])(satVoltage_+(supportsGain()+1));

    satVoltage_[NO_GAIN] = 0.0;
    linearCoeff_[NO_GAIN][0] = 1.0;
    linearCoeff_[NO_GAIN][1] = linearCoeff_[NO_GAIN][2] =
                               linearCoeff_[NO_GAIN][3] =
                               linearCoeff_[NO_GAIN][4] = 0.0;
    if (supportsGain())
    {
        satVoltage_[HIGH_GAIN] = 0.0;
        linearCoeff_[HIGH_GAIN][0] = 1.0;
        linearCoeff_[HIGH_GAIN][1] = linearCoeff_[HIGH_GAIN][2] =
                                     linearCoeff_[HIGH_GAIN][3] =
                                     linearCoeff_[HIGH_GAIN][4] = 0.0;
    }

    // only read saved spectrometer state it if enabled
    if (baseEEPROM_>=0)
    {
        // Get ADC voltages - first
        EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_ADC_VOLTAGES], adcVoltages_);
        if (int)(5.0 - adcVoltages_[ADC_5V]) != 0)
        {
            adcVoltages_[ADC_2_5V]   = 2.5;
            adcVoltages_[ADC_3V]     = 3.0;
            adcVoltages_[ADC_4_096V] = 4.096;
            adcVoltages_[ADC_5V]     = 5.0;
        }

        // read saved data and set defaults
        EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_CALIBRATION_COEF], calibration_);
        EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_STATE],            specState_);
        EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_AVG_BIAS],         avgBias_);

        EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_SAT_VOLTAGES],       satVoltage_[0]);
        EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_LINEARISATION_COEF], linearCoeff_[0]);
        if (supportsGain())
        {
            // read high gain linearisation and saturation array
            EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_SAT_VOLTAGES]
                       + sizeof(satVoltage_[0]),                       satVoltage_[1]);
            EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_LINEARISATION_COEF]
                       + sizeof(linearCoeff_[0]),                      linearCoeff_[1]);
            if (std::isnan(linearCoeff_[HIGH_GAIN][0])
                || std::isnan(linearCoeff_[HIGH_GAIN][1])
                || std::isnan(linearCoeff_[HIGH_GAIN][2])
                || std::isnan(linearCoeff_[HIGH_GAIN][3])
                || std::isnan(linearCoeff_[HIGH_GAIN][4]))
            {
                linearCoeff_[HIGH_GAIN][0] = 1.0;
                linearCoeff_[HIGH_GAIN][1] = linearCoeff_[HIGH_GAIN][2] =
                                             linearCoeff_[HIGH_GAIN][3] =
                                             linearCoeff_[HIGH_GAIN][4] = 0.0;
            }
        }

        // check linearisations
        if (std::isnan(linearCoeff_[NO_GAIN][0]) || std::isnan(linearCoeff_[NO_GAIN][1])
            || std::isnan(linearCoeff_[NO_GAIN][2]) || std::isnan(linearCoeff_[NO_GAIN][3])
            || std::isnan(linearCoeff_[NO_GAIN][4]))
        {
            linearCoeff_[NO_GAIN][0] = 1.0;
            linearCoeff_[NO_GAIN][1] = linearCoeff_[NO_GAIN][2] =
                                       linearCoeff_[NO_GAIN][3] =
                                       linearCoeff_[NO_GAIN][4] = 0.0;
        }
    }

    // checks and set defaults
    if (std::isnan(avgBias_) || avgBias_ < 0.0 || avgBias_ > adcVoltages_[ADC_2_5V])
        avgBias_ = 0.0;

    biasInvalid_ = avgBias_ == 0.0;

    if ((specState_ & ADC_REF_MASK) >= ADC_MAX_VOLTAGES)
    {
        specState_ &= ~ADC_REF_MASK;
        specState_ |= ADC_5V;
    }

    // check and set saturation voltages against Hamamatsu spec limits
    float minSatVoltage = 0.0;
    float maxSatVoltage = 0.0;
    getSensorConstraint(MIN_SAT_VOLTAGE, &minSatVoltage);
    getSensorConstraint(MAX_SAT_VOLTAGE, &maxSatVoltage);
    if (std::isnan(satVoltage_[NO_GAIN])
        || satVoltage_[NO_GAIN] < minSatVoltage
        || satVoltage_[NO_GAIN] > maxSatVoltage)
        satVoltage_[NO_GAIN] = minSatVoltage;
    if (supportsGain())
    {
        getSensorConstraint(MIN_SAT_VOLTAGE_HIGH_GAIN, &minSatVoltage);
        getSensorConstraint(MAX_SAT_VOLTAGE_HIGH_GAIN, &maxSatVoltage);
        if (std::isnan(satVoltage_[HIGH_GAIN])
            || satVoltage_[HIGH_GAIN] < minSatVoltage
            || satVoltage_[HIGH_GAIN] > maxSatVoltage)
            satVoltage_[HIGH_GAIN] = minSatVoltage;
    }

    // check the wavelength calibration
    if (defaultCalibration_ &&
        (std::isnan(calibration_[0]) || std::isnan(calibration_[1])
         || std::isnan(calibration_[2]) || std::isnan(calibration_[3])
         || std::isnan(calibration_[4]) || std::isnan(calibration_[5])
         || calibration_[0] < 100
         || calibration_[0] > 500))  // first coeff should be around 300
    {
        setWavelengthCalibrationInternal(defaultCalibration_);
    }

    // initialise ranges and arrays
    int minWavelength, maxWavelength;
    getSensorRangeInternal(minWavelength, maxWavelength);
    setSensorRangeInternal(minWavelength, maxWavelength);

    // Initialize arrays - bias, black and normalisation
    const int normCoeffOffs = baseEEPROM_ + offsEEPROM_[EEPROM_NORM_COEF_ARRAY];
    for (int i=0; i<rangePixels_; i++)
    {
        blackLevels_[i] = 0.0;
        meas_[i] = normCoef_[i] = 0.0;
        bias_[i] = avgBias_;

        // read spectral response normalisation
        if (baseEEPROM_>=0)
            EEPROM.get(normCoeffOffs+i*sizeof(float), normCoef_[i]);

        if (std::isnan(normCoef_[i])
            || normCoef_[i]<0.000001
            || normCoef_[i]>10000)
            normCoef_[i] = 1.0;
    }

    // Setup pins
    pinMode(adc_ref_sel1_, OUTPUT);
    pinMode(adc_ref_sel2_, OUTPUT);

    // set defaults
    setAdcRefInternal(getAdcReference());
    if (supportsGain())
        setHighGain(getHighGain(), false);

    return true;
}

// Reset all stored values to default
void CSpectrometer::resetToDefaults()
{
    adcVoltages_[ADC_2_5V]   = 2.5;
    adcVoltages_[ADC_3V]     = 3.0;
    adcVoltages_[ADC_4_096V] = 4.096;
    adcVoltages_[ADC_5V]     = 5.0;
    avgBias_ = 0.0;
    lastMeasScale_ = 1.0;
    biasInvalid_ = true;

    if (supportsGain())
    {
        // don't write the state in EEPROM yet
        setHighGain(false, false);
        calibrateGain();
    }

    // reset ADC and write state
    setAdcReference(ADC_5V, true);
    lastMeasSpecState_ = specState_;
    setMeasScale();

    if (supportsGain())
    {
        getSensorConstraint(MIN_SAT_VOLTAGE_HIGH_GAIN, &satVoltage_[HIGH_GAIN]);

        linearCoeff_[0][0] = linearCoeff_[1][0] = 1.0;
        linearCoeff_[0][1] = linearCoeff_[0][2] = linearCoeff_[0][3] = linearCoeff_[0][4]
                           = linearCoeff_[1][1] = linearCoeff_[1][2] = linearCoeff_[1][3]
                           = linearCoeff_[1][4] = 0.0;
    }
    else
    {
        linearCoeff_[0][0] = 1.0;
        linearCoeff_[0][1] = linearCoeff_[0][2] = linearCoeff_[0][3]
                           = linearCoeff_[0][4] = 0.0;
    }

    getSensorConstraint(MIN_SAT_VOLTAGE, &satVoltage_[NO_GAIN]);

    if (baseEEPROM_>=0)
    {
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_ADC_VOLTAGES],       adcVoltages_);
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_STATE],              specState_);
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_AVG_BIAS],           avgBias_);
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_SAT_VOLTAGES],       satVoltage_[0]);
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_LINEARISATION_COEF], linearCoeff_[0]);
        if (supportsGain())
        {
            EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_SAT_VOLTAGES]
                       + sizeof(satVoltage_[0]),                       satVoltage_[1]);
            EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_LINEARISATION_COEF]
                       + sizeof(linearCoeff_[0]),                      linearCoeff_[1]);
        }
    }

    setIntTime(100.0, T_MS, true);
    setExtTrgMeasDelay(-1, T_MS, true);
    setWavelengthCalibrationInternal(defaultCalibration_);

    setSensorRange(-1, -1);
    calibrateSpectralResponse(0);
}

// Obtains sensor range from saved EEPROM
void CSpectrometer::getSensorRangeInternal(int& minWavelength, int& maxWavelength)
{
    if (baseEEPROM_>=0)
    {
        EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_SPEC_RANGE_MIN], minWavelength);
        EEPROM.get(baseEEPROM_+offsEEPROM_[EEPROM_SPEC_RANGE_MAX], maxWavelength);
    }
    else
        minWavelength = maxWavelength = -1;

    if (minWavelength != -1 && minWavelength < 100)
        minWavelength = -1;
    if (maxWavelength != -1 && maxWavelength > 1000)
        maxWavelength = -1;

    if (maxWavelength>0 && maxWavelength>0 && maxWavelength<=minWavelength)
        minWavelength = maxWavelength = -1;
}

// Sets sensor spectral range - this will set internal indexes, pixel numbers
// and array reallocations according to the new range. This method needs
// valid wavelength calibration coefficients to work.
void CSpectrometer::setSensorRangeInternal(int& minWavelength, int& maxWavelength)
{
    int rangeEndIdx = rangeStartIdx_ + rangePixels_;
    int rangeStartIdx = rangeStartIdx_;

    if (minWavelength>0 && maxWavelength>0 && maxWavelength<=minWavelength)
        return;

    // reset the range start for wavelength calculations
    rangeStartIdx_ = 0;
    int specPixels = 0;
    getSensorConstraint(MAX_PIXELS, &specPixels);

    if (getWavelength(0) <= 0)
    {
        // wavlength calibration is not set - use full range
        minWavelength = maxWavelength = -1;
        rangeStartIdx = 0;
        rangeEndIdx = specPixels;
    }
    else
    {
        // update lower bound
        if (minWavelength != 0)
        {
            if (minWavelength < 0)
                // value from Hamamatsu spec
                getSensorConstraint(MIN_WAVELENGTH, &minWavelength);
            // search for lower bound index
            rangeStartIdx = 0;
            for (int i=0; i<specPixels; ++i)
                if (minWavelength <= (int)getWavelength(i))
                {
                    rangeStartIdx = i;
                    break;
                }
            if (!rangeStartIdx)
                // no valid one was found or exceeds the range
                minWavelength = getWavelength(0);
            // take one more pixel to enclose the range
            if (rangeStartIdx)
                --rangeStartIdx;
        }

        // update upper bound
        if (maxWavelength != 0)
        {
            if (maxWavelength < 0)
                // value from Hamamatsu spec
                getSensorConstraint(MAX_WAVELENGTH, &maxWavelength);
            // search for lower bound index
            rangeEndIdx = specPixels;
            for (int i=specPixels; i>rangeStartIdx; --i)
                if (maxWavelength >= (int)getWavelength(i-1))
                {
                    rangeEndIdx = i;
                    break;
                }
            if (rangeEndIdx==specPixels)
                // no valid one was found or exceeds the range
                maxWavelength = getWavelength(specPixels-1);
            // take one more pixel to enclose the range
            if (rangeEndIdx < specPixels)
                ++rangeEndIdx;
        }
    }

    // setup arrays
    if (rangePixels_ < rangeEndIdx - rangeStartIdx)
    {
        // deallocate existing arrays
        delete[] meas_;
        meas_ = blackLevels_ = normCoef_ = bias_ = 0;
    }

    rangeStartIdx_ = rangeStartIdx;
    rangePixels_ = rangeEndIdx - rangeStartIdx;

    if (!meas_)
    {
        meas_ = new float[4*rangePixels_];
        blackLevels_ = meas_ + rangePixels_;
        normCoef_ = blackLevels_ + rangePixels_;
        bias_ = normCoef_ + rangePixels_;
    }
}

// Sets the spectrometer sensor range in nanometers. If the range is
// wider than current one, then this will reset spectral response
// normalisation. It also resets measured value and black levels.
//
// Specifying either value as 0 will not update that value.
//
// Specifying either value as -1 will reset to the spectrometer default
// for that value.
//
// Generally, the spectral range should be chosen at the beginning for
// the specific application, sensor calibrated with it and then left alone.
void CSpectrometer::setSensorRange(int minWavelength, int maxWavelength)
{
    // no action if in measurement
    if (measuringData_)
        return;

    // prevent measurement whilst resetting
    measuringData_ = true;

    int savedStartIdx = rangeStartIdx_;
    int savedRangePixels = rangePixels_;

    // update sensor data
    setSensorRangeInternal(minWavelength, maxWavelength);

    // store in EEPROM
    if (baseEEPROM_>=0)
    {
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_SPEC_RANGE_MIN], minWavelength);
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_SPEC_RANGE_MAX], maxWavelength);
    }

    // check if we need to reset data
    if (savedStartIdx != rangeStartIdx_ || savedRangePixels != rangePixels_)
    {
        // reset data
        const int normCoeffOffs =
            baseEEPROM_+offsEEPROM_[EEPROM_NORM_COEF_ARRAY];
        for (int i=0; i<rangePixels_; ++i)
        {
            normCoef_[i] = 1.0;
            blackLevels_[i] = 0.0;
            bias_[i] = avgBias_;
            meas_[i] = 0.0;
            if (baseEEPROM_>=0)
                // write spectral response normalisation
                EEPROM.put(normCoeffOffs+i*sizeof(float), normCoef_[i]);
        }
        biasInvalid_ = avgBias_ == 0.0;
        lastMeasSpecState_ = specState_;
        setMeasScale();
    }

    measuringData_ = false;
}

// Sets the wavelength calibration coeffiecients only and preserves them in EEPROM.
// Usually provided by Hamamatsu but can be overriden by user calculated ones.
//
// Return true if the calibration coefficients are changed
bool CSpectrometer::setWavelengthCalibrationInternal(const double* wavelengthCal)
{
    if (!wavelengthCal)
        return false;

    bool changed = calibration_[0] != wavelengthCal[0] ||
                   calibration_[1] != wavelengthCal[1] ||
                   calibration_[2] != wavelengthCal[2] ||
                   calibration_[3] != wavelengthCal[3] ||
                   calibration_[4] != wavelengthCal[4] ||
                   calibration_[5] != wavelengthCal[5];

    calibration_[0] = wavelengthCal[0];
    calibration_[1] = wavelengthCal[1];
    calibration_[2] = wavelengthCal[2];
    calibration_[3] = wavelengthCal[3];
    calibration_[4] = wavelengthCal[4];
    calibration_[5] = wavelengthCal[5];

    if (changed && baseEEPROM_>=0)
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_CALIBRATION_COEF], calibration_);

    return changed;
}

// Sets the wavelength calibration coeffiecients. Usually provided
// by Hamamatsu but can be overriden by user calculated ones.
//
// Setting this will reset spectral response normalisation.
bool CSpectrometer::setWavelengthCalibration(const double* wavelengthCal)
{
    // no action we are in measurement alerady
    if (measuringData_)
        return false;

    // prevent measurement whilst resetting
    measuringData_ = true;

    const double* wvCal = wavelengthCal ? wavelengthCal : defaultCalibration_;
    bool result = setWavelengthCalibrationInternal(wvCal);

    if (result)
    {
        int minWavelength, maxWavelength;
        getSensorRangeInternal(minWavelength, maxWavelength);
        setSensorRangeInternal(minWavelength, maxWavelength);

        // reset data
        const int normCoeffOffs =
            baseEEPROM_+offsEEPROM_[EEPROM_NORM_COEF_ARRAY];
        for (int i=0; i<rangePixels_; ++i)
        {
            normCoef_[i] = 1.0;
            blackLevels_[i] = 0.0;
            bias_[i] = avgBias_;
            meas_[i] = 0.0;
            if (baseEEPROM_>=0)
                // write spectral response normalisation
                EEPROM.put(normCoeffOffs+i*sizeof(float), normCoef_[i]);
        }

        biasInvalid_ = avgBias_ == 0.0;
        lastMeasSpecState_ = specState_;
        setMeasScale();
    }

    measuringData_ = false;

    return result;
}

// Set ADC reference voltage to one of the specified values. This is
// internal function without reentrance checks.
//
// Using ADC_AUTO setting results in selecting closest ADC voltage larger than
// saturation limit.
void CSpectrometer::setAdcRefInternal(int adcRef)
{
    if (adcRef == ADC_AUTO)
    {
        float satVoltage = satVoltage_[getHighGain()];
        adcRef = 0;

        while ((satVoltage-adcVoltages_[adcRef]) > 0.001 && adcRef < ADC_MAX_VOLTAGES-1)
            ++adcRef;
    }

    adcRef &= ADC_REF_MASK;
    specState_ &= ~ADC_REF_MASK;
    specState_ |= adcRef;

    if (adcRef & 1)
        pinSetFast(adc_ref_sel1_);
    else
        pinResetFast(adc_ref_sel1_);

    if (adcRef & 2)
        pinSetFast(adc_ref_sel2_);
    else
        pinResetFast(adc_ref_sel2_);
}

// Set ADC reference voltage to one of the specified values. Defines maximum
// analogue signal voltage for conversion. This generally should be used
// together with setting the spectrometer gain (high gain will mean higher
// reference voltage) but it is decoupled for flexibility of ADC control.
void CSpectrometer::setAdcReference(adc_ref_t ref, bool saveState)
{
    // no action if in measurement
    if (measuringData_)
        return;

    // physically setup the ADC reference
    setAdcRefInternal(ref);

    if (saveState && baseEEPROM_>=0)
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_STATE], specState_);

    // delay to stabilise the changes
    delay(200);
}

// Set integration time in chosen units for a single measurement cycle.
// The real measurement time can be more that this period in which case
// several measurement cycles will be taken sequentially and averaged out.
//
// NOTE: if not within sensor allowed boundaries then it will be set to
//       closest minimum or maximum allowed value
void CSpectrometer::setIntTime(double time, time_units_t units, bool saveState)
{
    double timeUs = time<0 ? 0.0 : timeUnitsToUsec[units] * time;

    setIntTimeInternal(timeUs, saveState);
}

// Sets the external trigger to measurement delay time. This defines time
// interval in specified units that offsets external trigger from
// the measurement. I.e. external trigger is raised and after this delay
// the integration and measurement starts.
//
// Specifying -1 as  delay will disable the external trigger
void CSpectrometer::setExtTrgMeasDelay(double delayTime, time_units_t units, bool saveState)
{
    double delayUs = delayTime<0 ? -1.0 : timeUnitsToUsec[units] * delayTime;

    setExtTrgMeasDelayInternal(delayUs, saveState);
}

// calculates and sets last measurement scale
void CSpectrometer::setMeasScale()
{
    double gainFactor = getHighGain() ? getGainFactor() : 1.0;

    lastMeasScale_ = 1/(getIntTime(T_SEC)*gainFactor);
}

// Sets the saturation voltage. These are used to in auto integration mode
// of measurement. Passing values outside of ranges from Hamamatsu spec
// will reset approprite value to a default ones.
void CSpectrometer::setSaturationVoltage(float satVoltage, bool highGain)
{
    float minSatVoltage = 0.0;
    float maxSatVoltage = 0.0;

    // get the limits
    highGain = highGain && supportsGain();

    if (highGain)
    {
        getSensorConstraint(MIN_SAT_VOLTAGE_HIGH_GAIN, &minSatVoltage);
        getSensorConstraint(MAX_SAT_VOLTAGE_HIGH_GAIN, &maxSatVoltage);
    }
    else
    {
        getSensorConstraint(MIN_SAT_VOLTAGE, &minSatVoltage);
        getSensorConstraint(MAX_SAT_VOLTAGE, &maxSatVoltage);
    }

    // check the voltage set
    if (satVoltage < minSatVoltage)
        satVoltage = minSatVoltage;
    else if (satVoltage > maxSatVoltage)
        satVoltage = maxSatVoltage;

    satVoltage_[highGain] = satVoltage;

    if (baseEEPROM_>=0)
        EEPROM.put(
            baseEEPROM_+offsEEPROM_[EEPROM_SAT_VOLTAGES]+sizeof(satVoltage_[0])*highGain,
            satVoltage_[highGain]);
}

// Saturation is calculated as minimal value across fully lit pixels with
// where linearity deviation starts exceeding 3%
float CSpectrometer::findSaturatedLimit(float minV, float maxV, TProgressFunc progress)
{
    double minIntTimeUs = 0.0;
    double maxIntTimeUs = 0.0;

    getSensorConstraint(MIN_INT_TIME_US, &minIntTimeUs);
    getSensorConstraint(MAX_INT_TIME_US, &maxIntTimeUs);

    // do max int time to see if sensor saturated
    setIntTimeInternal(maxIntTimeUs, false);
    readSpectrometer(-1, false, false);
    float stddev = 0.0;
    float maxRead = processMeasurement(meas_, false, &stddev);

    if (stddev>0.07 || maxRead<minV || maxRead>maxV)
        return -1.0;

    // find lowest saturated pixel (C12666 at least is unevently saturated in gainless mode)
    int satIdx = -1;
    float calcSatVoltage = 10000.0;
    for (int i=1; i+1<rangePixels_; ++i)
    {
        float measVal = (meas_[i-1]+meas_[i]+meas_[i+1])/3.0;
        if (measVal < calcSatVoltage)
        {
            calcSatVoltage = measVal;
            satIdx = i;
        }
    }

    // if bias is not measured - return the adjusted
    // minimum as there is not much we can do
    if (bias_[satIdx]==0.0 || bias_[satIdx]==avgBias_ || biasInvalid_)
        return calcSatVoltage*0.99;

    calcSatVoltage = (getRawMeasValue(satIdx-1,true)+
                      getRawMeasValue(satIdx,  true)+
                      getRawMeasValue(satIdx+1,true))/3.0;

    // now keep lowering integration until we get into more or less linear area
    // less than 70% of the current saturation
    double satIntTimeUs = maxIntTimeUs;
    double measSatVoltage = calcSatVoltage;
    do
    {
        if (progress)
            (*progress)(1, true);

        satIntTimeUs /= 2;
        setIntTimeInternal(satIntTimeUs, false);
        readSpectrometer(-1, false, false);
        measSatVoltage = (getRawMeasValue(satIdx-1,true)+
                          getRawMeasValue(satIdx,  true)+
                          getRawMeasValue(satIdx+1,true))/3.0;
    }
    while (satIntTimeUs > minIntTimeUs && measSatVoltage > calcSatVoltage*0.7);

    if (satIntTimeUs < minIntTimeUs)
        return -1.0;

    // calculate saturation voltage integration time linearly and measure
    setIntTimeInternal(getIntTime(T_USEC)*calcSatVoltage/measSatVoltage, false);
    satIntTimeUs = getIntTime(T_USEC);
    readSpectrometer(std::max(satIntTimeUs*4.1, 500000.0), false, false);
    measSatVoltage = (getRawMeasValue(satIdx-1,true)+
                      getRawMeasValue(satIdx,  true)+
                      getRawMeasValue(satIdx+1,true))/3.0;

    // keep approximating saturation until we are within 2.9-3% of linearity
    int iterations = 16;
    while (iterations-- > 0 &&
           (calcSatVoltage*0.97>measSatVoltage || calcSatVoltage*0.971<measSatVoltage))
    {
        if (progress)
            (*progress)(1, true);

        // adjust the integration time linearly to the goal (3% from calculated linear value)
        setIntTimeInternal(satIntTimeUs*measSatVoltage/(calcSatVoltage*0.97), false);

        // recalculate saturation voltage linearly and re-measure
        calcSatVoltage *= getIntTime(T_USEC)/satIntTimeUs;
        satIntTimeUs = getIntTime(T_USEC);
        readSpectrometer(std::max(satIntTimeUs*4.1, 500000.0), false, false);
        measSatVoltage = (getRawMeasValue(satIdx-1,true)+
                          getRawMeasValue(satIdx,  true)+
                          getRawMeasValue(satIdx+1,true))/3.0;
    }

    return (getRawMeasValue(satIdx-1,false)+
            getRawMeasValue(satIdx,  false)+
            getRawMeasValue(satIdx+1,false))/3.0;
}

// Automatic measurement of the saturation voltages. These are used in
// auto integration mode of measurement.
//
// Automatic setting works by exposing sensor to bright light and then
// calling this method to work out saturation voltages.
//
// If progress callback is supplied it will be called during calibration runs
// periodically to refresh progression of the calibration.
bool CSpectrometer::measureSaturation(TProgressFunc progress)
{
    // no action if in measurement
    if (measuringData_)
        return false;

    bool result = true;

    float minSatVoltage = 0.0;
    float maxSatVoltage = 0.0;

    // save current state and integration time
    uint8_t savedSpecState_ = specState_;
    double  savedIntTimeUs  = getIntTime(T_USEC);

    // set the ADC to the maximum
    setAdcRefInternal(ADC_MAX_VOLTAGES-1);

    if (supportsGain())
    {
        getSensorConstraint(MIN_SAT_VOLTAGE_HIGH_GAIN, &minSatVoltage);
        getSensorConstraint(MAX_SAT_VOLTAGE_HIGH_GAIN, &maxSatVoltage);

        // set high gain first
        setHighGain(true, false);

        // delay to stabilise the changes
        delay(50);

        // find high gain saturated voltage
        float satVoltage = findSaturatedLimit(minSatVoltage, maxSatVoltage, progress);
        if (satVoltage>0.0)
            // round store satVoltage rounded to 2dp
            setSaturationVoltage(satVoltage);
        else
            result = false;

        // report half way there
        if (progress)
            (*progress)(50, false);

        // set no gain next
        setHighGain(false, false);
    }

    getSensorConstraint(MIN_SAT_VOLTAGE, &minSatVoltage);
    getSensorConstraint(MAX_SAT_VOLTAGE, &maxSatVoltage);

    // delay to stabilise the changes
    delay(50);

    // find no gain saturated voltage
    float satVoltage = findSaturatedLimit(minSatVoltage, maxSatVoltage, progress);
    if (satVoltage>0.0)
        // round store satVoltage rounded to 2dp
        setSaturationVoltage(satVoltage);
    else
        result = false;

    // restore state and integration
    specState_ = savedSpecState_;
    setAdcRefInternal(getAdcReference());
    if (supportsGain())
        setHighGain(getHighGain(), false);
    setIntTimeInternal(savedIntTimeUs, false);

    measuringData_ = false;

    return result;
}

// Measurement of the bias.
//
// Measurement works by reading a series of dark exposure of short integrations
// series and interpolating at each point with rational polynomial by integration
// time. Bias is taken as an interpolated value at integration time of 0.
//
// Each calibration updates the the average bias that is used after startup
// before first calibration run. If avgMinOnly is set then average bias is
// only updated if new average is less than previously recorded one, othwerise
// it is updated regardless.
//
// If progress callback is supplied it will be called during calibration runs
// periodically to refresh progression of the calibration.
//
bool CSpectrometer::measureBias(bool avgMinOnly, TProgressFunc progress)
{
    // determine how many measures we can hold and allocate arrays
    uint32_t availMemMeasures = System.freeMemory() / (rangePixels_ * sizeof(float));
    if (availMemMeasures < 8)
        return false;   // too little

    measuringData_ = true;

    // save current state
    uint8_t savedSpecState_ = specState_;
    double  savedIntTimeUs  = getIntTime(T_USEC);

    // set the 2.048V reference - to make black measuremeent more precise
    setAdcRefInternal(ADC_2_5V);

    // set no gain
    setHighGain(false, false);

    // delay to stabilise the changes
    delay(50);

    int numMeas = availMemMeasures - 2; // to make sure we don't use all of free mem
    if (numMeas > 28)
        numMeas  = 28;  // limit to max reasonable reads

    // allocate readings array, set exposure parameters and get started
    float* biasMeas = new float[numMeas*(rangePixels_+1)];
    float* intTimes = biasMeas + numMeas*rangePixels_;
    double curIntTimeUs = 0.0;
    double maxIntTimeUs = 0.0;

    getSensorConstraint(MIN_INT_TIME_US, &curIntTimeUs);
    getSensorConstraint(MAX_BIAS_INT_TIME_US, &maxIntTimeUs);
    double intTimeStepUs = (maxIntTimeUs - curIntTimeUs) / (numMeas-1);

    curIntTimeUs = maxIntTimeUs; // start from the longest int. time

    // start measurement loop
    for (int i=0; i<numMeas; ++i, curIntTimeUs -= intTimeStepUs)
    {
        setIntTimeInternal(curIntTimeUs, false);
        // measure for 0.5 sec or int.time x 4 - whichever is longer
        readSpectrometer(std::max(curIntTimeUs*4.1, 500000.0), false, false);

        // process raw reading
        for (int px=0; px<rangePixels_; ++px)
            biasMeas[i+numMeas*px] = getRawMeasValue(px+rangeStartIdx_,false);

        // store current int time in secs in array
        intTimes[i] = getIntTime(T_SEC);

        // update progress
        if (progress)
            (*progress)(i*100/numMeas, false);
    }

    // now interpolate for each pixel
    double avgBias = 0;
    for (int px=0; px<rangePixels_; ++px)
    {
        // fit rational curve and store value at 0 as bias
        bias_[px] = fitRational2(intTimes, biasMeas+numMeas*px, numMeas)[0];
        // accumulate averages
        avgBias += bias_[px];
    }

    // deallocate
    delete[] biasMeas;

    // calculate and store the average bias
    avgBias /= rangePixels_;
    if (!avgMinOnly || avgBias_ <= 0.0 || avgBias_ > avgBias)
    {
        avgBias_ = avgBias;
        if (baseEEPROM_>=0)
            EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_AVG_BIAS], avgBias_);
    }

    // restore state
    specState_ = savedSpecState_;
    setAdcRefInternal(getAdcReference());
    if (supportsGain())
        setHighGain(getHighGain(), false);
    setIntTimeInternal(savedIntTimeUs, false);

    measuringData_ = false;

    return true;
}

// Linearise sensor response.
//
// Linearisation is based on the MDPI article by M.Nehir, C.Frank, S.Asmann and
// E.P.Achterberg "Improving Optical Measurements: Non-Linearity Compensation of
// Compact Charge-Coupled Device (CCD) Spectrometers" available here
//     https://www.mdpi.com/1424-8220/19/12/2833/pdf
// and a few other sources.
//
// General approach is as follows (sensor saturation should be already calibrated):
//     1) measure/calibrate bias prior to linearisation (dark sensor)
//     2) use high stability constant current LED source to light spectrometer via
//        integrating sphere or diffusing or ND filter such that standard deviation
//        at short integration time is between 30 and 100 ADC units
//     3) at shortest integration time with useful signal as above select pixel with
//        peak value and use it for linearsation (this prevents effects of cross talk
//        or blooming)
//     4) determine longest integration time such that this selected pixel reads near
//        its saturation limit
//     3) obtain a series of measurements for the selectes pixel intensity between
//        shortest and longest integration times equally spaced and take away bias
//     4) find linear gradient k for exposure represented as meas=k*time by
//        linear regression and calculate expected values of measurements
//        against measured values
//     5) calculate linearisation 5th degree polynomial correcting response of the
//        above to the calculated expected measured values
//
// If progress callback is supplied it will be called during calibration runs
// periodically to refresh progression of the calibration.
//
bool CSpectrometer::linearise(TProgressFunc progress)
{
    measuringData_ = true;
    bool success = true;

    // number of measurements to do to calculate linearisation
    const int numMeas = 32;
    int pixelIdxLow  = -1;
    int pixelIdxMed  = -1;
    int pixelIdxHigh = -1;

    // save current state
    uint8_t savedSpecState_ = specState_;
    double  savedIntTimeUs  = getIntTime(T_USEC);

    // allocate readings arrays
    float* intTimes = new float[19*numMeas];
    float* linMeas      = intTimes + numMeas;       // readings for 3 pixels with low values
    float* linMeasMed   = linMeas + 3*numMeas;      // readings for 3 pixels with medium values
    float* linMeasHigh  = linMeasMed + 3*numMeas;   // readings for 3 pixels with high values
    float* corrMeas     = linMeasHigh + 3*numMeas;  // corrected readings for 3 pixels with low values
    float* corrMeasMed  = corrMeas + 3*numMeas;     // corrected readings for 3 pixels with medium values
    float* corrMeasHigh = corrMeasMed + 3*numMeas;  // corrected readings for 3 pixels with high values

    // for gain enabled spectrometer start with high gain
    if (supportsGain())
        setHighGain(true, false);

    bool keepGoing = true;
    do
    {
        // include saturation limit
        setAdcRefInternal(ADC_AUTO);
        float satVoltage = getSatVoltage();

        // delay to stabilise the changes
        delay(50);

        // determine central pixel for linearisation and integration time limits
        double minIntTimeUs = 0.0;
        double maxIntTimeUs = 0.0;
        getSensorConstraint(MIN_INT_TIME_US, &minIntTimeUs);
        getSensorConstraint(MAX_INT_TIME_US, &maxIntTimeUs);

        // min and max stddev to determine the darkest area of signal to
        // linearise 30-100 ADC units
        float stddevMin = 60*getAdcRefVoltage()/ADC_MAX_VALUE;
        float stddevMax = 150*getAdcRefVoltage()/ADC_MAX_VALUE;

        // determine min time
        float stddev = 0.0;
        do
        {
            setIntTimeInternal(minIntTimeUs, false);
            minIntTimeUs = getIntTime(T_USEC) * 2.0;
             // measure for 0.5 sec to average short reads
            readSpectrometer(500000.0, false, false);
            processMeasurement(meas_, false, &stddev);
            if (stddev > stddevMin && stddev < stddevMax)
            {
                float maxVal = -1;
                float minVal = 10000;
                for (int i=0; i<rangePixels_; ++i)
                    if (meas_[i] < minVal)
                        minVal = meas_[pixelIdxLow=i];
                    else if (meas_[i] > maxVal)
                        maxVal = meas_[pixelIdxHigh=i];
                float medVal = (maxVal+minVal)/2;
                for (int i=1; pixelIdxMed<0 && i<rangePixels_; ++i)
                    if (meas_[i-1] < medVal && medVal <= meas_[i])
                        pixelIdxMed = i;
                if (pixelIdxLow == 0)
                    ++pixelIdxLow;
                else if (pixelIdxLow == rangePixels_-1)
                    --pixelIdxLow;
                if (pixelIdxMed == 0)
                    ++pixelIdxMed;
                else if (pixelIdxMed == rangePixels_-1)
                    --pixelIdxMed;
                if (pixelIdxHigh == 0)
                    ++pixelIdxHigh;
                else if (pixelIdxHigh == rangePixels_-1)
                    --pixelIdxHigh;
            }

            if (progress)
                (*progress)(1, true);
        }
        while (pixelIdxHigh < 0 && stddev < stddevMin && minIntTimeUs < maxIntTimeUs/100);

        success = pixelIdxHigh > 0;
        if (!success)
            break;

        minIntTimeUs = getIntTime(T_USEC);  // got the min integration time

        // now establish max integration time
        double curIntTimeUs = minIntTimeUs;
        float measVal = getRawMeasValue(pixelIdxHigh, false);
        while (curIntTimeUs < maxIntTimeUs && measVal < satVoltage*0.999)
        {
            curIntTimeUs = getIntTime(T_USEC) * satVoltage/measVal;
            setIntTimeInternal(curIntTimeUs, false);
            // measure for 0.5 sec to average short reads
            readSpectrometer(500000.0, false, false);
            measVal = getRawMeasValue(pixelIdxHigh, false);
            if (progress)
                (*progress)(1, true);
        }
        success = curIntTimeUs < maxIntTimeUs;
        if (!success)
            break;
        maxIntTimeUs = getIntTime(T_USEC);  // got the max integration time

        curIntTimeUs = maxIntTimeUs;
        double intTimeStepUs = (maxIntTimeUs-minIntTimeUs) / (numMeas-1);

        // start measurement loop
        for (int i=0; i<numMeas; ++i, curIntTimeUs -= intTimeStepUs)
        {
            setIntTimeInternal(curIntTimeUs, false);

            // store current int time in secs in array
            intTimes[i] = getIntTime(T_SEC);

            // measure for 0.5 sec or int.time x 4 - whichever is longer
            readSpectrometer(std::max(curIntTimeUs*4.1, 500000.0), false, false);

            // average up readings without bias
            linMeas[i]               = getRawMeasValue(pixelIdxLow-1, true);
            linMeas[numMeas+i]       = getRawMeasValue(pixelIdxLow,   true);
            linMeas[numMeas*2+i]     = getRawMeasValue(pixelIdxLow+1, true);
            linMeasMed[i]            = getRawMeasValue(pixelIdxMed-1, true);
            linMeasMed[numMeas+i]    = getRawMeasValue(pixelIdxMed,   true);
            linMeasMed[numMeas*2+i]  = getRawMeasValue(pixelIdxMed+1, true);
            linMeasHigh[i]           = getRawMeasValue(pixelIdxHigh-1,true);
            linMeasHigh[numMeas+i]   = getRawMeasValue(pixelIdxHigh,  true);
            linMeasHigh[numMeas*2+i] = getRawMeasValue(pixelIdxHigh+1,true);

            // refresh cloud and progress
            if (progress)
                (*progress)(1, true);
        }

        // calculate gradient by fitting linear 0,0 based curve
        double lowK0  = fitLinearZero(intTimes, linMeas, numMeas);
        double lowK1  = fitLinearZero(intTimes, linMeas+numMeas, numMeas);
        double lowK2  = fitLinearZero(intTimes, linMeas+2*numMeas, numMeas);
        double medK0  = fitLinearZero(intTimes, linMeasMed, numMeas);
        double medK1  = fitLinearZero(intTimes, linMeasMed+numMeas, numMeas);
        double medK2  = fitLinearZero(intTimes, linMeasMed+2*numMeas, numMeas);
        double highK0 = fitLinearZero(intTimes, linMeasHigh, numMeas);
        double highK1 = fitLinearZero(intTimes, linMeasHigh+numMeas, numMeas);
        double highK2 = fitLinearZero(intTimes, linMeasHigh+2*numMeas, numMeas);

        // fill in expected corrected values
        for (int i=0; i<numMeas; ++i)
        {
            corrMeas[i]               = lowK0 > 0.0 ? lowK0*intTimes[i] : linMeas[i];
            corrMeas[numMeas+i]       = lowK1 > 0.0 ? lowK1*intTimes[i] : linMeas[numMeas+i];
            corrMeas[numMeas*2+i]     = lowK2 > 0.0 ? lowK2*intTimes[i] : linMeas[numMeas*2+i];
            corrMeasMed[i]            = medK0 > 0.0 ? medK0*intTimes[i] : linMeasMed[i];
            corrMeasMed[numMeas+i]    = medK1 > 0.0 ? medK1*intTimes[i] : linMeasMed[numMeas+i];
            corrMeasMed[numMeas*2+i]  = medK2 > 0.0 ? medK2*intTimes[i] : linMeasMed[numMeas*2+i];
            corrMeasHigh[i]           = highK0 > 0.0 ? highK0*intTimes[i] : linMeasHigh[i];
            corrMeasHigh[numMeas+i]   = highK1 > 0.0 ? highK1*intTimes[i] : linMeasHigh[numMeas+i];
            corrMeasHigh[numMeas*2+i] = highK2 > 0.0 ? highK2*intTimes[i] : linMeasHigh[numMeas*2+i];
        }

        // calculate linearisation polynomial
        fitPolyZero5(linMeas, corrMeas, numMeas*9, linearCoeff_[getHighGain()]);

        // write calibrations
        if (baseEEPROM_>=0)
            EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_LINEARISATION_COEF]
                                  +sizeof(linearCoeff_[0])*getHighGain(),
                       linearCoeff_[getHighGain()]);

        // end of the loop, for gain enabled spec switch to low gain
        keepGoing = getHighGain();
        if (supportsGain())
            setHighGain(false, false);
    }
    while (keepGoing);

    // deallocate
    delete[] intTimes;

    // restore state
    specState_ = savedSpecState_;
    setAdcRefInternal(getAdcReference());
    if (supportsGain())
        setHighGain(getHighGain(), false);
    setIntTimeInternal(savedIntTimeUs, false);

    measuringData_ = false;

    return success;
}

// Sets linearisation polynomial coefficients explicitly for given high gainю
// If linearisation array is null then resets current linearisation to unity.
bool CSpectrometer::setLinearisation(const double* linearisation, bool highGain)
{
    bool gain = highGain && supportsGain();
    bool result = true;

    if (linearisation)
    {
        if (std::isnan(linearisation[0]) || std::isnan(linearisation[1]) ||
            std::isnan(linearisation[2]) || std::isnan(linearisation[3]) ||
            std::isnan(linearisation[4]))
            result = false;
        else
        {
            linearCoeff_[gain][0] = linearisation[0];
            linearCoeff_[gain][1] = linearisation[1];
            linearCoeff_[gain][2] = linearisation[2];
            linearCoeff_[gain][3] = linearisation[3];
            linearCoeff_[gain][4] = linearisation[4];
        }
    }
    else
    {
        // reset
        linearCoeff_[gain][0] = 1.0;
        linearCoeff_[gain][1] = linearCoeff_[gain][2] = linearCoeff_[gain][3]
                              = linearCoeff_[gain][4] = 0.0;
    }
    // write calibrations
    if (result && baseEEPROM_>=0)
        EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_LINEARISATION_COEF]
                              +sizeof(linearCoeff_[0])*gain,
                   linearCoeff_[gain]);

    return result;
}

// This method calibrates sensor relative spectral response.
//
// It should only be invoked after calibrating saturation, bias and linerisation.
//
// It expects the sensor to be exposed to stabilised tungsten light source
// of the specified temperature, with black levels captured, measures
// sensor response for selected parameters, calculates expected theoretical
// response (relative against largest wavelength) for Planckian blackbody
// corrected for tungsten source, and then calculates corrections for
// measured sensor response (with blacks subtracted).
//
// Normalisation and calibration is done relative to the specified wavelength
// or measured maximim if 0 was specified.
//
// Procedure for calibration:
// 1) run the tungsten light source on stabilised power supply for at
//    least 20 mins, measuring its temperature (using voltage/current
//    measurement, lamp resistance against lamp resistance at room
//    temperature) - see O. Harang, M. J. Kosch "Absolute Optical
//    Calibrations Using a Simple Tungsten Bulb:Theory" for details
// 2) do automatic measurement to capture spectrometer measurement and
//    parameters at minumal ADC voltage and high gain for higher
//    dynamic range
// 3) capture black levels with exposure parameters established by (2)
//    running measurement for 5-10 seconds (averaging out)
// 4) run normal measurement with exposure parameters established by (2)
//    running measurement for 5-10 seconds (averaging out)
// 5) call this method to do the calibration specifying calculated lamp
//    temperature at (1) and use the measurement done at (4) - default)
//
// NOTE: This call can invalidate current measurement results
//
void CSpectrometer::calibrateSpectralResponse(float lampTempK,
                                              bool useCurrentMeasurement,
                                              uint32_t normWvlNm)
{
    const double AIR_REFRACTION = 1.00028;  // standard air refraction

    // no action if in measurement
    if (measuringData_)
        return;

    measuringData_ = true;

    // reset coesfficients
    if (lampTempK <= 0.0)
        for (int i=0; i<rangePixels_; ++i)
            normCoef_[i] = 1.0;
    else
    {
        if (!useCurrentMeasurement)
        {
            // no external triggering and measuring with existing parameters
            // measure for 1 sec or int.time x 4 - whichever is longer
            readSpectrometer(std::max(getIntTime(T_USEC)*4.1, 1000000.0), false, false);
            processMeasurement(meas_);
        }

        // process data
        int measNormIdx = 0;
        double measNormVal = 0.0;

        // first pass - calculate norm wavelength idx
        bool useMax = normWvlNm < getWavelength(0) ||
                      normWvlNm > getWavelength(rangePixels_-1);
        for (int i=0; i<rangePixels_; ++i)
        {
            double measVal = getLinearisedMeas(i, meas_, getHighGain(LAST_MEAS));
            if (useMax && measVal > measNormVal)
            {
                measNormVal = measVal;
                measNormIdx = i;
            }
            else if (!useMax && getWavelength(i) > normWvlNm)
            {
                measNormVal = measVal;
                measNormIdx = i;
                break;
            }
        }

        if (measNormVal <= 0.0)
        {
            measuringData_ = false;
            return;
        }

        // second pass scale measurement relative to max and calculate normalisation
        double normWv = getWavelength(measNormIdx);  // normalised to measured max
        double hckTA = 6.62606957293*2.99792458/1.38064881313*1000000./lampTempK/AIR_REFRACTION;
        double normVal = normWv*normWv*normWv*normWv*normWv*(exp(hckTA/normWv)-1.0);
        double normEmvTungst = emvTungst(normWv, lampTempK);
        float maxNorm = 0.0;
        for (int i=0; i<rangePixels_; ++i)
        {
            double measuredRelVal = getLinearisedMeas(i, meas_, getHighGain(LAST_MEAS))/measNormVal;
            double curWv = getWavelength(i);
            double calcRelVal = normVal*emvTungst(curWv, lampTempK) /
                    (normEmvTungst*curWv*curWv*curWv*curWv*curWv*(exp(hckTA/curWv)-1.0));
            normCoef_[i] = measuredRelVal>0 ? calcRelVal/measuredRelVal : 0;
            if (useMax && maxNorm < normCoef_[i])
                maxNorm = normCoef_[i];
            else if (!useMax && measNormIdx == i)
                maxNorm = normCoef_[i];
        }

        // third pass - normalise calibration coefficients
        for (int i=0; i<rangePixels_; ++i)
            normCoef_[i] = normCoef_[i] ? normCoef_[i]/maxNorm : 1.0;
    }

    measuringData_ = false;

    if (baseEEPROM_>=0)
    {
        const int normCoeffOffs =
            baseEEPROM_+offsEEPROM_[EEPROM_NORM_COEF_ARRAY];
        for (int i=0; i<rangePixels_; ++i)
            // write spectral response normalisation
            EEPROM.put(normCoeffOffs+i*sizeof(float), normCoef_[i]);
    }
}

// This method calibrates ADC voltages.
//
// This will attempt to set more precise ADC voltages using the given ADC
// reference as a base. The other voltages will be calculated relative to
// the given one. This calibration needs a stable LED light directed at
// spectrometer.
//
// It should be calibrated for accurate bias and linearisation but after
// saturation calibration. Since linearisation is performed for the closest
// larger than saturation voltage, it is generally a good idea to use that
// ADC voltage as a base.
//
void CSpectrometer::calibrateADCVoltages(adc_ref_t baseADC)
{
    // save current state
    uint8_t savedSpecState_ = specState_;
    double  savedIntTimeUs  = getIntTime(T_USEC);

    // store base reference for this calibration
    adc_ref_t baseADC = getAdcReference();

    // maximize measurement for smallest reference and high gain
    setAdcRefInternal(ADC_2_5V);
    if (supportsGain())
        setHighGain(true, false);
    takeAutoMeasurement(AUTO_FOR_SET_REF_GAIN);

    // find range close to upper saturation area
    int startIdx = -1;
    for (int i=0; i<rangePixels_; ++i)
    {
        float measVal = getRawMeasValue(i, false);
        if (measVal > 2.0)
        {
           startIdx = i<5 ? 0 : (i+5 >= rangePixels_ ? rangePixels_-10 : i-5);
           break;
        }
    }
    // exposed pixels
    if (startIdx >= 0)
    {
        static const double refADC[ADC_MAX_VOLTAGES] = {2.5, 3.0, 4.096, 5.0};
        measuringData_ = true;

        double rawMeas[ADC_MAX_VOLTAGES][10];

        for (adc_ref_t curRef = ADC_2_5V; curRef<ADC_MAX_VOLTAGES; ++curRef)
        {
            setAdcRefInternal(curRef);
            readSpectrometer(500000.0, false, false);
            for (int i=0; i<10; ++i)
            {
                uint16_t rawIdx = rangeStartIdx_ + startIdx + i;

                rawMeas[curRef][i] = rawMeasCounts_[rawIdx]
                                        ? ((double)rawMeas_[rawIdx]) /
                                          ((double)rawMeasCounts_[rawIdx])
                                        : 0.0;

                // store base measurements as voltage
                if (curRef == baseADC)
                    rawMeas[curRef][i] *= refADC[baseADC]/ADC_MAX_VALUE;
            }
        }

        // go through measured values and calibrate ADC relative
        // to the base one
        adcVoltages_[baseADC] = refADC[baseADC];
        for (adc_ref_t curRef = ADC_2_5V; curRef<ADC_MAX_VOLTAGES; ++curRef)
        {
            if (curRef != baseADC)
            {
                double curADCVoltage = 0.0;
                int count = 0;
                for (int i=0; i<10; ++i)
                {
                    if (rawMeas[curRef][i])
                    {
                        curADCVoltage += (rawMeas[baseADC][i] * ADC_MAX_VALUE) /
                                          rawMeas[curRef][i];
                        ++count;
                    }
                }
                if (count)
                {
                    adcVoltages_[curRef] = curADCVoltage/count;
                }
            }
        }

        // save the new ADC voltages
        if (baseEEPROM_>=0)
            EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_ADC_VOLTAGES], adcVoltages_);
    }

    // restore state
    specState_ = savedSpecState_;
    setAdcRefInternal(getAdcReference());
    if (supportsGain())
        setHighGain(getHighGain(), false);
    setIntTimeInternal(savedIntTimeUs, false);

    measuringData_ = false;
}

// Returns calculated voltage value from the raw buffer
float CSpectrometer::getRawMeasValue(uint16_t pixelIdx, bool removeBias)
{
    float adcRefVoltage = getAdcRefVoltage();
    uint16_t rawIdx = rangeStartIdx_ + pixelIdx;

    float meas = rawMeasCounts_[rawIdx]
                    ? ((float)rawMeas_[rawIdx]*adcRefVoltage) /
                      ((float)rawMeasCounts_[rawIdx]*ADC_MAX_VALUE)
                    : 0.0;
    if (removeBias)
        meas = meas < bias_[pixelIdx] ? 0.0 : meas - bias_[pixelIdx];

    return meas;
}

// Convert aggregated readouts to voltage measurement floating point data,
// average with the previous reading if specified and return the max value.
// If supplied also calculates std.deviation.
float CSpectrometer::processMeasurement(float* measurement,
                                        bool   avgPrevious,
                                        float* stddev)
{
    if (!rawMeas_ || !rawMeasCounts_)
        return 0.0;

    float maxVal = 0.0;
    float avgVal = 0.0;
    if (stddev)
        *stddev = 0.0;
    for (int i=0; i<rangePixels_; ++i)
    {
        float measVal = getRawMeasValue(i, false);

        if (measVal > 0.0)
            measurement[i] = avgPrevious ? (measurement[i]+measVal)/2.0 : measVal;

        if (stddev)
        {
            if (i)
            {
                float delta = measurement[i] - avgVal;
                avgVal += delta / (i+1);
                *stddev += delta * (measurement[i] - avgVal);
            }
            else
                avgVal = measurement[i];
        }

        if (measurement[i] > maxVal)
            maxVal = measurement[i];
    }

    // update stddev
    if (stddev)
        *stddev = sqrt(*stddev/rangePixels_);

    // store static data ref for this measurement
    if (measurement == meas_)
    {
        lastMeasSpecState_ = specState_;
        setMeasScale();
    }

    return maxVal;
}

// Take spectrometer reading in automatic mode. This does not require
// integration time. Also as a result of measurement it will set the
// integration time measured as well as gain and ADC voltage reference
// (the latter only in some modes). The automatic measurement is tuned to
// use sensor setup to maximise the output ADC resolution/range.
//
// The automatic measurement can be tuned to achieve different results.
// The behavior of it is controlled controlled by autoType parameter and
// has the following modes:
//
//    AUTO_FOR_SET_REF_GAIN - Aims to maximise ADC reading within currently
//                            set reference voltage and gain. I.e. achieving
//                            maximum resolution within selected gain and
//                            reference voltage or saturation limit (whichever
//                            is smaller). Only integration is changed in this
//                            mode.
//
//    AUTO_FOR_SET_REF      - Aims to maximise ADC reading within currently
//                            set reference voltage. I.e. achieving maximum
//                            resolution within selected reference voltage or
//                            saturation limit (whichever is smaller). Only
//                            gain and integration are changed in this mode.
//
//    AUTO_ALL_MIN_INTEG    - Aims to maximise ADC reading across all ADC
//                            reference voltages whilst achieving minimum
//                            integration time. This essentially attempts
//                            to achieve maximum resolution on smallest
//                            reference voltage (to get shortest integration).
//
//    AUTO_ALL_MAX_RANGE    - Aims to maximise ADC reading across all ADC
//                            reference voltages to maximise sensor output.
//                            This method attempts to achieve maximum use of
//                            the sensor output range and attempts to achieve
//                            maximum reading close to sensor saturation.
//
// NOTE: it is essential to set/measure sensor saturation levels for this
//       function to work!
//
// NOTE2: Because it changes exposure time, this mode will reset black level
//        measurements to minimum calibrated by default. This can be omitted
//        if specified. It generally is a good idea to recapture black levels
//        with established exposure parameters after this call to make
//        measurement more precise.
//
void CSpectrometer::takeAutoMeasurement(auto_measure_t autoType,
                                        double measTime,
                                        time_units_t units,
                                        bool doBlackReset,
                                        bool doExtTriggering,
                                        bool saveState)
{
    // no action if in measurement
    if (measuringData_)
        return;

    measuringData_ = true;

    // reset blacks to calibrated minimum
    if (doBlackReset)
        resetBlackLevels();

    if (supportsGain() && autoType != AUTO_FOR_SET_REF_GAIN)
        // High gain is preferable since it allows more optimal filling of
        // ADC reference range. So start with high gain. and check if it's
        // not satuared at shortest exposure
        setHighGain(true, false);

    // change ADC ref if enabled
    if (autoType != AUTO_FOR_SET_REF)
        setAdcRefInternal(ADC_AUTO);

    float satVoltage = getSatVoltage();

    if (satVoltage > getAdcRefVoltage())
        satVoltage = getAdcRefVoltage();

    // delay to stabilise the changes
    delay(50);

    // try to make shortest reading
    setIntTimeInternal(0.0, false);
    readSpectrometer(-1, false, doExtTriggering);
    float maxMeasuredVoltage = processMeasurement(meas_);

    // check for saturated data and repeat with no gain if we support it
    if (supportsGain() &&
        autoType != AUTO_FOR_SET_REF_GAIN &&
        maxMeasuredVoltage > 0.97*satVoltage)
    {
        // too much - set to no gain
        setHighGain(false, false);

        // change ADC ref if enabled
        if (autoType != AUTO_FOR_SET_REF)
            setAdcRefInternal(ADC_AUTO);

        satVoltage = getSatVoltage();

        if (satVoltage > getAdcRefVoltage())
            satVoltage = getAdcRefVoltage();

        // delay to stabilise the changes
        delay(50);

        // repeat shortest measurement
        readSpectrometer(-1, false, doExtTriggering);
        maxMeasuredVoltage = processMeasurement(meas_);
    }

    // Initial setup now done and we have the shortest measurement in a selected
    // gain and encompassing saturation limits (if allowed). Check that it is
    // less then maximum and attempt to maximise it
    if (maxMeasuredVoltage < satVoltage && autoType == AUTO_ALL_MIN_INTEG)
    {
        // For shortest integration time find the smallest encompassing ADC voltage
        uint8_t adcRef = 0;

        while (maxMeasuredVoltage > adcVoltages_[adcRef] && adcRef < ADC_MAX_VOLTAGES-1)
            ++adcRef;

        setAdcRefInternal(adcRef);

        // correct saturation voltage
        if (satVoltage > adcVoltages_[adcRef])
            satVoltage = adcVoltages_[adcRef];

        // delay to stabilise the changes
        delay(50);
    }

    // The measurement aim within 1% from saturation point.
    float targetVoltage      = satVoltage;
    float targetVoltageLower = satVoltage*0.99;

    // get the limits
    double minIntTimeUs = 0.0;
    double maxIntTimeUs = 0.0;
    getSensorConstraint(MIN_INT_TIME_US, &minIntTimeUs);
    getSensorConstraint(MAX_INT_TIME_US, &maxIntTimeUs);

    // now go up the integration within the range until we maximise the exposure
    bool stillGoing = maxMeasuredVoltage < targetVoltageLower;
    while (stillGoing)
    {
        double curIntTimeUs = getIntTime(T_USEC);

        // keep Particle connection alive
        if (Particle.connected())
            Particle.process();

        // calculate new integration time
        if (maxMeasuredVoltage > targetVoltage)
            curIntTimeUs /= 2;
        else if (5*maxMeasuredVoltage < targetVoltage)
            // too little exposure to apply ratio - double it
            curIntTimeUs *= 2;
        else
            curIntTimeUs *= targetVoltage/maxMeasuredVoltage;

        if (curIntTimeUs < minIntTimeUs)
            curIntTimeUs = minIntTimeUs;
        else if (curIntTimeUs > maxIntTimeUs)
            curIntTimeUs = maxIntTimeUs;

        setIntTimeInternal(curIntTimeUs, false);

        // do new reading
        readSpectrometer(-1, false, doExtTriggering);
        maxMeasuredVoltage = processMeasurement(meas_);

        // check exit conditions
        if (maxMeasuredVoltage >= targetVoltageLower && maxMeasuredVoltage < satVoltage)
            stillGoing = false;
        else if (curIntTimeUs >= maxIntTimeUs && maxMeasuredVoltage < targetVoltage)
            stillGoing = false;
        else if (curIntTimeUs <= minIntTimeUs && maxMeasuredVoltage > targetVoltage)
            stillGoing = false;
    }

    // do measurement with established parameters and triggering
    double measTimeUs = time<0 ? 0.0 : timeUnitsToUsec[units] * measTime;
    readSpectrometer(measTimeUs, doExtTriggering, doExtTriggering);
    processMeasurement(meas_);

    lastMeasSpecState_ = specState_;
    setMeasScale();

    measuringData_ = false;

    // save data in EEPROM if specified
    if (saveState)
    {
        setIntTimeInternal(getIntTime(T_USEC), true);

        if (baseEEPROM_>=0)
            EEPROM.put(baseEEPROM_+offsEEPROM_[EEPROM_STATE], specState_);
    }
}

// Take spectrometer reading for time specified (in specified units). If supplied
// time is larger than integration time then take several measurement cycles at
// integration time to fit the specified time period. If specified time is 0 then
// take measurement at set integration time.
void CSpectrometer::takeMeasurement(double measTime, time_units_t units, bool doExtTriggering)
{
    // no action if in measurement
    if (measuringData_)
        return;

    measuringData_ = true;

    // read main measurement data
    double timeUs = time<0 ? 0.0 : timeUnitsToUsec[units] * measTime;
    readSpectrometer(timeUs, doExtTriggering, doExtTriggering);
    processMeasurement(meas_);

    measuringData_ = false;
}

// Take normal spectrometer reading at specified integration time and
// stores it as black level. The time and units parameters are the same as
// described above in takeMeasurement() function.
//
// This allows to do manually controlled measurement of the black level
// reference so it should be taken with spectrometer in the non-illuminated
// conditions.
//
// If avgPrevious is set, then average the measurement with the previous one.
// Averaging like that only makes sense at the same gain and integration time.
void CSpectrometer::takeBlackMeasurement(double measTime, time_units_t units,
                                         bool doExtTriggering, bool avgPrevious)
{
    // no action if in measurement
    if (measuringData_)
        return;

    measuringData_ = true;

    // read black if needed
    double timeUs = time<0 ? 0.0 : timeUnitsToUsec[units] * measTime;
    readSpectrometer(timeUs, doExtTriggering, false);
    processMeasurement(blackLevels_, avgPrevious);

    for (int i=0, biasInvalid_=false; !biasInvalid_ && i<rangePixels_; ++i)
        biasInvalid_ = blackLevels_[i] < bias_[i];

    measuringData_ = false;
}

// Reset black levels to 0
void CSpectrometer::resetBlackLevels(float resetVoltage)
{
    // Initialize arrays
    biasInvalid_ = false;
    if (resetVoltage < 0.0)
        resetVoltage = 0.0;
    for (int i=0; i<rangePixels_; i++)
        blackLevels_[i] = resetVoltage;
}

// Get linearised unscaled measured data for specified pixel
double CSpectrometer::getLinearisedMeas(uint16_t pixelIdx, float* measurement, bool highGain)
{
    double data = measurement[pixelIdx];

    double* linear = linearCoeff_[highGain];

    // take away bias and correct linearity
    data -= bias_[pixelIdx];
    if (data < 0)
        data = 0;

    // linearise - 5 degree polinomial with 0,0 origin
    data = data*(linear[0]+data*(linear[1]+data*(linear[2]+data*(linear[3]+data*linear[4]))));

    // take the black levels out
    data -= getBlackLevel(pixelIdx);
    if (data < 0.0)
        data = 0.0;

    return data;
}

// Get measured data for specified pixel
// Note: Applying bandpass correction can go out of range
double CSpectrometer::getMeasurement(uint16_t pixelIdx, bool rawMeas, bool applyBandpassCorrection)
{
    double data = meas_[pixelIdx];

    if (!rawMeas)
    {
        bool lastHighGain = getHighGain(LAST_MEAS);
        bool saturated = data > satVoltage_[lastHighGain];

        // take away bias and correct linearity
        data = getLinearisedMeas(pixelIdx, meas_, lastHighGain);

        // scale to the gain and integration time
        data *= lastMeasScale_;

        // apply spectral response corrections
        if (!saturated)
            data *= normCoef_[pixelIdx];
    }

    if (applyBandpassCorrection)
    {
        // Stearns and Stearns (1988) bandpass correction
        // Note: recursive calls so definitely off with bandpass in them
        if (pixelIdx == 0)
            data = 1.083*data - 0.083*getMeasurement(pixelIdx+1, rawMeas, false);
        else if (pixelIdx == rangePixels_-1)
            data = 1.083*data - 0.083*getMeasurement(pixelIdx-1, rawMeas, false);
        else
            data = 1.166*data - 0.083*getMeasurement(pixelIdx-1, rawMeas, false)
                              - 0.083*getMeasurement(pixelIdx+1, rawMeas, false);
    }

    return data;
}

// get the wavelength for specified pixel
double CSpectrometer::getWavelength(uint16_t pixelNumber)
{
    // pixelNumber in formula start with 1
    double p = pixelNumber+rangeStartIdx_+1;
    return calibration_[0]
           + p*(calibration_[1]
           + p*(calibration_[2]
           + p*(calibration_[3]
           + p*(calibration_[4]
           + p* calibration_[5]))));
}

// Get ADC reference
adc_ref_t CSpectrometer::getAdcReference(bool lastMeas)
{
    return (adc_ref_t)((lastMeas ? lastMeasSpecState_ : specState_) & ADC_REF_MASK);
}

// Get ADC reference
adc_ref_t CSpectrometer::getAdcReference(double voltage)
{
    ref = 0;

    while (std::abs(voltage-adcVoltages_[ref]) > 0.001 && ref < ADC_MAX_VOLTAGES-1)
        ++ref;

    return (adc_ref_t)ref;
}

// Get ADC reference voltage
double CSpectrometer::getAdcRefVoltage(bool lastMeas)
{
    return adcVoltages_[getAdcReference(lastMeas)];
}
// Get ADC reference voltage
double CSpectrometer::getAdcRefVoltage(adc_ref_t ref)
{
    if (ref == ADC_AUTO)
    {
        float satVoltage = satVoltage_[getHighGain()];
        ref = 0;

        while ((satVoltage-adcVoltages_[ref]) > 0.001 && ref < ADC_MAX_VOLTAGES-1)
            ++ref;
    }
    return adcVoltages_[ref];
}