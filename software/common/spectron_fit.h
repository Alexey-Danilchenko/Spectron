/*
 *  spectron_fit.h - Implementation of Gaussian and polynomial fitting for 
 *                   Spectron project
 *
 *  Copyright 2018-2019 Alexey Danilchenko
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
#ifndef _SPECTRAL_FIT_H
#define _SPECTRAL_FIT_H

#include <vector>
#include <algorithm>
#include <stdlib.h>

/**
 * Performs a polynomial regression. I.e. fitting a polynomial curve of a given
 * order to a set of points using least-squares approach.
 *
 * We can model the expected value y as an nth degree polynomial, yielding
 * the general polynomial regression model:
 *
 * y = a0 + a1 * x + a2 * x^2 + ... + an * x^n
 *
 * Adapted from Chris Engelsma open implementation 
 *     https://gist.github.com/chrisengelsma/108f7ab0a746323beaaf7d6634cf4add
 */
template <class TYPE>
bool fitPoly(const std::vector<TYPE>& x,
             const std::vector<TYPE>& y,
             const int                order,
             std::vector<TYPE>&       result)
{
    // Check the sizes to be non empty and equal
    if (x.size() != y.size() || x.empty() || y.empty())
        return false;

    int nSamples = x.size();
    int n = order;
    int np1 = n + 1;
    int np2 = n + 2;
    int tnp1 = 2 * n + 1;
    TYPE tmp;

    // X = vector that stores values of sigma(xi^2n)
    // Y = vector to store values of sigma(xi^n * yi)
    std::vector<TYPE> X(tnp1);
    std::vector<TYPE> Y(np1);
    for (int i = 0; i < tnp1; ++i) 
    {
        X[i] = 0;
        if (i< np1)
            Y[i] = 0;
        for (int j = 0; j < nSamples; ++j)
        {
            TYPE xM = i==0 ? 1 : (i==1 ? x[j] : (TYPE)pow(x[j], i));
            X[i] += xM;
            if (i< np1)
                Y[i] += xM*y[j];
        }
    }

    // resize result vector to store final coefficients 
    result.resize(np1);

    // B = normal augmented matrix that stores the equations.
    std::vector<std::vector<TYPE> > B(np1, std::vector<TYPE> (np2, 0));

    for (int i = 0; i <= n; ++i) 
        for (int j = 0; j <= n; ++j) 
              B[i][j] = X[i + j];

    // Load values of Y as last column of B
    for (int i = 0; i <= n; ++i) 
        B[i][np1] = Y[i];

    n += 1;
    int nm1 = n-1;

    // Pivotisation of the B matrix.
    for (int i = 0; i < n; ++i) 
        for (int k = i+1; k < n; ++k) 
            if (B[i][i] < B[k][i]) 
                for (int j = 0; j <= n; ++j) 
                {
                    tmp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = tmp;
                }

    // Performs the Gaussian elimination.
    // (1) Make all elements below the pivot equals to zero
    //     or eliminate the variable.
    for (int i=0; i<nm1; ++i)
        for (int k =i+1; k<n; ++k) 
        {
            TYPE t = B[k][i] / B[i][i];
            for (int j=0; j<=n; ++j)
                B[k][j] -= t*B[i][j];         // (1)
        }

    // Back substitution.
    // (1) Set the variable as the rhs of last equation
    // (2) Subtract all lhs values except the target coefficient.
    // (3) Divide rhs by coefficient of variable being calculated.
    for (int i=nm1; i >= 0; --i) 
    {
        result[i] = B[i][n];                        // (1)
        for (int j = 0; j<n; ++j)
            if (j != i)
                result[i] -= B[i][j] * result[j];   // (2)
        result[i] /= B[i][i];                       // (3)
    }

    return true;
}

/**
 * Performs a Gaussian fit. I.e. fitting a Gaussian curve to a set of points 
 * using Caruana approach. The approach does not work well with tail values 
 * of Gaussian curve especially if they contain substantial errors.  In that
 * case iterative fit should be used (see fitGaussIterative).
 *
 * Gaussian function being fit is considered to be in this format:
 *             -(x-m)^2/2q^2
 *       y = Ae
 *
 * Resulting fit sets the following return values:
 *     middle - m from above formula
 *     width  - q from above formula
 *     height - A from the above formula
 */
template <class TYPE>
bool fitGauss(const std::vector<TYPE>& x,
              const std::vector<TYPE>& y,
              TYPE& middle,
              TYPE& width,
              TYPE& height)
{
    // Check the sizes to be non empty and equal
    if (x.size() != y.size() || x.empty() || y.empty())
        return false;
    
    int nSamples = x.size();
    std::vector<TYPE> logY(nSamples);
    std::vector<TYPE> quadParams;
    
    // convert vector y to logarithmic
    for (int i=0; i<nSamples; i++)
        logY[i] = (TYPE)log(y[i]);

    // perform quadratic fit
    if (!fitPoly(x, logY, 2, quadParams))
        return false;
    
    // calculate Gaussian curve parameters
    // m = -b / 2c
    middle = -quadParams[1] / (2*quadParams[2]);
    // q = sqrt(-1/2c)
    width = sqrt(-1 / (2*quadParams[2]));
    //      a-b^2/4c
    // A = e
    height = exp(quadParams[0] - (quadParams[1]*quadParams[1] / (4*quadParams[2])));
    
    return true;
}

/**
 * Performs a Gaussian fit. I.e. fitting a Gaussian curve to a set of points 
 * using modification of Caruana. The only limitation of this method is that
 * it fails when data is offset from 0 - i.e. for captured sensor data the
 * offset is present in captured signal. Offsetting the set of data by minimal 
 * value helps to cater for such cases.
 *
 * For more details refer to https://www.researchgate.net/publication/252062037 
 *
 * Gaussian function being fit is considered to be in this format:
 *             -(x-m)^2/2q^2
 *       y = Ae
 *
 * Resulting fit sets the following return values:
 *     middle - m from above formula
 *     width  - q from above formula
 *     height - A from the above formula
 *
 * Retuns 0 if unsuccessful and number of iterations if it is
 */
template <class TYPE>
int fitGaussIterative(const std::vector<TYPE>& x,
                      const std::vector<TYPE>& y,
                      TYPE& middle,
                      TYPE& width,
                      TYPE& height,
                      const int maxIterations = 10)
{
    const double PARAM_FITTING_PRECISION = 0.00001;
    
    // Check the sizes to be non empty and equal
    if (x.size() != y.size() || x.empty() || y.empty() || maxIterations < 1)
        return 0;

    int nSamples = x.size();
    TYPE tmp;

    // X = vector that stores values of sigma(y^2*xi^2n)
    std::vector<TYPE> X(5);
    std::vector<TYPE> Y(3);
    std::vector<TYPE> coef(3);
    // B = normal augmented matrix that stores the equations.
    std::vector<std::vector<TYPE> > B(3, std::vector<TYPE> (4, 0));

    TYPE prevMiddle = 0, prevWidth = 0, prevHeight = 0;

    // start iterative fitting
    int iter = 0;
    for (; iter < maxIterations; iter++)
    {
        // check if precision has been reached
        if (iter > 1 && coef[2] < 0 &&
            fabs(prevMiddle - middle) < PARAM_FITTING_PRECISION &&
            fabs(prevWidth -  width)  < PARAM_FITTING_PRECISION &&
            fabs(prevHeight - height) < PARAM_FITTING_PRECISION)
            break;
    
        // Fill in X with sigma(yM^2 * xi^2n) and simultaneously set
        // Y = vector to store values of sigma(xi^2 * yM^2 * ln(yi))
        for (int i = 0; i < 5; ++i) 
        {
            X[i] = 0;
            if (i<3)
                Y[i] = 0;
            for (int j = 0; j < nSamples; ++j)
                if (y[j]>0)
                {
                    TYPE yM = 
                        iter > 0 ? exp (coef[0] + coef[1]*x[j] + coef[2]*x[j]*x[j]) 
                                 : y[j];
                    TYPE xM = (TYPE)pow(x[j], i)*yM*yM;
                    X[i] += xM;
                    if (i<3)
                         Y[i] += xM*log(y[j]);
                }
        }

        // Populate main matrix from X
        for (int i = 0; i <= 2; ++i) 
            for (int j = 0; j <= 2; ++j) 
                  B[i][j] = X[i + j];

        // Load values of Y as last column of B
        for (int i = 0; i <= 2; ++i) 
            B[i][3] = Y[i];

        // Pivotisation of the B matrix.
        for (int i = 0; i < 3; ++i) 
            for (int k = i+1; k < 3; ++k) 
                if (B[i][i] < B[k][i]) 
                    for (int j = 0; j <= 3; ++j) 
                    {
                        tmp = B[i][j];
                        B[i][j] = B[k][j];
                        B[k][j] = tmp;
                    }

        // Performs the Gaussian elimination.
        // (1) Make all elements below the pivot equals to zero
        //     or eliminate the variable.
        for (int i=0; i<2; ++i)
            for (int k =i+1; k<3; ++k) 
            {
                TYPE t = B[k][i] / B[i][i];
                for (int j=0; j<=3; ++j)
                    B[k][j] -= t*B[i][j];         // (1)
            }

        // Back substitution.
        // (1) Set the variable as the rhs of last equation
        // (2) Subtract all lhs values except the target coefficient.
        // (3) Divide rhs by coefficient of variable being calculated.
        for (int i=2; i >= 0; --i) 
        {
            coef[i] = B[i][3];                        // (1)
            for (int j = 0; j<3; ++j)
                if (j != i)
                    coef[i] -= B[i][j] * coef[j];     // (2)
            coef[i] /= B[i][i];                       // (3)
        }
        
        // calculate Gaussian curve parameters for this iteration
        // m = -b / 2c
        prevMiddle = middle;
        middle = -coef[1] / (2*coef[2]);
        
        // q = sqrt(-1/2c)
        prevWidth = width;
        width = sqrt(-1 / (2*coef[2]));
        
        //      a-b^2/4c
        // A = e
        prevHeight = height;
        height = exp(coef[0] - (coef[1]*coef[1] / (4*coef[2])));
    }

    return iter;
}

/**
 * Performs a rational polynomial curve fitting to a set of points using
 * least-squares approach.
 * 
 * The rational functions used in this calculation is limited to this
 * representation (any general representation with b0 != 1 
 * could be normalised to this by dividing both parts by b0):
 *
 *    y = (a0 + a1*x + a2*x^2 + ... + ak*x^k) / (1 + b1*x + ... + bm*x^m)
 *
 * Results retuned as a vector of: a0,...,ak,b1,...,bm
 */
template <class TYPE>
bool fitRational(const std::vector<TYPE>& x,
                 const std::vector<TYPE>& y,
                 const int                numeratorOrder,
                 const int                denominatorOrder,
                 std::vector<TYPE>&       result)
{
    // Check the sizes to be non empty and equal
    if (x.size() != y.size() || x.empty() || y.empty())
        return false;

    int nSamples = x.size();
    int k = numeratorOrder;
    int m = denominatorOrder;
    int n = k + m;
    int tnp1 = 2 * n + 1;
    int tkp1 = 2 * k + 1;
    int tm   = 2 * m;
    int maxIter = std::max(tkp1, std::max(k+m, tm));

    // X   = vector that stores values of  sigma(xi^2k)       (0..2k)
    // XY  = vector that stores values of -sigma(yi*xi^(k+m)) (1..k+m)
    // XY2 = vector that stores values of  sigma(yi^2*xi^2m)  (2..2m)
    // Y   = vector that stores values of  sigma(xi^n*yi) and -sigma(xi^m*yi^2)
    std::vector<TYPE> X(tkp1);
    std::vector<TYPE> XY(k+m);
    std::vector<TYPE> XY2(tm);
    std::vector<TYPE> Y(n+1);
    for (int i = 0; i < maxIter; ++i) 
    {
        if (i<tkp1)
            X[i] = 0;
        if (i && i<=k+m)
            XY[i-1] = 0;
        if (i>1 && i<=tm)
            XY2[i-2] = 0;
        if (i<=k)
            Y[i] = 0;
        if (i && i<=m)
            Y[i+k] = 0;
        for (int j = 0; j < nSamples; ++j)
        {
            TYPE xM = i==0 ? 1 : (i==1 ? x[j] : (TYPE)pow(x[j], i));

            if (i<tkp1)
                X[i] += xM;
            if (i && i<=k+m)
                XY[i-1] -= xM*y[j];
            if (i>1 && i<=tm)
                XY2[i-2] += xM*y[j]*y[j];
            if (i<=k)
                Y[i] += xM*y[j];
            if (i && i<=m)
                Y[i+k] -= xM*y[j]*y[j];
        }
    }

    // Resize result vector to store final coefficients 
    result.resize(n+1);

    // B = normal augmented matrix that stores the equations.
    std::vector<std::vector<TYPE> > B(n+1, std::vector<TYPE> (n+2, 0));
    
    // Populate matrix from the aggergated vectors
    for (int i = 0; i <= n; ++i)
        for (int j = 0; j <= n; ++j)
            if (i<=k && j<=k)
                B[i][j] = X[i+j];
            else if (i<=k || j<=k)
                B[i][j] = XY[i+j-k-1];
            else
                B[i][j] = XY2[i+j-tkp1-1];

    // Load values of Y as last column of B
    for (int i = 0; i <= n; ++i) 
        B[i][n+1] = Y[i];

    // Matrix now loaded - perform Gaussian elimination
    n += 1;
    int nm1 = n-1;

    // Pivotisation of the B matrix.
    for (int i=0; i<n; ++i) 
        for (int k=i+1; k<n; ++k) 
            if (B[i][i] < B[k][i]) 
                for (int j = 0; j <= n; ++j) 
                {
                    TYPE tmp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = tmp;
                }

    // Performs the Gaussian elimination.
    // (1) Make all elements below the pivot equals to zero
    //     or eliminate the variable.
    for (int i=0; i<nm1; ++i)
        for (int k=i+1; k<n; ++k) 
        {
            TYPE t = B[k][i] / B[i][i];
            for (int j=0; j<=n; ++j)
                B[k][j] -= t*B[i][j];         // (1)
        }

    // Back substitution.
    // (1) Set the variable as the rhs of last equation
    // (2) Subtract all lhs values except the target coefficient.
    // (3) Divide rhs by coefficient of variable being calculated.
    for (int i=nm1; i >= 0; --i) 
    {
        result[i] = B[i][n];                        // (1)
        for (int j = 0; j<n; ++j)
            if (j != i)
                result[i] -= B[i][j] * result[j];   // (2)
        result[i] /= B[i][i];                       // (3)
    }

    return true;
}

/**
 * Performs a rational polynomial 2/2 curve fitting to a set of points using
 * least-squares approach. This implementation is tuned for embedded devices.
 * 
 * The rational function fitted here is:
 *
 *    y = (a0 + a1*x + a2*x^2) / (1 + b1*x + b2*x^2)
 *
 * Results retuned as a vector of doubles: a0,a1,a2,b1,b2. 
 * Results array change between invocations so need to be saved.
 */
double* fitRational2(const double *x,
                     const double *y,
                     const double nSamples)
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
    
    // conatsnts and copy values 
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

#endif
