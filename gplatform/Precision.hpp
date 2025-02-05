//
// Created by Gaëtan Blaise-Cazalet on 13/01/2025.
//

#ifndef PRECISION_HPP
#define PRECISION_HPP

#include "Defines.hpp"

#define SINGLE_PRECISION 1
#define DOUBLE_PRECISION 0

#if SINGLE_PRECISION

    using real = f32;
    /** Defines the highest value for the real number. */
    #define REAL_MAX FLT_MAX

    /** Defines the precision of the square root operator. */
    #define RealSqrt sqrtf

    /** Defines the precision of the absolute magnitude operator. */
    #define RealAbs fabsf

    /** Defines the precision of the sine operator. */
    #define RealSin sinf

    /** Defines the precision of the cosine operator. */
    #define RealCos cosf

    /** Defines the precision of the exponent operator. */
    #define RealExp expf

    /** Defines the precision of the power operator. */
    #define RealPow powf

    /** Defines the precision of the floating point modulo operator. */
    #define RealFMod fmodf

    /** Defines the number e on which 1+e == 1 **/
    #define realEpsilon FLT_EPSILON

    #define PI 3.14159f

#elif DOUBLE_PRECISION

    using real = f64;
    #define REAL_MAX DBL_MAX
    #define RealSqrt sqrt
    #define RealAbs fabs
    #define RealSin sin
    #define RealCos cos
    #define RealExp exp
    #define RealPow pow
    #define RealFMod fmod
    #define realEpsilon DBL_EPSILON
    #define PI 3.14159265358979

#endif

#endif //PRECISION_HPP
