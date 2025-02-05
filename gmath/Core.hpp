//
// Created by gaetz on 02/02/2025.
//

#ifndef GMATH_CORE_HPP
#define GMATH_CORE_HPP

#include <Defines.hpp>

namespace gmath {

    inline float Lerp(f32 a, f32 b, float f) {
        return a + f * (b - a);
    }

    inline float Lerp(f64 a, f64 b, float f) {
        return a + f * (b - a);
    }

}


#endif //GMATH_CORE_HPP
