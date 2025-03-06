#pragma once

namespace MathUtils {
    // Does the pythagorean theorem
    template <typename T>
    T pythag(T a, T b) {
        return std::sqrt(a * a + b * b);
    }
}