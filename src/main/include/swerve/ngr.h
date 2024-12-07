#pragma once

#ifndef CFG_NO_DRIVEBASE

#include <type_traits>
#include <array>

constexpr bool debugging = true;

/******************************************************************/
/*               Public Helper Function Definitions               */
/******************************************************************/
namespace ngr
{
    [[nodiscard]] constexpr bool valueInRange(double const &value, double const &min, double const &max)
    {
        return value > min && value < max;
    }
    static_assert(valueInRange(1, 0, 2) == true, "Error with valueInRange()");
    static_assert(valueInRange(0, 0, 1) == false, "Error with valueInRange()");
    static_assert(valueInRange(1, 0, 1) == false, "Error with valueInRange()");
    static_assert(valueInRange(2, 0, 1) == false, "Error with valueInRange()");
    static_assert(valueInRange(-1, 0, 1) == false, "Error with valueInRange()");

    // Constexpr floating point comparison
    [[nodiscard]] constexpr bool isCloseTo(double value,
                                           double target,
                                           double tol = 0.00001)
    {
        return std::abs(value - target) < tol;
    }

    static_assert(isCloseTo(.5, .500000001) == true, "Error in isCloseTo()");
    static_assert(isCloseTo(.1, .999989) == false, "Error in isCloseTo()");
    static_assert(isCloseTo(.1, .10002) == false, "Error in isCloseTo()");

    // Scale a value from one range of values to another
    [[nodiscard]] constexpr double scaleOutput(double inputMin, double inputMax, double outputMin, double outputMax, double input)
    {
        return ((input - inputMin) / (inputMax - inputMin)) * ((outputMax - outputMin)) + outputMin;
    }

    static_assert(isCloseTo(scaleOutput(0, 1, -1, 1, 0), -1), "Error with scaleOutput()");
    static_assert(isCloseTo(scaleOutput(0, 1, -1, 1, 1), 1), "Error with scaleOutput()");
    static_assert(isCloseTo(scaleOutput(0, 1, -1, 1, .5), 0), "Error with scaleOutput()");

    // Bad, but good enough implementation of std::midpoint from C++20
    // Can be removed if upgraded to C++20
    template <typename A, typename B>
    constexpr std::common_type_t<A, B> midpoint(A const &a, B const &b)
    {
        return (a + b) / 2;
    }

    static_assert(midpoint(1, 3) == 2, "Error with midpoint()");
    static_assert(midpoint(-1, 3) == 1, "Error with midpoint()");
    static_assert(midpoint(-10, -4) == -7, "Error with midpoint()");

    // Constexpr version of isSorted
    // Can be removed if upgraded to C++20
    template <class ForwardIt, class SortingPredicate>
    [[nodiscard]] constexpr bool isSorted(ForwardIt first, ForwardIt last,
                                          SortingPredicate const &sorting_test)
    {
        auto current = first;
        auto next = first;

        while (++next < last)
        {
            if (!sorting_test(*(current++), *next))
                return false;
        }
        return true;
    }

    static_assert([]()
                  {
                    constexpr std::array<double, 3> ascending_array{-.09, 1, 10000};
                    constexpr std::array<double, 3> descending_array{1, -0.8, -33000};
                    
                    return isSorted(ascending_array.begin(), ascending_array.end(),
                                    [](auto const &l, auto const &r)
                                    { return r > l; }) &&
                           isSorted(descending_array.begin(), descending_array.end(),
                                    [](auto const &l, auto const &r)
                                    { return l > r; }) &&
                           !isSorted(ascending_array.begin(), ascending_array.end(),
                                     [](auto const &l, auto const &r)
                                     { return l > r; }) &&
                           !isSorted(descending_array.begin(), descending_array.end(),
                                     [](auto const &l, auto const &r)
                                     { return r > l; }); }(),
                  "Error with isSorted()");

    // Constexpr version of findIf
    // Can be removed if upgraded to C++20
    template <class InputIt, class UnaryPredicate>
    constexpr InputIt findIf(InputIt first, InputIt last, UnaryPredicate find_test)
    {
        for (auto current = first; current != last; current++)
        {
            if (find_test(*current))
                return current;
        }
        return last;
    }

    static_assert([]()
                  {
                    constexpr std::array<double, 3> array{-1, 1, 10000};

                    return findIf(array.begin(), array.end(),
                                    [](auto const &i)
                                    { return i == -1; }) == array.begin() &&
                           findIf(array.begin(), array.end(),
                                   [](auto const &i)
                                    { return i == 1; }) == array.begin() + 1 &&
                           findIf(array.begin(), array.end(),
                                    [](auto const &i)
                                    { return i == 10000; }) == array.begin() + 2 &&
                           findIf(array.begin(), array.end(),
                                     [](auto const &i)
                                    { return i == -234908.2; }) == array.end(); }(),
                  "Error with findIf()");

} // namespace ngr
#endif
