#pragma once
#include <frc/geometry/Pose2d.h>
#include <units/math.h>

namespace MathUtils
{
    template <typename T>
    T pythag(T a, T b);

    
    units::meter_t distPythagorean(units::meter_t a, units::meter_t b);
    frc::Pose2d getMiddlePose(frc::Pose2d p1, frc::Pose2d p2);
    units::meter_t getDistance(frc::Pose2d p1, frc::Pose2d p2);
}