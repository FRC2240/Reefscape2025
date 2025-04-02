#include "utility/MathUtils.h"

template <typename T>
T MathUtils::pythag(T a, T b)
{
    return std::sqrt(a * a + b * b);
}

template double MathUtils::pythag<double>(double, double);
template float MathUtils::pythag<float>(float, float);

units::meter_t MathUtils::distPythagorean(units::meter_t a, units::meter_t b)
{
    return units::math::sqrt(a * a + b * b);
}

frc::Pose2d MathUtils::getMiddlePose(frc::Pose2d p1, frc::Pose2d p2)
{
    frc::Translation2d middleT = (p1.Translation() + p2.Translation()) / 2;
    frc::Rotation2d middleR = (p1.Rotation() + p2.Rotation()) / 2;

    return frc::Pose2d(middleT, middleR);
}

units::meter_t MathUtils::getDistance(frc::Pose2d p1, frc::Pose2d p2)
{
    units::meter_t xDist = p1.Translation().X() - p2.Translation().X();
    units::meter_t yDist = p1.Translation().Y() - p2.Translation().Y();

    return distPythagorean(xDist, yDist);
}