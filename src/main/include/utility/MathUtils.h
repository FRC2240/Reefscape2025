#pragma once
#include <frc/geometry/Pose2d.h>


namespace MathUtils {
    // Does the pythagorean theorem
    template <typename T>
    T pythag(T a, T b) {
        return std::sqrt(a * a + b * b);
    }

    frc::Pose2d getMiddlePose(frc::Pose2d p1, frc::Pose2d p2) {
        frc::Translation2d middleT = (p1.Translation() + p2.Translation()) / 2;
        frc::Rotation2d middleR = (p1.Rotation() + p2.Rotation()) / 2;

        return frc::Pose2d(middleT, middleR);
    }

    units::meter_t getDistance(frc::Pose2d p1, frc::Pose2d p2) {
        units::meter_t xDist = p1.Translation().X() - p2.Translation().X();
        units::meter_t yDist = p1.Translation().Y() - p2.Translation().Y();

        return pythag(xDist, yDist)
    }
}