#pragma once

#include "utility/BetterSubsystemBase.h"
#include "utility/MotorUtils.h"
#include "Constants.h"

#include <units/angle.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "frc2/command/CommandPtr.h"


class Wrist : public BetterSubsystemBase
{
public:

    Wrist();

    void set_angle(units::angle::degree_t angle);

    units::degree_t get_angle();

    frc2::CommandPtr set_angle_command(units::degree_t pos);

private:
    ctre::phoenix6::hardware::TalonFX m_motor {CONSTANTS::WRIST::WRIST_ID};
};