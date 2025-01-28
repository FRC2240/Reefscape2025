#pragma once

#include "Constants.h"
#include "utility/MotorUtils.h"

#include <units/angle.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "frc2/command/CommandPtr.h"
#include <frc2/command/RunCommand.h>

class Climber : public frc2::SubsystemBase
{
public:
    Climber();

    frc2::CommandPtr set_position_command(units::angle::turn_t pos);
    frc2::CommandPtr idle_command();
    frc2::CommandPtr climb_command();
    frc2::CommandPtr extend_command();

    units::degree_t get_angle();
    void set_position(units::angle::turn_t pos);

private:
    ctre::phoenix6::hardware::TalonFX m_motor{CONSTANTS::CLIMBER::CLIMBER_ID};
};