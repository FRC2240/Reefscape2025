#pragma once

#include "Constants.h"
#include "utility/BetterSubsystemBase.h"
#include "utility/MotorUtils.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "frc2/command/CommandPtr.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/RunCommand.h>

class Elevator: public BetterSubsystemBase {

    public:
        Elevator();

        frc2::CommandPtr set_position_command(units::angle::turn_t pos);
        frc2::CommandPtr idle_command();    
        frc2::CommandPtr follow_joystick_command(frc2::CommandXboxController* stick);

        units::angle::turn_t get_position();
        void set_position(units::angle::turn_t pos);

    private:
        ctre::phoenix6::hardware::TalonFX m_motor{CONSTANTS::ELEVATOR::ELEVATOR_ID};
};