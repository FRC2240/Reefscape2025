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

        frc2::CommandPtr setPositionCommand(units::angle::turn_t pos);
        frc2::CommandPtr idleCommand();    
        frc2::CommandPtr followJoystickCommand(frc2::CommandXboxController* stick);

        units::angle::turn_t getPosition();
        void setPosition(units::angle::turn_t pos);

    private:
        ctre::phoenix6::hardware::TalonFX m_motor{CONSTANTS::ELEVATOR::ELEVATOR_ID};
};