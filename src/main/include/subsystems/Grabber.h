#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <utility/MotorUtils.h>
#include <TimeOfFlight.h>
#include <frc2/command/Commands.h>

class Grabber : public frc2::SubsystemBase
{
public:
    Grabber();
    frc2::CommandPtr intake_command(units::turns_per_second_t speed);
    frc2::CommandPtr extake_command(units::turns_per_second_t speed, units::second_t time);

private:
    ctre::phoenix6::hardware::TalonFX m_right_motor{CONSTANTS::GRABBER::RIGHT_ID};
    ctre::phoenix6::hardware::TalonFX m_left_motor{CONSTANTS::GRABBER::LEFT_ID};
    pwf::TimeOfFlight Grabber_sensor{CONSTANTS::GRABBER::TOF_ID};

    void spin(units::turns_per_second_t speed);
};