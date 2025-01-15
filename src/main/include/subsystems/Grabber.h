#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <utility/MotorUtils.h>
#include "utility/BetterSubsystemBase.h"
#include <TimeOfFlight.h>
#include <frc2/command/Commands.h>


class Grabber : public BetterSubsystemBase
{
public:
    Grabber();

private:
    ctre::phoenix6::hardware::TalonFX m_right_motor{CONSTANTS::GRABBER::RIGHT_ID};
    ctre::phoenix6::hardware::TalonFX m_left_motor{CONSTANTS::GRABBER::LEFT_ID};
    pwf::TimeOfFlight Grabber_sensor{CONSTANTS::GRABBER::TOF_ID};

    void spin(units::turns_per_second_t speed);
    frc2::CommandPtr intake(units::turns_per_second_t speed);
    frc2::CommandPtr extake(units::turns_per_second_t speed, units::second_t time);
};