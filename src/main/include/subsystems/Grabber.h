#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <utility/MotorUtils.h>

class Grabber : public frc2::SubsystemBase
{
public:
    Grabber();

private:
    ctre::phoenix6::hardware::TalonFX m_right_motor{CONSTANTS::GRABBER::RIGHT_CAN_ID};
    ctre::phoenix6::hardware::TalonFX m_left_motor{CONSTANTS::GRABBER::LEFT_CAN_ID};
};