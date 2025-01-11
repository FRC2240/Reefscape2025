#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"
#include <frc2/command/SubsystemBase.h>

class Grabber : public frc2::SubsystemBase {
    public:
        Grabber();
    private:
        ctre::phoenix6::hardware::TalonFX right_grabber_motor{CONSTANTS::GRABBER::right_grabber_motor_CAN_ID };
        ctre::phoenix6::hardware::TalonFX left_grabber_motor{CONSTANTS::GRABBER::left_grabber_motor_CAN_ID};
};