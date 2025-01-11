#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
<<<<<<< HEAD
#include <utility/MotorUtils.h>
=======
>>>>>>> 1f3fd3a6838768b023e540b947cab00886cb9cc3

class Grabber : public frc2::SubsystemBase {
    public:
        Grabber();
    private:
<<<<<<< HEAD
        ctre::phoenix6::hardware::TalonFX m_right_motor{CONSTANTS::GRABBER::right_grabber_motor_CAN_ID };
        ctre::phoenix6::hardware::TalonFX m_left_motor{CONSTANTS::GRABBER::left_grabber_motor_CAN_ID};
=======
        ctre::phoenix6::hardware::TalonFX right_grabber_motor{CONSTANTS::GRABBER::right_grabber_motor_CAN_ID };
        ctre::phoenix6::hardware::TalonFX left_grabber_motor{CONSTANTS::GRABBER::left_grabber_motor_CAN_ID};
>>>>>>> 1f3fd3a6838768b023e540b947cab00886cb9cc3
};