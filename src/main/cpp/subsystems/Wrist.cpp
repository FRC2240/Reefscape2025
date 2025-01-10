#include "subsystems/Wrist.h"
#include "Constants.h"

#include "frc2/command/FunctionalCommand.h"

Wrist::Wrist() {
    MotorUtils::Motor::LogValues logValues {true, true, true};
    MotorUtils::Motor wristMotor{&m_motor, CONSTANTS::WRIST::PidValue, logValues};
    AddPID(wristMotor);
    SetPID();

    m_motor.SetPosition(0_tr);
}

void Wrist::set_angle(units::angle::degree_t angle) {
    // Not sure if this is how I'm supposed to do it but it can change
    ctre::phoenix6::controls::PositionVoltage req{angle};
    m_motor.SetControl(req);
}

units::degree_t Wrist::get_angle()
{
  return m_motor.GetPosition().GetValue();
}

frc2::CommandPtr Wrist::set_angle_cmd(units::degree_t angle) {
    std::function<void()> init = [this] {};
    std::function<void()> periodic = [this, &angle]()
    {
        try
        {
            set_angle(angle);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    };

    std::function<bool()> is_finished = [this, &angle]() -> bool 
    {
        try
        {
            return CONSTANTS::IN_THRESHOLD<units::turn_t>(get_angle(), angle, 1_deg);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        return false;
    };

    std::function<void(bool IsInterrupted)> end = [this](bool IsInterrupted) {};

    return frc2::FunctionalCommand(init, periodic, end, is_finished, {this}).ToPtr();
}