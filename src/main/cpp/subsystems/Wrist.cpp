#include "subsystems/Wrist.h"

#include "frc2/command/RunCommand.h"

Wrist::Wrist() {
    MotorUtils::Motor::LogValues logValues {true, true, true};
    MotorUtils::Motor wristMotor{&m_motor, CONSTANTS::WRIST::PidValue, logValues};
    AddPID(wristMotor);
    SetPID();

    m_motor.SetPosition(CONSTANTS::WRIST::DEFAULT_POSITION);
}

void Wrist::set_angle(units::angle::degree_t angle) {
    m_motor.SetControl(ctre::phoenix6::controls::PositionVoltage {angle});
}

units::degree_t Wrist::get_angle()
{
  return m_motor.GetPosition().GetValue();
}

frc2::CommandPtr Wrist::set_angle_command(units::degree_t pos) {
    return frc2::RunCommand([this, pos] {
        set_angle(pos);
    },
    {this}).Until([this, pos] -> bool {
        CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), pos, CONSTANTS::WRIST::POSITION_THRESHOLD);
    }).WithName("Set Wrist Angle");
}
