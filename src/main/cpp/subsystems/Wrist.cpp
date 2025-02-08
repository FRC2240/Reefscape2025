#include "subsystems/Wrist.h"

#include "frc2/command/RunCommand.h"

Wrist::Wrist() {
  // MotorUtils::Motor::LogValues logValues{true, true, true};
  // MotorUtils::Motor wristMotor{&m_motor, CONSTANTS::WRIST::PidValue,
  // logValues};
  SetPID();

  m_motor.SetPosition(CONSTANTS::WRIST::DEFAULT_POSITION);
}

void Wrist::InitSendable(wpi::SendableBuilder &builder) {
  builder.SetSmartDashboardType("Wrist");
  MotorUtils::BuildSender(builder, &coeff);
  MotorUtils::BuildSender(builder, &m_motor);
}

void Wrist::SetPID() { MotorUtils::SetPID(m_motor, coeff); }

void Wrist::set_angle(units::angle::degree_t angle) {
  m_motor.SetControl(m_control_req.WithPosition(angle));
}

units::degree_t Wrist::get_angle() { return m_motor.GetPosition().GetValue(); }

frc2::CommandPtr Wrist::rezero() {
  return frc2::RunCommand([this] { m_motor.SetPosition(0_tr); }, {this})
      .WithName("RE-zero");
}

frc2::CommandPtr Wrist::set_angle_command(units::degree_t pos) {
  return frc2::RunCommand(
             [this, pos] {
               frc::SmartDashboard::PutNumber("Wrist Setpoint", pos.value());
               frc::SmartDashboard::PutNumber(
                   "Wrist position", m_motor.GetPosition().GetValueAsDouble());
               set_angle(pos);
             },
             {this})
      .WithName("Set Wrist Angle")
      .Until([this, pos] {
        return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(
            get_angle(), pos, CONSTANTS::WRIST::POSITION_THRESHOLD);
      });
}
