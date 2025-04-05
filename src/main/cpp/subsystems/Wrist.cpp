#include "subsystems/Wrist.h"

#include "frc2/command/RunCommand.h"

Wrist::Wrist()
{

  ctre::phoenix6::configs::TalonFXConfiguration conf{};
  conf.MotionMagic.MotionMagicAcceleration = 350_tr_per_s_sq;
  conf.MotionMagic.MotionMagicCruiseVelocity = 35_tps;
  conf.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
  m_motor.GetConfigurator().Apply(conf);

  SetPID();
}

void Wrist::InitSendable(wpi::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("Wrist");
  MotorUtils::BuildSender(builder, &coeff);
  MotorUtils::BuildSender(builder, &m_motor);
}

void Wrist::SetPID() { MotorUtils::SetPID(m_motor, coeff); }

void Wrist::set_angle(units::angle::degree_t angle)
{
  m_motor.SetControl(m_control_req.WithPosition(angle));
}

units::degree_t Wrist::get_angle()
{
  return m_motor.GetPosition().GetValue();
}

frc2::CommandPtr Wrist::offset_command(units::degree_t amount)
{
  return frc2::cmd::RunOnce([this, amount]
                            { return set_angle(get_angle() + amount); });
}

frc2::CommandPtr Wrist::rezero()
{
  return frc2::RunCommand([this]
                          { m_motor.SetControl(ctre::phoenix6::controls::VoltageOut{-3_V}); },
                          {this})
      .Until([this]
             { return m_motor.GetVelocity().GetValue() < 1_tps; })
      .WithName("Rezero Wrist")
      .AndThen(frc2::cmd::RunOnce([this]
                                  { m_motor.SetPosition(0_tr);
                                          m_motor.SetControl(ctre::phoenix6::controls::VoltageOut{0_V}); },
                                  {this}));
}

frc2::CommandPtr Wrist::set_angle_command(units::degree_t pos)
{
  return frc2::RunCommand(
             [this, pos]
             {
               frc::SmartDashboard::PutNumber("Wrist Setpoint", pos.value());
               frc::SmartDashboard::PutNumber(
                   "Wrist position", m_motor.GetPosition().GetValueAsDouble());

               set_angle(pos);
             },
             {this})
      .WithName("Set Wrist Angle")
      .Until([this, pos]
             { return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(
                   get_angle(), pos, CONSTANTS::WRIST::POSITION_THRESHOLD + 3_tr); });
}
