#include "subsystems/Ground.h"
#include <frc2/command/RunCommand.h>





// Change Ground to Arm asp
// not the class but the motor, coeff, etc.






Ground::Ground() 
{
    ctre::phoenix6::configs::TalonFXConfiguration conf{};

    conf.MotionMagic.MotionMagicAcceleration = 0_tr_per_s_sq; // Change Me!
    conf.MotionMagic.MotionMagicCruiseVelocity = 0_tps; // Change Me!
    conf.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine; // Change Me!

    m_ground.GetConfigurator().Apply(conf);
    SetPID(m_ground, ground_coeff);

    conf.MotionMagic.MotionMagicAcceleration = 0_tr_per_s_sq; // Change Me!
    conf.MotionMagic.MotionMagicCruiseVelocity = 0_tps; // Change Me!
    conf.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine; // Change Me!

    m_intake.GetConfigurator().Apply(conf);
    SetPID(m_intake, intake_coeff);

    conf.MotionMagic.MotionMagicAcceleration = 0_tr_per_s_sq; // Change Me!
    conf.MotionMagic.MotionMagicCruiseVelocity = 0_tps; // Change Me!
    conf.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine; // Change Me!

    m_index.GetConfigurator().Apply(conf);
    SetPID(m_index, index_coeff);
}

void Ground::InitSendable(wpi::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("Ground");
  MotorUtils::BuildSender(builder, &ground_coeff);
  MotorUtils::BuildSender(builder, &m_ground);

  builder.SetSmartDashboardType("Intake");
  MotorUtils::BuildSender(builder, &intake_coeff);
  MotorUtils::BuildSender(builder, &m_intake);

  builder.SetSmartDashboardType("Index");
  MotorUtils::BuildSender(builder, &index_coeff);
  MotorUtils::BuildSender(builder, &m_index);
}

void Ground::SetPID(ctre::phoenix6::hardware::TalonFX& motor, CONSTANTS::PidCoeff coeff) { MotorUtils::SetPID(motor, coeff);}

void Ground::set_angle(ctre::phoenix6::hardware::TalonFX& motor, units::angle::degree_t angle) 
{
    motor.SetControl(m_control_req.WithPosition(angle));
}

units::degree_t Ground::get_angle(ctre::phoenix6::hardware::TalonFX& motor) 
{
    return motor.GetPosition().GetValue();
}

// Add command stuff

// Ground swaps states on click
// Intake for onToggleTrue
// Indexer whop knows lol
