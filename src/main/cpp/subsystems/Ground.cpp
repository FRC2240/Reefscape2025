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

void Ground::SetPID(ctre::phoenix6::hardware::TalonFX& m_motor, CONSTANTS::PidCoeff coeff) { MotorUtils::SetPID(m_motor, coeff);}

void Ground::set_angle(ctre::phoenix6::hardware::TalonFX& m_motor, units::angle::degree_t angle) 
{
    m_motor.SetControl(m_control_req.WithPosition(angle));
}

units::degree_t Ground::get_angle(ctre::phoenix6::hardware::TalonFX& m_motor) 
{
    return m_motor.GetPosition().GetValue();
}

void Ground::intake() 
{
    set_angle(m_intake, CONSTANTS::GROUND::INTAKE_SPEED);
}

void Ground::eject() 
{
    set_angle(m_intake,-CONSTANTS::GROUND::INTAKE_SPEED);
}

bool Ground::inThresholdExtended() 
{
    CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(m_ground), CONSTANTS::GROUND::EXTENDED, CONSTANTS::GROUND::POSITION_THRESHOLD + 0_tr);
}

bool Ground::inThresholdIdle() 
{
    CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(m_ground), CONSTANTS::GROUND::IDLE, CONSTANTS::GROUND::POSITION_THRESHOLD + 0_tr);
}

bool hasGP() 
{
    if(CONSTANTS::GROUND::test_sensor = true)
    {
        return true;
    }
}

bool Ground::IsAuto()
{
    return frc::DriverStation::IsAutonomousEnabled();
};

void Ground::Periodic() 
{
    if(IsAuto() && setup && !HASGP) { state = IDLE; } //Checks if the arm position is at its idle position
    else if(setup && !HASGP) { state = IDLE; }

    switch (state)
    {
    case INIT:
        if(true)
            bool setup = true;
        break;

    case IDLE:
        if(press == "A") 
        {
            intake_command();
            state = HASGP;
        }
        break;
    case HASGP:
        if(press == "A") 
        {
            eject();
            state = IDLE;
        }
        break;
    }
}

frc2::CommandPtr Ground::intake_command()
{
  return frc2::RunCommand(
             [this]
             {
               set_angle(m_ground, CONSTANTS::GROUND::EXTENDED);
             },
             {this})
      .AndThen(
             [this]
             {
               m_ground.SetControl(ctre::phoenix6::controls::VoltageOut{0_V});
               intake();
             },
             {this})
      .Until([this]
             { return hasGP(); }) //add manual stop
      .AndThen(
             [this]
             {
               set_angle(m_ground, CONSTANTS::GROUND::IDLE);
             },
             {this})
       .Until([this] //potentially add stop here for faster time on mistakes
             { return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(
                   get_angle(m_ground), CONSTANTS::GROUND::IDLE, CONSTANTS::GROUND::POSITION_THRESHOLD + 0_tr); });
}

frc2::CommandPtr Ground::eject_command()
{
  return frc2::RunCommand(
             [this]
             {
               set_angle(m_ground, CONSTANTS::GROUND::EXTENDED);
             },
             {this})
      .AndThen(
             [this]
             {
               m_ground.SetControl(ctre::phoenix6::controls::VoltageOut{0_V});
               eject();
             },
             {this})
      .WithTimeout(1_s) //add manual stop and change time
      .AndThen(
             [this]
             {
               set_angle(m_ground, CONSTANTS::GROUND::IDLE);
             },
             {this})
       .Until([this] //potentially continues the manual stop option here
             { return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(
                   get_angle(m_ground), CONSTANTS::GROUND::IDLE, CONSTANTS::GROUND::POSITION_THRESHOLD + 0_tr); });
}

// Indexer who knows lol may not even be necessary
// Add controller inputs
// Add code to feed from Intake/Indexer to Indexer/Elevator
// Add manual overrides(up/down) for Javi
