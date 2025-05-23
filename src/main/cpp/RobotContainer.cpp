// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc/geometry/Pose2d.h>
#include <iostream>

RobotContainer::RobotContainer()
{
  ConfigureBindings();
  m_odometry.putField2d();
  add_named_commands();
  autoChooser = AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

  frc::SmartDashboard::PutData("Elevator", &m_elevator);
  frc::SmartDashboard::PutData("Wrist", &m_wrist);
}

void RobotContainer::add_named_commands()
{
  pathplanner::NamedCommands::registerCommand("l4", frc2::cmd::Print("start l4").AndThen(set_state(CONSTANTS::MANIPULATOR_STATES::L4)).AndThen(frc2::cmd::Print("end l4")));
  pathplanner::NamedCommands::registerCommand("l2", frc2::cmd::Print("start l2").AndThen(set_state(CONSTANTS::MANIPULATOR_STATES::L2)).AndThen(frc2::cmd::Print("end l2")));
  pathplanner::NamedCommands::registerCommand("score_l4", frc2::cmd::Print("start score l4").AndThen(score(CONSTANTS::MANIPULATOR_STATES::L4)).WithTimeout(0.25_s).AndThen(frc2::cmd::Print("end score l4")));
  pathplanner::NamedCommands::registerCommand("intake", frc2::cmd::Print("start intake").AndThen(coral_intake()).AndThen(frc2::cmd::Print("end intake")));
  pathplanner::NamedCommands::registerCommand("idle", frc2::cmd::Print("start idle").AndThen(set_state(CONSTANTS::MANIPULATOR_STATES::IDLE)).AndThen(frc2::cmd::Print("end idle")));
  pathplanner::NamedCommands::registerCommand("algae_intake", frc2::cmd::Print("start algae intake").AndThen(algae_intake()).AndThen(frc2::cmd::Print("end intake")));
  pathplanner::NamedCommands::registerCommand("algae_l2", frc2::cmd::Print("start l2").AndThen(set_state(CONSTANTS::MANIPULATOR_STATES::ALGAE_L2)).AndThen(frc2::cmd::Print("end l2")));
  pathplanner::NamedCommands::registerCommand("algae_score", frc2::cmd::Print("start algae score").AndThen(score_algae()).AndThen(frc2::cmd::Print("end algae score")));
  pathplanner::NamedCommands::registerCommand("barge", frc2::cmd::Print("start barge").AndThen(set_state(CONSTANTS::MANIPULATOR_STATES::BARGE)).AndThen(frc2::cmd::Print("end barge")));
}

void RobotContainer::SetPID()
{
  m_wrist.SetPID();
  m_elevator.SetPID();
}
void RobotContainer::LogDashboard()
{
}

void RobotContainer::ConfigureBindings()
{
  frc::SmartDashboard::PutData(&m_elevator);
  m_trajectory.SetDefaultCommand(m_trajectory.manual_drive());
  m_grabber.SetDefaultCommand(m_grabber.idle());
  m_poweredfun.SetDefaultCommand(m_poweredfun.spin());


  // https://files.slack.com/files-pri/T0CS7MN06-F08BXRSU770/image.png

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.LeftBumper().Get() && !this->m_stick0.LeftTrigger().Get(); })
      .OnTrue(coral_intake());

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.LeftBumper().Get() && this->m_stick0.LeftTrigger().Get(); })
      .OnTrue(algae_intake());

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.A().Get() && this->m_stick0.LeftTrigger().Get(); })
      .OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::ALGAE_L2).AlongWith(m_grabber.idle()));

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.X().Get() && this->m_stick0.LeftTrigger().Get(); })
      .OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::ALGAE_L3).AlongWith(m_grabber.idle()));

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.LeftTrigger().Get() && this->m_stick0.Y().Get(); })
      .OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::BARGE));
  /*
  frc2::Trigger([this] () -> bool {
    return this->m_stick0.RightBumper().Get() && this->m_stick0.LeftTrigger().Get();
  }).OnTrue(m_grabber.extake());
  */

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.RightBumper().Get() && !this->m_stick0.LeftTrigger().Get(); })
      .OnTrue(score(current_state));

  frc2::Trigger([this]() -> bool {
    return this->m_stick0.Y().Get() && !this->m_stick0.LeftTrigger().Get();
  }).OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L4));

  // frc2::Trigger([this]() -> bool
  //               { return this->m_stick0.B().Get() && this->m_stick0.LeftTrigger().Get(); }); // proc

  // frc2::Trigger([this]() -> bool
  //               { return this->m_stick0.B().Get() && !this->m_stick0.LeftTrigger().Get(); })
  //     .OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L1));

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.RightBumper().Get() && this->m_stick0.LeftTrigger().Get(); }) // L2 algae
      .OnTrue(score_algae());

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.B().Get() && this->m_stick0.LeftTrigger().Get(); }) // L2 algae
      .OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::PROCESSOR));

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.X().Get() && !this->m_stick0.LeftTrigger().Get(); })
      .OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L3));

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.A().Get() && this->m_stick0.LeftTrigger().Get(); }); // L1 algae

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.A().Get() && !this->m_stick0.LeftTrigger().Get(); })
      .OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L2));

  m_stick0.RightTrigger().OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::IDLE).AlongWith(m_grabber.idle()));

  // Driver 2 overrides
  m_stick1.Start().WhileTrue(m_wrist.rezero());

  m_stick1.Y().OnTrue(m_wrist.offset_command(CONSTANTS::WRIST::OFFSET_AMOUNT));
  m_stick1.B().OnTrue(m_wrist.offset_command(-CONSTANTS::WRIST::OFFSET_AMOUNT));

  m_stick1.X().OnTrue(m_elevator.offset_command(CONSTANTS::ELEVATOR::OFFSET_AMOUNT));
  m_stick1.A().OnTrue(m_elevator.offset_command(-CONSTANTS::ELEVATOR::OFFSET_AMOUNT));

  m_stick0.POVLeft().OnTrue(m_trajectory.reef_align_command(CONSTANTS::FIELD_POSITIONS::REEF_SIDE_SIDE::LEFT));
  m_stick0.POVRight().OnTrue(m_trajectory.reef_align_command(CONSTANTS::FIELD_POSITIONS::REEF_SIDE_SIDE::RIGHT));

  // ground algae intake(driver 2)
  frc2::Trigger([this]() -> bool
                { return this->m_stick1.RightTrigger().Get(); })
      .OnTrue(m_wrist.set_angle_command(CONSTANTS::MANIPULATOR_STATES::GROUND_ALGAE.wrist_pos).AndThen(m_elevator.set_position_command(CONSTANTS::MANIPULATOR_STATES::GROUND_ALGAE.elevtor_pos)));

  // togglable funnel 
  frc2::Trigger([this]() -> bool
              {return this->m_stick1.LeftTrigger().Get(); })
      .ToggleOnTrue(m_poweredfun.spin(0_A));

  // L1 state command on driver 2
  frc2::Trigger([this]() -> bool
                { return this->m_stick1.LeftBumper().Get(); })
      .OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L1));

}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  return autoChooser.GetSelected();
}

frc2::CommandPtr RobotContainer::set_state(CONSTANTS::MANIPULATOR_STATES::ManipulatorState target)
{
  CONSTANTS::MANIPULATOR_STATES::ManipulatorState last_state = current_state;
  current_state = target;

  if (target == CONSTANTS::MANIPULATOR_STATES::IDLE)
  {
    return m_elevator.set_position_command(target.elevtor_pos).AndThen(m_wrist.set_angle_command(target.wrist_pos));
  }
  if (target == CONSTANTS::MANIPULATOR_STATES::L2)
  {
    return m_wrist.set_angle_command(target.wrist_pos).AndThen(m_elevator.set_position_command(target.elevtor_pos));
  }

  if (last_state == CONSTANTS::MANIPULATOR_STATES::BARGE)
  {
    return m_wrist.set_angle_command(target.wrist_pos).AndThen(m_elevator.set_position_command(target.elevtor_pos));
  }
  // if (target == CONSTANTS::MANIPULATOR_STATES::L4 && last_state == CONSTANTS::MANIPULATOR_STATES::IDLE && !frc::DriverStation::IsAutonomous())
  if (target == CONSTANTS::MANIPULATOR_STATES::L4)
  {
    return m_elevator.set_position_command(target.elevtor_pos).AndThen(m_wrist.set_angle_command(target.wrist_pos));
  }
  {
    return m_wrist.set_angle_command(target.wrist_pos).AndThen(m_elevator.set_position_command(target.elevtor_pos));
  }

  return m_wrist.set_angle_command(target.wrist_pos).AlongWith(m_elevator.set_position_command(target.elevtor_pos));
}

frc2::CommandPtr RobotContainer::score(CONSTANTS::MANIPULATOR_STATES::ManipulatorState target)
{
  CONSTANTS::MANIPULATOR_STATES::ManipulatorState last_state = current_state;
  
  if(last_state == CONSTANTS::MANIPULATOR_STATES::L1) 
  {
    return m_grabber.extake(CONSTANTS::GRABBER::L1_EXTAKE_VELOCITY);
  }
  else 
  {
    return m_wrist.set_angle_command(CONSTANTS::MANIPULATOR_STATES::POST_SCORE.wrist_pos).AlongWith(m_grabber.coast());
  }
}

frc2::CommandPtr RobotContainer::coral_intake()
{
  return set_state(CONSTANTS::MANIPULATOR_STATES::INTAKE).AlongWith(m_grabber.intake(CONSTANTS::GRABBER::INTAKE_CORAL_VELOCITY));
}

frc2::CommandPtr RobotContainer::score_algae()
{
  return m_grabber.extake(CONSTANTS::GRABBER::ALGAE_SCORE_VELOCITY);
}

frc2::CommandPtr RobotContainer::algae_intake()
{
  return m_grabber.intake_algae(CONSTANTS::GRABBER::INTAKE_ALGAE_VELOCITY);
}
