// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer()
{
  ConfigureBindings();
  m_odometry.putField2d();
  autoChooser = AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}

void RobotContainer::SetPID()
{
  m_climber.SetPID();
  m_elevator.SetPID();
  m_wrist.SetPID();
  m_grabber.SetPID();
}
void RobotContainer::LogDashboard()
{
  m_climber.LogDashboard();
  m_elevator.LogDashboard();
  m_wrist.LogDashboard();
  m_grabber.LogDashboard();
}

void RobotContainer::ConfigureBindings()
{
  frc::SmartDashboard::PutData(&m_elevator);
  m_trajectory.SetDefaultCommand(m_trajectory.manual_drive());
  frc2::Trigger{
      [this] -> bool
      {
        return m_stick0.A().Get() && m_stick0.RightBumper().Get();
      }}
      .OnTrue(score_prepare(CONSTANTS::SCORING_TARGETS::L1));
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  return autoChooser.GetSelected();
}

frc2::CommandPtr RobotContainer::score_prepare(CONSTANTS::SCORING_TARGETS::TargetProfile target)
{
  return m_elevator.set_position_command(target.elevtor_pos).AlongWith(m_wrist.set_angle_command(target.wrist_pos));
}

frc2::CommandPtr RobotContainer::score_execute(CONSTANTS::SCORING_TARGETS::TargetProfile target)
{
  if (target == CONSTANTS::SCORING_TARGETS::L1 || target == CONSTANTS::SCORING_TARGETS::PROCESSOR)
  {
    return m_grabber.extake();
  }
  else
  {
    // To score, you need to lower the elevator and hold the grabber at the current position
    return score_prepare(
        CONSTANTS::SCORING_TARGETS::TargetProfile{target.elevtor_pos - CONSTANTS::SCORING_TARGETS::POST_SCORE_DELTA, target.wrist_pos});
  }
}
