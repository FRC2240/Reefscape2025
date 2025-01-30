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

  frc::SmartDashboard::PutData("Wrist", &m_wrist);
}

void RobotContainer::SetPID() {
  m_wrist.SetPID();
  m_climber.SetPID();
  m_elevator.SetPID();
  m_grabber.SetPID();
}

void RobotContainer::ConfigureBindings()
{
  m_trajectory.SetDefaultCommand(m_trajectory.manual_drive());
  m_stick0.A().OnTrue(m_wrist.set_angle_command(10_tr));
  m_stick0.B().OnTrue(m_wrist.set_angle_command(0_tr));
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  return autoChooser.GetSelected();
}
