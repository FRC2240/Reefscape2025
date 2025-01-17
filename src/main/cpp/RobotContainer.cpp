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
  // m_wrist.SetPID();
}
void RobotContainer::LogDashboard()
{
  // m_wrist.LogDashboard();
}

void RobotContainer::ConfigureBindings()
{
  m_trajectory.SetDefaultCommand(m_trajectory.manual_drive());
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  return autoChooser.GetSelected();
}
