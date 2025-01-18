// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "utility/DriverUtils.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/DriverStation.h>

Robot::Robot()
{
  frc::DataLogManager::Start();
  frc::DriverStation::SilenceJoystickConnectionWarning(1);
}

void Robot::RobotPeriodic()
{
  m_container.m_odometry.update();
  try
  {
    frc2::CommandScheduler::GetInstance().Run();
  }
  catch (const std::exception &e)
  {
    frc::DataLogManager::Log(e.what());
  }
  m_container.LogDashboard();
}

void Robot::DisabledInit()
{

  m_container.m_drivetrain.set_brake_mode(1);
  disabled_timer.Restart();
}

void Robot::DisabledPeriodic()
{

  if (disabled_timer.Get() > CONSTANTS::DRIVE::BRAKE_TIME && !frc::DriverStation::IsEStopped())
  {
    m_container.m_drivetrain.set_brake_mode(0);
    disabled_timer.Reset();
    disabled_timer.Stop();
  }
}

void Robot::DisabledExit()
{
  if (!frc::DriverStation::IsFMSAttached())
  {
    m_container.SetPID();
  }
}

void Robot::AutonomousInit()
{
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand.value()->Schedule();
    // &m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
  if (m_autonomousCommand)
  {
    m_autonomousCommand.value()->Cancel();
  }
}

void Robot::TeleopPeriodic()
{
  DriverUtils::DriverTeleopPeriodic();
}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
