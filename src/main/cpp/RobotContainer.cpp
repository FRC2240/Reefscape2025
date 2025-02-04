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

  frc::SmartDashboard::PutData("Elevator", &m_elevator);
}

void RobotContainer::SetPID()
{
  m_climber.SetPID();
  m_wrist.SetPID();
  m_grabber.SetPID();
  m_elevator.SetPID();
}
void RobotContainer::LogDashboard()
{
  m_climber.LogDashboard();
  m_wrist.LogDashboard();
  m_grabber.LogDashboard();
}

void RobotContainer::ConfigureBindings()
{
  frc::SmartDashboard::PutData(&m_elevator);
  m_trajectory.SetDefaultCommand(m_trajectory.manual_drive());
  m_stick0.A().OnTrue(intake());
  m_stick0.B().OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L2));
  m_stick0.Y().OnTrue(score(CONSTANTS::MANIPULATOR_STATES::L2));
  m_stick0.X().OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::IDLE));
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  return autoChooser.GetSelected();
}

frc2::CommandPtr RobotContainer::set_state(CONSTANTS::MANIPULATOR_STATES::ManipulatorState target)
{
  return m_elevator.set_position_command(target.elevtor_pos).AlongWith(m_wrist.set_angle_command(target.wrist_pos));
}

frc2::CommandPtr RobotContainer::score(CONSTANTS::MANIPULATOR_STATES::ManipulatorState target)
{
  // Verify that the manipulator is in the correct position, if it isn't fall back to last state
  if (CONSTANTS::IN_THRESHOLD<units::turn_t>(m_wrist.get_angle(), target.wrist_pos, CONSTANTS::WRIST::POSITION_THRESHOLD) &&
      CONSTANTS::IN_THRESHOLD<units::turn_t>(m_elevator.get_position(), target.elevtor_pos, CONSTANTS::ELEVATOR::POSITION_THRESHOLD))
  {
    {
      if (target == CONSTANTS::MANIPULATOR_STATES::L1 || target == CONSTANTS::MANIPULATOR_STATES::PROCESSOR)
      {
        return m_grabber.extake();
      }
      else
      {
        // To score, you need to lower the elevator and hold the grabber at the current position
        return set_state(
            CONSTANTS::MANIPULATOR_STATES::ManipulatorState{target.elevtor_pos, target.wrist_pos - CONSTANTS::MANIPULATOR_STATES::POST_SCORE_DELTA});
      }
    }
  }
  else
  {
    return set_state(target);
  }
}

frc2::CommandPtr RobotContainer::intake()
{
  return set_state(CONSTANTS::MANIPULATOR_STATES::INTAKE).Until([this] -> bool
                                                                { return m_grabber.has_gp(); })
      .AndThen(m_elevator.set_position_command(CONSTANTS::MANIPULATOR_STATES::IDLE_W_GP.elevtor_pos).AndThen(m_wrist.set_angle_command(CONSTANTS::MANIPULATOR_STATES::IDLE_W_GP.wrist_pos)));
}