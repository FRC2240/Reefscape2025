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
  frc::SmartDashboard::PutData("Wrist", &m_wrist);
}

void RobotContainer::SetPID()
{
  m_climber.SetPID();
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
  /*
  m_stick0.A().OnTrue(intake());
  m_stick0.B().OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L4));
  m_stick0.Y().OnTrue(score(CONSTANTS::MANIPULATOR_STATES::L4));
  m_stick0.X().OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::IDLE));
  m_stick0.RightBumper().OnTrue(m_wrist.rezero());
  // m_stick0.X().OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L4));
  */

  frc2::Trigger([this] () -> bool {
    return this->m_stick0.LeftBumper().Get() && this->m_stick0.LeftTrigger().Get();
  }).OnTrue(intake());

  frc2::Trigger([this] () -> bool {
    return this->m_stick0.LeftBumper().Get() && ! this->m_stick0.LeftTrigger().Get();
  }); // coral pickup

  frc2::Trigger([this] () -> bool {
    return this->m_stick0.RightBumper().Get() && this->m_stick0.LeftTrigger().Get();
  }).OnTrue(m_grabber.extake());

  frc2::Trigger([this] () -> bool {
    return this->m_stick0.RightBumper().Get() && ! this->m_stick0.LeftTrigger().Get();
  }).OnTrue(score());

  m_stick0.Y().OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L4));

  frc2::Trigger([this] () -> bool {
    return this->m_stick0.B().Get() && this->m_stick0.LeftTrigger().Get();
  }); // proc

  frc2::Trigger([this] () -> bool {
    return this->m_stick0.B().Get() && ! this->m_stick0.LeftTrigger().Get();
  }).OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L1));

  frc2::Trigger([this] () -> bool {
    return this->m_stick0.X().Get() && this->m_stick0.LeftTrigger().Get();
  }); // L2 algae

  frc2::Trigger([this] () -> bool {
    return this->m_stick0.X().Get() && ! this->m_stick0.LeftTrigger().Get();
  }).OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L3));

  frc2::Trigger([this] () -> bool {
    return this->m_stick0.A().Get() && this->m_stick0.LeftTrigger().Get();
  }); // L1 algae

  frc2::Trigger([this] () -> bool {
    return this->m_stick0.A().Get() && ! this->m_stick0.LeftTrigger().Get();
  }).OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L2));

  m_stick0.RightTrigger().OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::IDLE));

  m_stick0.Start().OnTrue(m_wrist.rezero());

}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  return autoChooser.GetSelected();
}

frc2::CommandPtr RobotContainer::set_state(CONSTANTS::MANIPULATOR_STATES::ManipulatorState target)
{
  current_state = target;
  if (target == CONSTANTS::MANIPULATOR_STATES::IDLE)
  {

    return m_elevator.set_position_command(target.elevtor_pos).AndThen(m_wrist.set_angle_command(target.wrist_pos));
  }
  return m_elevator.set_position_command(target.elevtor_pos).AlongWith(m_wrist.set_angle_command(target.wrist_pos));
}

frc2::CommandPtr RobotContainer::score() {
  if (
      current_state == CONSTANTS::MANIPULATOR_STATES::L1 ||
      current_state == CONSTANTS::MANIPULATOR_STATES::L2 ||
      current_state == CONSTANTS::MANIPULATOR_STATES::L3 ||
      current_state == CONSTANTS::MANIPULATOR_STATES::L4
    ) {
      return score(current_state);
    } else {
      return frc2::InstantCommand([] () {return;}, {}).ToPtr();
    }
}

frc2::CommandPtr RobotContainer::score(CONSTANTS::MANIPULATOR_STATES::ManipulatorState target)
{
  return set_state(
      CONSTANTS::MANIPULATOR_STATES::ManipulatorState{target.elevtor_pos, CONSTANTS::MANIPULATOR_STATES::POST_SCORE.wrist_pos});

  // Verify that the manipulator is in the correct position, if it isn't fall back to last state
  if (CONSTANTS::IN_THRESHOLD<units::turn_t>(m_wrist.get_angle(), target.wrist_pos, CONSTANTS::WRIST::POSITION_THRESHOLD) &&
      CONSTANTS::IN_THRESHOLD<units::turn_t>(m_elevator.get_position(), target.elevtor_pos, CONSTANTS::ELEVATOR::POSITION_THRESHOLD))
  {
    {
      if (target == CONSTANTS::MANIPULATOR_STATES::L1 || target == CONSTANTS::MANIPULATOR_STATES::PROCESSOR)
      {
        //return m_grabber.extake();
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
  return set_state(CONSTANTS::MANIPULATOR_STATES::INTAKE);
  // .Until([this] -> bool
  // { return m_grabber.has_gp(); })
  // .AndThen(m_elevator.set_position_command(CONSTANTS::MANIPULATOR_STATES::IDLE_W_GP.elevtor_pos).AndThen(m_wrist.set_angle_command(CONSTANTS::MANIPULATOR_STATES::IDLE_W_GP.wrist_pos)));
}