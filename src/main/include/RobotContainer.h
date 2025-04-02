// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// FRC
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/Trigger.h>
#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SendableChooser.h>

// Misc
#include "Constants.h"
#include <pathplanner/lib/auto/NamedCommands.h>

// Subsystems
#include "subsystems/Candle.h"
#include "subsystems/Elevator.h"
#include "subsystems/Grabber.h"
#include "subsystems/Wrist.h"

// Swerve
#include "swerve/Drivetrain.h"
#include "swerve/Odometry.h"
#include "swerve/Trajectory.h"
#include "swerve/Vision.h"
#include "utility/ForceLog.h"

#include "subsystems/Wrist.h"
#include "subsystems/Elevator.h"
#include <frc/DataLogManager.h>
#include "subsystems/Grabber.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/auto/NamedCommands.h>
// #include <ForceLog.h>
#include "subsystems/Candle.h"
// TODO: Add w/ merge
class RobotContainer
{
public:
  bool has_flipped = false;
  RobotContainer();

  void add_named_commands();

  frc2::Command *GetAutonomousCommand(); // This is a raw pointer because AutoBuilder::buildAutoChooser() returns frc::SendableChooser<frc2::Command*>

  frc::SendableChooser<frc2::Command *> autoChooser;

  frc2::CommandXboxController m_stick0{0};
  frc2::CommandXboxController m_stick1{1};

  // Moves wrist and elevelator to position
  frc2::CommandPtr set_state(CONSTANTS::MANIPULATOR_STATES::ManipulatorState target);

  // Brings elevator down and moves wrist
  frc2::CommandPtr score();
  frc2::CommandPtr score(CONSTANTS::MANIPULATOR_STATES::ManipulatorState target);

  frc2::CommandPtr coral_intake();
  frc2::CommandPtr algae_intake();

  frc2::CommandPtr score_algae();

  Drivetrain m_drivetrain;
  Odometry m_odometry{&m_drivetrain, &m_vision};
  Elevator m_elevator;
  Wrist m_wrist;
  Grabber m_grabber;
  Candle m_candle{[this]() -> bool
                  { return m_grabber.has_gp(); }};

  Vision m_vision{
      [this]() -> units::degree_t
      {
        return m_drivetrain.getAngle();
      }};

  void ConfigureBindings();
  void SetPID();
  void LogDashboard();

  CONSTANTS::MANIPULATOR_STATES::ManipulatorState prev_state = CONSTANTS::MANIPULATOR_STATES::IDLE;
  CONSTANTS::MANIPULATOR_STATES::ManipulatorState current_state = CONSTANTS::MANIPULATOR_STATES::IDLE;

  std::vector<std::optional<frc::Pose2d>> bot_pose = m_vision.get_bot_position();

  Trajectory m_trajectory{&m_drivetrain, &m_odometry, &m_stick0, &m_vision};
};
