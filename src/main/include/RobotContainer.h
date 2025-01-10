// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
// #include "commands/Autos.h"
#include "swerve/Drivetrain.h"
#include "swerve/Odometry.h"
#include "swerve/Trajectory.h"
#include "swerve/Vision.h"
#include "subsystems/Wrist.h"
#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/auto/NamedCommands.h>
// #include <ForceLog.h>
#include "subsystems/Candle.h"
// TODO: Add w/ merge
class RobotContainer
{
public:
  RobotContainer();
  void SetPID();
  void LogDashboard();

  void add_named_commands();

  frc2::Command *GetAutonomousCommand(); // This is a raw pointer because AutoBuilder::buildAutoChooser() returns frc::SendableChooser<frc2::Command*>

  frc::SendableChooser<frc2::Command *> autoChooser;

  frc2::CommandXboxController m_stick0{0};
  frc2::CommandXboxController m_stick1{1};

  Drivetrain m_drivetrain;

  Vision m_vision{
      [this]() -> units::degree_t
      {
        return m_drivetrain.getAngle();
      }};

  Odometry m_odometry{&m_drivetrain, &m_vision};

  Wrist m_wrist;

  void ConfigureBindings();

  std::vector<std::optional<frc::Pose2d>> bot_pose = m_vision.get_bot_position();

  Trajectory m_trajectory{&m_drivetrain, &m_odometry, &m_stick0, &m_vision};

  
};
