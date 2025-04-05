// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc/geometry/Pose2d.h>

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

  /*
  frc2::Trigger([this] () -> bool {
    return this->m_stick0.RightBumper().Get() && this->m_stick0.LeftTrigger().Get();
  }).OnTrue(m_grabber.extake());
  */

  frc2::Trigger([this]() -> bool
                { return this->m_stick0.RightBumper().Get() && !this->m_stick0.LeftTrigger().Get(); })
      .OnTrue(score(current_state));

  m_stick0.Y().OnTrue(set_state(CONSTANTS::MANIPULATOR_STATES::L4));

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

  m_stick1.LeftBumper().OnTrue(m_trajectory.reef_align_command(CONSTANTS::FIELD_POSITIONS::REEF_SIDE_SIDE::LEFT));
  m_stick1.RightBumper().OnTrue(m_trajectory.reef_align_command(CONSTANTS::FIELD_POSITIONS::REEF_SIDE_SIDE::RIGHT));

  // ground algae intake
  frc2::Trigger([this]() -> bool
                { return this->m_stick1.RightTrigger().Get(); })
      .OnTrue(m_wrist.set_angle_command(CONSTANTS::MANIPULATOR_STATES::GROUND_ALGAE.wrist_pos).AndThen(m_elevator.set_position_command(CONSTANTS::MANIPULATOR_STATES::GROUND_ALGAE.elevtor_pos)));
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

frc2::CommandPtr RobotContainer::score(CONSTANTS::MANIPULATOR_STATES::ManipulatorState target)
{
  return m_wrist.set_angle_command(CONSTANTS::MANIPULATOR_STATES::POST_SCORE.wrist_pos).AlongWith(m_grabber.coast());
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


void RobotContainer::SelfTest() {
  
  bool MOTORS_OK     = false;
  bool BATTERY_OK    = false;
  bool ELEVATOR_OK   = false;
  bool WRIST_OK      = false;
  bool GP_LOADED     = false;
  bool JOYSTICKS_OK  = false;
  bool ODOMETRY_OK   = false;
  bool AUTO_SELECTED = false;
//bool STARTPOS_OK   = false; // this is probably impossible without reimplementing the auto chooser


  // No motors report issues ***

  for (auto motor : motors) {
    if ( // not entirely sure if this will work
      motor->GetFaultField().GetValue() + motor->GetStickyFaultField().GetValue() != 0
    ) {
      MOTORS_OK = false;
    }
  }

  // MOTORS_OK = true;
  // for (auto motor : motors) {
  //   int faults = 0;
  //   faults += motor->GetFault_ForwardSoftLimit().GetValue();
  //   faults += motor->GetFault_ReverseSoftLimit().GetValue();
  //   faults += motor->GetFault_ForwardHardLimit().GetValue();
  //   faults += motor->GetFault_ReverseHardLimit().GetValue();
  //   faults += motor->GetFault_UnstableSupplyV().GetValue();
  //   faults += motor->GetFault_OverSupplyV().GetValue();
  //   faults += motor->GetFault_RemoteSensorPosOverflow().GetValue();
  //   faults += motor->GetFault_MissingDifferentialFX().GetValue();
  //   faults += motor->GetFault_RemoteSensorReset().GetValue();
  //   faults += motor->GetFault_BridgeBrownout().GetValue();
  //   faults += motor->GetFault_UnlicensedFeatureInUse().GetValue();
  //   faults += motor->GetFault_BootDuringEnable().GetValue();
  //   faults += motor->GetFault_Undervoltage().GetValue();
  //   faults += motor->GetFault_DeviceTemp().GetValue();
  //   faults += motor->GetFault_ProcTemp().GetValue();
  //   faults += motor->GetFault_Hardware().GetValue();
  //   faults += motor->GetFault_StaticBrakeDisabled().GetValue();
  //   faults += motor->GetFault_UsingFusedCANcoderWhileUnlicensed().GetValue();
  //   faults += motor->GetFault_SupplyCurrLimit().GetValue();
  //   faults += motor->GetFault_StatorCurrLimit().GetValue();
  //   faults += motor->GetFault_FusedSensorOutOfSync().GetValue();
  //   faults += motor->GetStickyFault_RemoteSensorDataInvalid().GetValue();
  //   faults += motor->GetFault_MissingHardLimitRemote().GetValue();
  //   faults += motor->GetFault_MissingSoftLimitRemote().GetValue();
  //   if (faults) {
  //     MOTORS_OK = false;
  //     break;
  //   }
  // }


  // Battery voltage > 12.3

  if (frc::DriverStation::GetBatteryVoltage() >= CONSTANTS::SELFTEST::BATTERY_VOLTAGE_THRESHOLD) {
    BATTERY_OK = true;
  }

  // elevator down

  if (CONSTANTS::IN_THRESHOLD<units::turn_t>(
    m_elevator.get_position(),
    CONSTANTS::ELEVATOR::BOTTOM_POS,
    CONSTANTS::ELEVATOR::POSITION_THRESHOLD
  )) {
    ELEVATOR_OK = true;
  }

  // wrist pos ~= intake state

  if (CONSTANTS::IN_THRESHOLD<units::degree_t>(
    m_wrist.get_angle(),
    CONSTANTS::MANIPULATOR_STATES::INTAKE.wrist_pos,
    CONSTANTS::WRIST::POSITION_THRESHOLD
  )) {
    WRIST_OK = true;
  }

  // game piece loaded

  if (m_grabber.has_gp()) {
    GP_LOADED = true;
  }

  // 2 joysticks

  if (
    frc::DriverStation::IsJoystickConnected(0) &&
    frc::DriverStation::IsJoystickConnected(1)
  ) {
    JOYSTICKS_OK = true;
  }

  // apriltag angle +/- 10deg from gyro angle ***

  units::degree_t vision_pose = LimelightHelpers::toPose2D(LimelightHelpers::getBotpose()).Rotation().Degrees();
  units::degree_t gyro_pose = m_drivetrain.getAngle();
  if (
    CONSTANTS::IN_THRESHOLD<units::degree_t>(vision_pose, gyro_pose, CONSTANTS::SELFTEST::YAW_ERROR_THRESHOLD)
  ) {
    ODOMETRY_OK = true;
  }

  // auto selected (not sure if this works)

  frc2::CommandPtr lval_none = frc2::cmd::None();
  if (autoChooser.GetSelected() != lval_none.get()) {
    AUTO_SELECTED = true;
  }

  int ERRORS =
    MOTORS_OK +
    BATTERY_OK +
    ELEVATOR_OK +
    WRIST_OK +
    GP_LOADED +
    JOYSTICKS_OK +
    ODOMETRY_OK +
    AUTO_SELECTED;

  if (ERRORS) {
    fail_selftest.Set(true);
  } else {
    fail_selftest.Set(false);
  }

  frc::SmartDashboard::PutBoolean("selftest/motors_ok", MOTORS_OK);
  frc::SmartDashboard::PutBoolean("selftest/battery_ok", BATTERY_OK);
  frc::SmartDashboard::PutBoolean("selftest/elevator_ok", ELEVATOR_OK);
  frc::SmartDashboard::PutBoolean("selftest/wrist_ok", WRIST_OK);
  frc::SmartDashboard::PutBoolean("selftest/gp_loaded", GP_LOADED);
  frc::SmartDashboard::PutBoolean("selftest/joysticks_ok", JOYSTICKS_OK);
  frc::SmartDashboard::PutBoolean("selftest/odometry_ok", ODOMETRY_OK);
  frc::SmartDashboard::PutBoolean("selftest/auto_selected", AUTO_SELECTED);

}

std::vector<ctre::phoenix6::hardware::TalonFX*> RobotContainer::get_motors() {

  // IMPORTANT: Add new motors to this if any new motors are added!!!
  std::vector<ctre::phoenix6::hardware::TalonFX*> motors = {
    &m_elevator.m_motor,
    &m_elevator.m_follower_motor,
    &m_grabber.m_motor,
    &m_wrist.m_motor,
    &Module::front_right->driver,
    &Module::front_right->turner,
    &Module::front_left->driver,
    &Module::front_left->turner,
    &Module::back_right->driver,
    &Module::back_right->turner,
    &Module::back_left->driver,
    &Module::back_left->turner,
  };

}