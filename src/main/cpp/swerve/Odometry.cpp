#include "swerve/Odometry.h"

#ifndef CFG_NO_DRIVEBASE
// frc::TimeOfFlight tof_sensor{1};
/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// extern frc::SwerveDriveKinematics<4> const kinematics;

// This is not how it should be but doing it "correctly" (++,+-,-+,--) causes
// the wheels to form an "X" instead of diamond while turning.
// It's wrong but it works, no touchy.

frc::Field2d field2d;

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/
Odometry::Odometry(Drivetrain *drivetrain, Vision *vision)
    : m_drivetrain{drivetrain}, m_vision{vision}
{
  // Bjarne Sjourstup, why have you forsaken me?
  estimator.SetVisionMeasurementStdDevs(wpi::array<double, 3>(std::move(std::array{5.0d, 5.0d, 5.0d})));
}

frc2::CommandPtr Odometry::set_pose_cmd(frc::Pose2d pose)
{
  return frc2::cmd::RunOnce([this, &pose]
                            { resetPosition(pose, frc::Rotation2d(0_rad)); },
                            {})
      .AndThen(frc2::PrintCommand("reset odometry").ToPtr());
}

void Odometry::putField2d()
{
  frc::SmartDashboard::PutData("Odometry Field", &field2d);
}

void Odometry::update()
{
  frc::Pose2d const pose = estimator.Update(m_drivetrain->getCCWHeading(),
                                            m_drivetrain->getModulePositions());
  // if constexpr (CONSTANTS::DEBUGGING)
  field2d.SetRobotPose(pose.X(), pose.Y(), pose.Rotation());
  frc::SmartDashboard::PutNumber("odometry/X", pose.X().value());
  frc::SmartDashboard::PutNumber("odometry/Y", pose.Y().value());
  frc::SmartDashboard::PutNumber("odometry/rot", pose.Rotation().Degrees().value());
  frc::SmartDashboard::PutNumber("odometry/CCW", m_drivetrain->getCCWHeading().Degrees().value());
  frc::SmartDashboard::PutString("Odometry: ", fmt::format("Pose X: {}, Y: {}, Z (Degrees): {}\n", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()));
}

frc::Pose2d Odometry::getPose() { return estimator.Update(m_drivetrain->getCCWHeading(),
                                                          m_drivetrain->getModulePositions()); }

frc::ChassisSpeeds const Odometry::getFieldRelativeSpeeds()
{
  // Init for first
  static frc::Timer speed_timer;
  speed_timer.Start();
  static frc::Pose2d previous_pose{};

  frc::Pose2d const current_pose = estimator.GetEstimatedPosition();

  frc::Pose2d const delta_pose = current_pose.RelativeTo(previous_pose);

  auto const time_elapsed = speed_timer.Get();
  units::meters_per_second_t const X = delta_pose.X() / time_elapsed;

  units::meters_per_second_t const Y = delta_pose.Y() / time_elapsed;

  units::degrees_per_second_t const rot{delta_pose.Rotation().Degrees() / time_elapsed};

  previous_pose = estimator.GetEstimatedPosition(); // Set the previous_pose for the next time this loop is run

  // estimator.
  speed_timer.Reset(); // Time how long until next call

  return frc::ChassisSpeeds{X, Y, rot};
}

void Odometry::reset_position_from_vision(const frc::Pose2d &bot_pose)
{
  estimator.ResetPosition(m_drivetrain->getCCWHeading(),
                          m_drivetrain->getModulePositions(),
                          bot_pose);
}

// void Odometry::reset_from_distance()
// {
//     units::millimeter_t raw_dist {tof_sensor.GetRange()};
//     units::meter_t dist {raw_dist};
//     units::meter_t x;
//     units::meter_t y{Odometry::getPose().X()};
//     if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
//     {
//         x = 7.94_m - dist;
//     }
//     else
//     {
//         x = -7.94_m + dist;
//     }
//     frc::Pose2d pose{x, y, m_drivetrain->getCCWHeading()};

//     odometry.ResetPosition(m_drivetrain->getCCWHeading(),
//                            m_drivetrain->getModulePositions(),
//                            pose
//                            );
// }

void Odometry::resetPosition(const frc::Pose2d &bot_pose, const frc::Rotation2d &gyro_angle)
{
  estimator.ResetPosition(gyro_angle, m_drivetrain->getModulePositions(), bot_pose);
}

frc::FieldObject2d *Odometry::getField2dObject(std::string_view name)
{
  return field2d.GetObject(name);
}

void Odometry::add_vision_measurment(const frc::Pose2d &pose)
{
  estimator.AddVisionMeasurement(pose, frc::Timer::GetFPGATimestamp());
}

void Odometry::update_from_vision()
{
  auto pose = estimator.GetEstimatedPosition();
  frc::SmartDashboard::PutNumber("odometry/X", pose.X().value());
  frc::SmartDashboard::PutNumber("odometry/Y", pose.Y().value());

  for (std::optional<frc::Pose2d> i : m_vision->get_bot_position())
  {
    if (i)
    {
      estimator.AddVisionMeasurement(frc::Pose2d(i.value().Translation(), m_drivetrain->getCCWHeading()), frc::Timer::GetFPGATimestamp());
    }
  }
}

std::optional<units::degree_t> Odometry::get_coral()
{
}

std::optional<units::meter_t> Odometry::get_dist_to_tgt()
{
  frc::SmartDashboard::PutBoolean("tv", m_limelight->GetBoolean("tv", 0));
  if (m_limelight->GetNumber("tv", 0))
  {
    auto results = m_limelight->GetNumberArray("targetpose_robotspace", std::vector<double>(6));
    return units::meter_t{(std::sqrt(std::pow(results[0], 0) + std::pow(results[1], 2)))};
  }
  else
  {
    return std::nullopt;
  }
}

units::turn_t Odometry::get_shooter_angle()
{
  auto pose = getPose();
  double x = pose.X().value();
  double y = units::math::fabs(pose.Y() - 5.548_m).value();
  x = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
  // units::turn_t angle = units::turn_t{-(0.047 * std::pow(x, 2)) + (1.62 * x) - 17.3};
  if (frc::DriverStation::GetAlliance().has_value() && frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed)
  {
    x = (16.9) - x;
  }
  units::turn_t angle = units::turn_t{19.5 - (8.1 * x) + (1.31 * std::pow(x, 2)) + (-0.0752 * std::pow(x, 3))};
  frc::SmartDashboard::PutNumber("shooter/eq", 0);

  if (x > 4.25)
  {
    frc::SmartDashboard::PutNumber("shooter/eq", 1);

    angle = units::turn_t{26.9 - (11.8 * x) + (1.82 * std::pow(x, 2)) - (0.096 * std::pow(x, 3))};
  }
  if (x > 7.7)
  {
    angle = 0_tr;
  }
  frc::SmartDashboard::PutNumber("shooter/auto angle", angle.value());
  frc::SmartDashboard::PutNumber("shooter/auto x", x);

  return angle;
}
#endif
