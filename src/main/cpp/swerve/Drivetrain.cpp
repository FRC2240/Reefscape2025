#include "swerve/Drivetrain.h"
#ifndef CFG_NO_DRIVEBASE
/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// static std::unique_ptr<AHRS> navx;
//  These are "public" (not static) bc they are accessed by the Trajectory namespace

namespace Module
{
  std::unique_ptr<SwerveModule> front_left;
  std::unique_ptr<SwerveModule> front_right;
  std::unique_ptr<SwerveModule> back_left;
  std::unique_ptr<SwerveModule> back_right;
}
// This is not how it should be but doing it "correctly" (++,+-,-+,--) causes
// the wheels to form an "X" instead of diamond while turning.
// It's wrong but it works, no touchy.
/*
frc::SwerveDriveKinematics<4> kinematics{frc::Translation2d{12.25_in, -12.25_in},
                                         frc::Translation2d{12.25_in, 12.25_in},
                                         frc::Translation2d{-12.25_in, -12.25_in},
                                         frc::Translation2d{-12.25_in, 12.25_in}};
                                         */

Drivetrain::Drivetrain()
{
  ctre::phoenix6::configs::Pigeon2Configuration gyro_conf{};
  gyro_conf.MountPose.MountPoseYaw = 180_deg;
  gyro.GetConfigurator().Apply(gyro_conf);
  using namespace Module;
  front_left = std::make_unique<SwerveModule>(60, 61, 14, CONSTANTS::DRIVE::CONFIG::FL.offset);
  front_right = std::make_unique<SwerveModule>(50, 51, 13, CONSTANTS::DRIVE::CONFIG::FR.offset);
  back_left = std::make_unique<SwerveModule>(30, 31, 11, CONSTANTS::DRIVE::CONFIG::BL.offset);
  back_right = std::make_unique<SwerveModule>(40, 41, 12, CONSTANTS::DRIVE::CONFIG::BR.offset);
}

void Drivetrain::set_brake_mode(bool on)
{
}

double Drivetrain::get_pitch()
{
  return gyro.GetPitch().GetValue().value();
}

void Drivetrain::flip()
{
}

double Drivetrain::get_offset()
{
  // return navx.GetAngleAdjustment();
}

void Drivetrain::zero_adjustment()
{
  // navx.ResetDisplacement();
  // navx.SetAngleAdjustment(0);
}

void Drivetrain::log_accel()
{
  frc::SmartDashboard::PutNumber("accel/X", gyro.GetAccelerationX().GetValue().convert<units::meters_per_second_squared>().value());
  frc::SmartDashboard::PutNumber("accel/Y", gyro.GetAccelerationY().GetValue().convert<units::meters_per_second_squared>().value());
  frc::SmartDashboard::PutNumber("accel/Z", gyro.GetAccelerationZ().GetValue().convert<units::meters_per_second_squared>().value());
}

void Drivetrain::zero_yaw()
{
  gyro.SetYaw(0_deg);
}

void Drivetrain::print_angle()
{
  using namespace Module;
  {
    frc::SmartDashboard::PutNumber("fl/angle", front_left->getState().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("fl/speed", front_left->getState().speed.value());

    frc::SmartDashboard::PutNumber("fr/angle", front_right->getAngle().value());
    frc::SmartDashboard::PutNumber("fr/speed", front_right->getState().speed.value());

    frc::SmartDashboard::PutNumber("br/angle", back_right->getState().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("br/speed", back_right->getState().speed.value());

    frc::SmartDashboard::PutNumber("bl/angle", back_left->getState().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("bl/speed", back_left->getState().speed.value());

  } // namespace Module
}
/******************************************************************/
/*                        Public Variables                        */
/******************************************************************/

// All variables interacting with hardware need initalized in init()
// to avoid issues with initalizing before wpilib

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Drivetrain::init()
{
}

// Returns values with 0 being front and positive angles going CW
units::degree_t Drivetrain::getAngle()
{
  // static bool first_time_getting_angle = true;

  // if (first_time_getting_angle)
  // {
  //   navx.ZeroYaw(); // This can't be called in init() since the gyro will still be calibrating
  //   first_time_getting_angle = false;
  // }
  // return units::degree_t{-navx.GetAngle()};
  return gyro.GetYaw().GetValue();
}
// IMPORTANT: CCW (counterclockwise) must not be inverted and CW (clockwise)
// must be. If CCW is negative and CW is positive, a 90 degree turn will
// cause feild centric inputs to be inverted.
// It's weird but the inversion as it stands is good and works, even though
// it seems odd.
frc::Rotation2d Drivetrain::getCCWHeading() { return {getAngle()}; }
// or navx.GetRotation()

frc::Rotation2d Drivetrain::getCWHeading() { return {-getAngle()}; }

units::degree_t Drivetrain::get_absolute_angle()
{
  auto angle = Drivetrain::getAngle().value();
  auto a = angle / 360.0;
  double b = 0.0;
  double c = 0.0;
  b = 360.0 * (a - floor(a));
  if (b < -180.0)
    c = b + 360.0;
  else if (b > 180.0)
    c = b - 360.0;
  else
    c = b;

  units::degree_t d{c};
  return d;
}

wpi::array<double, 4> Drivetrain::getDriverTemps()
{
  using namespace Module;
  return {front_left->getDriverTemp(),
          front_right->getDriverTemp(),
          back_left->getDriverTemp(),
          back_right->getDriverTemp()};
}
void Drivetrain::debug_angles()
{
  /*
    frc::SmartDashboard::PutNumber("front left alignment", fl_old.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("front right alignment", fr_old.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("back left alignment", bl_old.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("back right alignment", br_old.angle.Degrees().value());

    frc::SmartDashboard::PutNumber("front left pos", Module::front_left->getEncoder());
    frc::SmartDashboard::PutNumber("front right pos", Module::front_right->getEncoder());
    frc::SmartDashboard::PutNumber("back left ps", Module::back_left->getEncoder());
    frc::SmartDashboard::PutNumber("back right pos", Module::back_right->getEncoder());
  */
}
wpi::array<double, 4> Drivetrain::getTurnerTemps()
{
  using namespace Module;
  return {front_left->getTurnerTemp(),
          front_right->getTurnerTemp(),
          back_left->getTurnerTemp(),
          back_right->getTurnerTemp()};
}

frc::ChassisSpeeds Drivetrain::getRobotRelativeSpeeds()
{
  return kinematics.ToChassisSpeeds(Module::front_left->getState(),
                                    Module::front_right->getState(),
                                    Module::back_left->getState(),
                                    Module::back_right->getState());
}

wpi::array<frc::SwerveModuleState, 4> Drivetrain::getModuleStates()
{
  return {Module::front_left->getState(),
          Module::front_right->getState(),
          Module::back_left->getState(),
          Module::back_right->getState()};
}

wpi::array<frc::SwerveModulePosition, 4> Drivetrain::getModulePositions()
{
  return {Module::front_left->getPosition(),
          Module::front_right->getPosition(),
          Module::back_left->getPosition(),
          Module::back_right->getPosition()};
}
/******************************************************************/
/*                       Driving Functions                        */
/******************************************************************/

void Drivetrain::tankDrive(double const &l_speed, double const &r_speed)
{
  using namespace Module;
  front_left->percentOutputControl(l_speed);
  front_right->percentOutputControl(-r_speed);
  back_left->percentOutputControl(l_speed);
  back_right->percentOutputControl(-r_speed);
}
// Converts inputted speeds into a frc::ChassisSpeeds object
void Drivetrain::drive(units::meters_per_second_t const &xSpeed,
                       units::meters_per_second_t const &ySpeed,
                       units::radians_per_second_t const &rot,
                       bool const &fieldRelative)
{
  auto const speeds = fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getCCWHeading())
                                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot};

  frc::SmartDashboard::PutNumber("cspeed/vy", speeds.vx.value());
  frc::SmartDashboard::PutNumber("cspeed/vx", speeds.vy.value());
  drive(speeds);
}

// Takes the speed & direction the robot should be going and figures out the states for each indivdual module
void Drivetrain::drive(frc::ChassisSpeeds const &speeds)
{
  drive(kinematics.ToSwerveModuleStates(speeds));

  if constexpr (debugging)
  {
    // frc::SmartDashboard::PutNumber("Target VX Speed", speeds.vx.value());
    // frc::SmartDashboard::PutNumber("Target VY Speed", speeds.vy.value());
    // frc::SmartDashboard::PutNumber("Target Omega Speed (CCW is +)", units::degrees_per_second_t{speeds.omega}.value() / 720);
  }
}

// Sets each module to the desired state
void Drivetrain::drive(wpi::array<frc::SwerveModuleState, 4> states)
{
  // frc::SmartDashboard::PutNumber("robot speed", std::sqrt(std::pow(navx.GetVelocityX(), 2) + std::pow(navx.GetVelocityY(), 2)));
  kinematics.DesaturateWheelSpeeds(&states, MODULE_MAX_SPEED);

  auto const [fl, fr, bl, br] = states;
  frc::SmartDashboard::PutNumber("fl/desiredstate", fl.speed.value());
  using namespace Module;
  front_left->setDesiredState(fl);
  front_right->setDesiredState(fr);
  back_left->setDesiredState(bl);
  back_right->setDesiredState(br);

  /*std::cout << "FL: " << front_left->getAngle().value()
            << "FR: " << front_right->getAngle().value()
            << "BL: " << back_left->getAngle().value()
            << "BR: " << back_right->getAngle().value()
            << "\n";*/

  if constexpr (debugging)
  {
    /*
    frc::SmartDashboard::PutString("Target Front Left Module", fmt::format("Speed (mps): {}, Direction: {}", fl.speed.value(), fl.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Target Front Right Module", fmt::format("Speed (mps): {}, Direction: {}", fr.speed.value(), fr.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Target Back Left Module", fmt::format("Speed (mps): {}, Direction: {}", bl.speed.value(), bl.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Target Back Right Module", fmt::format("Speed (mps): {}, Direction: {}", br.speed.value(), br.angle.Degrees().value()));

    auto const fl_old = front_left->getState();
    auto const fr_old = front_left->getState();
    auto const bl_old = back_left->getState();
    auto const br_old = back_right->getState();
    frc::SmartDashboard::PutString("Actual Front Left Module", fmt::format("Speed (mps): {}, Direction: {}", fl_old.speed, fl_old.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Actual Front Right Module", fmt::format("Speed (mps): {}, Direction: {}", fr_old.speed.value(), fr_old.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Actual Back Left Module", fmt::format("Speed (mps): {}, Direction: {}", bl_old.speed.value(), bl_old.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Actual Back Right Module", fmt::format("Speed (mps): {}, Direction: {}", br_old.speed.value(), br_old.angle.Degrees().value()));
    */
  }
}

void Drivetrain::stop()
{
  frc::SwerveModuleState stopped{0_mps, {}};

  using namespace Module;
  front_left->setDesiredState(stopped);
  front_right->setDesiredState(stopped);
  back_left->setDesiredState(stopped);
  back_right->setDesiredState(stopped);
}

/******************************************************************/
/*                        Facing Functions                        */
/******************************************************************/
bool Drivetrain::snap_to_zero()
{

  auto angle = Drivetrain::get_absolute_angle();
  if ((angle >= 179_deg && angle <= 181_deg) || (angle <= -179_deg && angle >= -181_deg))
  {
    // std::cout << "passed check\n";
    return true;
  }
  else
  {
    Drivetrain::faceDirection(0_mps, 0_mps, 180_deg, false, 4.5);
    // std::cout << "failed threshold check: " << Drivetrain::get_absolute_angle().value() << std::endl;
    return false;
  }
}

bool Drivetrain::human_player_snap()
{
  Drivetrain::faceDirection(0_mps, 0_mps, 0_deg, false, 6.5);
  auto angle = Drivetrain::get_absolute_angle();

  if ((angle >= -1_deg) && (angle <= 1_deg))
  {
    return true;
  }
  else
  {
    // std::cout << "failed threshold check: " << Drivetrain::get_absolute_angle().value() << std::endl;
    return false;
  }
}
void Drivetrain::faceDirection(units::meters_per_second_t const &dx,
                               units::meters_per_second_t const &dy,
                               units::degree_t const &theta,
                               bool const &field_relative,
                               double const &rot_p,
                               units::degrees_per_second_t const &max_rot_speed)
{
  int error_theta = (theta + getAngle()).to<int>() % 360; // Get difference between old and new angle;
                                                          // gets the equivalent value between -360 and 360

  if (error_theta < -180)
    error_theta += 360; // Ensure angle is between -180 and 360
  if (error_theta > 180)
    error_theta -= 360; // Optimizes angle if over 180
                        //  if (std::abs(error_theta) < 5)
  //  error_theta = 0; // Dead-zone to prevent oscillation

  double p_rotation = error_theta * rot_p; // Modifies error_theta in order to get a faster turning speed

  /*  if (std::abs(p_rotation) > max_rot_speed.value())
      p_rotation = max_rot_speed.value() * ((p_rotation > 0) ? 1 : -1); // Constrains turn speed
  */
  // p_rotation is negated since the robot actually turns ccw, not cw
  drive(dx, dy, units::degrees_per_second_t{-p_rotation}, field_relative);
}

void Drivetrain::faceClosest(units::meters_per_second_t const &dx,
                             units::meters_per_second_t const &dy,
                             bool const &field_relative,
                             double const &rot_p,
                             units::degrees_per_second_t const &max_rot_speed)
{
  int current_rotation = getAngle().to<int>() % 360; // Ensure angle is between -360 and 360

  if (current_rotation < 0)
    current_rotation += 360; // Ensure angle is between 0 and 360

  if (current_rotation <= 90 || current_rotation >= 270)
    faceDirection(dx, dy, 0_deg, field_relative, rot_p, max_rot_speed);
  else
    faceDirection(dx, dy, 180_deg, field_relative, rot_p, max_rot_speed);
}

void Drivetrain::manualPercentOutput(double const &percent_output)
{
  using namespace Module;
  front_left->percentOutputControl(percent_output);
  front_right->percentOutputControl(percent_output);
  back_left->percentOutputControl(percent_output);
  back_right->percentOutputControl(percent_output);
}

void Drivetrain::manualVelocity(double const &velocity_ticks_per_100ms)
{
  using namespace Module;
  front_left->manualVelocityContol(velocity_ticks_per_100ms);
  front_right->manualVelocityContol(velocity_ticks_per_100ms);
  back_left->manualVelocityContol(velocity_ticks_per_100ms);
  back_right->manualVelocityContol(velocity_ticks_per_100ms);
}

bool Drivetrain::face_direction(units::degree_t tgt, double feedback_device)
{
  turn_coral_pid.SetSetpoint(tgt.value());
  double pid_out = turn_coral_pid.Calculate(feedback_device);
  drive(0_mps, 0_mps, units::degrees_per_second_t{-pid_out}, false);
  frc::SmartDashboard::PutNumber("PID out", pid_out);
  frc::SmartDashboard::PutNumber("PID target", tgt.value());
  frc::SmartDashboard::PutNumber("Current  rotation", feedback_device);

  if ((feedback_device >= turn_pid.GetSetpoint() - 1) && (feedback_device <= turn_pid.GetSetpoint() + 1))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool Drivetrain::face_direction(units::degree_t tgt, units::meters_per_second_t dx, units::meters_per_second_t dy)
{
  auto angle = get_absolute_angle().value();
  turn_pid.SetSetpoint(tgt.value());
  auto delta = angle - tgt.value();
  if (delta > 180.0)
  {
    delta -= 360.0;
  }
  else if (delta < -180.0)
  {
    delta += 360.0;
  }
  double pid_out = turn_pid.Calculate(angle);
  frc::SmartDashboard::PutNumber("PID out", pid_out);
  frc::SmartDashboard::PutNumber("PID target", tgt.value());
  frc::SmartDashboard::PutNumber("Current rotation", angle);
  frc::SmartDashboard::PutNumber("Delta", delta);

  drive(-dx, -dy, units::degrees_per_second_t{-pid_out}, true);
  frc::SmartDashboard::PutNumber("PID Setpoint", turn_pid.GetSetpoint());
  if ((angle >= turn_pid.GetSetpoint() - .1) && (angle <= turn_pid.GetSetpoint() + .1))
  {
    return true;
    frc::SmartDashboard::PutBoolean("Face Direction", true);
  }
  else
  {
    return false;
    frc::SmartDashboard::PutBoolean("Face Direction", false);
  }
}
#endif
