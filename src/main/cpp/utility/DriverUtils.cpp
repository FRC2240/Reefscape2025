#include "utility/DriverUtils.h"

constexpr double ALERT_RUMBLE_DURATION = 1.0;

void DriverUtils::DriverTeleopPeriodic() {
  frc2::CommandXboxController m_stick{0};

  double time = frc::DriverStation::GetMatchTime().value();
  // double result = units::unit_cast<double>(time);
  frc::SmartDashboard::PutNumber("time", time);
  if (time <= 45 && time > 45 - ALERT_RUMBLE_DURATION) {
    m_stick.SetRumble(frc::GenericHID::kBothRumble, 1);
  } else {
    m_stick.SetRumble(frc::GenericHID::kBothRumble, 0);
  }
}