#include "utility/DriverUtils.h"


void DriverUtils::DriverTeleopPeriodic() {
  frc2::CommandXboxController m_stick{0};

  double time = frc::DriverStation::GetMatchTime().value();
  // double result = units::unit_cast<double>(time);
  frc::SmartDashboard::PutNumber("time", time);
  int duration = 1;
  if (time <= 45 && time > 45 - duration) {
    m_stick.SetRumble(frc::GenericHID::kBothRumble, 1);
  } else {
    m_stick.SetRumble(frc::GenericHID::kBothRumble, 0);
  }
}