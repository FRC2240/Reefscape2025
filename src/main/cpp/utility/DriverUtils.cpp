#include "utility/DriverUtils.h"


void DriverUtils::DriverTeleopPeriodic() {
  frc2::CommandXboxController m_stick{0};

  double time = frc::DriverStation::GetMatchTime().value();
 
  frc::SmartDashboard::PutNumber("time", time);
  if (time <= 45.0 && time > 45.0 - ALERT_RUMBLE_DURATION) {
    m_stick.SetRumble(frc::GenericHID::kBothRumble, 1);
  } else {
    m_stick.SetRumble(frc::GenericHID::kBothRumble, 0);
  }
}