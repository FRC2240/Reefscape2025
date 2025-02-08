#include "utility/MotorUtils.h"
#include "frc/DataLogManager.h"
#include "frc/smartdashboard/SmartDashboard.h"

void MotorUtils::SetPID(ctre::phoenix6::hardware::TalonFX &motor, CONSTANTS::PidCoeff coeff)
{
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  motor.GetConfigurator().Refresh(config);
  config.Slot0.kS = coeff.kS;
  config.Slot0.kP = coeff.kP;
  config.Slot0.kI = coeff.kI;
  config.Slot0.kD = coeff.kD;
  config.CurrentLimits.StatorCurrentLimit = CONSTANTS::DEFAULT_CURRENT_LIMIT;
  config.Audio.BeepOnConfig = true;
  config.Audio.BeepOnBoot = true;

  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int j = 0; j < MAX_CONFIG_APPLY_ATTEMPTS; j++)
  {
    status = motor.GetConfigurator().Apply(config);
    if (status.IsOK())
      break;
  }
  if (!status.IsOK())
  {
    std::string errString = status.GetName();
    frc::DataLogManager::Log("Could not apply configs, error code: " + errString);
  }

  frc::SmartDashboard::PutNumber("S-PID", config.Slot0.kS);
  frc::SmartDashboard::PutNumber("P-PID", config.Slot0.kP);
  frc::SmartDashboard::PutNumber("I-PID", config.Slot0.kI);
  frc::SmartDashboard::PutNumber("D-PID", config.Slot0.kD);
}

void MotorUtils::BuildSender(wpi::SendableBuilder &builder, CONSTANTS::PidCoeff *coeff)
{
  builder.AddDoubleProperty(
      "PID/P",
      [coeff] { return coeff->GetP(); },
      [coeff](double val) { coeff->SetP(val); });
  builder.AddDoubleProperty(
      "PID/I",
      [coeff] { return coeff->GetI(); },
      [coeff](double val) { coeff->SetI(val); });
  builder.AddDoubleProperty(
      "PID/D",
      [coeff] { return coeff->GetD(); },
      [coeff](double val) { coeff->SetD(val); });
  builder.AddDoubleProperty(
      "PID/S",
      [coeff] { return coeff->GetS(); },
      [coeff](double val) { coeff->SetS(val); });
  builder.AddDoubleProperty(
      "PID/G",
      [coeff] { return coeff->GetG(); },
      [coeff](double val) { coeff->SetG(val); });
  builder.AddDoubleProperty(
      "PID/Min",
      [coeff] { return coeff->GetMin(); },
      [coeff](double val) { coeff->SetMin(val); });
  builder.AddDoubleProperty(
      "PID/Max",
      [coeff] { return coeff->GetMax(); },
      [coeff](double val) { coeff->SetMax(val); });
}

void MotorUtils::BuildSender(wpi::SendableBuilder &builder, ctre::phoenix6::hardware::TalonFX *motor)
{
  builder.AddDoubleProperty(
      "LOG/Position",
      [motor]
      { return motor->GetPosition().GetValueAsDouble(); },
      nullptr);
  builder.AddDoubleProperty(
      "LOG/Velocity",
      [motor]
      { return motor->GetVelocity().GetValueAsDouble(); },
      nullptr);
  builder.AddDoubleProperty(
      "LOG/Acceleration",
      [motor]
      { return motor->GetAcceleration().GetValueAsDouble(); },
      nullptr);
  builder.AddDoubleProperty(
      "LOG/Temp",
      [motor]
      { return motor->GetDeviceTemp().GetValueAsDouble(); },
      nullptr);
  builder.AddDoubleProperty(
      "LOG/SupplyCurrent",
      [motor]
      { return motor->GetSupplyCurrent().GetValueAsDouble(); },
      nullptr);
  builder.AddDoubleProperty(
      "LOG/StatorCurrent",
      [motor]
      { return motor->GetStatorCurrent().GetValueAsDouble(); },
      nullptr
      );
}