#pragma once

#include "atomic"
#include "fmt/format.h"
#include "frc/Timer.h"
#include "iostream"
#include "string"
// Shuffuleboard API is compatable with Elastic. Use Elastic, not shuffleboard
#include <Elastic.h>
#include <fmt/color.h>
#include <frc/DataLogManager.h>
#include <frc/Timer.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace ForceLog {
/// @desc notify the driver about things to know about, but aren't problems
/// @param title This is in a slightly larger font in the notification
/// @param message the body of the message
static void info(std::string title, std::string message = "") {
  elastic::SendAlert(elastic::Notification{elastic::Notification::Level::INFO,
                                           title, message});
  frc::DataLogManager::Log(
      "[info] " + '(' + std::to_string(frc::Timer::GetFPGATimestamp().value()) +
      ") " + title + ": " + message + "\n");
}

/// @desc notify the driver about problems that interfere with robot
/// functionality and preformance, such as not seeing data in expected ranges
/// @param title This is in a slightly larger font in the notification
/// @param message the body of the message
static void warn(std::string title, std::string message = "") {
  elastic::SendAlert(elastic::Notification{
      elastic::Notification::Level::WARNING, title, message});
  frc::DataLogManager::Log(
      fmt::format(fmt::emphasis::bold | fg(fmt::color::yellow),
                  "[warning] " + '(' +
                      std::to_string(frc::Timer::GetFPGATimestamp().value()) +
                      ") " + title + ": " + message + "\n"));
}
/// @desc notify the driver about fatal faults. Only use this if things are very
/// wrong
/// @param title This is in a slightly larger font in the notification
/// @param message the body of the message
static void error(std::string title, std::string message = "") {
  elastic::SendAlert(elastic::Notification{elastic::Notification::Level::ERROR,
                                           title, message});
  frc::DataLogManager::Log(
      fmt::format(fmt::emphasis::bold | bg(fmt::color::yellow),
                  "[error] " + '(' +
                      std::to_string(frc::Timer::GetFPGATimestamp().value()) +
                      ") " + title + ": " + message + "\n"));
}

} // namespace ForceLog
