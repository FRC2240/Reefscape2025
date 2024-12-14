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
constexpr void tstfn(){
    int x = 1;
    std::vector<int> intvec;
    intvec.push_back(x);
    std::string s = "a";
    s.append(std::string{"b"});
}

/// @desc notify the driver about things to know about, but aren't problems
/// @param title This is in a slightly larger font in the notification
/// @param message the body of the message
std::string strval = std::to_string(frc::Timer::GetTimestamp().value());
static void info(std::string title, std::string message = "") {
  elastic::SendAlert(elastic::Notification{elastic::Notification::Level::INFO,
                                           title, message});
  frc::DataLogManager::Log(
      "[info] " + '(' + strval +
      ") " + title + ": " + message + "\n");
}

/// @desc notify the driver about problems that interfere with robot
/// functionality and preformance, such as not seeing data in expected ranges
/// @param title This is in a slightly larger font in the notification
/// @param message the body of the message
static void warn(std::string title, std::string message = "") {
  elastic::SendAlert(elastic::Notification{
      elastic::Notification::Level::WARNING, title, message});
      
std::string strval = std::to_string(frc::Timer::GetTimestamp().value());
  frc::DataLogManager::Log(
      fmt::format(fmt::emphasis::bold | fg(fmt::color::yellow),
                  "[warning] " + '(' +
                      strval +
                      ") " + title + ": " + message + "\n"));
}
/// @desc notify the driver about fatal faults. Only use this if things are very
/// wrong
/// @param title This is in a slightly larger font in the notification
/// @param message the body of the message
static void error(std::string title, std::string message = "") {
  elastic::SendAlert(elastic::Notification{elastic::Notification::Level::ERROR,
                                           title, message});
std::string timestr = std::to_string(frc::Timer::GetTimestamp().value());
std::string leader = "[error] (";
std::string trail = ") ";
std::string colon = ": ";
constexpr std::string resultant = leader.append(timestr).append(trail).append(title).append(colon).append(message);
  frc::DataLogManager::Log(
      fmt::format(fmt::emphasis::bold | bg(fmt::color::yellow),
                  resultant));
}

} // namespace ForceLog
