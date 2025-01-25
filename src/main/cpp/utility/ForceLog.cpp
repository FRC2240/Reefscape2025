#include <utility/ForceLog.h>

void ForceLog::info(const std::string &message)
{
    frc::DataLogManager::Log(fmt::format("INFO: {} ({})", message, frc::DriverStation::GetMatchTime().value()));
    auto old_arr = frc::SmartDashboard::GetStringArray("Log/info", std::vector<std::string>{});
    old_arr.push_back(message);
    frc::SmartDashboard::PutStringArray("Log/info", old_arr);
    frc::Alert{message, frc::Alert::AlertType::kInfo};
}
void ForceLog::warning(const std::string &message)
{
    frc::DataLogManager::Log(
        fmt::format(fg(fmt::color::yellow), "WARNING: {} ({})", message, frc::DriverStation::GetMatchTime().value()));
    auto old_arr = frc::SmartDashboard::GetStringArray("Log/warning", std::vector<std::string>{});
    old_arr.push_back(message);
    frc::SmartDashboard::PutStringArray("Log/warning", old_arr);
    frc::Alert{message, frc::Alert::AlertType::kWarning};
};

void ForceLog::debug(const std::string &message)
{
    frc::DataLogManager::Log(fmt::format("DEBUG: {} ({})", message, frc::DriverStation::GetMatchTime().value()));
    auto old_arr = frc::SmartDashboard::GetStringArray("Log/debug", std::vector<std::string>{});
    old_arr.push_back(message);
    frc::SmartDashboard::PutStringArray("Log/debug", old_arr);
}
void ForceLog::error(const std::string &message)
{
    frc::DataLogManager::Log(fmt::format(fg(fmt::color::red), "ERROR: {} ({})", message, frc::DriverStation::GetMatchTime().value()));
    frc::SmartDashboard::GetStringArray("Log/error", std::vector<std::string>{});
    auto old_arr = frc::SmartDashboard::GetStringArray("Log/error", std::vector<std::string>{});
    old_arr.push_back(message);
    frc::SmartDashboard::PutStringArray("Log/error", old_arr);
    frc::Alert{message, frc::Alert::AlertType::kError};
}
void ForceLog::fatal(const std::string &message)
{
    frc::DataLogManager::Log(
        fmt::format(bg(fmt::color::red) | fg(fmt::color::yellow) | fmt::emphasis::bold, "FATAL ERROR: {} ({})", message, frc::DriverStation::GetMatchTime().value()));
    auto old_arr = frc::SmartDashboard::GetStringArray("Log/fatal", std::vector<std::string>{});
    old_arr.push_back(message);
    frc::SmartDashboard::PutStringArray("Log/fatal", old_arr);
    frc::Alert{message, frc::Alert::AlertType::kError};
}