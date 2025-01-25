#include <string>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Alert.h>
#include <fmt/color.h>

namespace ForceLog
{
    void info(const std::string &message);
    void warning(const std::string &message);
    void error(const std::string &message);
    void fatal(const std::string &message);
    void debug(const std::string &message);

} // namespace ForceLog
