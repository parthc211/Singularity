#pragma once

#include <string_view>

namespace SGE {

enum class LogLevel { Info, Warning, Error };

void Log(LogLevel level, std::string_view message);

inline void LogInfo(std::string_view msg)  { Log(LogLevel::Info,    msg); }
inline void LogWarn(std::string_view msg)  { Log(LogLevel::Warning, msg); }
inline void LogError(std::string_view msg) { Log(LogLevel::Error,   msg); }

} // namespace SGE
