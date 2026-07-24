#include "Core/Logger.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>

#include <cstdio>
#include <format>
#include <string>

namespace SGE {

void Log(LogLevel level, std::string_view message)
{
    const char* prefix = nullptr;
    switch (level) {
    case LogLevel::Info:    prefix = "[INFO]   "; break;
    case LogLevel::Warning: prefix = "[WARN]   "; break;
    case LogLevel::Error:   prefix = "[ERROR]  "; break;
    }

    std::string line = std::format("{}{}\n", prefix, message);

    // Write to VS Output window
    OutputDebugStringA(line.c_str());

    // Also write to stdout so it shows in terminal builds
    fputs(line.c_str(), (level == LogLevel::Error) ? stderr : stdout);
}

} // namespace SGE
