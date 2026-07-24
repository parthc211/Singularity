#include "Renderer/ShaderLibrary.h"
#include "Renderer/DXC/DxcInstance.h"
#include "Core/Logger.h"

#include <format>

namespace SGE {

ShaderLibrary::ShaderLibrary()  = default;
ShaderLibrary::~ShaderLibrary() { Shutdown(); }

// -------------------------------------------------------------------------
// Initialization
// -------------------------------------------------------------------------

bool ShaderLibrary::Initialize(const std::filesystem::path& shaderDir)
{
    if (!DxcInstance::Get().Initialize()) return false;

    m_shaderDir = std::filesystem::absolute(shaderDir);
    m_stopEvent = CreateEventW(nullptr, FALSE, FALSE, nullptr);

    m_dir = CreateFileW(
        m_shaderDir.c_str(),
        FILE_LIST_DIRECTORY,
        FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
        nullptr,
        OPEN_EXISTING,
        FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OVERLAPPED,
        nullptr);

    if (m_dir == INVALID_HANDLE_VALUE) {
        LogWarn(std::format("Hot-reload unavailable: cannot open '{}'", m_shaderDir.string()));
    } else {
        m_watchThread = std::thread(&ShaderLibrary::WatcherThread, this);
        LogInfo(std::format("Watching shaders: {}", m_shaderDir.string()));
    }

    return true;
}

// -------------------------------------------------------------------------
// Compile / cache
// -------------------------------------------------------------------------

size_t ShaderLibrary::CacheKeyHash::operator()(const CacheKey& k) const
{
    auto h = std::hash<std::wstring>{}(k.path);
    h ^= std::hash<std::string>{}(k.entryPoint) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<std::string>{}(k.target)     + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
}

std::shared_ptr<ShaderBlob> ShaderLibrary::GetOrCompile(
    const std::wstring& relativePath,
    const char*         entryPoint,
    const char*         target)
{
    std::filesystem::path full = m_shaderDir / relativePath;
    CacheKey key { full.wstring(), entryPoint, target };

    auto it = m_cache.find(key);
    if (it != m_cache.end()) return it->second;

    auto blob = DxcInstance::Get().Compile(key.path, entryPoint, target);
    m_cache.emplace(key, blob);
    return blob;
}

void ShaderLibrary::OnReload(const std::wstring& relativePath, ReloadCallback cb)
{
    std::filesystem::path full = m_shaderDir / relativePath;
    m_callbacks[full.wstring()].push_back(std::move(cb));
}

// -------------------------------------------------------------------------
// Hot-reload (called from main thread, once per frame)
// -------------------------------------------------------------------------

bool ShaderLibrary::FlushReloads()
{
    std::vector<std::wstring> pending;
    {
        std::lock_guard lock(m_pendingMtx);
        pending = std::move(m_pending);
    }
    if (pending.empty()) return false;

    bool anyReloaded = false;

    for (const auto& relName : pending) {
        std::filesystem::path full = m_shaderDir / relName;

        // Recompile every cached entry whose path matches this file.
        for (auto& [key, blob] : m_cache) {
            if (!std::filesystem::equivalent(std::filesystem::path(key.path), full))
                continue;

            try {
                blob = DxcInstance::Get().Compile(key.path, key.entryPoint.c_str(), key.target.c_str());
                LogInfo(std::format("Hot-reloaded {}::{}", key.entryPoint, key.target));
                anyReloaded = true;
            } catch (const std::exception& e) {
                // Keep the old working shader — don't crash the running app.
                LogError(std::format("Hot-reload error: {}", e.what()));
            }
        }

        // Fire callbacks for this file (they rebuild PSOs etc.)
        auto it = m_callbacks.find(full.wstring());
        if (it != m_callbacks.end()) {
            for (auto& cb : it->second)
                cb();
        }
    }

    return anyReloaded;
}

// -------------------------------------------------------------------------
// File watcher (background thread)
// -------------------------------------------------------------------------

void ShaderLibrary::WatcherThread()
{
    alignas(DWORD) char buffer[4096];

    OVERLAPPED ov = {};
    ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    HANDLE events[] = { ov.hEvent, m_stopEvent };

    while (!m_stop.load())
    {
        BOOL ok = ReadDirectoryChangesW(
            m_dir, buffer, sizeof(buffer),
            FALSE,                         // don't recurse into subdirectories
            FILE_NOTIFY_CHANGE_LAST_WRITE,
            nullptr, &ov, nullptr);

        if (!ok) break;

        DWORD which = WaitForMultipleObjects(2, events, FALSE, INFINITE);

        if (which != WAIT_OBJECT_0) {
            // Stop event (or error) — cancel the pending IO and exit cleanly.
            CancelIoEx(m_dir, &ov);
            DWORD bytes;
            GetOverlappedResult(m_dir, &ov, &bytes, TRUE);
            break;
        }

        DWORD bytes = 0;
        GetOverlappedResult(m_dir, &ov, &bytes, FALSE);
        ResetEvent(ov.hEvent);

        if (bytes == 0) continue;

        // Walk the notification chain and push modified file names.
        auto* info = reinterpret_cast<FILE_NOTIFY_INFORMATION*>(buffer);
        for (;;) {
            if (info->Action == FILE_ACTION_MODIFIED) {
                std::wstring name(info->FileName, info->FileNameLength / sizeof(WCHAR));
                std::lock_guard lock(m_pendingMtx);
                // Deduplicate: editors often fire multiple MODIFIED events per save.
                bool dup = false;
                for (auto& p : m_pending) { if (p == name) { dup = true; break; } }
                if (!dup) m_pending.push_back(std::move(name));
            }
            if (info->NextEntryOffset == 0) break;
            info = reinterpret_cast<FILE_NOTIFY_INFORMATION*>(
                reinterpret_cast<char*>(info) + info->NextEntryOffset);
        }
    }

    CloseHandle(ov.hEvent);
}

// -------------------------------------------------------------------------
// Shutdown
// -------------------------------------------------------------------------

void ShaderLibrary::Shutdown()
{
    if (m_watchThread.joinable()) {
        m_stop.store(true);
        if (m_stopEvent) SetEvent(m_stopEvent);
        m_watchThread.join();
    }

    if (m_dir != INVALID_HANDLE_VALUE) { CloseHandle(m_dir); m_dir = INVALID_HANDLE_VALUE; }
    if (m_stopEvent)                   { CloseHandle(m_stopEvent); m_stopEvent = nullptr; }

    m_cache.clear();
    m_callbacks.clear();

    DxcInstance::Get().Shutdown();
    LogInfo("ShaderLibrary shut down.");
}

} // namespace SGE
