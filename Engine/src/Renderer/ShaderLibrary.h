#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>

#include "Renderer/DXC/ShaderBlob.h"

#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace SGE {

using ReloadCallback = std::function<void()>;

// Compiles and caches shaders via DXC. Watches the shader directory for file changes
// and hot-reloads affected shaders at the start of each frame.
class ShaderLibrary
{
public:
    ShaderLibrary();
    ~ShaderLibrary();

    // Initialize DXC and start watching shaderDir for changes.
    bool Initialize(const std::filesystem::path& shaderDir);
    void Shutdown();

    // Compile (or return cached result for) a shader. relativePath is relative to shaderDir.
    std::shared_ptr<ShaderBlob> GetOrCompile(
        const std::wstring& relativePath,
        const char*         entryPoint,
        const char*         target
    );

    // Register a callback invoked when relativePath is modified on disk.
    void OnReload(const std::wstring& relativePath, ReloadCallback cb);

    // Call once per frame before drawing. Recompiles any changed shaders and fires their callbacks.
    // Returns true if at least one shader was reloaded.
    bool FlushReloads();

private:
    struct CacheKey {
        std::wstring path;
        std::string  entryPoint;
        std::string  target;
        bool operator==(const CacheKey&) const = default;
    };
    struct CacheKeyHash {
        size_t operator()(const CacheKey& k) const;
    };

    void WatcherThread();

    std::filesystem::path m_shaderDir;

    std::unordered_map<CacheKey, std::shared_ptr<ShaderBlob>, CacheKeyHash> m_cache;
    std::unordered_map<std::wstring, std::vector<ReloadCallback>>            m_callbacks;

    // File watcher
    std::thread       m_watchThread;
    std::atomic<bool> m_stop  { false };
    HANDLE            m_dir   = INVALID_HANDLE_VALUE;
    HANDLE            m_stopEvent = nullptr;

    // Written by watcher thread, consumed by main thread in FlushReloads.
    std::mutex               m_pendingMtx;
    std::vector<std::wstring> m_pending; // relative file names
};

} // namespace SGE
