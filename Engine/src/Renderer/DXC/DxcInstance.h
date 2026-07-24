#pragma once

#include "Renderer/DXC/ShaderBlob.h"

#include <memory>
#include <string>

namespace SGE {

// Singleton that owns IDxcCompiler3 and IDxcUtils, loaded dynamically from dxcompiler.dll.
// Call Initialize() once (via ShaderLibrary) before any Compile() calls.
// DXC COM types are hidden behind PIMPL so dxcapi.h is never included in this header.
class DxcInstance
{
public:
    static DxcInstance& Get();

    bool Initialize();
    void Shutdown();

    std::shared_ptr<ShaderBlob> Compile(
        const std::wstring& fullPath,
        const char*         entryPoint,
        const char*         target      // e.g. "vs_6_0", "ps_6_0"
    );

private:
    DxcInstance();
    ~DxcInstance();

    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SGE
