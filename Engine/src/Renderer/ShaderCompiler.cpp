#include "Renderer/ShaderCompiler.h"
#include "Core/Logger.h"

#include <d3dcompiler.h>
#include <stdexcept>
#include <format>

namespace SGE {

ComPtr<ID3DBlob> CompileShader(const std::wstring& path, const char* entryPoint, const char* target)
{
    UINT flags = 0;
#ifdef SGE_DEBUG
    flags |= D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

    ComPtr<ID3DBlob> code;
    ComPtr<ID3DBlob> errors;

    HRESULT hr = D3DCompileFromFile(
        path.c_str(),
        nullptr,
        D3D_COMPILE_STANDARD_FILE_INCLUDE,
        entryPoint,
        target,
        flags, 0,
        &code,
        &errors
    );

    if (errors)
        LogWarn(static_cast<const char*>(errors->GetBufferPointer()));

    if (FAILED(hr)) {
        int len = WideCharToMultiByte(CP_UTF8, 0, path.c_str(), -1, nullptr, 0, nullptr, nullptr);
        std::string p(len, '\0');
        WideCharToMultiByte(CP_UTF8, 0, path.c_str(), -1, p.data(), len, nullptr, nullptr);
        throw std::runtime_error(std::format("Shader compile failed: {} [{}::{}]", p, target, entryPoint));
    }

    return code;
}

} // namespace SGE
