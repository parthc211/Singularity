#include "Renderer/DX12/DX12Common.h"  // <Windows.h> must precede dxcapi.h
#include <initguid.h>                   // defines CLSID_DxcUtils / CLSID_DxcCompiler (ONE TU only)
#include <dxcapi.h>
#include <d3d12shader.h>

#include "Renderer/DXC/DxcInstance.h"
#include "Core/Logger.h"

#include <format>
#include <stdexcept>

namespace SGE {

// -------------------------------------------------------------------------
// PIMPL — all DXC COM types live here, invisible to headers
// -------------------------------------------------------------------------

struct DxcInstance::Impl
{
    HMODULE                    module          = nullptr;
    ComPtr<IDxcUtils>          utils;
    ComPtr<IDxcCompiler3>      compiler;
    ComPtr<IDxcIncludeHandler> includeHandler;
};

DxcInstance::DxcInstance()  : m_impl(std::make_unique<Impl>()) {}
DxcInstance::~DxcInstance() = default;

// -------------------------------------------------------------------------

DxcInstance& DxcInstance::Get()
{
    static DxcInstance instance;
    return instance;
}

bool DxcInstance::Initialize()
{
    if (m_impl->module) return true;

    m_impl->module = LoadLibraryW(L"dxcompiler.dll");
    if (!m_impl->module) {
        LogError("Failed to load dxcompiler.dll — ensure it is beside the executable.");
        return false;
    }

    auto pfnCreate = reinterpret_cast<DxcCreateInstanceProc>(
        GetProcAddress(m_impl->module, "DxcCreateInstance"));
    if (!pfnCreate) {
        LogError("DxcCreateInstance not found in dxcompiler.dll.");
        return false;
    }

    SGE_THROW_IF_FAILED(pfnCreate(CLSID_DxcUtils,    IID_PPV_ARGS(&m_impl->utils)));
    SGE_THROW_IF_FAILED(pfnCreate(CLSID_DxcCompiler, IID_PPV_ARGS(&m_impl->compiler)));
    SGE_THROW_IF_FAILED(m_impl->utils->CreateDefaultIncludeHandler(&m_impl->includeHandler));

    LogInfo("DXC initialized.");
    return true;
}

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------

static DXGI_FORMAT FormatFromParam(const D3D12_SIGNATURE_PARAMETER_DESC& p)
{
    UINT n = 0;
    for (BYTE m = p.Mask; m; m >>= 1) n += (m & 1);

    switch (p.ComponentType) {
    case D3D_REGISTER_COMPONENT_FLOAT32:
        switch (n) {
        case 1: return DXGI_FORMAT_R32_FLOAT;
        case 2: return DXGI_FORMAT_R32G32_FLOAT;
        case 3: return DXGI_FORMAT_R32G32B32_FLOAT;
        case 4: return DXGI_FORMAT_R32G32B32A32_FLOAT;
        }
        break;
    case D3D_REGISTER_COMPONENT_UINT32:
        switch (n) {
        case 1: return DXGI_FORMAT_R32_UINT;
        case 2: return DXGI_FORMAT_R32G32_UINT;
        case 3: return DXGI_FORMAT_R32G32B32_UINT;
        case 4: return DXGI_FORMAT_R32G32B32A32_UINT;
        }
        break;
    case D3D_REGISTER_COMPONENT_SINT32:
        switch (n) {
        case 1: return DXGI_FORMAT_R32_SINT;
        case 2: return DXGI_FORMAT_R32G32_SINT;
        case 3: return DXGI_FORMAT_R32G32B32_SINT;
        case 4: return DXGI_FORMAT_R32G32B32A32_SINT;
        }
        break;
    }
    return DXGI_FORMAT_UNKNOWN;
}

static std::string WideToUtf8(const std::wstring& w)
{
    int len = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
    std::string s(len, '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, s.data(), len, nullptr, nullptr);
    return s;
}

// -------------------------------------------------------------------------
// Compile
// -------------------------------------------------------------------------

std::shared_ptr<ShaderBlob> DxcInstance::Compile(
    const std::wstring& fullPath,
    const char*         entryPoint,
    const char*         target)
{
    ComPtr<IDxcBlobEncoding> source;
    if (FAILED(m_impl->utils->LoadFile(fullPath.c_str(), nullptr, &source)))
        throw std::runtime_error(std::format("Shader not found: {}", WideToUtf8(fullPath)));

    DxcBuffer buf { source->GetBufferPointer(), source->GetBufferSize(), DXC_CP_UTF8 };

    std::wstring wEntry(entryPoint, entryPoint + strlen(entryPoint));
    std::wstring wTarget(target,    target    + strlen(target));

    std::vector<LPCWSTR> args = {
        fullPath.c_str(),
        L"-E", wEntry.c_str(),
        L"-T", wTarget.c_str(),
        L"-HV", L"2021",
        L"-Qstrip_reflect",
    };
#ifdef SGE_DEBUG
    args.push_back(L"-Zi");
    args.push_back(L"-Od");
#else
    args.push_back(L"-O3");
#endif

    ComPtr<IDxcResult> result;
    SGE_THROW_IF_FAILED(m_impl->compiler->Compile(
        &buf,
        args.data(), static_cast<UINT32>(args.size()),
        m_impl->includeHandler.Get(),
        IID_PPV_ARGS(&result)));

    ComPtr<IDxcBlobEncoding> errors;
    if (SUCCEEDED(result->GetOutput(DXC_OUT_ERRORS, IID_PPV_ARGS(&errors), nullptr))
        && errors && errors->GetBufferSize() > 0)
    {
        LogWarn(static_cast<const char*>(errors->GetBufferPointer()));
    }

    HRESULT status = S_OK;
    result->GetStatus(&status);
    if (FAILED(status))
        throw std::runtime_error(std::format("Compile failed: {}::{}", WideToUtf8(fullPath), entryPoint));

    auto blob = std::make_shared<ShaderBlob>();

    // Copy DXIL bytecode into std::vector<uint8_t>
    ComPtr<IDxcBlob> dxil;
    result->GetOutput(DXC_OUT_OBJECT, IID_PPV_ARGS(&dxil), nullptr);
    if (dxil && dxil->GetBufferSize() > 0) {
        const auto* ptr = static_cast<const uint8_t*>(dxil->GetBufferPointer());
        blob->bytecode.assign(ptr, ptr + dxil->GetBufferSize());
    }

    // Reflection — local only, used to build inputLayout
    ComPtr<IDxcBlob> reflData;
    if (SUCCEEDED(result->GetOutput(DXC_OUT_REFLECTION, IID_PPV_ARGS(&reflData), nullptr)) && reflData)
    {
        DxcBuffer reflBuf { reflData->GetBufferPointer(), reflData->GetBufferSize(), 0 };
        ComPtr<ID3D12ShaderReflection> refl;
        m_impl->utils->CreateReflection(&reflBuf, IID_PPV_ARGS(&refl));

        if (refl && wTarget.starts_with(L"vs_")) {
            D3D12_SHADER_DESC sd = {};
            refl->GetDesc(&sd);

            blob->inputLayout.reserve(sd.InputParameters);
            blob->semanticNames.reserve(sd.InputParameters);

            for (UINT i = 0; i < sd.InputParameters; ++i) {
                D3D12_SIGNATURE_PARAMETER_DESC pd = {};
                refl->GetInputParameterDesc(i, &pd);

                if (pd.SystemValueType != D3D_NAME_UNDEFINED) continue;

                blob->semanticNames.push_back(pd.SemanticName);

                D3D12_INPUT_ELEMENT_DESC elem = {};
                elem.SemanticName         = blob->semanticNames.back().c_str();
                elem.SemanticIndex        = pd.SemanticIndex;
                elem.Format               = FormatFromParam(pd);
                elem.AlignedByteOffset    = D3D12_APPEND_ALIGNED_ELEMENT;
                elem.InputSlotClass       = D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA;
                blob->inputLayout.push_back(elem);
            }
        }
    }

    LogInfo(std::format("Compiled {}::{}", entryPoint, target));
    return blob;
}

// -------------------------------------------------------------------------

void DxcInstance::Shutdown()
{
    m_impl->includeHandler.Reset();
    m_impl->compiler.Reset();
    m_impl->utils.Reset();
    if (m_impl->module) { FreeLibrary(m_impl->module); m_impl->module = nullptr; }
}

} // namespace SGE
