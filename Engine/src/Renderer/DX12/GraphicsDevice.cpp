#include "Renderer/DX12/GraphicsDevice.h"
#include "Core/Logger.h"

#include <format>

// Kept in the .cpp so the class layout is identical in debug and release builds.
#ifdef SGE_DEBUG
static Microsoft::WRL::ComPtr<ID3D12Debug1> s_debugController;
#endif

namespace SGE {

bool GraphicsDevice::Initialize(bool enableDebugLayer)
{
    try {
#ifdef SGE_DEBUG
        if (enableDebugLayer) {
            ComPtr<ID3D12Debug> debug0;
            if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debug0)))) {
                debug0.As(&s_debugController);
                s_debugController->EnableDebugLayer();
                s_debugController->SetEnableGPUBasedValidation(TRUE);
                LogInfo("DX12 debug layer enabled.");
            }
        }
#endif

        UINT factoryFlags = 0;
#ifdef SGE_DEBUG
        factoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
#endif
        SGE_THROW_IF_FAILED(CreateDXGIFactory2(factoryFlags, IID_PPV_ARGS(&m_factory)));

        SelectAdapter();

        SGE_THROW_IF_FAILED(D3D12CreateDevice(
            m_adapter.Get(),
            D3D_FEATURE_LEVEL_11_0,
            IID_PPV_ARGS(&m_device)
        ));

        D3D12_COMMAND_QUEUE_DESC queueDesc = {};
        queueDesc.Type     = D3D12_COMMAND_LIST_TYPE_DIRECT;
        queueDesc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL;
        queueDesc.Flags    = D3D12_COMMAND_QUEUE_FLAG_NONE;
        queueDesc.NodeMask = 0;
        SGE_THROW_IF_FAILED(m_device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&m_commandQueue)));

        DetectCapabilities();

        {
            DXGI_ADAPTER_DESC1 desc;
            m_adapter->GetDesc1(&desc);
            // std::format doesn't support wchar_t in all MSVC versions; convert via WideCharToMultiByte
            int len = WideCharToMultiByte(CP_UTF8, 0, desc.Description, -1, nullptr, 0, nullptr, nullptr);
            std::string name(len, '\0');
            WideCharToMultiByte(CP_UTF8, 0, desc.Description, -1, name.data(), len, nullptr, nullptr);
            LogInfo(std::format("GPU: {}", name));
        }

        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void GraphicsDevice::SelectAdapter()
{
    // Prefer the highest-performance (discrete) GPU.
    for (UINT i = 0;
         m_factory->EnumAdapterByGpuPreference(i, DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS(&m_adapter)) != DXGI_ERROR_NOT_FOUND;
         ++i)
    {
        DXGI_ADAPTER_DESC1 desc;
        m_adapter->GetDesc1(&desc);

        if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE)
            continue; // skip the Basic Render Driver (software fallback)

        // Check DX12 support without actually creating a device yet.
        if (SUCCEEDED(D3D12CreateDevice(m_adapter.Get(), D3D_FEATURE_LEVEL_11_0, _uuidof(ID3D12Device), nullptr)))
            return;
    }

    throw std::runtime_error("No DX12-capable GPU found.");
}

void GraphicsDevice::DetectCapabilities()
{
    // Bindless needs SM 6.6 (the ResourceDescriptorHeap[] intrinsic) and
    // resource-binding tier 3 (unbounded, fully dynamic descriptor indexing).
    // CheckFeatureSupport caps HighestShaderModel to what the driver actually
    // supports; an older driver may not know 6.6 and returns E_INVALIDARG.
    D3D12_FEATURE_DATA_SHADER_MODEL sm = { D3D_SHADER_MODEL_6_6 };
    const bool sm66 =
        SUCCEEDED(m_device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &sm, sizeof(sm)))
        && sm.HighestShaderModel >= D3D_SHADER_MODEL_6_6;

    D3D12_FEATURE_DATA_D3D12_OPTIONS opt = {};
    const bool tier3 =
        SUCCEEDED(m_device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS, &opt, sizeof(opt)))
        && opt.ResourceBindingTier >= D3D12_RESOURCE_BINDING_TIER_3;

    m_supportsBindless = sm66 && tier3;
    LogInfo(m_supportsBindless ? "Bindless supported (SM 6.6 + resource-binding tier 3)."
                               : "Bindless NOT supported on this GPU/driver.");

    // DXR: raytracing tier 1.0+ (acceleration structures, DispatchRays).
    D3D12_FEATURE_DATA_D3D12_OPTIONS5 opt5 = {};
    m_supportsRaytracing =
        SUCCEEDED(m_device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS5, &opt5, sizeof(opt5)))
        && opt5.RaytracingTier >= D3D12_RAYTRACING_TIER_1_0;
    LogInfo(m_supportsRaytracing ? "Raytracing (DXR) supported."
                                 : "Raytracing (DXR) NOT supported on this GPU/driver.");

    // Mesh shaders: shader model 6.5 (the amplification/mesh-shader stages) plus
    // mesh-shader tier 1. Requesting 6.5 lets the runtime clamp HighestShaderModel
    // to the real maximum, so this is true on any GPU that actually supports them.
    D3D12_FEATURE_DATA_SHADER_MODEL sm65 = { D3D_SHADER_MODEL_6_5 };
    const bool haveSm65 =
        SUCCEEDED(m_device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &sm65, sizeof(sm65)))
        && sm65.HighestShaderModel >= D3D_SHADER_MODEL_6_5;
    D3D12_FEATURE_DATA_D3D12_OPTIONS7 opt7 = {};
    const bool meshTier =
        SUCCEEDED(m_device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &opt7, sizeof(opt7)))
        && opt7.MeshShaderTier >= D3D12_MESH_SHADER_TIER_1;
    m_supportsMeshShaders = haveSm65 && meshTier;
    LogInfo(m_supportsMeshShaders ? "Mesh shaders supported (SM 6.5 + mesh-shader tier 1)."
                                  : "Mesh shaders NOT supported on this GPU/driver.");
}

void GraphicsDevice::Shutdown()
{
    m_commandQueue.Reset();
    m_device.Reset();
    m_adapter.Reset();
    m_factory.Reset();
#ifdef SGE_DEBUG
    s_debugController.Reset();
#endif
}

} // namespace SGE
