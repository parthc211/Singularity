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
