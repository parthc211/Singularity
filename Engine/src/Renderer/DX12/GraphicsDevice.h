#pragma once

#include "Renderer/DX12/DX12Common.h"

namespace SGE {

class GraphicsDevice
{
public:
    bool Initialize(bool enableDebugLayer = true);
    void Shutdown();

    ID3D12Device*        GetDevice()       const { return m_device.Get();       }
    ID3D12CommandQueue*  GetCommandQueue() const { return m_commandQueue.Get(); }
    IDXGIFactory6*       GetFactory()      const { return m_factory.Get();      }

    // True when the GPU supports Shader Model 6.6 + resource-binding tier 3, i.e.
    // ResourceDescriptorHeap[] bindless access. Detected once at Initialize.
    bool                 SupportsBindless() const { return m_supportsBindless;   }

    // True when the GPU supports DXR (raytracing tier 1.0+). Detected at Initialize.
    bool                 SupportsRaytracing() const { return m_supportsRaytracing; }

private:
    void SelectAdapter();
    void DetectCapabilities();

    ComPtr<IDXGIFactory6>      m_factory;
    ComPtr<IDXGIAdapter1>      m_adapter;
    ComPtr<ID3D12Device>       m_device;
    ComPtr<ID3D12CommandQueue> m_commandQueue;
    bool                       m_supportsBindless   = false;
    bool                       m_supportsRaytracing = false;
};

} // namespace SGE
