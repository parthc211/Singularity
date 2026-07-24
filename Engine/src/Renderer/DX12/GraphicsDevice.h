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

private:
    void SelectAdapter();

    ComPtr<IDXGIFactory6>      m_factory;
    ComPtr<IDXGIAdapter1>      m_adapter;
    ComPtr<ID3D12Device>       m_device;
    ComPtr<ID3D12CommandQueue> m_commandQueue;
};

} // namespace SGE
