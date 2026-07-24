#pragma once

#include "Renderer/DX12/DX12Common.h"

namespace SGE {

class SwapChain
{
public:
    bool Initialize(IDXGIFactory6* factory, ID3D12CommandQueue* queue,
                    ID3D12Device* device, HWND hwnd,
                    uint32_t width, uint32_t height);
    void Shutdown();

    void Present();
    void Resize(ID3D12Device* device, uint32_t width, uint32_t height);

    uint32_t                    GetCurrentBackBufferIndex() const;
    ID3D12Resource*             GetCurrentBackBuffer()     const;
    D3D12_CPU_DESCRIPTOR_HANDLE GetCurrentRTV()            const;

private:
    void CreateRTVs(ID3D12Device* device);

    ComPtr<IDXGISwapChain3>       m_swapChain;
    ComPtr<ID3D12DescriptorHeap>  m_rtvHeap;
    ComPtr<ID3D12Resource>        m_renderTargets[FrameCount];
    uint32_t                      m_rtvDescriptorSize = 0;
};

} // namespace SGE
