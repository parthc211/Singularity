#include "Renderer/DX12/SwapChain.h"
#include "Core/Logger.h"

namespace SGE {

bool SwapChain::Initialize(IDXGIFactory6* factory, ID3D12CommandQueue* queue,
                            ID3D12Device* device, HWND hwnd,
                            uint32_t width, uint32_t height)
{
    try {
        DXGI_SWAP_CHAIN_DESC1 desc = {};
        desc.Width       = width;
        desc.Height      = height;
        desc.Format      = DXGI_FORMAT_R8G8B8A8_UNORM;
        desc.Stereo      = FALSE;
        desc.SampleDesc  = { 1, 0 };                        // no MSAA on the swap chain in DX12
        desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        desc.BufferCount = FrameCount;
        desc.Scaling     = DXGI_SCALING_STRETCH;
        desc.SwapEffect  = DXGI_SWAP_EFFECT_FLIP_DISCARD;  // required flip model for DX12
        desc.AlphaMode   = DXGI_ALPHA_MODE_UNSPECIFIED;
        desc.Flags       = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;

        ComPtr<IDXGISwapChain1> sc1;
        SGE_THROW_IF_FAILED(factory->CreateSwapChainForHwnd(queue, hwnd, &desc, nullptr, nullptr, &sc1));

        // Disable Alt+Enter built-in fullscreen toggle — we'll handle it ourselves later.
        factory->MakeWindowAssociation(hwnd, DXGI_MWA_NO_ALT_ENTER);

        SGE_THROW_IF_FAILED(sc1.As(&m_swapChain));

        CreateRTVs(device);

        LogInfo("Swap chain created.");
        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void SwapChain::CreateRTVs(ID3D12Device* device)
{
    D3D12_DESCRIPTOR_HEAP_DESC heapDesc = {};
    heapDesc.NumDescriptors = FrameCount;
    heapDesc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    heapDesc.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_NONE; // RTVs don't need to be shader-visible
    SGE_THROW_IF_FAILED(device->CreateDescriptorHeap(&heapDesc, IID_PPV_ARGS(&m_rtvHeap)));

    m_rtvDescriptorSize = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

    D3D12_CPU_DESCRIPTOR_HANDLE handle = m_rtvHeap->GetCPUDescriptorHandleForHeapStart();
    for (uint32_t i = 0; i < FrameCount; ++i) {
        SGE_THROW_IF_FAILED(m_swapChain->GetBuffer(i, IID_PPV_ARGS(&m_renderTargets[i])));
        device->CreateRenderTargetView(m_renderTargets[i].Get(), nullptr, handle);
        handle.ptr += m_rtvDescriptorSize;
    }
}

void SwapChain::Resize(ID3D12Device* device, uint32_t width, uint32_t height)
{
    for (auto& rt : m_renderTargets)
        rt.Reset();

    SGE_THROW_IF_FAILED(m_swapChain->ResizeBuffers(
        FrameCount, width, height,
        DXGI_FORMAT_R8G8B8A8_UNORM,
        DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH
    ));

    CreateRTVs(device);
}

void SwapChain::Present()
{
    // SyncInterval=1 enables vsync. Use 0 to run uncapped (no vsync).
    SGE_THROW_IF_FAILED(m_swapChain->Present(1, 0));
}

uint32_t SwapChain::GetCurrentBackBufferIndex() const
{
    return m_swapChain->GetCurrentBackBufferIndex();
}

ID3D12Resource* SwapChain::GetCurrentBackBuffer() const
{
    return m_renderTargets[m_swapChain->GetCurrentBackBufferIndex()].Get();
}

D3D12_CPU_DESCRIPTOR_HANDLE SwapChain::GetCurrentRTV() const
{
    D3D12_CPU_DESCRIPTOR_HANDLE handle = m_rtvHeap->GetCPUDescriptorHandleForHeapStart();
    handle.ptr += m_swapChain->GetCurrentBackBufferIndex() * m_rtvDescriptorSize;
    return handle;
}

void SwapChain::Shutdown()
{
    for (auto& rt : m_renderTargets)
        rt.Reset();
    m_rtvHeap.Reset();
    m_swapChain.Reset();
}

} // namespace SGE
