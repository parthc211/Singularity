#pragma once

#include "Renderer/DX12/DX12Common.h"
#include "Renderer/DX12/GraphicsDevice.h"
#include "Renderer/DX12/SwapChain.h"
#include "Renderer/DX12/CommandContext.h"
#include "Renderer/DX12/DepthBuffer.h"
#include "Renderer/DX12/GpuHeap.h"

#include <cstdint>

namespace SGE {

class Renderer
{
public:
    bool Initialize(HWND hwnd, uint32_t width, uint32_t height);
    void Shutdown();

    void BeginFrame();
    void EndFrame();

    // Execute externally-recorded command lists as part of THIS frame, sequenced
    // correctly on the single GPU queue: the shared frame list so far (BeginFrame's
    // clear + render-target setup) is flushed FIRST, then the caller's lists, in
    // array order. The shared list is then re-opened so the frame can keep
    // recording (e.g. the ImGui pass) before EndFrame. Used by the threaded
    // command-list demo, which records draws across worker threads.
    void ExecuteFrameLists(ID3D12CommandList* const* lists, uint32_t count);

    // Blocks until all in-flight GPU work is complete. Call before releasing resources.
    void WaitForGPU();

    void OnResize(uint32_t width, uint32_t height);

    ID3D12Device*              GetDevice()      const;
    ID3D12GraphicsCommandList* GetCommandList() const;
    uint32_t                   GetFrameIndex()  const { return m_frameIndex; }

    // Shared UPLOAD-type heap that static mesh VB/IB sub-allocate placed resources
    // from (Phase 5). Owned here so it outlives every Mesh and is torn down after
    // WaitForGPU in Shutdown.
    GpuHeap& GetGeometryHeap() { return m_geometryHeap; }

    uint32_t GetWidth()  const { return m_width;  }
    uint32_t GetHeight() const { return m_height; }
    // Back buffer + depth handles, for passes that re-bind targets mid-frame
    // (e.g. deferred rendering: geometry pass -> G-buffer, lighting pass -> back buffer).
    D3D12_CPU_DESCRIPTOR_HANDLE GetBackBufferRTV() const { return m_swapChain.GetCurrentRTV(); }
    D3D12_CPU_DESCRIPTOR_HANDLE GetDepthDSV()      const { return m_depthBuffer.GetDSV(); }

private:
    GraphicsDevice  m_graphicsDevice;
    SwapChain       m_swapChain;
    CommandContext  m_commandContext;
    DepthBuffer     m_depthBuffer;
    GpuHeap         m_geometryHeap;

    ComPtr<ID3D12Fence> m_fence;
    HANDLE              m_fenceEvent    = nullptr;
    uint64_t            m_fenceCounter  = 0;
    uint64_t            m_fenceValues[FrameCount] = {};

    uint32_t m_frameIndex = 0;
    uint32_t m_width      = 0;
    uint32_t m_height     = 0;
};

} // namespace SGE
