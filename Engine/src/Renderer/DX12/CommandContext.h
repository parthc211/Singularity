#pragma once

#include "Renderer/DX12/DX12Common.h"

namespace SGE {

// Wraps per-frame command allocators and a single reusable command list.
// Each frame index has its own allocator so we don't reset one the GPU is still reading.
class CommandContext
{
public:
    bool Initialize(ID3D12Device* device);
    void Shutdown();

    void Reset(uint32_t frameIndex);
    void Close();

    // Re-open the list for more recording WITHOUT resetting the allocator. Legal
    // even while the GPU is still executing a prior submission of this same list
    // (unlike an allocator reset, a command-list reset only restarts CPU-side
    // recording). Used to continue a frame after mid-frame ExecuteCommandLists.
    void ReopenList(uint32_t frameIndex);

    void ResourceBarrier(ID3D12Resource* resource,
                         D3D12_RESOURCE_STATES before,
                         D3D12_RESOURCE_STATES after);

    void ClearRenderTarget(D3D12_CPU_DESCRIPTOR_HANDLE rtv, const float color[4]);

    ID3D12GraphicsCommandList* GetCommandList() const { return m_commandList.Get(); }

private:
    ComPtr<ID3D12CommandAllocator>     m_commandAllocators[FrameCount];
    ComPtr<ID3D12GraphicsCommandList>  m_commandList;
};

} // namespace SGE
