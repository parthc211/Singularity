#include "Renderer/DX12/CommandContext.h"
#include "Core/Logger.h"

namespace SGE {

bool CommandContext::Initialize(ID3D12Device* device)
{
    try {
        for (uint32_t i = 0; i < FrameCount; ++i) {
            SGE_THROW_IF_FAILED(device->CreateCommandAllocator(
                D3D12_COMMAND_LIST_TYPE_DIRECT,
                IID_PPV_ARGS(&m_commandAllocators[i])
            ));
        }

        // Command list starts in the closed state; Reset() opens it before each frame.
        SGE_THROW_IF_FAILED(device->CreateCommandList(
            0,
            D3D12_COMMAND_LIST_TYPE_DIRECT,
            m_commandAllocators[0].Get(),
            nullptr,                           // no initial pipeline state
            IID_PPV_ARGS(&m_commandList)
        ));
        m_commandList->Close();

        LogInfo("Command context created.");
        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void CommandContext::Reset(uint32_t frameIndex)
{
    // Safe to reset once the GPU has finished this allocator's commands (caller must ensure this).
    SGE_THROW_IF_FAILED(m_commandAllocators[frameIndex]->Reset());
    SGE_THROW_IF_FAILED(m_commandList->Reset(m_commandAllocators[frameIndex].Get(), nullptr));
}

void CommandContext::Close()
{
    SGE_THROW_IF_FAILED(m_commandList->Close());
}

void CommandContext::ReopenList(uint32_t frameIndex)
{
    // Reset the LIST only — NOT the allocator. The allocator keeps the commands
    // from the segment we just executed; we append the next segment into the same
    // allocator (which is only reset next time this frame index comes around, after
    // its fence). This is why it's safe while the GPU is mid-execute.
    SGE_THROW_IF_FAILED(m_commandList->Reset(m_commandAllocators[frameIndex].Get(), nullptr));
}

void CommandContext::ResourceBarrier(ID3D12Resource* resource,
                                     D3D12_RESOURCE_STATES before,
                                     D3D12_RESOURCE_STATES after)
{
    D3D12_RESOURCE_BARRIER barrier = {};
    barrier.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Flags                  = D3D12_RESOURCE_BARRIER_FLAG_NONE;
    barrier.Transition.pResource   = resource;
    barrier.Transition.StateBefore = before;
    barrier.Transition.StateAfter  = after;
    barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    m_commandList->ResourceBarrier(1, &barrier);
}

void CommandContext::ClearRenderTarget(D3D12_CPU_DESCRIPTOR_HANDLE rtv, const float color[4])
{
    m_commandList->ClearRenderTargetView(rtv, color, 0, nullptr);
}

void CommandContext::Shutdown()
{
    m_commandList.Reset();
    for (auto& alloc : m_commandAllocators)
        alloc.Reset();
}

} // namespace SGE
