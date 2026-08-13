#include "Renderer/DX12/GpuBuffer.h"
#include "Core/Logger.h"

#include <cstring>

namespace SGE {

bool GpuBuffer::Create(ID3D12Device* device, uint64_t sizeBytes, bool allowUav)
{
    try {
        Reset();
        D3D12_HEAP_PROPERTIES heap = {};
        heap.Type = D3D12_HEAP_TYPE_DEFAULT;

        D3D12_RESOURCE_DESC desc = {};
        desc.Dimension        = D3D12_RESOURCE_DIMENSION_BUFFER;
        desc.Width            = sizeBytes;
        desc.Height           = 1;
        desc.DepthOrArraySize = 1;
        desc.MipLevels        = 1;
        desc.SampleDesc.Count = 1;
        desc.Layout           = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
        desc.Flags            = allowUav ? D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS
                                         : D3D12_RESOURCE_FLAG_NONE;

        SGE_THROW_IF_FAILED(device->CreateCommittedResource(
            &heap, D3D12_HEAP_FLAG_NONE, &desc,
            D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&m_buf)));
        m_state = D3D12_RESOURCE_STATE_COMMON;
        m_size  = sizeBytes;
        return true;
    } catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

bool GpuBuffer::Upload(ID3D12Device* device, ID3D12CommandQueue* queue,
                       const void* data, uint64_t sizeBytes)
{
    try {
        if (!Create(device, sizeBytes, false)) return false;

        D3D12_HEAP_PROPERTIES up = {};
        up.Type = D3D12_HEAP_TYPE_UPLOAD;
        D3D12_RESOURCE_DESC desc = m_buf->GetDesc();
        desc.Flags = D3D12_RESOURCE_FLAG_NONE;

        ComPtr<ID3D12Resource> staging;
        SGE_THROW_IF_FAILED(device->CreateCommittedResource(
            &up, D3D12_HEAP_FLAG_NONE, &desc,
            D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&staging)));

        void* mapped = nullptr;
        D3D12_RANGE noRead = {};
        SGE_THROW_IF_FAILED(staging->Map(0, &noRead, &mapped));
        std::memcpy(mapped, data, size_t(sizeBytes));
        staging->Unmap(0, nullptr);

        ComPtr<ID3D12CommandAllocator>    allocator;
        ComPtr<ID3D12GraphicsCommandList> cmd;
        SGE_THROW_IF_FAILED(device->CreateCommandAllocator(
            D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&allocator)));
        SGE_THROW_IF_FAILED(device->CreateCommandList(
            0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator.Get(), nullptr, IID_PPV_ARGS(&cmd)));

        cmd->CopyBufferRegion(m_buf.Get(), 0, staging.Get(), 0, sizeBytes);
        D3D12_RESOURCE_BARRIER b = {};
        b.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource   = m_buf.Get();
        b.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
        b.Transition.StateAfter  = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        cmd->ResourceBarrier(1, &b);
        SGE_THROW_IF_FAILED(cmd->Close());

        // Buffers in DEFAULT heaps decay/promote: the copy promotes COMMON ->
        // COPY_DEST implicitly, so only the explicit after-state is tracked.
        ID3D12CommandList* lists[] = { cmd.Get() };
        queue->ExecuteCommandLists(1, lists);

        ComPtr<ID3D12Fence> fence;
        SGE_THROW_IF_FAILED(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));
        HANDLE event = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (!event) throw std::runtime_error("GpuBuffer::Upload: CreateEvent failed");
        SGE_THROW_IF_FAILED(queue->Signal(fence.Get(), 1));
        SGE_THROW_IF_FAILED(fence->SetEventOnCompletion(1, event));
        WaitForSingleObject(event, INFINITE);
        CloseHandle(event);

        m_state = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
        return true;
    } catch (const std::exception& e) {
        LogError(e.what());
        Reset();
        return false;
    }
}

void GpuBuffer::Reset()
{
    m_buf.Reset();
    m_state = D3D12_RESOURCE_STATE_COMMON;
    m_size  = 0;
}

void GpuBuffer::TransitionTo(ID3D12GraphicsCommandList* cmd, D3D12_RESOURCE_STATES state)
{
    if (m_state == state || !m_buf) return;
    D3D12_RESOURCE_BARRIER b = {};
    b.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    b.Transition.pResource   = m_buf.Get();
    b.Transition.StateBefore = m_state;
    b.Transition.StateAfter  = state;
    b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    cmd->ResourceBarrier(1, &b);
    m_state = state;
}

void GpuBuffer::UavBarrier(ID3D12GraphicsCommandList* cmd) const
{
    D3D12_RESOURCE_BARRIER b = {};
    b.Type          = D3D12_RESOURCE_BARRIER_TYPE_UAV;
    b.UAV.pResource = m_buf.Get();
    cmd->ResourceBarrier(1, &b);
}

} // namespace SGE
