#include "Renderer/Mesh.h"
#include "Core/Logger.h"

#include <cstring>
#include <stdexcept>

namespace SGE {

namespace {

// Minimal buffer resource desc.
D3D12_RESOURCE_DESC BufferDesc(uint64_t bytes)
{
    D3D12_RESOURCE_DESC d = {};
    d.Dimension        = D3D12_RESOURCE_DIMENSION_BUFFER;
    d.Width            = bytes;
    d.Height           = 1;
    d.DepthOrArraySize = 1;
    d.MipLevels        = 1;
    d.Format           = DXGI_FORMAT_UNKNOWN;
    d.SampleDesc.Count = 1;
    d.Layout           = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    d.Flags            = D3D12_RESOURCE_FLAG_NONE;
    return d;
}

ComPtr<ID3D12Resource> CreateBuffer(ID3D12Device* device, uint64_t bytes,
                                    D3D12_HEAP_TYPE heapType, D3D12_RESOURCE_STATES state)
{
    D3D12_HEAP_PROPERTIES hp = {};
    hp.Type = heapType;
    const D3D12_RESOURCE_DESC desc = BufferDesc(bytes);
    ComPtr<ID3D12Resource> res;
    SGE_THROW_IF_FAILED(device->CreateCommittedResource(
        &hp, D3D12_HEAP_FLAG_NONE, &desc, state, nullptr, IID_PPV_ARGS(&res)));
    return res;
}

} // namespace

bool Mesh::Upload(ID3D12Device*                  device,
                  ID3D12CommandQueue*            queue,
                  const std::vector<MeshVertex>& vertices,
                  const std::vector<uint32_t>&   indices)
{
    if (vertices.empty() || indices.empty() || !queue) return false;

    try {
        Reset();

        const uint64_t vbBytes = uint64_t(vertices.size()) * sizeof(MeshVertex);
        const uint64_t ibBytes = uint64_t(indices.size())  * sizeof(uint32_t);

        // DEFAULT-heap destinations (VRAM). Created in COMMON; the copy promotes
        // them to COPY_DEST implicitly, then we transition to GENERIC_READ.
        m_vb = CreateBuffer(device, vbBytes, D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_COMMON);
        m_ib = CreateBuffer(device, ibBytes, D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_COMMON);

        // Temporary UPLOAD staging buffers (CPU-writable).
        ComPtr<ID3D12Resource> vbStaging =
            CreateBuffer(device, vbBytes, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ);
        ComPtr<ID3D12Resource> ibStaging =
            CreateBuffer(device, ibBytes, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ);

        auto fill = [](ID3D12Resource* staging, const void* src, uint64_t bytes) {
            void* mapped = nullptr;
            D3D12_RANGE noRead = { 0, 0 };
            SGE_THROW_IF_FAILED(staging->Map(0, &noRead, &mapped));
            std::memcpy(mapped, src, size_t(bytes));
            staging->Unmap(0, nullptr);
        };
        fill(vbStaging.Get(), vertices.data(), vbBytes);
        fill(ibStaging.Get(), indices.data(),  ibBytes);

        // One-shot command list: copy both, transition both to a read state.
        ComPtr<ID3D12CommandAllocator>    allocator;
        ComPtr<ID3D12GraphicsCommandList> cmd;
        SGE_THROW_IF_FAILED(device->CreateCommandAllocator(
            D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&allocator)));
        SGE_THROW_IF_FAILED(device->CreateCommandList(
            0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator.Get(), nullptr, IID_PPV_ARGS(&cmd)));

        cmd->CopyBufferRegion(m_vb.Get(), 0, vbStaging.Get(), 0, vbBytes);
        cmd->CopyBufferRegion(m_ib.Get(), 0, ibStaging.Get(), 0, ibBytes);

        // GENERIC_READ is a superset of VERTEX_AND_CONSTANT_BUFFER | INDEX_BUFFER |
        // NON_PIXEL_SHADER_RESOURCE | INDIRECT_ARGUMENT, so the finished mesh serves
        // raster IA binds, ExecuteIndirect, BLAS build input and shader reads with
        // no further transitions.
        D3D12_RESOURCE_BARRIER barriers[2] = {};
        for (int i = 0; i < 2; ++i) {
            barriers[i].Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            barriers[i].Transition.pResource   = (i == 0) ? m_vb.Get() : m_ib.Get();
            barriers[i].Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
            barriers[i].Transition.StateAfter  = D3D12_RESOURCE_STATE_GENERIC_READ;
            barriers[i].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        }
        cmd->ResourceBarrier(2, barriers);
        SGE_THROW_IF_FAILED(cmd->Close());

        ID3D12CommandList* lists[] = { cmd.Get() };
        queue->ExecuteCommandLists(1, lists);

        // Block until the copy completes so the staging buffers stay alive through it.
        ComPtr<ID3D12Fence> fence;
        SGE_THROW_IF_FAILED(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));
        HANDLE event = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (!event) throw std::runtime_error("Mesh::Upload: CreateEvent failed");
        SGE_THROW_IF_FAILED(queue->Signal(fence.Get(), 1));
        SGE_THROW_IF_FAILED(fence->SetEventOnCompletion(1, event));
        WaitForSingleObject(event, INFINITE);
        CloseHandle(event);

        m_vertexCount = uint32_t(vertices.size());
        m_indexCount  = uint32_t(indices.size());

        m_vbv.BufferLocation = m_vb->GetGPUVirtualAddress();
        m_vbv.SizeInBytes    = UINT(vbBytes);
        m_vbv.StrideInBytes  = sizeof(MeshVertex);

        m_ibv.BufferLocation = m_ib->GetGPUVirtualAddress();
        m_ibv.SizeInBytes    = UINT(ibBytes);
        m_ibv.Format         = DXGI_FORMAT_R32_UINT;

        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        Reset();
        return false;
    }
}

void Mesh::Draw(ID3D12GraphicsCommandList* cmd) const
{
    cmd->IASetVertexBuffers(0, 1, &m_vbv);
    cmd->IASetIndexBuffer(&m_ibv);
    cmd->DrawIndexedInstanced(m_indexCount, 1, 0, 0, 0);
}

void Mesh::Reset()
{
    m_vb.Reset();
    m_ib.Reset();
    m_vbv         = {};
    m_ibv         = {};
    m_indexCount  = 0;
    m_vertexCount = 0;
}

} // namespace SGE
