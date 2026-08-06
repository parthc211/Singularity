#include "Renderer/DX12/SrvHeap.h"
#include "Core/Logger.h"

namespace SGE {

bool SrvHeap::Create(ID3D12Device* device, uint32_t capacity)
{
    Reset();
    D3D12_DESCRIPTOR_HEAP_DESC desc = {};
    desc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    desc.NumDescriptors = capacity;
    desc.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    if (FAILED(device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&m_heap)))) {
        LogError("SrvHeap: CreateDescriptorHeap failed");
        return false;
    }
    m_increment = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
    m_capacity  = capacity;
    return true;
}

void SrvHeap::Reset()
{
    m_heap.Reset();
    m_increment = 0;
    m_capacity  = 0;
}

void SrvHeap::BindTable(ID3D12GraphicsCommandList* cmd, UINT rootParam,
                        uint32_t firstSlot) const
{
    ID3D12DescriptorHeap* heaps[] = { m_heap.Get() };
    cmd->SetDescriptorHeaps(1, heaps);
    cmd->SetGraphicsRootDescriptorTable(rootParam, Gpu(firstSlot));
}

D3D12_CPU_DESCRIPTOR_HANDLE SrvHeap::Cpu(uint32_t slot) const
{
    D3D12_CPU_DESCRIPTOR_HANDLE h = m_heap->GetCPUDescriptorHandleForHeapStart();
    h.ptr += size_t(slot) * m_increment;
    return h;
}

D3D12_GPU_DESCRIPTOR_HANDLE SrvHeap::Gpu(uint32_t slot) const
{
    D3D12_GPU_DESCRIPTOR_HANDLE h = m_heap->GetGPUDescriptorHandleForHeapStart();
    h.ptr += size_t(slot) * m_increment;
    return h;
}

} // namespace SGE
