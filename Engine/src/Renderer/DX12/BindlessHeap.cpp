#include "Renderer/DX12/BindlessHeap.h"
#include "Core/Logger.h"

namespace SGE {

bool BindlessHeap::Create(ID3D12Device* device, uint32_t capacity)
{
    try {
        Reset();
        D3D12_DESCRIPTOR_HEAP_DESC desc = {};
        desc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        desc.NumDescriptors = capacity;
        desc.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        SGE_THROW_IF_FAILED(device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&m_heap)));

        m_increment = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
        m_capacity  = capacity;
        m_next      = 0;
        m_freeList.clear();
        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void BindlessHeap::Reset()
{
    m_heap.Reset();
    m_increment = 0;
    m_capacity  = 0;
    m_next      = 0;
    m_freeList.clear();
}

uint32_t BindlessHeap::Allocate()
{
    if (!m_freeList.empty()) {
        const uint32_t i = m_freeList.back();
        m_freeList.pop_back();
        return i;
    }
    if (m_next < m_capacity)
        return m_next++;
    LogError("BindlessHeap: out of descriptors");
    return kInvalid;
}

void BindlessHeap::Free(uint32_t index)
{
    if (index < m_capacity)
        m_freeList.push_back(index);
}

D3D12_CPU_DESCRIPTOR_HANDLE BindlessHeap::Cpu(uint32_t index) const
{
    D3D12_CPU_DESCRIPTOR_HANDLE h = m_heap->GetCPUDescriptorHandleForHeapStart();
    h.ptr += SIZE_T(index) * m_increment;
    return h;
}

void BindlessHeap::Bind(ID3D12GraphicsCommandList* cmd) const
{
    ID3D12DescriptorHeap* heaps[] = { m_heap.Get() };
    cmd->SetDescriptorHeaps(1, heaps);
}

} // namespace SGE
