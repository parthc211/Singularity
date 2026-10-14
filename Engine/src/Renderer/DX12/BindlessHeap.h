#pragma once
// ---------------------------------------------------------------------------
// BindlessHeap — a large shader-visible CBV/SRV/UAV descriptor heap addressed by
// integer index, the backing store for bindless rendering.
//
// Traditional D3D12 binds a descriptor TABLE per draw so a shader can reach its
// textures; the root signature names a fixed, small set of slots. Bindless
// inverts that: the whole heap is exposed to every shader (SM 6.6's
// ResourceDescriptorHeap[i]), and a resource is referenced by a plain integer —
// so a material is "texture #4718", data rather than binding state. The heap is
// bound ONCE; draws differ only by the indices they pass (a root constant/CBV),
// with zero per-draw descriptor-table churn. This is also the prerequisite for
// GPU-driven rendering (a compute pass picks indices) and ray tracing (a hit
// anywhere must reach any material).
//
// Indices are persistent: Allocate() hands one out (reused from a free list once
// Free'd), and the descriptor written at Cpu(index) stays valid until freed.
// ---------------------------------------------------------------------------
#include "Renderer/DX12/DX12Common.h"

#include <cstdint>
#include <vector>

namespace SGE {

class BindlessHeap {
public:
    static constexpr uint32_t kInvalid = UINT32_MAX;

    bool Create(ID3D12Device* device, uint32_t capacity);
    void Reset();

    // Reserve an index (from the free list, else the next unused). kInvalid if full.
    uint32_t Allocate();
    void     Free(uint32_t index);

    // CPU handle of a slot — create the resource's SRV/UAV/CBV here (e.g. via a
    // resource's CreateSrvInto). The matching integer is the index itself.
    D3D12_CPU_DESCRIPTOR_HANDLE Cpu(uint32_t index) const;

    // Make this the active heap so ResourceDescriptorHeap[] resolves against it.
    void Bind(ID3D12GraphicsCommandList* cmd) const;

    ID3D12DescriptorHeap* Heap()     const { return m_heap.Get(); }
    uint32_t              Capacity() const { return m_capacity; }
    bool                  IsValid()  const { return m_heap != nullptr; }

private:
    ComPtr<ID3D12DescriptorHeap> m_heap;
    uint32_t                     m_increment = 0;
    uint32_t                     m_capacity  = 0;
    uint32_t                     m_next      = 0;   // high-water mark of never-used slots
    std::vector<uint32_t>        m_freeList;         // recycled indices
};

} // namespace SGE
