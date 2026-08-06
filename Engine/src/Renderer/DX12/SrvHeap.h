#pragma once
// ---------------------------------------------------------------------------
// SrvHeap — a small shader-visible CBV/SRV/UAV heap with slot-indexed writes,
// part of the renderer abstraction layer. Scenes were each hand-rolling the
// same ritual: create a heap, compute increment-sized offsets, CreateSrvInto
// each resource, SetDescriptorHeaps + SetGraphicsRootDescriptorTable. Now:
//
//   m_srvs.Create(device, 2);
//   m_srvs.Write(device, 0, m_albedo);     // anything with CreateSrvInto()
//   m_srvs.Write(device, 1, m_normal);
//   ...
//   m_srvs.BindTable(cmd, /*rootParam*/ 2);   // heap + table in one call
//
// Contiguous slots form the table, matching RootSignatureBuilder::SrvTable.
// ---------------------------------------------------------------------------
#include "Renderer/DX12/DX12Common.h"

#include <cstdint>

namespace SGE {

class SrvHeap {
public:
    bool Create(ID3D12Device* device, uint32_t capacity);
    void Reset();

    // Writes src's SRV into a slot. Works with any type exposing
    // CreateSrvInto(ID3D12Device*, D3D12_CPU_DESCRIPTOR_HANDLE) —
    // Texture2D, RenderTexture, ShadowMap, GBuffer attachments, ...
    template <typename T>
    void Write(ID3D12Device* device, uint32_t slot, const T& src) const {
        src.CreateSrvInto(device, Cpu(slot));
    }

    // SetDescriptorHeaps + SetGraphicsRootDescriptorTable(firstSlot) in one go.
    void BindTable(ID3D12GraphicsCommandList* cmd, UINT rootParam,
                   uint32_t firstSlot = 0) const;

    D3D12_CPU_DESCRIPTOR_HANDLE Cpu(uint32_t slot) const;
    D3D12_GPU_DESCRIPTOR_HANDLE Gpu(uint32_t slot) const;
    ID3D12DescriptorHeap*       Heap() const { return m_heap.Get(); }
    bool                        IsValid() const { return m_heap != nullptr; }

private:
    ComPtr<ID3D12DescriptorHeap> m_heap;
    uint32_t m_increment = 0;
    uint32_t m_capacity  = 0;
};

} // namespace SGE
