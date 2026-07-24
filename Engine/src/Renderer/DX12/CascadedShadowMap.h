#pragma once
#include "Renderer/DX12/DX12Common.h"
#include <cstdint>

namespace SGE {

// A depth Texture2D *array* — one slice per shadow cascade. The light renders the
// scene depth into each slice (a per-slice DSV); the main pass samples the whole
// thing as one Texture2DArray SRV and picks a slice per pixel by view distance.
// Same typeless depth trick as ShadowMap (R32_TYPELESS -> D32 DSV + R32 SRV),
// extended to an array.
class CascadedShadowMap {
public:
    static constexpr uint32_t kMaxCascades = 4;

    bool Create(ID3D12Device* device, uint32_t size, uint32_t cascadeCount);
    void Reset();

    // Transition the whole array to DEPTH_WRITE before rendering the cascades.
    void BeginCascadePass(ID3D12GraphicsCommandList* cmd);
    // Bind cascade i's DSV, clear it, set the shadow viewport. Call per cascade.
    void BindCascade(ID3D12GraphicsCommandList* cmd, uint32_t i);
    // Transition to PIXEL_SHADER_RESOURCE for sampling in the main pass.
    void EndCascadePass(ID3D12GraphicsCommandList* cmd);

    ID3D12DescriptorHeap*       SrvHeap() const { return m_srvHeap.Get(); }
    D3D12_GPU_DESCRIPTOR_HANDLE Srv()     const { return m_srvHeap->GetGPUDescriptorHandleForHeapStart(); }
    uint32_t                    Size()    const { return m_size; }
    uint32_t                    CascadeCount() const { return m_count; }

private:
    Microsoft::WRL::ComPtr<ID3D12Resource>       m_texture; // Texture2DArray, R32_TYPELESS
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_dsvHeap; // N DSVs (CPU)
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_srvHeap; // 1 array SRV (shader-visible)
    uint32_t m_dsvStride = 0;
    uint32_t m_size  = 0;
    uint32_t m_count = 0;
    D3D12_RESOURCE_STATES m_state = D3D12_RESOURCE_STATE_COMMON;
};

} // namespace SGE
