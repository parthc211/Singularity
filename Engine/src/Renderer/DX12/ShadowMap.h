#pragma once
#include "Renderer/DX12/DX12Common.h"
#include <cstdint>

namespace SGE {

// ---------------------------------------------------------------------------
// ShadowMap — a square depth texture used both as a depth target (the light
// renders scene depth into it) and as a shader resource (the main pass samples
// it to test occlusion).
//
// The key trick is the *typeless* resource: a single R32_TYPELESS texture gets
// two typed views — a D32_FLOAT depth-stencil view to render into, and an
// R32_FLOAT shader-resource view to sample. A depth resource can't be written
// and read at once, so it transitions DEPTH_WRITE <-> PIXEL_SHADER_RESOURCE
// each frame (this class emits the barriers).
// ---------------------------------------------------------------------------
class ShadowMap {
public:
    bool Create(ID3D12Device* device, uint32_t size);
    void Reset();

    // Shadow pass: transition to DEPTH_WRITE, clear, bind as the (only, depth-only)
    // target, and set the viewport/scissor to the shadow-map resolution.
    void BeginShadowPass(ID3D12GraphicsCommandList* cmd);
    // After drawing occluders: transition to PIXEL_SHADER_RESOURCE for sampling.
    void EndShadowPass(ID3D12GraphicsCommandList* cmd);

    ID3D12DescriptorHeap*       SrvHeap() const { return m_srvHeap.Get(); }
    D3D12_GPU_DESCRIPTOR_HANDLE Srv()     const { return m_srvHeap->GetGPUDescriptorHandleForHeapStart(); }
    uint32_t                    Size()    const { return m_size; }

private:
    Microsoft::WRL::ComPtr<ID3D12Resource>       m_texture;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_dsvHeap; // 1 DSV (CPU)
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_srvHeap; // 1 SRV (shader-visible)
    D3D12_RESOURCE_STATES m_state = D3D12_RESOURCE_STATE_COMMON;
    uint32_t m_size = 0;
};

} // namespace SGE
