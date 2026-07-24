#pragma once
#include "Renderer/DX12/DX12Common.h"
#include <cstdint>

namespace SGE {

// ---------------------------------------------------------------------------
// GBuffer — the deferred-rendering geometry buffer.
//
// Deferred rendering splits shading into two passes:
//   1. Geometry pass: draw all geometry ONCE, writing surface attributes into
//      several render targets at the same time (MRT) — here albedo, world-space
//      normal, world-space position.
//   2. Lighting pass: a single fullscreen draw reads those targets as textures
//      and accumulates every light. Cost becomes (pixels x lights) instead of
//      (objects x lights), and each pixel is shaded once regardless of overdraw.
//
// This class owns everything the two passes need to talk to each other:
//   * 3 screen-sized DEFAULT-heap textures created with ALLOW_RENDER_TARGET,
//   * a CPU (non-shader-visible) RTV heap so the geometry pass can render INTO
//     them via OMSetRenderTargets,
//   * a SHADER-VISIBLE SRV heap holding the 3 SRVs *contiguously*, so the
//     lighting pass can bind them as one descriptor table (t0,t1,t2).
//
// Render targets and shader-resource views are two different "views" of the same
// texture, and a resource can't be both at once — so each frame the textures
// ping-pong between RENDER_TARGET (being written) and PIXEL_SHADER_RESOURCE
// (being sampled). This class tracks the current state and emits the barriers.
// ---------------------------------------------------------------------------
class GBuffer {
public:
    static constexpr uint32_t kCount = 3;
    enum Target { Albedo = 0, Normal = 1, Position = 2 };

    // The render-target format of target i (for building the geometry-pass PSO).
    static DXGI_FORMAT Format(uint32_t i);

    // The underlying texture (so callers can create SRVs into their own heap,
    // e.g. SSAO sampling normal/position alongside its own targets).
    ID3D12Resource* Resource(uint32_t i) const { return m_textures[i].Get(); }

    bool Create(ID3D12Device* device, uint32_t width, uint32_t height);
    void Reset();

    uint32_t Width()  const { return m_width;  }
    uint32_t Height() const { return m_height; }
    bool     IsValid() const { return m_textures[0] != nullptr; }

    // CPU RTV handle for OMSetRenderTargets in the geometry pass.
    D3D12_CPU_DESCRIPTOR_HANDLE Rtv(uint32_t i) const;
    // GPU base of the SRV table (Albedo=t0, Normal=t1, Position=t2).
    D3D12_GPU_DESCRIPTOR_HANDLE SrvTable() const;
    ID3D12DescriptorHeap*       SrvHeap() const { return m_srvHeap.Get(); }

    // Barriers for all 3 targets between render-target and shader-resource.
    void TransitionToRenderTargets(ID3D12GraphicsCommandList* cmd);
    void TransitionToShaderResources(ID3D12GraphicsCommandList* cmd);

    // Clear all 3 targets to 0. Call after TransitionToRenderTargets + OMSet.
    void ClearRenderTargets(ID3D12GraphicsCommandList* cmd);

private:
    static const DXGI_FORMAT kFormats[kCount];

    void TransitionAll(ID3D12GraphicsCommandList* cmd, D3D12_RESOURCE_STATES to);

    ComPtr<ID3D12Resource>       m_textures[kCount];
    ComPtr<ID3D12DescriptorHeap> m_rtvHeap; // CPU, kCount RTVs
    ComPtr<ID3D12DescriptorHeap> m_srvHeap; // shader-visible, kCount SRVs (contiguous)
    uint32_t m_rtvStride = 0;
    D3D12_RESOURCE_STATES m_state = D3D12_RESOURCE_STATE_COMMON; // shared by all 3
    uint32_t m_width = 0;
    uint32_t m_height = 0;
};

} // namespace SGE
