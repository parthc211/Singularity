#pragma once
#include "Renderer/DX12/DX12Common.h"
#include <cstdint>

namespace SGE {

// A single off-screen colour render target: one texture + its RTV, plus a helper
// to write an SRV into a caller-owned shader-visible heap (so several render
// textures can share one heap and be sampled together as a descriptor table —
// what the bloom composite pass needs). Tracks its own state for the
// RENDER_TARGET <-> PIXEL_SHADER_RESOURCE ping-pong of post-processing.
class RenderTexture {
public:
    bool Create(ID3D12Device* device, uint32_t width, uint32_t height,
                DXGI_FORMAT format, const float clearColor[4]);
    void Reset();

    // Create this texture's SRV at the given (shader-visible heap) CPU handle.
    void CreateSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const;

    D3D12_CPU_DESCRIPTOR_HANDLE Rtv() const { return m_rtvHeap->GetCPUDescriptorHandleForHeapStart(); }

    void TransitionTo(ID3D12GraphicsCommandList* cmd, D3D12_RESOURCE_STATES state);
    void ClearRtv(ID3D12GraphicsCommandList* cmd); // must be in RENDER_TARGET state

    ID3D12Resource* Resource() const { return m_tex.Get(); }
    uint32_t Width()  const { return m_width; }
    uint32_t Height() const { return m_height; }
    bool     IsValid() const { return m_tex != nullptr; }

private:
    Microsoft::WRL::ComPtr<ID3D12Resource>       m_tex;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_rtvHeap; // 1 RTV (CPU)
    D3D12_RESOURCE_STATES m_state  = D3D12_RESOURCE_STATE_COMMON;
    DXGI_FORMAT           m_format = DXGI_FORMAT_UNKNOWN;
    uint32_t              m_width  = 0;
    uint32_t              m_height = 0;
    float                 m_clear[4] = { 0, 0, 0, 0 };
};

} // namespace SGE
