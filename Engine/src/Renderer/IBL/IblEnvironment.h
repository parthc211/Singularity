#pragma once
// ---------------------------------------------------------------------------
// IblEnvironment — the four GPU products of image-based lighting, baked once
// from a procedural sky by compute shaders:
//
//   1. Environment cube (HDR, mipped)  — the source radiance (a procedural sky).
//   2. Irradiance cube                 — diffuse IBL: the cosine-convolved env.
//   3. Prefiltered env cube (mipped)   — specular IBL: env convolved per roughness
//                                        (mip 0 mirror ... last mip fully rough).
//   4. BRDF integration LUT (2D)       — the environment-independent split-sum
//                                        scale/bias, so specular = prefiltered*(F0*A+B).
//
// Bake() is deliberately self-contained and blocking (its own command
// allocator/list, executed on the given queue and fenced) — the same pattern as
// Texture2D::Create — so a scene can call it from OnLoad regardless of the frame
// list's state, and re-call it when the sky parameters change. It owns its
// compute pipelines/root signatures internally; the caller only supplies a
// ShaderLibrary. The SRVs are written into a caller-owned shader-visible heap so
// the scene can lay irradiance/prefiltered/LUT out as one descriptor table.
// ---------------------------------------------------------------------------
#include "Renderer/DX12/DX12Common.h"

#include <cstdint>

namespace SGE {

class ShaderLibrary;

struct IblBakeParams {
    float sunDir[3]    = { 0.35f, 0.55f, 0.45f }; // direction TO the sun
    float sunIntensity = 45.0f;                   // HDR peak around the sun
    float exposure     = 1.0f;                    // overall sky brightness
};

class IblEnvironment {
public:
    bool Bake(ID3D12Device* device, ID3D12CommandQueue* queue,
              ShaderLibrary& shaders, const IblBakeParams& params);
    void Reset();

    // Write each product's SRV at a CPU handle in a caller-owned shader-visible
    // CBV/SRV/UAV heap. Env/irradiance/prefiltered are TextureCube; LUT is 2D.
    void CreateEnvSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const;
    void CreateIrradianceSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const;
    void CreatePrefilteredSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const;
    void CreateBrdfLutSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const;

    // Top mip index of the prefiltered cube, for roughness*maxMip lookups.
    float PrefilteredMaxMip() const { return float(kPrefilterMips - 1); }
    bool  IsValid() const { return m_env != nullptr; }

private:
    static constexpr uint32_t   kEnvSize        = 256;
    static constexpr uint32_t   kEnvMips        = 9;   // 256,128,...,1 (log2(256)+1)
    static constexpr uint32_t   kIrradianceSize = 32;
    static constexpr uint32_t   kPrefilterSize  = 128;
    static constexpr uint32_t   kPrefilterMips  = 5;   // 128,64,32,16,8
    static constexpr uint32_t   kBrdfLutSize    = 256;
    static constexpr DXGI_FORMAT kHdrFormat     = DXGI_FORMAT_R16G16B16A16_FLOAT;
    static constexpr DXGI_FORMAT kLutFormat     = DXGI_FORMAT_R16G16_FLOAT;

    ComPtr<ID3D12Resource> m_env;         // environment cube (kEnvMips)
    ComPtr<ID3D12Resource> m_irradiance;  // irradiance cube (1 mip)
    ComPtr<ID3D12Resource> m_prefiltered; // prefiltered cube (kPrefilterMips)
    ComPtr<ID3D12Resource> m_brdfLut;     // 2D LUT
};

} // namespace SGE
