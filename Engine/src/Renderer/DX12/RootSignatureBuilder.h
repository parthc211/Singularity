#pragma once
// ---------------------------------------------------------------------------
// RootSignatureBuilder — the declarative half of the renderer abstraction
// layer. Every scene root signature in the codebase is some arrangement of
// root CBVs + one SRV descriptor table + static samplers; this builder turns
// the ~40 lines of D3D12_ROOT_PARAMETER/D3D12_STATIC_SAMPLER_DESC filling
// each scene repeated into:
//
//   RootSignatureBuilder()
//       .Cbv(0).Cbv(1)
//       .SrvTable(0, 2)
//       .SamplerAnisoWrap(0)
//       .Build(device, m_rootSig);
//
// Root parameter indices are assigned in call order (first Cbv -> param 0...),
// which is also the binding order scenes already use. This is deliberately a
// convenience over D3D12, not a portability layer — the escape hatches accept
// raw D3D12 descs.
// ---------------------------------------------------------------------------
#include "Renderer/DX12/DX12Common.h"
#include "Renderer/DX12/RootSignature.h"

#include <vector>

namespace SGE {

class RootSignatureBuilder {
public:
    // Root CBV at register b<reg>. Returns the builder for chaining; the
    // parameter's root index is its position in the call sequence.
    RootSignatureBuilder& Cbv(UINT reg,
                              D3D12_SHADER_VISIBILITY vis = D3D12_SHADER_VISIBILITY_ALL);

    // Root descriptor SRV/UAV at t<reg>/u<reg> — raw or structured BUFFERS
    // only (textures need a descriptor table). Bind with
    // SetGraphicsRoot/SetComputeRoot ShaderResourceView/UnorderedAccessView.
    RootSignatureBuilder& Srv(UINT reg,
                              D3D12_SHADER_VISIBILITY vis = D3D12_SHADER_VISIBILITY_ALL);
    RootSignatureBuilder& Uav(UINT reg,
                              D3D12_SHADER_VISIBILITY vis = D3D12_SHADER_VISIBILITY_ALL);

    // Descriptor table of `count` contiguous SRVs starting at t<baseReg>.
    RootSignatureBuilder& SrvTable(UINT baseReg, UINT count,
                                   D3D12_SHADER_VISIBILITY vis = D3D12_SHADER_VISIBILITY_PIXEL);

    // Static sampler presets covering every sampler the scenes use.
    RootSignatureBuilder& SamplerAnisoWrap(UINT reg);    // material textures
    RootSignatureBuilder& SamplerLinearClamp(UINT reg);  // post-process chains
    RootSignatureBuilder& SamplerPointClamp(UINT reg);   // G-buffer reads
    RootSignatureBuilder& SamplerShadowPcf(UINT reg);    // comparison, border=white
    RootSignatureBuilder& Sampler(const D3D12_STATIC_SAMPLER_DESC& desc); // escape hatch

    // Builds into `out`. The builder can be discarded afterwards.
    bool Build(ID3D12Device* device, RootSignature& out,
               D3D12_ROOT_SIGNATURE_FLAGS flags =
                   D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

private:
    struct TableRange { size_t paramIndex; D3D12_DESCRIPTOR_RANGE range; };

    std::vector<D3D12_ROOT_PARAMETER>      m_params;
    std::vector<TableRange>                m_tables;   // pointers patched in Build
    std::vector<D3D12_STATIC_SAMPLER_DESC> m_samplers;
};

} // namespace SGE
