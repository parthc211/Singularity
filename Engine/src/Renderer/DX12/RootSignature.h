#pragma once

#include "Renderer/DX12/DX12Common.h"

namespace SGE {

class RootSignature
{
public:
    // Empty root signature (no parameters).
    bool Create(ID3D12Device* device,
                D3D12_ROOT_SIGNATURE_FLAGS flags =
                    D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

    // Root signature with explicit parameters (root CBVs, descriptor tables, etc.).
    bool Create(ID3D12Device* device,
                const D3D12_ROOT_PARAMETER* params, UINT numParams,
                D3D12_ROOT_SIGNATURE_FLAGS flags =
                    D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

    // As above, plus static (immutable) samplers — used by the deferred lighting
    // pass, which samples the G-buffer SRVs through a fixed point sampler.
    bool Create(ID3D12Device* device,
                const D3D12_ROOT_PARAMETER* params, UINT numParams,
                const D3D12_STATIC_SAMPLER_DESC* samplers, UINT numSamplers,
                D3D12_ROOT_SIGNATURE_FLAGS flags =
                    D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

    void Reset() { m_rootSig.Reset(); }

    ID3D12RootSignature* Get() const { return m_rootSig.Get(); }

private:
    ComPtr<ID3D12RootSignature> m_rootSig;
};

} // namespace SGE
