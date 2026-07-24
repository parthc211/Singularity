#include "Renderer/DX12/RootSignature.h"
#include "Core/Logger.h"

namespace SGE {

bool RootSignature::Create(ID3D12Device* device, D3D12_ROOT_SIGNATURE_FLAGS flags)
{
    return Create(device, nullptr, 0, flags);
}

bool RootSignature::Create(ID3D12Device* device,
                           const D3D12_ROOT_PARAMETER* params, UINT numParams,
                           D3D12_ROOT_SIGNATURE_FLAGS flags)
{
    return Create(device, params, numParams, nullptr, 0, flags);
}

bool RootSignature::Create(ID3D12Device* device,
                           const D3D12_ROOT_PARAMETER* params, UINT numParams,
                           const D3D12_STATIC_SAMPLER_DESC* samplers, UINT numSamplers,
                           D3D12_ROOT_SIGNATURE_FLAGS flags)
{
    try {
        D3D12_ROOT_SIGNATURE_DESC desc = {};
        desc.NumParameters     = numParams;
        desc.pParameters       = params;
        desc.NumStaticSamplers = numSamplers;
        desc.pStaticSamplers   = samplers;
        desc.Flags             = flags;

        ComPtr<ID3DBlob> serialized;
        ComPtr<ID3DBlob> errors;
        HRESULT hr = D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1_0, &serialized, &errors);
        if (errors)
            LogWarn(static_cast<const char*>(errors->GetBufferPointer()));
        SGE_THROW_IF_FAILED(hr);

        SGE_THROW_IF_FAILED(device->CreateRootSignature(
            0,
            serialized->GetBufferPointer(),
            serialized->GetBufferSize(),
            IID_PPV_ARGS(&m_rootSig)));

        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

} // namespace SGE
