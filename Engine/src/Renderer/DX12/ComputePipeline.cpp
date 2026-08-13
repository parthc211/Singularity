#include "Renderer/DX12/ComputePipeline.h"
#include "Core/Logger.h"

namespace SGE {

bool ComputePipeline::Create(ID3D12Device* device, ID3D12RootSignature* rootSig,
                             std::shared_ptr<ShaderBlob> cs)
{
    if (!cs) return false;
    D3D12_COMPUTE_PIPELINE_STATE_DESC desc = {};
    desc.pRootSignature = rootSig;
    desc.CS             = cs->GetBytecode();
    if (FAILED(device->CreateComputePipelineState(&desc, IID_PPV_ARGS(&m_pso)))) {
        LogError("ComputePipeline: CreateComputePipelineState failed");
        return false;
    }
    return true;
}

} // namespace SGE
