#pragma once
// ---------------------------------------------------------------------------
// ComputePipeline — the compute counterpart of GraphicsPipeline: one compute
// shader + a root signature. Bind with SetComputeRootSignature +
// SetPipelineState, feed root parameters via the SetComputeRoot* calls, then
// Dispatch.
// ---------------------------------------------------------------------------
#include "Renderer/DX12/DX12Common.h"
#include "Renderer/DXC/ShaderBlob.h"

#include <memory>

namespace SGE {

class ComputePipeline
{
public:
    bool Create(ID3D12Device* device, ID3D12RootSignature* rootSig,
                std::shared_ptr<ShaderBlob> cs);
    void Reset() { m_pso.Reset(); }

    ID3D12PipelineState* Get() const { return m_pso.Get(); }

private:
    ComPtr<ID3D12PipelineState> m_pso;
};

} // namespace SGE
