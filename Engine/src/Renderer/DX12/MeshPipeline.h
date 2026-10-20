#pragma once

#include "Renderer/DX12/DX12Common.h"
#include "Renderer/DXC/ShaderBlob.h"

#include <memory>

namespace SGE {

// Pipeline for the mesh-shader geometry front end (amplification -> mesh -> pixel).
// Unlike a classic VS/PS PSO, a mesh-shader PSO is created from a *stream* of
// subobjects via ID3D12Device2::CreatePipelineState — there is no input assembler
// (the mesh shader fetches and emits geometry itself), so no input layout or
// primitive-topology type is set.
struct MeshPipelineDesc
{
    ID3D12RootSignature*        rootSignature = nullptr;
    std::shared_ptr<ShaderBlob> as;   // optional amplification shader (per-meshlet cull)
    std::shared_ptr<ShaderBlob> ms;   // mesh shader (required)
    std::shared_ptr<ShaderBlob> ps;   // pixel shader (required)
    DXGI_FORMAT     rtvFormat   = DXGI_FORMAT_R8G8B8A8_UNORM;
    DXGI_FORMAT     dsvFormat   = DXGI_FORMAT_D32_FLOAT;
    bool            depthEnable = true;
    D3D12_CULL_MODE cullMode    = D3D12_CULL_MODE_BACK;
};

class MeshPipeline
{
public:
    bool Create(ID3D12Device* device, const MeshPipelineDesc& desc);
    void Reset() { m_pso.Reset(); }

    ID3D12PipelineState* Get() const { return m_pso.Get(); }

private:
    ComPtr<ID3D12PipelineState> m_pso;
};

} // namespace SGE
