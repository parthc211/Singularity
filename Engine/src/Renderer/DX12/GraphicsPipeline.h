#pragma once

#include "Renderer/DX12/DX12Common.h"
#include "Renderer/DXC/ShaderBlob.h"

#include <memory>

namespace SGE {

struct GraphicsPipelineDesc
{
    ID3D12RootSignature*        rootSignature = nullptr;
    std::shared_ptr<ShaderBlob> vs;
    std::shared_ptr<ShaderBlob> ps;
    std::shared_ptr<ShaderBlob> hs;  // optional — hull shader (tessellation)
    std::shared_ptr<ShaderBlob> ds;  // optional — domain shader (tessellation)
    DXGI_FORMAT                 rtvFormat   = DXGI_FORMAT_R8G8B8A8_UNORM; // used when rtvCount <= 1
    DXGI_FORMAT                 dsvFormat   = DXGI_FORMAT_D32_FLOAT;
    bool                        depthEnable = true;
    // Multiple render targets (deferred G-buffer). When rtvCount > 1, rtvFormats[0..n-1]
    // are used and rtvFormat is ignored; otherwise rtvFormat drives a single RT.
    uint32_t                    rtvCount       = 1;
    DXGI_FORMAT                 rtvFormats[8]  = {};
    // Topology class — set PATCH for tessellation. Rasterizer knobs:
    D3D12_PRIMITIVE_TOPOLOGY_TYPE topologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    D3D12_CULL_MODE               cullMode     = D3D12_CULL_MODE_BACK;
    bool                          wireframe    = false;
    // Input layout is derived automatically from vs->inputLayout (via reflection);
    // a VS with no vertex inputs (e.g. a fullscreen-triangle pass) yields an empty one.
};

class GraphicsPipeline
{
public:
    bool Create(ID3D12Device* device, const GraphicsPipelineDesc& desc);

    // Swap in new shader blobs and rebuild the PSO. Call after WaitForGPU().
    void UpdateShaders(std::shared_ptr<ShaderBlob> vs, std::shared_ptr<ShaderBlob> ps);
    bool Rebuild(ID3D12Device* device);

    void Reset() { m_pso.Reset(); }

    ID3D12PipelineState* Get() const { return m_pso.Get(); }

private:
    bool BuildPSO(ID3D12Device* device);

    GraphicsPipelineDesc        m_desc;
    ComPtr<ID3D12PipelineState> m_pso;
};

} // namespace SGE
