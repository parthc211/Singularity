#include "Renderer/DX12/MeshPipeline.h"
#include "Core/Logger.h"

#include <cstddef>

namespace SGE {

namespace {

// One subobject in a pipeline-state stream: a type tag followed by its payload,
// each entry aligned to a pointer boundary (the stream is parsed tag-by-tag).
template <typename T, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE Type>
struct alignas(void*) StreamSubobject
{
    D3D12_PIPELINE_STATE_SUBOBJECT_TYPE type = Type;
    T                                   inner = {};
};

// AS is placed LAST so an amplification-less pipeline can be expressed simply by
// trimming the stream's SizeInBytes to exclude it (subobjects may appear in any
// order, and the parser stops at SizeInBytes).
struct MeshPsoStream
{
    StreamSubobject<ID3D12RootSignature*,     D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_ROOT_SIGNATURE>        rootSig;
    StreamSubobject<D3D12_SHADER_BYTECODE,    D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_MS>                    ms;
    StreamSubobject<D3D12_SHADER_BYTECODE,    D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_PS>                    ps;
    StreamSubobject<D3D12_RASTERIZER_DESC,    D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_RASTERIZER>            raster;
    StreamSubobject<D3D12_DEPTH_STENCIL_DESC, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DEPTH_STENCIL>         depth;
    StreamSubobject<D3D12_BLEND_DESC,         D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_BLEND>                 blend;
    StreamSubobject<DXGI_FORMAT,              D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DEPTH_STENCIL_FORMAT>  dsvFormat;
    StreamSubobject<D3D12_RT_FORMAT_ARRAY,    D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_RENDER_TARGET_FORMATS> rtvFormats;
    StreamSubobject<DXGI_SAMPLE_DESC,         D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_SAMPLE_DESC>           sampleDesc;
    StreamSubobject<UINT,                     D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_SAMPLE_MASK>           sampleMask;
    StreamSubobject<D3D12_SHADER_BYTECODE,    D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_AS>                    as;  // LAST (optional)
};

} // namespace

bool MeshPipeline::Create(ID3D12Device* device, const MeshPipelineDesc& desc)
{
    if (!desc.ms || !desc.ps || !desc.rootSignature) {
        LogError("MeshPipeline::Create: mesh shader, pixel shader and root signature are required.");
        return false;
    }

    // Mesh-shader PSOs need the stream-based CreatePipelineState (ID3D12Device2).
    ComPtr<ID3D12Device2> device2;
    if (FAILED(device->QueryInterface(IID_PPV_ARGS(&device2)))) {
        LogError("MeshPipeline::Create: ID3D12Device2 unavailable (no mesh-shader support).");
        return false;
    }

    MeshPsoStream stream = {};
    stream.rootSig.inner = desc.rootSignature;
    stream.ms.inner      = desc.ms->GetBytecode();
    stream.ps.inner      = desc.ps->GetBytecode();

    D3D12_RASTERIZER_DESC& r = stream.raster.inner;
    r.FillMode              = D3D12_FILL_MODE_SOLID;
    r.CullMode              = desc.cullMode;
    r.FrontCounterClockwise = FALSE;   // engine convention: CW-from-outside is front
    r.DepthClipEnable       = TRUE;

    D3D12_DEPTH_STENCIL_DESC& d = stream.depth.inner;
    d.DepthEnable    = desc.depthEnable ? TRUE : FALSE;
    d.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
    d.DepthFunc      = D3D12_COMPARISON_FUNC_LESS;

    D3D12_BLEND_DESC& b = stream.blend.inner;
    b.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

    stream.dsvFormat.inner = desc.dsvFormat;

    D3D12_RT_FORMAT_ARRAY& rt = stream.rtvFormats.inner;
    rt.NumRenderTargets = 1;
    rt.RTFormats[0]     = desc.rtvFormat;

    stream.sampleDesc.inner.Count = 1;
    stream.sampleMask.inner       = UINT_MAX;

    // Amplification shader is optional: include it only when present by covering it
    // in the stream's byte length (it is the last subobject).
    if (desc.as) stream.as.inner = desc.as->GetBytecode();
    const SIZE_T size = desc.as ? sizeof(MeshPsoStream) : offsetof(MeshPsoStream, as);

    D3D12_PIPELINE_STATE_STREAM_DESC streamDesc = {};
    streamDesc.SizeInBytes                   = size;
    streamDesc.pPipelineStateSubobjectStream = &stream;

    if (FAILED(device2->CreatePipelineState(&streamDesc, IID_PPV_ARGS(&m_pso)))) {
        LogError("MeshPipeline::Create: CreatePipelineState failed.");
        return false;
    }
    return true;
}

} // namespace SGE
