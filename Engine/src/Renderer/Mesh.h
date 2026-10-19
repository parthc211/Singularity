#pragma once

#include "Renderer/DX12/DX12Common.h"

#include <vector>
#include <cstdint>

namespace SGE {

// Field order matters: input layouts are reflected from each VS with
// D3D12_APPEND_ALIGNED_ELEMENT, so a shader's inputs must be a prefix of this
// struct in this order (POSITION, NORMAL, TEXCOORD, TANGENT). Appending new
// fields at the END is always safe — old shaders read their prefix and the
// grown stride skips the rest.
struct MeshVertex
{
    float position[3];
    float normal[3];
    float texCoord[2];
    float tangent[4];   // xyz = unit tangent (+U direction), w = handedness (±1):
                        // bitangent = cross(normal, tangent) * w
};

// A geometry resource: one VB + IB pair uploaded once to GPU-local (DEFAULT-heap)
// memory and consumable by *any* render path, not just a built-in draw.
//
// Why this shape (the point of the refactor): a mesh that only exposes Draw() can
// serve forward/deferred rasterization and nothing else. GPU-driven rendering must
// bind the VB/IB itself and feed ExecuteIndirect; DXR must hand the vertex/index
// GPU addresses to an acceleration-structure build and read the vertices back in a
// hit shader. Both need the raw buffers, their addresses, views, counts, stride
// and formats — so this class exposes all of them. One asset (e.g. cube.obj),
// every path.
//
// Residency: the buffers live in a DEFAULT heap (VRAM) and are filled by a blocking
// staged copy through a temporary UPLOAD buffer — the standard pattern for static
// geometry a game reads every frame (and every ray). They end in GENERIC_READ,
// which is a superset of VERTEX_AND_CONSTANT_BUFFER | INDEX_BUFFER |
// NON_PIXEL_SHADER_RESOURCE | INDIRECT_ARGUMENT, so one mesh serves raster VB/IB,
// ExecuteIndirect, BLAS build input and StructuredBuffer<MeshVertex> reads with no
// per-frame barriers. (Dynamically-managed buffers still use the GpuHeap free-list
// allocator; a static mesh wants a stable VRAM home instead.)
class Mesh
{
public:
    // Staged upload into DEFAULT-heap VB + IB. Blocking (one-time at load): records
    // one copy command list, executes on `queue`, and waits on a fence.
    bool Upload(ID3D12Device*                   device,
                ID3D12CommandQueue*             queue,
                const std::vector<MeshVertex>&  vertices,
                const std::vector<uint32_t>&    indices);

    // Binds VB + IB and issues DrawIndexedInstanced.
    void Draw(ID3D12GraphicsCommandList* cmd) const;

    void Reset();

    bool     IsValid()     const { return m_indexCount  > 0; }
    uint32_t IndexCount()  const { return m_indexCount;      }
    uint32_t VertexCount() const { return m_vertexCount;     }

    // --- geometry-resource accessors: let any render path consume this mesh ---
    // Views for IA bind (raster + ExecuteIndirect).
    const D3D12_VERTEX_BUFFER_VIEW& VertexView() const { return m_vbv; }
    const D3D12_INDEX_BUFFER_VIEW&  IndexView()  const { return m_ibv; }
    // Raw GPU addresses for BLAS build input and root/structured-buffer binds.
    D3D12_GPU_VIRTUAL_ADDRESS VertexAddress() const { return m_vb ? m_vb->GetGPUVirtualAddress() : 0; }
    D3D12_GPU_VIRTUAL_ADDRESS IndexAddress()  const { return m_ib ? m_ib->GetGPUVirtualAddress() : 0; }
    // Underlying resources for barriers / descriptor (SRV) creation.
    ID3D12Resource* VertexResource() const { return m_vb.Get(); }
    ID3D12Resource* IndexResource()  const { return m_ib.Get(); }
    // Layout the above buffers follow.
    uint32_t    VertexStride() const { return uint32_t(sizeof(MeshVertex)); }
    DXGI_FORMAT IndexFormat()  const { return DXGI_FORMAT_R32_UINT; }

private:
    ComPtr<ID3D12Resource>   m_vb;          // DEFAULT-heap vertex buffer (VRAM)
    ComPtr<ID3D12Resource>   m_ib;          // DEFAULT-heap index buffer  (VRAM)
    D3D12_VERTEX_BUFFER_VIEW m_vbv = {};
    D3D12_INDEX_BUFFER_VIEW  m_ibv = {};
    uint32_t                 m_indexCount  = 0;
    uint32_t                 m_vertexCount = 0;
};

} // namespace SGE
