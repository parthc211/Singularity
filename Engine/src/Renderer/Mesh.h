#pragma once

#include "Renderer/DX12/VertexBuffer.h"
#include "Renderer/DX12/IndexBuffer.h"
#include "Renderer/DX12/GpuHeap.h"   // Mesh::Upload takes a GpuHeap&

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

// Owns a VertexBuffer + IndexBuffer pair uploaded once to the GPU.
class Mesh
{
public:
    bool Upload(ID3D12Device*                   device,
                GpuHeap&                        heap,
                const std::vector<MeshVertex>&  vertices,
                const std::vector<uint32_t>&    indices);

    // Binds VB + IB and issues DrawIndexedInstanced.
    void Draw(ID3D12GraphicsCommandList* cmd) const;

    void Reset();

    bool     IsValid()     const { return m_indexCount  > 0; }
    uint32_t IndexCount()  const { return m_indexCount;      }
    uint32_t VertexCount() const { return m_vertexCount;     }

private:
    VertexBuffer m_vertexBuffer;
    IndexBuffer  m_indexBuffer;
    uint32_t     m_indexCount  = 0;
    uint32_t     m_vertexCount = 0;
};

} // namespace SGE
