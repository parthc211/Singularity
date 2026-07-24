#pragma once

#include "Renderer/DX12/VertexBuffer.h"
#include "Renderer/DX12/IndexBuffer.h"
#include "Renderer/DX12/GpuHeap.h"   // Mesh::Upload takes a GpuHeap&

#include <vector>
#include <cstdint>

namespace SGE {

struct MeshVertex
{
    float position[3];
    float normal[3];
    float texCoord[2];
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
