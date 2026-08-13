#pragma once

#include "Assets/GltfLoader.h"          // SkinnedVertex
#include "Renderer/DX12/VertexBuffer.h"
#include "Renderer/DX12/IndexBuffer.h"
#include "Renderer/DX12/GpuHeap.h"

#include <cstdint>
#include <vector>

namespace SGE {

// GPU-resident skinned mesh: a Mesh with the fatter SkinnedVertex layout.
// Vertices stay in bind pose; all deformation happens in the vertex shader
// via the bone palette.
class SkinnedMesh
{
public:
    bool Upload(ID3D12Device*                     device,
                GpuHeap&                          heap,
                const std::vector<SkinnedVertex>& vertices,
                const std::vector<uint32_t>&      indices);

    void Draw(ID3D12GraphicsCommandList* cmd) const;
    // One draw for a whole crowd — the VS distinguishes instances via
    // SV_InstanceID (e.g. indexing a GPU-computed palette buffer).
    void DrawInstanced(ID3D12GraphicsCommandList* cmd, uint32_t instances) const;

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
