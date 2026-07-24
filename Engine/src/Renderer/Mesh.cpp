#include "Renderer/Mesh.h"

namespace SGE {

bool Mesh::Upload(ID3D12Device*                  device,
                  GpuHeap&                       heap,
                  const std::vector<MeshVertex>& vertices,
                  const std::vector<uint32_t>&   indices)
{
    if (vertices.empty() || indices.empty()) return false;

    if (!m_vertexBuffer.Upload(device, heap,
                               vertices.data(),
                               uint32_t(vertices.size() * sizeof(MeshVertex)),
                               sizeof(MeshVertex)))
        return false;

    if (!m_indexBuffer.Upload(device, heap,
                              indices.data(),
                              uint32_t(indices.size())))
        return false;

    m_vertexCount = uint32_t(vertices.size());
    m_indexCount  = uint32_t(indices.size());
    return true;
}

void Mesh::Draw(ID3D12GraphicsCommandList* cmd) const
{
    auto vbv = m_vertexBuffer.GetView();
    auto ibv = m_indexBuffer.GetView();
    cmd->IASetVertexBuffers(0, 1, &vbv);
    cmd->IASetIndexBuffer(&ibv);
    cmd->DrawIndexedInstanced(m_indexCount, 1, 0, 0, 0);
}

void Mesh::Reset()
{
    m_vertexBuffer.Reset();
    m_indexBuffer.Reset();
    m_indexCount  = 0;
    m_vertexCount = 0;
}

} // namespace SGE
