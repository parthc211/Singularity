#pragma once
// ---------------------------------------------------------------------------
// GpuBuffer — a DEFAULT-heap buffer for compute/structured-buffer work, bound
// through ROOT descriptors (SetGraphicsRoot/SetComputeRoot SRV/UAV take the
// GPU virtual address directly — no descriptor heap involved).
//
// Two flavors:
//   Upload(...)  — static data (bake results, tables): staging-copy like
//                  Texture2D, blocking, ends in NON_PIXEL_SHADER_RESOURCE.
//   Create(uav)  — GPU-written scratch (e.g. the pose palette a compute pass
//                  fills and the vertex shader reads); use TransitionTo to
//                  ping-pong between UNORDERED_ACCESS and shader-read states,
//                  and UavBarrier between dependent dispatches.
// ---------------------------------------------------------------------------
#include "Renderer/DX12/DX12Common.h"

#include <cstdint>

namespace SGE {

class GpuBuffer
{
public:
    // Creates the buffer (allowUav adds the UAV flag). Initial state COMMON.
    bool Create(ID3D12Device* device, uint64_t sizeBytes, bool allowUav);

    // Create + fill from CPU data via a blocking one-shot staging copy
    // (same self-contained pattern as Texture2D::Create); ends in
    // NON_PIXEL_SHADER_RESOURCE, ready for VS/CS reads.
    bool Upload(ID3D12Device* device, ID3D12CommandQueue* queue,
                const void* data, uint64_t sizeBytes);

    void Reset();

    void TransitionTo(ID3D12GraphicsCommandList* cmd, D3D12_RESOURCE_STATES state);
    void UavBarrier(ID3D12GraphicsCommandList* cmd) const;

    D3D12_GPU_VIRTUAL_ADDRESS Gpu() const { return m_buf ? m_buf->GetGPUVirtualAddress() : 0; }
    ID3D12Resource*           Resource() const { return m_buf.Get(); }
    uint64_t                  Size() const { return m_size; }
    bool                      IsValid() const { return m_buf != nullptr; }

private:
    ComPtr<ID3D12Resource> m_buf;
    D3D12_RESOURCE_STATES  m_state = D3D12_RESOURCE_STATE_COMMON;
    uint64_t               m_size  = 0;
};

} // namespace SGE
