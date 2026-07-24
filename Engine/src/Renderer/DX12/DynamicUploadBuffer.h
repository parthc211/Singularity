#pragma once
#include "DX12Common.h" // ComPtr, ID3D12*, DXGI, SGE_THROW_IF_FAILED, FrameCount
#include <cstdint>

namespace SGE {

// A linear (arena) allocator over one persistently-mapped UPLOAD-heap buffer,
// used for constant data that changes every frame (per-object MVPs, etc.).
//
// The buffer is split into FrameCount equal regions. Each frame we write only
// into the current frame index's region; the Renderer's fence guarantees the
// GPU has finished reading that region's previous contents before we reuse it,
// so we never overwrite data still in flight. (Same idea as your CPU arena
// allocator — just living in GPU-visible upload memory.)
//
// Allocate() hands back 256-byte-aligned blocks (the CBV placement-alignment
// requirement) and returns both a CPU pointer to fill and the GPU virtual
// address to bind with SetGraphicsRootConstantBufferView.
class DynamicUploadBuffer {
public:
    struct Allocation {
        void*                     Cpu = nullptr; // write your struct here
        D3D12_GPU_VIRTUAL_ADDRESS Gpu = 0;       // pass to SetGraphicsRootCBV
    };

    bool Init(ID3D12Device* device, std::size_t bytesPerFrame);
    void Shutdown();

    // Call once per frame, before recording draws, with the renderer's frame
    // index. Selects this frame's region and resets its bump cursor to 0.
    void BeginFrame(uint32_t frameIndex);

    // 256-aligned sub-allocation inside the current frame's region. If the
    // region is exhausted it asserts and returns { nullptr, 0 }.
    Allocation Allocate(std::size_t sizeBytes);

private:
    static constexpr std::size_t CBAlign = 256; // D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT

    Microsoft::WRL::ComPtr<ID3D12Resource> m_resource;
    uint8_t*                  m_mappedBase    = nullptr; // CPU base of whole buffer
    D3D12_GPU_VIRTUAL_ADDRESS m_gpuBase       = 0;       // GPU base of whole buffer
    std::size_t               m_bytesPerFrame = 0;       // size of one region
    std::size_t               m_frameOffset   = 0;       // base of current region
    std::size_t               m_cursor        = 0;       // bump cursor within region
};

} // namespace SGE
