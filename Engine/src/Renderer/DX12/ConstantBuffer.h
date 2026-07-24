#pragma once

#include "Renderer/DX12/DX12Common.h"

namespace SGE {

// Per-frame upload-heap constant buffer.
// Allocates FrameCount 256-byte-aligned slots in one contiguous buffer and keeps
// the mapping open permanently so the CPU can write each frame without a Map/Unmap cycle.
class ConstantBuffer
{
public:
    bool Create(ID3D12Device* device, uint32_t dataSizeBytes);
    void Reset();

    void                      Update(uint32_t frameIndex, const void* data, uint32_t sizeBytes);
    D3D12_GPU_VIRTUAL_ADDRESS GetGPUAddress(uint32_t frameIndex) const;

private:
    ComPtr<ID3D12Resource> m_buffer;
    uint8_t*               m_mapped      = nullptr;
    uint32_t               m_alignedSize = 0;
};

} // namespace SGE
