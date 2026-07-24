#include "DynamicUploadBuffer.h"
#include "../../Core/Logger.h" // LogError — adjust if your logger's API differs
#include <cassert>

namespace SGE {

static std::size_t AlignUp(std::size_t v, std::size_t a) {
    return (v + (a - 1)) & ~(a - 1);
}

bool DynamicUploadBuffer::Init(ID3D12Device* device, std::size_t bytesPerFrame) {
    try {
        m_bytesPerFrame = AlignUp(bytesPerFrame, CBAlign);
        const std::size_t totalSize = m_bytesPerFrame * FrameCount;

        D3D12_HEAP_PROPERTIES heapProps = {};
        heapProps.Type = D3D12_HEAP_TYPE_UPLOAD;

        D3D12_RESOURCE_DESC desc = {};
        desc.Dimension        = D3D12_RESOURCE_DIMENSION_BUFFER;
        desc.Width            = totalSize;
        desc.Height           = 1;
        desc.DepthOrArraySize = 1;
        desc.MipLevels        = 1;
        desc.Format           = DXGI_FORMAT_UNKNOWN;
        desc.SampleDesc.Count = 1;
        desc.Layout           = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
        desc.Flags            = D3D12_RESOURCE_FLAG_NONE;

        // Upload-heap resources must be created in (and stay in) GENERIC_READ.
        SGE_THROW_IF_FAILED(device->CreateCommittedResource(
            &heapProps, D3D12_HEAP_FLAG_NONE, &desc,
            D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
            IID_PPV_ARGS(&m_resource)));

        // Persistently map: upload memory is CPU-write / GPU-read, so we map
        // once and never unmap (until Shutdown). The {0,0} read range tells the
        // runtime we won't read back on the CPU.
        D3D12_RANGE noRead = { 0, 0 };
        void* mapped = nullptr;
        SGE_THROW_IF_FAILED(m_resource->Map(0, &noRead, &mapped));
        m_mappedBase = static_cast<uint8_t*>(mapped);
        m_gpuBase    = m_resource->GetGPUVirtualAddress();
        return true;
    } catch (...) {
        LogError("DynamicUploadBuffer::Init failed");
        return false;
    }
}

void DynamicUploadBuffer::Shutdown() {
    if (m_resource) {
        m_resource->Unmap(0, nullptr);
        m_resource.Reset();
    }
    m_mappedBase    = nullptr;
    m_gpuBase       = 0;
    m_bytesPerFrame = 0;
    m_frameOffset   = 0;
    m_cursor        = 0;
}

void DynamicUploadBuffer::BeginFrame(uint32_t frameIndex) {
    m_frameOffset = static_cast<std::size_t>(frameIndex) * m_bytesPerFrame;
    m_cursor      = 0;
}

DynamicUploadBuffer::Allocation DynamicUploadBuffer::Allocate(std::size_t sizeBytes) {
    const std::size_t aligned = AlignUp(sizeBytes, CBAlign);
    Allocation out;
    if (m_cursor + aligned > m_bytesPerFrame) {
        assert(false && "DynamicUploadBuffer region exhausted — raise bytesPerFrame");
        return out; // Cpu == nullptr signals failure
    }
    const std::size_t offset = m_frameOffset + m_cursor;
    out.Cpu = m_mappedBase + offset;
    out.Gpu = m_gpuBase + offset;
    m_cursor += aligned;
    return out;
}

} // namespace SGE
