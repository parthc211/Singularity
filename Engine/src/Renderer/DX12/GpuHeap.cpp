#include "Renderer/DX12/GpuHeap.h"
#include "Core/Logger.h"

namespace SGE {

// Buffers must be placed on 64 KB boundaries, so the whole allocator works in
// 64 KB units.
static constexpr uint64_t kBlockAlign = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT; // 64 KB

static uint64_t AlignUp(uint64_t v, uint64_t a) {
    return (v + (a - 1)) & ~(a - 1);
}

// Minimal buffer resource desc (matches the pattern used elsewhere in the engine).
static D3D12_RESOURCE_DESC BufferDesc(uint64_t width) {
    D3D12_RESOURCE_DESC d = {};
    d.Dimension        = D3D12_RESOURCE_DIMENSION_BUFFER;
    d.Width            = width;
    d.Height           = 1;
    d.DepthOrArraySize = 1;
    d.MipLevels        = 1;
    d.Format           = DXGI_FORMAT_UNKNOWN;
    d.SampleDesc.Count = 1;
    d.Layout           = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    d.Flags            = D3D12_RESOURCE_FLAG_NONE;
    return d;
}

bool GpuHeap::Init(ID3D12Device* device, uint64_t sizeBytes, D3D12_HEAP_TYPE heapType) {
    try {
        m_size = AlignUp(sizeBytes, kBlockAlign);

        D3D12_HEAP_DESC desc = {};
        desc.SizeInBytes     = m_size;
        desc.Properties.Type = heapType;
        desc.Alignment       = 0; // 0 => 64 KB default heap alignment
        // Buffer-only heap: portable across Resource Heap Tier 1/2 (Tier 1 forbids
        // mixing buffers and textures in one heap).
        desc.Flags           = D3D12_HEAP_FLAG_ALLOW_ONLY_BUFFERS;

        SGE_THROW_IF_FAILED(device->CreateHeap(&desc, IID_PPV_ARGS(&m_heap)));

        // Start with one big free block covering the whole heap.
        m_blocks.clear();
        m_blocks.push_back({ 0, m_size, true });
        m_used       = 0;
        m_allocCount = 0;

        LogInfo("GpuHeap created.");
        return true;
    } catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void GpuHeap::Shutdown() {
    m_blocks.clear();
    m_heap.Reset();
    m_size = m_used = 0;
    m_allocCount = 0;
}

GpuAllocation GpuHeap::AllocateBuffer(ID3D12Device* device, uint64_t sizeBytes,
                                      D3D12_RESOURCE_STATES initialState) {
    if (!m_heap || sizeBytes == 0)
        return {};

    // Ask the runtime for the real heap footprint (for buffers this rounds the
    // width up to 64 KB). Using the reported size keeps our bookkeeping exactly
    // in step with what CreatePlacedResource will consume.
    D3D12_RESOURCE_DESC desc = BufferDesc(sizeBytes);
    const D3D12_RESOURCE_ALLOCATION_INFO info = device->GetResourceAllocationInfo(0, 1, &desc);
    const uint64_t need = info.SizeInBytes; // already 64 KB-aligned

    // First-fit: the first free block large enough wins.
    for (std::size_t i = 0; i < m_blocks.size(); ++i) {
        Block& b = m_blocks[i];
        if (!b.Free || b.Size < need)
            continue;

        const uint64_t offset = b.Offset;

        Microsoft::WRL::ComPtr<ID3D12Resource> res;
        const HRESULT hr = device->CreatePlacedResource(
            m_heap.Get(), offset, &desc, initialState, nullptr, IID_PPV_ARGS(&res));
        if (FAILED(hr)) {
            LogError("GpuHeap::AllocateBuffer: CreatePlacedResource failed");
            return {};
        }

        // Split the free block: [offset, need) becomes used; any remainder stays
        // free as a new block right after it.
        if (b.Size > need) {
            const Block remainder{ offset + need, b.Size - need, true };
            b.Offset = offset;
            b.Size   = need;
            b.Free   = false;
            m_blocks.insert(m_blocks.begin() + i + 1, remainder);
        } else {
            b.Free = false;
        }

        m_used += need;
        ++m_allocCount;

        GpuAllocation a;
        a.Resource = std::move(res);
        a.Offset   = offset;
        a.Size     = need;
        return a;
    }

    // Nothing fit — out of memory or too fragmented (a free-list's weakness, and
    // exactly what the visualizer is meant to make visible).
    return {};
}

void GpuHeap::Free(GpuAllocation& alloc) {
    if (!alloc.IsValid())
        return;

    for (std::size_t i = 0; i < m_blocks.size(); ++i) {
        Block& b = m_blocks[i];
        if (b.Free || b.Offset != alloc.Offset)
            continue;

        b.Free = true;
        m_used -= b.Size;
        --m_allocCount;

        // Coalesce with the following block if it's also free...
        if (i + 1 < m_blocks.size() && m_blocks[i + 1].Free) {
            m_blocks[i].Size += m_blocks[i + 1].Size;
            m_blocks.erase(m_blocks.begin() + i + 1);
        }
        // ...and with the preceding block (merges into i-1, dropping i).
        if (i > 0 && m_blocks[i - 1].Free) {
            m_blocks[i - 1].Size += m_blocks[i].Size;
            m_blocks.erase(m_blocks.begin() + i);
        }
        break;
    }

    // Releasing the placed resource frees the GPU object; the heap memory it sat
    // in is already back in the free list above.
    alloc.Resource.Reset();
    alloc.Offset = 0;
    alloc.Size   = 0;
}

uint64_t GpuHeap::LargestFreeBlock() const {
    uint64_t largest = 0;
    for (const Block& b : m_blocks)
        if (b.Free && b.Size > largest)
            largest = b.Size;
    return largest;
}

} // namespace SGE
