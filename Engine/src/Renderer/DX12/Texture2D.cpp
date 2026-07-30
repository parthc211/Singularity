#include "Renderer/DX12/Texture2D.h"
#include "Core/Logger.h"

#include <cstring>

namespace SGE {

bool Texture2D::Create(ID3D12Device* device, ID3D12CommandQueue* queue,
                       const std::vector<Image>& mips, DXGI_FORMAT format)
{
    try {
        Reset();
        if (mips.empty() || !mips[0].IsValid()) {
            LogError("Texture2D::Create: empty or invalid mip chain");
            return false;
        }

        m_width     = mips[0].width;
        m_height    = mips[0].height;
        m_mipLevels = uint32_t(mips.size());
        m_format    = format;

        // --- destination texture (DEFAULT heap, starts as a copy target) ---
        D3D12_HEAP_PROPERTIES defaultHeap = {};
        defaultHeap.Type = D3D12_HEAP_TYPE_DEFAULT;

        D3D12_RESOURCE_DESC texDesc = {};
        texDesc.Dimension        = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        texDesc.Width            = m_width;
        texDesc.Height           = m_height;
        texDesc.DepthOrArraySize = 1;
        texDesc.MipLevels        = UINT16(m_mipLevels);
        texDesc.Format           = format;
        texDesc.SampleDesc.Count = 1;

        SGE_THROW_IF_FAILED(device->CreateCommittedResource(
            &defaultHeap, D3D12_HEAP_FLAG_NONE, &texDesc,
            D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&m_tex)));

        // --- staging layout: one placed footprint per mip, rows padded to 256 B ---
        std::vector<D3D12_PLACED_SUBRESOURCE_FOOTPRINT> footprints(m_mipLevels);
        std::vector<UINT>   numRows(m_mipLevels);
        std::vector<UINT64> rowSizes(m_mipLevels);
        UINT64 totalBytes = 0;
        device->GetCopyableFootprints(&texDesc, 0, m_mipLevels, 0,
                                      footprints.data(), numRows.data(),
                                      rowSizes.data(), &totalBytes);

        D3D12_HEAP_PROPERTIES uploadHeap = {};
        uploadHeap.Type = D3D12_HEAP_TYPE_UPLOAD;

        D3D12_RESOURCE_DESC bufDesc = {};
        bufDesc.Dimension        = D3D12_RESOURCE_DIMENSION_BUFFER;
        bufDesc.Width            = totalBytes;
        bufDesc.Height           = 1;
        bufDesc.DepthOrArraySize = 1;
        bufDesc.MipLevels        = 1;
        bufDesc.SampleDesc.Count = 1;
        bufDesc.Layout           = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

        ComPtr<ID3D12Resource> staging;
        SGE_THROW_IF_FAILED(device->CreateCommittedResource(
            &uploadHeap, D3D12_HEAP_FLAG_NONE, &bufDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&staging)));

        // Copy each mip row by row: source rows are tight (width*4), destination
        // rows sit at the footprint's 256-byte-aligned RowPitch.
        uint8_t* mapped = nullptr;
        D3D12_RANGE noRead = {};
        SGE_THROW_IF_FAILED(staging->Map(0, &noRead, reinterpret_cast<void**>(&mapped)));
        for (uint32_t m = 0; m < m_mipLevels; ++m) {
            const Image& img = mips[m];
            const auto&  fp  = footprints[m];
            if (img.width != fp.Footprint.Width || img.height != fp.Footprint.Height) {
                staging->Unmap(0, nullptr);
                LogError("Texture2D::Create: mip dimensions do not match the chain");
                Reset();
                return false;
            }
            for (UINT row = 0; row < numRows[m]; ++row) {
                std::memcpy(mapped + fp.Offset + size_t(row) * fp.Footprint.RowPitch,
                            &img.pixels[size_t(row) * img.width * 4],
                            size_t(rowSizes[m]));
            }
        }
        staging->Unmap(0, nullptr);

        // --- one-shot copy: own allocator/list, execute, fence, wait ---
        ComPtr<ID3D12CommandAllocator> allocator;
        ComPtr<ID3D12GraphicsCommandList> cmd;
        SGE_THROW_IF_FAILED(device->CreateCommandAllocator(
            D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&allocator)));
        SGE_THROW_IF_FAILED(device->CreateCommandList(
            0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator.Get(), nullptr,
            IID_PPV_ARGS(&cmd)));

        for (uint32_t m = 0; m < m_mipLevels; ++m) {
            D3D12_TEXTURE_COPY_LOCATION dst = {};
            dst.pResource        = m_tex.Get();
            dst.Type             = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
            dst.SubresourceIndex = m;

            D3D12_TEXTURE_COPY_LOCATION src = {};
            src.pResource       = staging.Get();
            src.Type            = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
            src.PlacedFootprint = footprints[m];

            cmd->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
        }

        D3D12_RESOURCE_BARRIER barrier = {};
        barrier.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barrier.Transition.pResource   = m_tex.Get();
        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
        barrier.Transition.StateAfter  = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        cmd->ResourceBarrier(1, &barrier);

        SGE_THROW_IF_FAILED(cmd->Close());
        ID3D12CommandList* lists[] = { cmd.Get() };
        queue->ExecuteCommandLists(1, lists);

        ComPtr<ID3D12Fence> fence;
        SGE_THROW_IF_FAILED(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));
        HANDLE event = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (!event) throw std::runtime_error("Texture2D::Create: CreateEvent failed");
        SGE_THROW_IF_FAILED(queue->Signal(fence.Get(), 1));
        SGE_THROW_IF_FAILED(fence->SetEventOnCompletion(1, event));
        WaitForSingleObject(event, INFINITE);
        CloseHandle(event);

        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        Reset();
        return false;
    }
}

bool Texture2D::CreateFromFile(ID3D12Device* device, ID3D12CommandQueue* queue,
                               const wchar_t* path, bool srgb)
{
    Image base;
    if (!LoadImageRGBA8(path, base))
        return false;
    return CreateFromImage(device, queue, base, srgb);
}

bool Texture2D::CreateFromImage(ID3D12Device* device, ID3D12CommandQueue* queue,
                                const Image& base, bool srgb)
{
    std::vector<Image> mips;
    GenerateMipChain(base, srgb ? ColorSpace::SRGB : ColorSpace::Linear, mips);
    return Create(device, queue, mips,
                  srgb ? DXGI_FORMAT_R8G8B8A8_UNORM_SRGB : DXGI_FORMAT_R8G8B8A8_UNORM);
}

void Texture2D::CreateSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const
{
    D3D12_SHADER_RESOURCE_VIEW_DESC srv = {};
    srv.Format                  = m_format;
    srv.ViewDimension           = D3D12_SRV_DIMENSION_TEXTURE2D;
    srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srv.Texture2D.MipLevels     = m_mipLevels;
    device->CreateShaderResourceView(m_tex.Get(), &srv, dst);
}

void Texture2D::Reset()
{
    m_tex.Reset();
    m_format    = DXGI_FORMAT_UNKNOWN;
    m_width     = 0;
    m_height    = 0;
    m_mipLevels = 0;
}

} // namespace SGE
