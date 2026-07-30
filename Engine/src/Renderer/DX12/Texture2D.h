#pragma once
// ---------------------------------------------------------------------------
// Texture2D — a sampled (SRV) texture in a DEFAULT heap with a full mip chain.
//
// Unlike buffers (which meshes sub-allocate from the UPLOAD-type GpuHeap),
// D3D12 forbids TEXTURE2D resources in UPLOAD heaps — sampled textures must
// live in DEFAULT memory and be filled through a staging copy:
//
//   CPU pixels -> UPLOAD staging buffer (rows padded to 256 B footprints)
//              -> CopyTextureRegion per mip
//              -> barrier to PIXEL_SHADER_RESOURCE
//
// Create() is deliberately blocking and self-contained: it records the copy on
// its own one-shot allocator/list, executes on the given queue, and fences.
// That makes it callable from a scene's OnLoad regardless of the shared frame
// list's recording state, and lets the staging buffer be freed before returning.
// Fine for load-time; a streaming path would keep the staging ring alive instead.
//
// CreateSrvInto mirrors RenderTexture: the SRV is written into a caller-owned
// shader-visible heap, so several textures can sit side by side in one heap
// and bind as a single descriptor table (t0 = albedo, t1 = normal map, ...).
// ---------------------------------------------------------------------------
#include "Renderer/DX12/DX12Common.h"
#include "Renderer/ImageLoader.h"

#include <cstdint>
#include <vector>

namespace SGE {

class Texture2D
{
public:
    // Upload an explicit mip chain (mips[0] = base level; dimensions must
    // floor-halve per level, as produced by GenerateMipChain). The format's
    // *_SRGB / plain UNORM choice decides how samplers decode the texels.
    bool Create(ID3D12Device* device, ID3D12CommandQueue* queue,
                const std::vector<Image>& mips, DXGI_FORMAT format);

    // Decode file -> generate mips -> upload. srgb = true for color/albedo
    // textures (R8G8B8A8_UNORM_SRGB + linear-light mip averaging); false for
    // data textures like normal maps (R8G8B8A8_UNORM + raw averaging).
    bool CreateFromFile(ID3D12Device* device, ID3D12CommandQueue* queue,
                        const wchar_t* path, bool srgb);

    // Same, from an already-built CPU image (procedural fallbacks).
    bool CreateFromImage(ID3D12Device* device, ID3D12CommandQueue* queue,
                         const Image& base, bool srgb);

    // Write this texture's SRV (all mips) at the given CPU handle of a
    // caller-owned shader-visible CBV/SRV/UAV heap.
    void CreateSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const;

    void Reset();

    ID3D12Resource* Resource()  const { return m_tex.Get(); }
    uint32_t        Width()     const { return m_width;     }
    uint32_t        Height()    const { return m_height;    }
    uint32_t        MipLevels() const { return m_mipLevels; }
    bool            IsValid()   const { return m_tex != nullptr; }

private:
    ComPtr<ID3D12Resource> m_tex;
    DXGI_FORMAT            m_format    = DXGI_FORMAT_UNKNOWN;
    uint32_t               m_width     = 0;
    uint32_t               m_height    = 0;
    uint32_t               m_mipLevels = 0;
};

} // namespace SGE
