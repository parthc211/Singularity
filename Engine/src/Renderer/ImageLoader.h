#pragma once
// ---------------------------------------------------------------------------
// CPU-side image handling for sampled textures.
//
// LoadImageRGBA8 decodes any WIC-supported file (PNG/JPG/BMP/TGA-via-codec...)
// into a tightly-packed 8-bit RGBA buffer. WIC ships with Windows, so this adds
// no third-party dependency — image *decoding* is "boring plumbing" under the
// project's rules (same category as ImGui); everything downstream of the decode
// (mip generation, GPU upload) is hand-written.
//
// GenerateMipChain builds the full chain on the CPU with a 2x2 box filter.
// Albedo textures are stored gamma-encoded, so averaging raw bytes darkens
// every mip — pass ColorSpace::SRGB to average in linear light and re-encode.
// Data textures (normal maps) must average their raw values: pass Linear.
// ---------------------------------------------------------------------------
#include <cstdint>
#include <vector>

namespace SGE {

struct Image
{
    uint32_t             width  = 0;
    uint32_t             height = 0;
    std::vector<uint8_t> pixels;      // RGBA8, tightly packed (row pitch = width * 4)

    bool IsValid() const { return width > 0 && height > 0 && pixels.size() == size_t(width) * height * 4; }
};

enum class ColorSpace { Linear, SRGB };

// Decode an image file to RGBA8. Returns false (and logs) on failure.
bool LoadImageRGBA8(const wchar_t* path, Image& out);

// Decode an in-memory image (PNG/JPEG bytes — e.g. a GLB-embedded texture)
// to RGBA8 through the same WIC path.
bool LoadImageFromMemoryRGBA8(const uint8_t* data, size_t sizeBytes, Image& out);

// Full mip chain down to 1x1. outMips[0] is a copy of base; each further level
// floor-halves the dimensions (matching D3D12's mip sizing for NPOT textures).
void GenerateMipChain(const Image& base, ColorSpace space, std::vector<Image>& outMips);

// Procedural fallbacks so demo scenes still run when asset files are missing.
Image MakeCheckerboard(uint32_t size, uint32_t cells,
                       const uint8_t colorA[4], const uint8_t colorB[4]);
Image MakeFlatNormalMap(uint32_t size);   // uniform (128,128,255) = +Z tangent space

} // namespace SGE
