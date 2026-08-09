#include "Renderer/ImageLoader.h"
#include "Core/Logger.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <objbase.h>
#include <wincodec.h>
#include <wrl/client.h>

#include <algorithm>
#include <cmath>
#include <format>

namespace SGE {

using Microsoft::WRL::ComPtr;

// --- WIC decode --------------------------------------------------------------

// WIC is COM: make sure COM is initialized once on this thread. Never
// uninitialized — this is process-lifetime plumbing, like the DXGI factory.
// RPC_E_CHANGED_MODE (someone already initialized STA) is fine for WIC use.
static bool EnsureComInitialized()
{
    static const HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    return SUCCEEDED(hr) || hr == RPC_E_CHANGED_MODE;
}

bool LoadImageRGBA8(const wchar_t* path, Image& out)
{
    out = {};
    if (!EnsureComInitialized()) {
        LogError("LoadImageRGBA8: COM initialization failed");
        return false;
    }

    // Narrow a wide string for the logger (lossy is fine for diagnostics).
    auto narrow = [](const wchar_t* w) {
        std::string s;
        for (; *w; ++w) s += (*w < 128) ? char(*w) : '?';
        return s;
    };
    auto fail = [&](const wchar_t* what, HRESULT hr) {
        LogError(std::format("LoadImageRGBA8('{}'): {} failed (0x{:08X})",
                             narrow(path), narrow(what), static_cast<uint32_t>(hr)));
        return false;
    };

    ComPtr<IWICImagingFactory> factory;
    HRESULT hr = CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                  IID_PPV_ARGS(&factory));
    if (FAILED(hr)) return fail(L"CoCreateInstance(WICImagingFactory)", hr);

    ComPtr<IWICBitmapDecoder> decoder;
    hr = factory->CreateDecoderFromFilename(path, nullptr, GENERIC_READ,
                                            WICDecodeMetadataCacheOnDemand, &decoder);
    if (FAILED(hr)) return fail(L"CreateDecoderFromFilename", hr);

    ComPtr<IWICBitmapFrameDecode> frame;
    hr = decoder->GetFrame(0, &frame);
    if (FAILED(hr)) return fail(L"GetFrame", hr);

    // Convert whatever the file stores (palettized, BGR, 16-bit, ...) to RGBA8.
    ComPtr<IWICBitmapSource> converted;
    hr = WICConvertBitmapSource(GUID_WICPixelFormat32bppRGBA, frame.Get(), &converted);
    if (FAILED(hr)) return fail(L"WICConvertBitmapSource", hr);

    UINT w = 0, h = 0;
    hr = converted->GetSize(&w, &h);
    if (FAILED(hr) || w == 0 || h == 0) return fail(L"GetSize", hr);

    out.width  = w;
    out.height = h;
    out.pixels.resize(size_t(w) * h * 4);
    hr = converted->CopyPixels(nullptr, w * 4, UINT(out.pixels.size()), out.pixels.data());
    if (FAILED(hr)) { out = {}; return fail(L"CopyPixels", hr); }

    return true;
}

bool LoadImageFromMemoryRGBA8(const uint8_t* data, size_t sizeBytes, Image& out)
{
    out = {};
    if (!data || sizeBytes == 0) return false;
    if (!EnsureComInitialized()) {
        LogError("LoadImageFromMemoryRGBA8: COM initialization failed");
        return false;
    }

    auto fail = [](const char* what, HRESULT hr) {
        LogError(std::format("LoadImageFromMemoryRGBA8: {} failed (0x{:08X})",
                             what, static_cast<uint32_t>(hr)));
        return false;
    };

    ComPtr<IWICImagingFactory> factory;
    HRESULT hr = CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                  IID_PPV_ARGS(&factory));
    if (FAILED(hr)) return fail("CoCreateInstance(WICImagingFactory)", hr);

    ComPtr<IWICStream> stream;
    hr = factory->CreateStream(&stream);
    if (FAILED(hr)) return fail("CreateStream", hr);
    // WIC only reads during decode below, so the const_cast is not written through.
    hr = stream->InitializeFromMemory(const_cast<uint8_t*>(data), DWORD(sizeBytes));
    if (FAILED(hr)) return fail("InitializeFromMemory", hr);

    ComPtr<IWICBitmapDecoder> decoder;
    hr = factory->CreateDecoderFromStream(stream.Get(), nullptr,
                                          WICDecodeMetadataCacheOnDemand, &decoder);
    if (FAILED(hr)) return fail("CreateDecoderFromStream", hr);

    ComPtr<IWICBitmapFrameDecode> frame;
    hr = decoder->GetFrame(0, &frame);
    if (FAILED(hr)) return fail("GetFrame", hr);

    ComPtr<IWICBitmapSource> converted;
    hr = WICConvertBitmapSource(GUID_WICPixelFormat32bppRGBA, frame.Get(), &converted);
    if (FAILED(hr)) return fail("WICConvertBitmapSource", hr);

    UINT w = 0, h = 0;
    hr = converted->GetSize(&w, &h);
    if (FAILED(hr) || w == 0 || h == 0) return fail("GetSize", hr);

    out.width  = w;
    out.height = h;
    out.pixels.resize(size_t(w) * h * 4);
    hr = converted->CopyPixels(nullptr, w * 4, UINT(out.pixels.size()), out.pixels.data());
    if (FAILED(hr)) { out = {}; return fail("CopyPixels", hr); }

    return true;
}

// --- mip generation ------------------------------------------------------------

// sRGB <-> linear (exact piecewise curve, not the 2.2 approximation, so repeated
// down-sampling doesn't drift). Decode is a 256-entry table; encode is math.
static const float* SrgbDecodeTable()
{
    static float table[256];
    static const bool built = [] {
        for (int i = 0; i < 256; ++i) {
            const float c = float(i) / 255.0f;
            table[i] = (c <= 0.04045f) ? c / 12.92f
                                       : std::pow((c + 0.055f) / 1.055f, 2.4f);
        }
        return true;
    }();
    (void)built;
    return table;
}

static uint8_t SrgbEncode(float linear)
{
    linear = std::clamp(linear, 0.0f, 1.0f);
    const float c = (linear <= 0.0031308f) ? linear * 12.92f
                                           : 1.055f * std::pow(linear, 1.0f / 2.4f) - 0.055f;
    return uint8_t(c * 255.0f + 0.5f);
}

void GenerateMipChain(const Image& base, ColorSpace space, std::vector<Image>& outMips)
{
    outMips.clear();
    if (!base.IsValid()) return;

    outMips.push_back(base);
    const float* decode = SrgbDecodeTable();

    while (outMips.back().width > 1 || outMips.back().height > 1) {
        const Image& src = outMips.back();
        Image dst;
        dst.width  = std::max(1u, src.width  / 2);
        dst.height = std::max(1u, src.height / 2);
        dst.pixels.resize(size_t(dst.width) * dst.height * 4);

        for (uint32_t y = 0; y < dst.height; ++y) {
            // Clamp the 2x2 source block at the edges (handles odd/1-wide dims).
            const uint32_t sy0 = std::min(2 * y,     src.height - 1);
            const uint32_t sy1 = std::min(2 * y + 1, src.height - 1);
            for (uint32_t x = 0; x < dst.width; ++x) {
                const uint32_t sx0 = std::min(2 * x,     src.width - 1);
                const uint32_t sx1 = std::min(2 * x + 1, src.width - 1);
                const uint8_t* p00 = &src.pixels[(size_t(sy0) * src.width + sx0) * 4];
                const uint8_t* p10 = &src.pixels[(size_t(sy0) * src.width + sx1) * 4];
                const uint8_t* p01 = &src.pixels[(size_t(sy1) * src.width + sx0) * 4];
                const uint8_t* p11 = &src.pixels[(size_t(sy1) * src.width + sx1) * 4];
                uint8_t* d = &dst.pixels[(size_t(y) * dst.width + x) * 4];

                // RGB in the requested color space; alpha is coverage — always linear.
                for (int c = 0; c < 3; ++c) {
                    if (space == ColorSpace::SRGB) {
                        const float sum = decode[p00[c]] + decode[p10[c]]
                                        + decode[p01[c]] + decode[p11[c]];
                        d[c] = SrgbEncode(sum * 0.25f);
                    } else {
                        d[c] = uint8_t((uint32_t(p00[c]) + p10[c] + p01[c] + p11[c] + 2) / 4);
                    }
                }
                d[3] = uint8_t((uint32_t(p00[3]) + p10[3] + p01[3] + p11[3] + 2) / 4);
            }
        }
        outMips.push_back(std::move(dst));
    }
}

// --- procedural fallbacks -------------------------------------------------------

Image MakeCheckerboard(uint32_t size, uint32_t cells,
                       const uint8_t colorA[4], const uint8_t colorB[4])
{
    Image img;
    img.width = img.height = std::max(1u, size);
    img.pixels.resize(size_t(img.width) * img.height * 4);
    const uint32_t cell = std::max(1u, img.width / std::max(1u, cells));

    for (uint32_t y = 0; y < img.height; ++y) {
        for (uint32_t x = 0; x < img.width; ++x) {
            const bool a = ((x / cell) + (y / cell)) % 2 == 0;
            const uint8_t* c = a ? colorA : colorB;
            uint8_t* d = &img.pixels[(size_t(y) * img.width + x) * 4];
            d[0] = c[0]; d[1] = c[1]; d[2] = c[2]; d[3] = c[3];
        }
    }
    return img;
}

Image MakeFlatNormalMap(uint32_t size)
{
    Image img;
    img.width = img.height = std::max(1u, size);
    img.pixels.resize(size_t(img.width) * img.height * 4);
    for (size_t i = 0; i < img.pixels.size(); i += 4) {
        img.pixels[i + 0] = 128;  // x = 0
        img.pixels[i + 1] = 128;  // y = 0
        img.pixels[i + 2] = 255;  // z = 1  (straight out of the surface)
        img.pixels[i + 3] = 255;
    }
    return img;
}

} // namespace SGE
