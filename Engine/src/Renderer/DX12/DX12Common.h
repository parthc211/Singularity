#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <wrl/client.h>

#include <d3d12.h>
#include <dxgi1_6.h>
#include <d3dcompiler.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <format>

using Microsoft::WRL::ComPtr;

static constexpr uint32_t FrameCount = 2;

namespace SGE {

// Throws a descriptive exception on DX12/DXGI HRESULT failures.
inline void ThrowIfFailed(HRESULT hr, const char* file, int line)
{
    if (FAILED(hr))
        throw std::runtime_error(std::format("DX12 error 0x{:08X} at {}:{}", static_cast<uint32_t>(hr), file, line));
}

} // namespace SGE

#define SGE_THROW_IF_FAILED(hr) SGE::ThrowIfFailed((hr), __FILE__, __LINE__)
