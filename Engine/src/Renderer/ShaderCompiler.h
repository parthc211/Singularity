#pragma once

#include "Renderer/DX12/DX12Common.h"
#include <string>

namespace SGE {

// Wraps D3DCompileFromFile (FXC, SM 5.1). Shader Model 6+ upgrade via DXC comes in the shader system step.
ComPtr<ID3DBlob> CompileShader(const std::wstring& path, const char* entryPoint, const char* target);

} // namespace SGE
