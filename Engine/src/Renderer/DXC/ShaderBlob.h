#pragma once

#include "Renderer/DX12/DX12Common.h"

#include <string>
#include <vector>

namespace SGE {

// Result of one DXC compilation: DXIL bytecode and (for VS) auto-generated input layout.
// SemanticName pointers in inputLayout are backed by semanticNames — valid for ShaderBlob's lifetime.
struct ShaderBlob
{
    std::vector<uint8_t>                  bytecode;
    std::vector<D3D12_INPUT_ELEMENT_DESC> inputLayout;
    std::vector<std::string>              semanticNames;

    D3D12_SHADER_BYTECODE GetBytecode() const
    {
        return { bytecode.data(), bytecode.size() };
    }
};

} // namespace SGE
