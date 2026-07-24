#pragma once

#include "Renderer/DX12/DX12Common.h"

namespace SGE {

class DepthBuffer
{
public:
    bool Create(ID3D12Device* device, uint32_t width, uint32_t height);
    void Resize(ID3D12Device* device, uint32_t width, uint32_t height);
    void Reset() { m_resource.Reset(); }

    D3D12_CPU_DESCRIPTOR_HANDLE GetDSV() const;

private:
    ComPtr<ID3D12Resource>       m_resource;
    ComPtr<ID3D12DescriptorHeap> m_dsvHeap;
};

} // namespace SGE
