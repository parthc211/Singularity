#include "Renderer/DX12/RootSignatureBuilder.h"

namespace SGE {

RootSignatureBuilder& RootSignatureBuilder::Cbv(UINT reg, D3D12_SHADER_VISIBILITY vis)
{
    D3D12_ROOT_PARAMETER p          = {};
    p.ParameterType                 = D3D12_ROOT_PARAMETER_TYPE_CBV;
    p.Descriptor.ShaderRegister     = reg;
    p.ShaderVisibility              = vis;
    m_params.push_back(p);
    return *this;
}

RootSignatureBuilder& RootSignatureBuilder::Srv(UINT reg, D3D12_SHADER_VISIBILITY vis)
{
    D3D12_ROOT_PARAMETER p          = {};
    p.ParameterType                 = D3D12_ROOT_PARAMETER_TYPE_SRV;
    p.Descriptor.ShaderRegister     = reg;
    p.ShaderVisibility              = vis;
    m_params.push_back(p);
    return *this;
}

RootSignatureBuilder& RootSignatureBuilder::Uav(UINT reg, D3D12_SHADER_VISIBILITY vis)
{
    D3D12_ROOT_PARAMETER p          = {};
    p.ParameterType                 = D3D12_ROOT_PARAMETER_TYPE_UAV;
    p.Descriptor.ShaderRegister     = reg;
    p.ShaderVisibility              = vis;
    m_params.push_back(p);
    return *this;
}

RootSignatureBuilder& RootSignatureBuilder::SrvTable(UINT baseReg, UINT count,
                                                     D3D12_SHADER_VISIBILITY vis)
{
    D3D12_DESCRIPTOR_RANGE range            = {};
    range.RangeType                         = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    range.NumDescriptors                    = count;
    range.BaseShaderRegister                = baseReg;
    range.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    D3D12_ROOT_PARAMETER p                    = {};
    p.ParameterType                           = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    p.DescriptorTable.NumDescriptorRanges     = 1;
    p.ShaderVisibility                        = vis;
    m_tables.push_back({ m_params.size(), range });   // pointer patched in Build
    m_params.push_back(p);
    return *this;
}

namespace {
D3D12_STATIC_SAMPLER_DESC BaseSampler(UINT reg)
{
    D3D12_STATIC_SAMPLER_DESC s = {};
    s.AddressU         = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    s.AddressV         = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    s.AddressW         = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    s.MaxLOD           = D3D12_FLOAT32_MAX;
    s.ShaderRegister   = reg;
    s.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    return s;
}
} // namespace

RootSignatureBuilder& RootSignatureBuilder::SamplerAnisoWrap(UINT reg)
{
    D3D12_STATIC_SAMPLER_DESC s = BaseSampler(reg);
    s.Filter        = D3D12_FILTER_ANISOTROPIC;
    s.MaxAnisotropy = 8;
    s.AddressU = s.AddressV = s.AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    m_samplers.push_back(s);
    return *this;
}

RootSignatureBuilder& RootSignatureBuilder::SamplerLinearClamp(UINT reg)
{
    D3D12_STATIC_SAMPLER_DESC s = BaseSampler(reg);
    s.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    m_samplers.push_back(s);
    return *this;
}

RootSignatureBuilder& RootSignatureBuilder::SamplerPointClamp(UINT reg)
{
    D3D12_STATIC_SAMPLER_DESC s = BaseSampler(reg);
    s.Filter = D3D12_FILTER_MIN_MAG_MIP_POINT;
    m_samplers.push_back(s);
    return *this;
}

RootSignatureBuilder& RootSignatureBuilder::SamplerShadowPcf(UINT reg)
{
    // Hardware-PCF comparison sampler; border = opaque white so anything
    // outside the light frustum counts as lit.
    D3D12_STATIC_SAMPLER_DESC s = BaseSampler(reg);
    s.Filter         = D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT;
    s.AddressU = s.AddressV = s.AddressW = D3D12_TEXTURE_ADDRESS_MODE_BORDER;
    s.BorderColor    = D3D12_STATIC_BORDER_COLOR_OPAQUE_WHITE;
    s.ComparisonFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
    m_samplers.push_back(s);
    return *this;
}

RootSignatureBuilder& RootSignatureBuilder::Sampler(const D3D12_STATIC_SAMPLER_DESC& desc)
{
    m_samplers.push_back(desc);
    return *this;
}

bool RootSignatureBuilder::Build(ID3D12Device* device, RootSignature& out,
                                 D3D12_ROOT_SIGNATURE_FLAGS flags)
{
    // Range storage lives in m_tables (stable by now); patch the pointers.
    for (TableRange& t : m_tables)
        m_params[t.paramIndex].DescriptorTable.pDescriptorRanges = &t.range;

    return out.Create(device,
                      m_params.empty() ? nullptr : m_params.data(),
                      UINT(m_params.size()),
                      m_samplers.empty() ? nullptr : m_samplers.data(),
                      UINT(m_samplers.size()),
                      flags);
}

} // namespace SGE
