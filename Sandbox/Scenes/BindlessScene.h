#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/BindlessHeap.h"
#include "Renderer/DX12/Texture2D.h"
#include "Renderer/RenderGraph/RenderGraph.h"
#include "Renderer/ShaderLibrary.h"

#include <cstdint>
#include <vector>

namespace SGE { class Mesh; }

// Bindless rendering showcase (SM 6.6). A grid of cubes, each with a DIFFERENT
// texture, is drawn through a single root signature with no per-draw descriptor
// table: every texture lives in one shader-visible heap and each cube passes
// only its integer index, which the pixel shader resolves via
// ResourceDescriptorHeap[index]. Falls back to a message if the GPU lacks SM 6.6.
class BindlessScene : public SGE::DemoScene {
public:
    explicit BindlessScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "Bindless"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override;

private:
    SGE::Mesh*               m_cube = nullptr;
    SGE::ShaderLibrary       m_shaders;
    SGE::RenderGraph         m_graph;
    SGE::RootSignature       m_rootSig;
    SGE::GraphicsPipeline    m_pso;
    SGE::BindlessHeap        m_bindless;
    std::vector<SGE::Texture2D> m_textures;
    std::vector<uint32_t>       m_texIndex;   // bindless index per texture

    int   m_gridN     = 8;
    float m_spinSpeed = 0.3f;
    float m_time      = 0.0f;
    bool  m_supported = false;
    bool  m_ready     = false;
};
