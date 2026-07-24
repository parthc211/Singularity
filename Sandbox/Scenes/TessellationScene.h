#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/VertexBuffer.h"
#include "Renderer/DX12/IndexBuffer.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/ShaderLibrary.h"

#include <cstdint>

// Hardware-tessellation showcase: a coarse grid of quad patches is subdivided on
// the GPU (denser near the camera) and displaced by procedural fBm noise into
// terrain. A wireframe toggle makes the distance-based LOD visible. No mesh asset
// — the patch control points are generated on load.
class TessellationScene : public SGE::DemoScene {
public:
    const char* Name()        const override { return "Tessellation"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    // Start above the terrain looking down (the default y=2 camera would be
    // underneath the hills, looking at their undersides — appears upside down).
    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override {
        pos[0] = 0.0f; pos[1] = 22.0f; pos[2] = -34.0f;
        yaw = 0.0f; pitch = -0.5f; // face +Z, tilt down ~28 degrees
        return true;
    }

private:
    void BuildGrid(const SGE::DemoContext& ctx);
    bool BuildPipelines(const SGE::DemoContext& ctx);

    SGE::VertexBuffer    m_vb;           // patch control points (grid corners)
    SGE::IndexBuffer     m_ib;           // 4 control points per patch
    std::uint32_t        m_indexCount = 0;
    SGE::ShaderLibrary   m_shaders;
    SGE::RootSignature   m_rootSig;
    SGE::GraphicsPipeline m_solidPSO;
    SGE::GraphicsPipeline m_wirePSO;

    // ImGui-controlled terrain/LOD parameters.
    float m_maxTess     = 24.0f;
    float m_lodScale    = 200.0f;
    float m_heightScale = 9.0f;
    float m_noiseFreq   = 0.04f;
    bool  m_wireframe   = false;
    bool  m_ready       = false;
};
