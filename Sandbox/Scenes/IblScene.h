#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/SrvHeap.h"
#include "Renderer/RenderGraph/RenderGraph.h"
#include "Renderer/IBL/IblEnvironment.h"
#include "Renderer/ShaderLibrary.h"

namespace SGE { class Mesh; }

// Image-based lighting showcase. A procedural HDR sky is baked (by compute) into
// an environment cube, an irradiance cube (diffuse IBL), a prefiltered-mip cube
// (specular IBL) and a BRDF LUT. A grid of spheres — roughness across, metalness
// down — is then lit entirely from that environment (the split-sum
// approximation), drawn under the skybox. ImGui toggles diffuse/specular IBL and
// the analytic sun, and re-bakes when the sun/exposure change.
class IblScene : public SGE::DemoScene {
public:
    explicit IblScene(SGE::Mesh* sphere) : m_sphere(sphere) {}

    const char* Name()        const override { return "Image-Based Lighting"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override;

private:
    bool BuildPipelines(const SGE::DemoContext& ctx);
    void WriteSrvs(const SGE::DemoContext& ctx);   // (re)point the SRV heap at the baked cubes

    SGE::Mesh*            m_sphere = nullptr;
    SGE::ShaderLibrary    m_shaders;
    SGE::IblEnvironment   m_ibl;
    SGE::RenderGraph      m_graph;
    SGE::SrvHeap          m_srvs;   // [0]=env, [1]=irradiance, [2]=prefiltered, [3]=brdfLut
    SGE::RootSignature    m_skyRS,  m_objRS;
    SGE::GraphicsPipeline m_skyPSO, m_objPSO;

    SGE::IblBakeParams m_bake;
    int   m_gridN         = 7;
    bool  m_useDirect     = true;
    bool  m_diffuseIbl    = true;
    bool  m_specularIbl   = true;
    float m_iblIntensity  = 1.0f;
    float m_albedo[3]     = { 1.0f, 0.77f, 0.34f }; // gold-ish, reads well on metals
    bool  m_rebakeQueued  = false;
    bool  m_ready         = false;
};
