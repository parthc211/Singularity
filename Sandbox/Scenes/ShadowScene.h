#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/ShadowMap.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/ShaderLibrary.h"

#include <DirectXMath.h>
#include <vector>

namespace SGE { class Mesh; }

// Shadow-mapping showcase: a directional light renders the scene depth into a
// shadow map (light's POV), then the main pass samples it with a PCF comparison
// sampler so occluded pixels fall into soft shadow. Ground plane + boxes; the
// light can be animated and the bias/PCF tuned.
class ShadowScene : public SGE::DemoScene {
public:
    explicit ShadowScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "Shadow Mapping"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override {
        pos[0] = 0.0f; pos[1] = 14.0f; pos[2] = -26.0f;
        yaw = 0.0f; pitch = -0.45f;
        return true;
    }

private:
    struct Object { DirectX::XMFLOAT4X4 Model; DirectX::XMFLOAT4 Color; };

    bool BuildPipelines(const SGE::DemoContext& ctx);
    void BuildObjects();
    DirectX::XMMATRIX LightViewProj() const;

    SGE::Mesh*            m_cube = nullptr;
    SGE::ShadowMap        m_shadowMap;
    SGE::ShaderLibrary    m_shaders;
    SGE::RootSignature    m_shadowRootSig;
    SGE::RootSignature    m_litRootSig;
    SGE::GraphicsPipeline m_shadowPSO;
    SGE::GraphicsPipeline m_litPSO;
    std::vector<Object>   m_objects;

    float m_time         = 0.0f;
    bool  m_animateLight  = true;
    float m_lightAzimuth = 0.6f;   // radians, animated
    float m_lightElevation = 52.0f; // degrees above horizon
    float m_bias         = 0.0018f;
    int   m_pcfRadius    = 2;
    bool  m_ready        = false;

    static constexpr int kShadowSize = 2048;
};
