#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/CascadedShadowMap.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/ShaderLibrary.h"

#include <DirectXMath.h>
#include <vector>

namespace SGE { class Mesh; class Camera; }

// Cascaded shadow maps: the view frustum is split into depth ranges, each gets a
// tight light-space shadow map (an array slice), and the main pass picks the
// right cascade per pixel — crisp shadows near the camera, coverage far away.
// A large scene (vs the single-map ShadowScene) makes the cascades matter; a
// debug toggle tints pixels by cascade.
class CsmScene : public SGE::DemoScene {
public:
    explicit CsmScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "Cascaded Shadows"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override {
        pos[0] = 0.0f; pos[1] = 12.0f; pos[2] = -34.0f;
        yaw = 0.0f; pitch = -0.22f;
        return true;
    }

private:
    struct Object { DirectX::XMFLOAT4X4 Model; DirectX::XMFLOAT4 Color; };

    bool BuildPipelines(const SGE::DemoContext& ctx);
    void BuildObjects();
    DirectX::XMVECTOR LightDir() const;
    // Fills cascade light view-proj matrices + far split distances for the camera.
    void ComputeCascades(const SGE::Camera& camera, DirectX::XMFLOAT4X4 outVP[4],
                         float outSplits[4]) const;

    static constexpr int kCascades   = 4;
    static constexpr int kShadowSize = 2048;

    SGE::Mesh*               m_cube = nullptr;
    SGE::CascadedShadowMap   m_csm;
    SGE::ShaderLibrary       m_shaders;
    SGE::RootSignature       m_shadowRootSig, m_litRootSig;
    SGE::GraphicsPipeline    m_shadowPSO, m_litPSO;
    std::vector<Object>      m_objects;

    float m_time           = 0.0f;
    bool  m_animateLight    = true;
    float m_lightAzimuth   = 0.7f;
    float m_lightElevation = 48.0f;
    float m_bias           = 0.0025f;
    float m_lambda         = 0.6f;   // split blend: uniform <-> logarithmic
    float m_shadowFar      = 110.0f;
    bool  m_debugTint      = false;
    bool  m_ready          = false;
};
