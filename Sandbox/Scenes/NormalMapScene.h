#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/SrvHeap.h"
#include "Renderer/DX12/Texture2D.h"
#include "Renderer/ShaderLibrary.h"

#include <DirectXMath.h>
#include <vector>

namespace SGE { class Mesh; }

// Texture + normal mapping showcase: albedo and tangent-space normal maps on a
// brick ground and boxes, lit by an animatable directional light. Textures load
// from Assets/ when present, otherwise a hand-written procedural brick generator
// (height field -> central-difference normals) builds them at load. The panel
// A/Bs the technique: normal map on/off, strength, green-channel convention,
// sRGB-vs-linear gamma, and raw view modes for each stage.
class NormalMapScene : public SGE::DemoScene {
public:
    explicit NormalMapScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "Texture / Normal Mapping"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override {
        pos[0] = 0.0f; pos[1] = 5.0f; pos[2] = -11.0f;
        yaw = 0.0f; pitch = -0.35f;
        return true;
    }

private:
    struct Object {
        DirectX::XMFLOAT3 Scale;
        DirectX::XMFLOAT3 Pos;
        DirectX::XMFLOAT2 Tiling;
        bool              Spin = false;  // model matrix rebuilt with rotation each frame
    };

    bool BuildPipeline(const SGE::DemoContext& ctx);
    bool BuildTextures(const SGE::DemoContext& ctx);
    void BuildObjects();

    SGE::Mesh*            m_cube = nullptr;
    SGE::ShaderLibrary    m_shaders;
    SGE::RootSignature    m_rootSig;
    SGE::GraphicsPipeline m_pso;
    SGE::Texture2D        m_albedo;
    SGE::Texture2D        m_normal;
    SGE::SrvHeap          m_srvs;    // t0 albedo, t1 normal
    std::vector<Object>   m_objects;

    float m_time           = 0.0f;
    float m_spinAngle      = 0.0f;
    bool  m_animateLight   = true;
    float m_lightAzimuth   = 0.8f;
    float m_lightElevation = 45.0f;  // degrees above horizon
    bool  m_useNormalMap   = true;
    bool  m_flipGreen      = false;
    bool  m_gammaCorrect   = true;
    float m_normalStrength = 1.0f;
    float m_tilingScale    = 1.0f;
    int   m_viewMode       = 0;      // 0 lit, 1 albedo, 2 geometric N, 3 mapped N
    bool  m_fromFiles      = false;  // textures loaded from Assets/ vs procedural
    bool  m_ready          = false;
};
