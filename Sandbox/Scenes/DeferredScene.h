#pragma once
#include "Scene/DemoScene.h"
#include "Scene/World.h"
#include "Renderer/DX12/GBuffer.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/ShaderLibrary.h"

#include <DirectXMath.h>
#include <vector>
#include <cstdint>

namespace SGE { class Mesh; }

// Deferred-rendering showcase. Renders a grid of cubes into a 3-target G-buffer
// (albedo / world normal / world position), then a single fullscreen lighting
// pass accumulates many moving point lights from the G-buffer. ImGui exposes the
// light count, animation, and G-buffer debug views.
class DeferredScene : public SGE::DemoScene {
public:
    explicit DeferredScene(SGE::Mesh* cube) : m_cube(cube) {}
    ~DeferredScene() override;

    const char* Name()        const override { return "Deferred Rendering"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

private:
    static constexpr int kMaxLights = 64;

    // GPU-side light, must match GpuLight in DeferredLighting.hlsl (32 bytes).
    struct GpuLight {
        DirectX::XMFLOAT3 Position; float Radius;
        DirectX::XMFLOAT3 Color;    float Intensity;
    };
    // Whole lighting cbuffer, must match LightData in DeferredLighting.hlsl.
    struct LightData {
        DirectX::XMFLOAT4 CameraPos;
        DirectX::XMFLOAT4 Ambient;
        uint32_t          LightCount;
        uint32_t          DebugMode;
        uint32_t          _pad0;
        uint32_t          _pad1;
        GpuLight          Lights[kMaxLights];
    };
    // CPU-only per-light animation parameters.
    struct LightAnim {
        DirectX::XMFLOAT3 Center;
        float             OrbitRadius;
        float             Phase;
        float             Speed;
        DirectX::XMFLOAT3 Color;
        float             Intensity;
        float             LightRadius;
    };

    void BuildScene();
    void BuildLights();
    bool BuildPipelines(const SGE::DemoContext& ctx);

    SGE::Mesh*               m_cube = nullptr;
    SGE::World               m_world;
    SGE::GBuffer             m_gbuffer;
    SGE::ShaderLibrary       m_shaders;
    SGE::RootSignature       m_geoRootSig;
    SGE::GraphicsPipeline    m_geoPSO;
    SGE::RootSignature       m_lightRootSig;
    SGE::GraphicsPipeline    m_lightPSO;

    std::vector<LightAnim>   m_anims;
    LightData                m_lightData = {};
    float                    m_time      = 0.0f;

    // ImGui-controlled state.
    int   m_lightCount = 16;
    int   m_debugMode  = 0;
    bool  m_animate    = true;
    float m_ambient    = 0.06f;
    bool  m_ready      = false;
};
