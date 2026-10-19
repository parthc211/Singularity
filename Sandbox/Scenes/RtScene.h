#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/ShaderLibrary.h"
#include "Renderer/Mesh.h"

#include <cstdint>

// Hardware ray tracing (DXR). The scene is assembled from the app's SHARED cube +
// sphere meshes: each mesh's VRAM-resident VB/IB feeds a bottom-level acceleration
// structure (BLAS), and a top-level AS (TLAS) instances them with per-instance
// transforms and materials. A ray-tracing pipeline (ray-gen / closest-hit / miss)
// and a shader binding table drive DispatchRays. Each pixel traces a primary ray, a
// hard shadow ray toward the sun, and — for reflective surfaces — a recursive
// reflection ray; reflections trace the real scene, so they show off-screen geometry
// that screen-space reflections cannot. The closest-hit shader fetches the hit mesh's
// vertices bindlessly (ResourceDescriptorHeap[]) via per-instance indices, so a
// single hit group shades every instance. Result is blitted to the back buffer.
// Falls back to a message if the GPU lacks DXR.
class RtScene : public SGE::DemoScene {
public:
    // Reuses the app's shared cube + sphere meshes: their VRAM-resident VB/IB feed
    // the BLAS builds, and the TLAS instances them with per-instance transforms.
    RtScene(const SGE::Mesh* cube, const SGE::Mesh* sphere)
        : m_cube(cube), m_sphere(sphere) {}

    const char* Name()        const override { return "Ray Tracing (DXR)"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override;

private:
    bool BuildGeometry(const SGE::DemoContext& ctx);
    bool BuildAccelerationStructures(const SGE::DemoContext& ctx);
    bool BuildPipeline(const SGE::DemoContext& ctx);
    bool BuildShaderTable(const SGE::DemoContext& ctx);
    bool CreateOutput(const SGE::DemoContext& ctx, uint32_t w, uint32_t h);

    SGE::ShaderLibrary    m_shaders;
    const SGE::Mesh*      m_cube   = nullptr;   // shared unit cube   (injected)
    const SGE::Mesh*      m_sphere = nullptr;   // shared unit sphere (injected)

    ComPtr<ID3D12Resource> m_cubeBlas, m_sphereBlas, m_tlas; // acceleration structures
    ComPtr<ID3D12Resource> m_instanceData;  // StructuredBuffer<InstanceData>, one per TLAS instance
    uint32_t               m_instanceCount = 0;
    uint32_t               m_triangleCount = 0;  // for the HUD

    SGE::RootSignature     m_globalRS;      // DXR global root signature
    ComPtr<ID3D12StateObject> m_rtpso;      // ray-tracing pipeline
    ComPtr<ID3D12Resource> m_sbt;           // shader binding table
    uint64_t               m_sbtMissOffset = 0, m_sbtHitOffset = 0;

    ComPtr<ID3D12Resource>       m_output;  // RT output texture (UAV + SRV)
    ComPtr<ID3D12DescriptorHeap> m_rtHeap;  // output UAV/SRV + the two meshes' VB/IB SRVs
    uint32_t                     m_rtHeapInc = 0;

    SGE::RootSignature    m_blitRS;
    SGE::GraphicsPipeline m_blitPSO;

    uint32_t m_w = 0, m_h = 0;
    float    m_sunYaw = 0.6f, m_sunPitch = 0.9f;
    int      m_bounces = 2;
    bool     m_spinSun = false;
    float    m_time = 0.0f;
    bool     m_supported = false;
    bool     m_ready = false;
};
