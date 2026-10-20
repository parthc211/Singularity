#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/MeshPipeline.h"
#include "Renderer/ShaderLibrary.h"
#include "Renderer/Mesh.h"

#include <cstdint>

// Mesh shaders (the amplification -> mesh -> pixel geometry pipeline). A dense
// sphere is partitioned on the CPU into meshlets (small vertex/triangle clusters);
// at draw time one DispatchMesh launches an amplification shader that frustum- and
// normal-cone-culls whole meshlets, then dispatches only the survivors as
// mesh-shader groups that expand each cluster into triangles directly — no input
// assembler, no index buffer bound. Per-cluster culling on the GPU, the
// Nanite-adjacent front end. A toggle tints each meshlet so the partition is
// visible. Falls back to a message if the GPU lacks mesh-shader support.
class MeshShaderScene : public SGE::DemoScene {
public:
    MeshShaderScene() = default;

    const char* Name()        const override { return "Mesh Shaders"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override;

private:
    bool BuildGeometry(const SGE::DemoContext& ctx);
    bool BuildPipeline(const SGE::DemoContext& ctx);

    SGE::ShaderLibrary m_shaders;
    SGE::Mesh          m_mesh;     // dense sphere (VRAM), read as a StructuredBuffer by the MS

    // Meshlet data as structured buffers (bound by GPU address as root SRVs).
    ComPtr<ID3D12Resource> m_meshlets, m_bounds, m_vertexIndices, m_primIndices;
    uint32_t               m_meshletCount  = 0;
    uint32_t               m_triangleCount = 0;

    SGE::RootSignature m_rootSig;
    SGE::MeshPipeline  m_pso;

    bool  m_cull          = true;
    bool  m_colorMeshlets = true;
    bool  m_spin          = true;
    float m_time          = 0.0f;
    bool  m_supported     = false;
    bool  m_ready         = false;
};
