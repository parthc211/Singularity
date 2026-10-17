#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/ComputePipeline.h"
#include "Renderer/DX12/GpuBuffer.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/ShaderLibrary.h"

#include <cstdint>
#include <vector>

namespace SGE { class Mesh; }

// Clustered (froxel) forward shading. The view frustum is diced into a 3D grid
// of froxels; a compute pass builds each froxel's AABB, a second compute pass
// culls hundreds of moving point lights into per-froxel index lists, and the
// forward pass shades each pixel looping ONLY its froxel's lights — so cost
// scales with local light density, not total light count (no deferred 64-light
// cap). A heatmap view shows lights-per-froxel so the culling is visible.
class ClusteredScene : public SGE::DemoScene {
public:
    explicit ClusteredScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "Clustered Shading"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override;

private:
    struct Light { float pos[3]; float radius; float color[3]; float intensity; };
    struct LightAnim { float base[3]; float orbit; float phase; float speed; };

    bool BuildPipelines(const SGE::DemoContext& ctx);
    bool CreateBuffers(const SGE::DemoContext& ctx, uint32_t w, uint32_t h);
    void BuildLights();

    static constexpr uint32_t kTileSize = 64;
    static constexpr uint32_t kSlices   = 24;
    static constexpr uint32_t kMaxPerFroxel = 64; // avg index-list budget / froxel

    SGE::Mesh*            m_cube = nullptr;
    SGE::ShaderLibrary    m_shaders;
    SGE::DynamicUploadBuffer m_upload;  // per-frame lights + cbuffers

    SGE::RootSignature    m_aabbRS, m_cullRS, m_shadeRS;
    SGE::ComputePipeline  m_aabbPSO, m_cullPSO, m_resetPSO;
    SGE::GraphicsPipeline m_shadePSO;

    SGE::GpuBuffer m_aabb, m_lightGrid, m_indexList, m_counter;

    std::vector<Light>     m_lights;
    std::vector<LightAnim> m_anims;

    uint32_t m_w = 0, m_h = 0;
    uint32_t m_tilesX = 0, m_tilesY = 0, m_froxels = 0, m_maxIndices = 0;
    bool     m_clustersDirty = true;

    int   m_lightCount = 512;
    bool  m_animate    = true;
    bool  m_heatmap    = false;
    float m_time       = 0.0f;
    bool  m_ready      = false;
};
