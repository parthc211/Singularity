#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/ComputePipeline.h"
#include "Renderer/DX12/GpuBuffer.h"
#include "Renderer/ShaderLibrary.h"
#include "Renderer/Mesh.h"

#include <cstdint>

// GPU-driven rendering. Thousands of cube instances are uploaded once; every
// frame a compute pass frustum-culls them and atomically compacts the surviving
// DrawIndexedInstanced commands into an indirect-args buffer, which a single
// ExecuteIndirect then draws. The CPU issues no per-object visibility or draw
// calls — the GPU decides what to draw. A read-back counter shows how many of
// the total instances survived culling as you fly the camera.
class GpuDrivenScene : public SGE::DemoScene {
public:
    GpuDrivenScene() = default;

    const char* Name()        const override { return "GPU-Driven Rendering"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override;

private:
    bool BuildGeometry(const SGE::DemoContext& ctx);
    void BuildInstances(const SGE::DemoContext& ctx);
    bool BuildPipelines(const SGE::DemoContext& ctx);

    SGE::ShaderLibrary    m_shaders;

    SGE::Mesh              m_cube;          // shared unit cube (cube.obj), VRAM-resident
    ComPtr<ID3D12Resource> m_instances;     // per-instance structured buffer (model + colour + bound)
    uint32_t               m_instanceCount = 0;

    SGE::GpuBuffer m_commands, m_counter;   // indirect args + survivor count (UAV)
    ComPtr<ID3D12Resource> m_readback;      // counter read-back (FrameCount slots)

    ComPtr<ID3D12CommandSignature> m_cmdSig;
    SGE::RootSignature    m_cullRS, m_drawRS;
    SGE::ComputePipeline  m_cullPSO, m_resetPSO;
    SGE::GraphicsPipeline m_drawPSO;

    uint32_t m_drawn = 0;
    bool     m_cull  = true;
    bool     m_faceColors = false;
    float    m_time  = 0.0f;
    bool     m_ready = false;
};
