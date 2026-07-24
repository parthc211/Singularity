#pragma once
#include "Scene/DemoScene.h"
#include "Jobs/JobSystem.h"
#include "Renderer/DX12/DX12Common.h"      // ComPtr, ID3D12*, FrameCount
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/ShaderLibrary.h"

#include <DirectXMath.h>
#include <cstdint>
#include <vector>

namespace SGE { class Mesh; }

// ---------------------------------------------------------------------------
// Job System — threaded command-list recording (Phase 10).
//
// Draws a large field of spinning cubes. The scene splits the objects into chunks
// and records each chunk into its OWN command list; in multithreaded mode those
// lists are recorded in parallel on the work-stealing JobSystem, then submitted
// together in order. A toggle switches to single-threaded recording (one list on
// the main thread) so you can watch the CPU record-time drop as cores are added.
//
// DX12 concept: command allocators and command lists are NOT thread-safe to share,
// so each worker owns its own pair. Recording is parallel on the CPU; the GPU still
// executes the lists serially in submission order, so depth testing across chunks
// stays correct. This CPU-side parallelism is the whole reason explicit APIs exist
// (DX11 serialized command generation on the driver thread).
// ---------------------------------------------------------------------------
class JobScene : public SGE::DemoScene
{
public:
    explicit JobScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "Job System"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override
    {
        pos[0] = 0.0f; pos[1] = 0.0f; pos[2] = -70.0f;
        yaw = 0.0f; pitch = 0.0f;
        return true;
    }

private:
    static constexpr uint32_t kMaxChunks  = 16;    // == max worker command lists
    static constexpr uint32_t kMaxObjects = 16384; // arena + address table sizing

    bool BuildPipeline(const SGE::DemoContext& ctx);
    bool BuildCommandObjects(ID3D12Device* device);
    void BuildObjects();
    // Record objects [begin, end) of this frame into worker list `chunk`.
    void RecordChunk(uint32_t chunk, uint32_t begin, uint32_t end,
                     uint32_t frameIndex,
                     D3D12_CPU_DESCRIPTOR_HANDLE rtv,
                     D3D12_CPU_DESCRIPTOR_HANDLE dsv,
                     const D3D12_VIEWPORT& vp, const D3D12_RECT& scissor);

    SGE::Mesh*               m_cube = nullptr;
    SGE::JobSystem           m_jobs;
    SGE::ShaderLibrary       m_shaders;
    SGE::RootSignature       m_rootSig;
    SGE::GraphicsPipeline    m_pso;
    SGE::DynamicUploadBuffer m_objectCB;   // scene-owned (the app's shared arena is tiny)

    // Per-chunk command recording. One list per chunk (reused each frame); one
    // allocator per (frame, chunk) so we never reset an allocator the GPU still reads.
    ComPtr<ID3D12CommandAllocator>    m_alloc[FrameCount][kMaxChunks];
    ComPtr<ID3D12GraphicsCommandList> m_list[kMaxChunks];

    // Per-object data.
    std::vector<DirectX::XMFLOAT3> m_basePos;
    std::vector<DirectX::XMFLOAT4> m_color;
    std::vector<float>             m_spinPhase;
    std::vector<D3D12_GPU_VIRTUAL_ADDRESS> m_gpuAddr; // filled each frame, read by workers

    // Controls / state.
    int   m_objectCount   = 4096;
    bool  m_multithreaded = true;
    bool  m_animate       = true;
    float m_time          = 0.0f;
    bool  m_ready         = false;

    // Rolling CPU record-time averages (ms) for each mode, for the A/B readout.
    float m_msSingle = 0.0f;
    float m_msMulti  = 0.0f;
};
