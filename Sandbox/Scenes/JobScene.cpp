#include "Scenes/JobScene.h"

#include "Core/Camera.h"
#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"

#include "imgui.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

using namespace SGE;
using namespace DirectX;

namespace {
// Must match the cbuffer in Triangle.hlsl (same layout RenderSystem uploads).
struct ObjectConstants
{
    XMFLOAT4X4 MVP;
    XMFLOAT4X4 Model;
    XMFLOAT4   BaseColor;
};

// Even split of [0,count) into `chunks` contiguous ranges: chunk c is [begin,end).
void ChunkRange(uint32_t count, uint32_t chunks, uint32_t c, uint32_t& begin, uint32_t& end)
{
    const uint32_t base = count / chunks;
    const uint32_t rem  = count % chunks;
    begin = c * base + std::min(c, rem);
    end   = begin + base + (c < rem ? 1u : 0u);
}
} // namespace

const char* JobScene::Description() const
{
    return "Threaded command-list recording. A field of spinning cubes is split "
           "into chunks; each chunk is recorded into its own command list. In "
           "multithreaded mode the chunks are recorded in parallel on a "
           "work-stealing job system, then submitted together in order. Toggle "
           "single vs multi and watch the CPU record time change. The GPU still "
           "executes the lists serially, so depth stays correct.";
}

void JobScene::BuildObjects()
{
    m_basePos.resize(kMaxObjects);
    m_color.resize(kMaxObjects);
    m_spinPhase.resize(kMaxObjects);
    m_gpuAddr.resize(kMaxObjects);

    // Arrange into a roughly cubic lattice centred on the origin.
    const uint32_t side = uint32_t(std::ceil(std::cbrt(double(kMaxObjects))));
    const float spacing = 2.2f;
    const float half    = (side - 1) * 0.5f * spacing;

    for (uint32_t i = 0; i < kMaxObjects; ++i)
    {
        const uint32_t x = i % side;
        const uint32_t y = (i / side) % side;
        const uint32_t z = i / (side * side);
        m_basePos[i] = { x * spacing - half, y * spacing - half, z * spacing - half };

        // Hue wheel so neighbours differ.
        const float k = (float(i) / float(kMaxObjects)) * XM_2PI;
        m_color[i] = { 0.5f + 0.5f * sinf(k),
                       0.5f + 0.5f * sinf(k + XM_2PI / 3.0f),
                       0.5f + 0.5f * sinf(k + 2.0f * XM_2PI / 3.0f),
                       1.0f };

        // Deterministic per-object phase (no Date/rand needed for variety).
        m_spinPhase[i] = float((i * 2654435761u) & 0xFFFF) / 65535.0f * XM_2PI;
    }
}

bool JobScene::BuildPipeline(const DemoContext& ctx)
{
    ID3D12Device* device = ctx.device;
    m_shaders.Initialize(L"Shaders");

    auto vs = m_shaders.GetOrCompile(L"Triangle.hlsl", "VSMain", "vs_6_0");
    auto ps = m_shaders.GetOrCompile(L"Triangle.hlsl", "PSMain", "ps_6_0");
    if (!vs || !ps) return false;

    // One root CBV at b0, visible to ALL stages (VS reads gMVP/gModel, PS reads
    // gBaseColor) — same contract as the app's forward pipeline.
    D3D12_ROOT_PARAMETER cbv      = {};
    cbv.ParameterType             = D3D12_ROOT_PARAMETER_TYPE_CBV;
    cbv.Descriptor.ShaderRegister = 0;
    cbv.ShaderVisibility          = D3D12_SHADER_VISIBILITY_ALL;
    if (!m_rootSig.Create(device, &cbv, 1))
        return false;

    GraphicsPipelineDesc desc;
    desc.rootSignature = m_rootSig.Get();
    desc.vs            = vs;
    desc.ps            = ps;
    // Defaults already match the back buffer (R8G8B8A8_UNORM) + depth (D32), CW/back-cull.
    return m_pso.Create(device, desc);
}

bool JobScene::BuildCommandObjects(ID3D12Device* device)
{
    for (uint32_t f = 0; f < FrameCount; ++f)
        for (uint32_t c = 0; c < kMaxChunks; ++c)
            if (FAILED(device->CreateCommandAllocator(
                    D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&m_alloc[f][c]))))
                return false;

    for (uint32_t c = 0; c < kMaxChunks; ++c)
    {
        if (FAILED(device->CreateCommandList(
                0, D3D12_COMMAND_LIST_TYPE_DIRECT, m_alloc[0][c].Get(),
                nullptr, IID_PPV_ARGS(&m_list[c]))))
            return false;
        m_list[c]->Close(); // created open; close so the first per-frame Reset is valid
    }
    return true;
}

void JobScene::OnLoad(const DemoContext& ctx)
{
    BuildObjects();
    if (!BuildPipeline(ctx))          return;
    if (!BuildCommandObjects(ctx.device)) return;

    // Scene-owned constant arena, sized for the whole field (the app's shared arena
    // is only ~256 objects). 256 B per object, split into FrameCount regions.
    m_objectCB.Init(ctx.device, std::size_t(kMaxObjects) * 256);

    m_jobs.Initialize(); // (cores - 1) workers
    m_ready = true;
}

void JobScene::OnUnload()
{
    m_jobs.Shutdown();
    m_objectCB.Shutdown();
    for (uint32_t f = 0; f < FrameCount; ++f)
        for (uint32_t c = 0; c < kMaxChunks; ++c)
            m_alloc[f][c].Reset();
    for (uint32_t c = 0; c < kMaxChunks; ++c)
        m_list[c].Reset();
    m_pso.Reset();
    m_rootSig.Reset();
    m_shaders.Shutdown();
    m_ready = false;
}

void JobScene::OnUpdate(const DemoContext& ctx)
{
    if (m_animate)
        m_time += ctx.dt;
}

void JobScene::RecordChunk(uint32_t chunk, uint32_t begin, uint32_t end,
                           uint32_t frameIndex,
                           D3D12_CPU_DESCRIPTOR_HANDLE rtv,
                           D3D12_CPU_DESCRIPTOR_HANDLE dsv,
                           const D3D12_VIEWPORT& vp, const D3D12_RECT& scissor)
{
    ID3D12GraphicsCommandList* list = m_list[chunk].Get();
    // Fresh list (allocator was already reset on the main thread this frame).
    list->Reset(m_alloc[frameIndex][chunk].Get(), m_pso.Get());
    list->SetGraphicsRootSignature(m_rootSig.Get());
    list->RSSetViewports(1, &vp);
    list->RSSetScissorRects(1, &scissor);
    list->OMSetRenderTargets(1, &rtv, FALSE, &dsv);
    list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    for (uint32_t i = begin; i < end; ++i)
    {
        list->SetGraphicsRootConstantBufferView(0, m_gpuAddr[i]);
        m_cube->Draw(list);
    }
    list->Close();
}

void JobScene::OnRender(const DemoContext& ctx)
{
    if (!m_ready || !m_cube || !m_cube->IsValid()) return;

    Renderer* renderer = ctx.renderer;
    const uint32_t frame = renderer->GetFrameIndex();

    D3D12_CPU_DESCRIPTOR_HANDLE rtv = renderer->GetBackBufferRTV();
    D3D12_CPU_DESCRIPTOR_HANDLE dsv = renderer->GetDepthDSV();
    D3D12_VIEWPORT vp = { 0.0f, 0.0f, float(renderer->GetWidth()), float(renderer->GetHeight()), 0.0f, 1.0f };
    D3D12_RECT scissor = { 0, 0, LONG(renderer->GetWidth()), LONG(renderer->GetHeight()) };

    const uint32_t count = std::clamp<uint32_t>(uint32_t(m_objectCount), 1u, kMaxObjects);

    // --- Precompute all per-object constants on the main thread ---------------
    // The upload arena's bump cursor is NOT thread-safe, so we allocate + fill every
    // object's CBV here, single-threaded, and hand the worker threads only the
    // finished GPU addresses to bind. This keeps the parallel section pure recording.
    m_objectCB.BeginFrame(frame);
    const XMMATRIX viewProj = ctx.camera->GetViewProjection();
    uint32_t recorded = count;
    for (uint32_t i = 0; i < count; ++i)
    {
        const XMMATRIX model =
            XMMatrixScaling(0.6f, 0.6f, 0.6f) *
            XMMatrixRotationRollPitchYaw(m_spinPhase[i] * 0.3f, m_time * 0.8f + m_spinPhase[i], 0.0f) *
            XMMatrixTranslation(m_basePos[i].x, m_basePos[i].y, m_basePos[i].z);

        ObjectConstants oc;
        XMStoreFloat4x4(&oc.MVP,   model * viewProj); // row-vector order, NO transpose
        XMStoreFloat4x4(&oc.Model, model);
        oc.BaseColor = m_color[i];

        const auto alloc = m_objectCB.Allocate(sizeof(ObjectConstants));
        if (!alloc.Cpu) { recorded = i; break; } // arena exhausted (shouldn't happen)
        std::memcpy(alloc.Cpu, &oc, sizeof(ObjectConstants));
        m_gpuAddr[i] = alloc.Gpu;
    }

    // --- Choose chunk count, reset this frame's allocators --------------------
    const uint32_t chunkCount = m_multithreaded
        ? std::clamp<uint32_t>(m_jobs.ThreadCount(), 1u, kMaxChunks)
        : 1u;

    for (uint32_t c = 0; c < chunkCount; ++c)
        m_alloc[frame][c]->Reset(); // safe: BeginFrame already fenced this frame index

    // --- Record (this is the part we time) ------------------------------------
    const auto t0 = std::chrono::high_resolution_clock::now();

    if (m_multithreaded && chunkCount > 1)
    {
        for (uint32_t c = 0; c < chunkCount; ++c)
        {
            uint32_t begin, end;
            ChunkRange(recorded, chunkCount, c, begin, end);
            m_jobs.Execute([this, c, begin, end, frame, rtv, dsv, vp, scissor]()
            {
                RecordChunk(c, begin, end, frame, rtv, dsv, vp, scissor);
            });
        }
        m_jobs.Wait(); // main thread helps record, then blocks until all lists are closed
    }
    else
    {
        RecordChunk(0, 0, recorded, frame, rtv, dsv, vp, scissor);
    }

    const auto t1 = std::chrono::high_resolution_clock::now();
    const float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
    float& ema = m_multithreaded ? m_msMulti : m_msSingle;
    ema = (ema <= 0.0f) ? ms : (ema * 0.9f + ms * 0.1f);

    // --- Submit the chunk lists, correctly sequenced after the frame's clear --
    ID3D12CommandList* lists[kMaxChunks];
    for (uint32_t c = 0; c < chunkCount; ++c)
        lists[c] = m_list[c].Get();
    renderer->ExecuteFrameLists(lists, chunkCount);
}

void JobScene::OnImGui()
{
    ImGui::TextWrapped(
        "Command lists are recorded on the CPU, then executed on the GPU's single "
        "queue in order. Multithreading speeds up the RECORDING (each thread fills "
        "its own list); the GPU work is unchanged.");
    ImGui::Separator();

    ImGui::SliderInt("Objects", &m_objectCount, 512, 8192);
    ImGui::Checkbox("Multithreaded recording", &m_multithreaded);
    ImGui::SameLine();
    ImGui::Checkbox("Animate", &m_animate);

    const uint32_t workers = m_jobs.ThreadCount();
    const uint32_t chunks  = m_multithreaded ? std::clamp<uint32_t>(workers, 1u, kMaxChunks) : 1u;
    ImGui::Text("Worker threads: %u   |   chunks this frame: %u", workers, chunks);

    ImGui::Separator();
    ImGui::Text("CPU record time (rolling avg):");
    ImGui::Text("  single-threaded: %.3f ms", m_msSingle);
    ImGui::Text("  multithreaded:   %.3f ms", m_msMulti);
    if (m_msMulti > 0.0f && m_msSingle > 0.0f)
        ImGui::Text("  speedup: %.2fx", m_msSingle / m_msMulti);
    ImGui::TextDisabled("(toggle the checkbox to populate both rows)");

    ImGui::Separator();
    if (ImGui::Button("Reset job stats"))
        m_jobs.ResetStats();
    ImGui::Text("Jobs run (steals) per runner:");
    for (uint32_t i = 0; i < workers; ++i)
        ImGui::Text("  worker %u: %llu  (%llu stolen)", i,
                    (unsigned long long)m_jobs.Executed(i),
                    (unsigned long long)m_jobs.Steals(i));
    ImGui::Text("  main:     %llu  (%llu stolen)",
                (unsigned long long)m_jobs.Executed(workers),
                (unsigned long long)m_jobs.Steals(workers));
}
