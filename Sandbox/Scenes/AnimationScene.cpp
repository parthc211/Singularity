#include "Scenes/AnimationScene.h"

#include "Core/Camera.h"
#include "Core/Logger.h"
#include "Renderer/Renderer.h"
#include "Renderer/DX12/RootSignatureBuilder.h"

#include "Assets/FbxLoader.h"

#include "imgui.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>

using namespace SGE;
using namespace DirectX;

namespace {

// Must match the cbuffers in SkinnedMesh.hlsl / DebugLines.hlsl.
struct ObjCB   { XMFLOAT4X4 MVP; XMFLOAT4X4 Model; XMFLOAT4 BaseColor; };
struct FrameCB { XMFLOAT4 LightDir; XMFLOAT4 CameraPos; };
struct LineCB  { XMFLOAT4X4 Transform; };

struct LineVertex { float pos[3]; float color[3]; };

constexpr int   kMaxGrid  = 8;      // up to 8x8 = 64 instances
constexpr float kSpacing  = 2.0f;   // world units between grid instances

// Worst case per frame: 64 palettes (24 joints, 256-aligned ~1.75 KB each)
// + 64 bone overlays + the floor grid. The shared per-object arena is only
// 64 KB/frame, hence the scene-owned arena.
constexpr size_t kArenaBytes = 384 * 1024;

} // namespace

const char* AnimationScene::Description() const {
    return "Skeletal animation end to end: a hand-written glTF 2.0 loader "
           "(JSON parser, GLB container, skins, clips) feeds keyframe "
           "sampling (slerp), crossfade blending (nlerp), pose propagation "
           "and a bone palette per character — all on the engine's SIMD math "
           "— and the vertex shader blends 4 weighted bone matrices. Scale "
           "the grid and A/B serial vs JobSystem pose evaluation below.";
}

bool AnimationScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* device = ctx.device;
    m_shaders.Initialize(L"Shaders");

    // --- skinned pass: b0 object, b1 frame, b2 bone palette,
    //     t0 base-color texture (param 3), s0 aniso sampler ---
    if (!RootSignatureBuilder()
             .Cbv(0).Cbv(1).Cbv(2)
             .SrvTable(0, 1)
             .SamplerAnisoWrap(0)
             .Build(device, m_skinRootSig))
        return false;

    auto svs = m_shaders.GetOrCompile(L"SkinnedMesh.hlsl", "VSMain", "vs_6_0");
    auto sps = m_shaders.GetOrCompile(L"SkinnedMesh.hlsl", "PSMain", "ps_6_0");
    if (!svs || !sps) return false;

    GraphicsPipelineDesc sd;
    sd.rootSignature = m_skinRootSig.Get();
    sd.vs = svs; sd.ps = sps;
    sd.rtvFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
    if (!m_skinPSO.Create(device, sd))
        return false;

    // --- line pass: one CBV, line topology; grid depth-tested, bones x-ray ---
    if (!RootSignatureBuilder().Cbv(0).Build(device, m_lineRootSig))
        return false;

    auto lvs = m_shaders.GetOrCompile(L"DebugLines.hlsl", "VSMain", "vs_6_0");
    auto lps = m_shaders.GetOrCompile(L"DebugLines.hlsl", "PSMain", "ps_6_0");
    if (!lvs || !lps) return false;

    GraphicsPipelineDesc ld;
    ld.rootSignature = m_lineRootSig.Get();
    ld.vs = lvs; ld.ps = lps;
    ld.rtvFormat    = DXGI_FORMAT_R8G8B8A8_UNORM;
    ld.topologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
    ld.cullMode     = D3D12_CULL_MODE_NONE;
    if (!m_gridPSO.Create(device, ld))
        return false;
    ld.depthEnable = false;                 // skeleton draws through the mesh
    return m_bonePSO.Create(device, ld);
}

void AnimationScene::RebuildInstances() {
    if (m_chars.empty()) return;
    const Character& c = m_chars[size_t(m_active)];
    const int count = m_gridN * m_gridN;
    m_instances.assign(size_t(count), Instance{});

    const float origin = -kSpacing * float(m_gridN - 1) * 0.5f;
    for (int i = 0; i < count; ++i) {
        Instance& inst = m_instances[size_t(i)];
        inst.X = origin + kSpacing * float(i % m_gridN);
        inst.Z = origin + kSpacing * float(i / m_gridN);
        // Stagger playback so the crowd doesn't move in lockstep.
        inst.TimeOffset = 0.37f * float(i);

        if (c.Loaded && !c.Data.Clips.empty()) {
            m_clipIndex = std::clamp(m_clipIndex, 0, int(c.Data.Clips.size()) - 1);
            inst.Player.SetClip(&c.Data.Clips[size_t(m_clipIndex)], m_loop);
            inst.Player.SetTime(inst.TimeOffset);
        }
    }
}

void AnimationScene::SelectClip(int clip) {
    if (m_chars.empty()) return;
    Character& c = m_chars[size_t(m_active)];
    if (!c.Loaded || c.Data.Clips.empty()) return;
    m_clipIndex = std::clamp(clip, 0, int(c.Data.Clips.size()) - 1);

    for (Instance& inst : m_instances) {
        // The old player keeps running while it fades out.
        const bool canFade = m_fadeDuration > 0.0f && inst.Player.Clip() != nullptr;
        inst.PrevPlayer = inst.Player;
        inst.Player.SetClip(&c.Data.Clips[size_t(m_clipIndex)], m_loop);
        inst.Player.SetTime(inst.TimeOffset);
        inst.Fade = canFade ? 0.0f : 1.0f;
    }
}

XMMATRIX AnimationScene::InstanceMatrix(const Instance& inst) const {
    const Character& c = m_chars[size_t(m_active)];
    return XMMatrixScaling(c.Scale, c.Scale, c.Scale)
         * XMMatrixRotationY(m_charYaw)
         * XMMatrixTranslation(inst.X, 0.0f, inst.Z);
}

namespace {
// Display scale for auto-discovered characters: normalize the SKINNED rest
// size (raw vertex units are meaningless for FBX, where unit conversion lives
// in the transforms) to a ~1.8-unit-tall figure.
float AutoScale(const SkeletalMeshData& data)
{
    std::vector<Anim::JointPose> pose = data.Skeleton.RestPose;
    if (!data.Clips.empty())
        Anim::SampleClip(data.Skeleton, data.Clips[0], 0.0f, pose);
    std::vector<Math::Mat4> globals, palette;
    Anim::ComputeGlobals(data.Skeleton, pose, globals);
    Anim::ComputePalette(data.Skeleton, globals, palette);

    float lo[3] = { 1e9f, 1e9f, 1e9f }, hi[3] = { -1e9f, -1e9f, -1e9f };
    for (const SkinnedVertex& sv : data.Vertices) {
        Math::Vec4 acc(0, 0, 0, 0);
        for (int k = 0; k < 4; ++k)
            acc += Math::Transform(
                       Math::Vec4(sv.position[0], sv.position[1], sv.position[2], 1.0f),
                       palette[sv.joints[k]]) * sv.weights[k];
        const float p[3] = { acc.x(), acc.y(), acc.z() };
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], p[a]);
            hi[a] = std::max(hi[a], p[a]);
        }
    }
    const float maxExtent = std::max({ hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2] });
    return maxExtent > 1e-5f ? 1.8f / maxExtent : 1.0f;
}
} // namespace

void AnimationScene::OnLoad(const DemoContext& ctx) {
    // Fixed glTF samples + every FBX dropped into Assets/ (the ufbx front-end:
    // e.g. a Mixamo download appears in the character combo on next load).
    m_chars.clear();
    auto add = [&](std::string file, std::string label, float scale, XMFLOAT4 color) {
        Character c;
        c.File  = std::move(file);
        c.Label = std::move(label);
        c.Scale = scale;
        c.Color = color;
        m_chars.push_back(std::move(c));
    };
    add("Assets/CesiumMan.glb", "CesiumMan", 1.0f,   { 0.62f, 0.72f, 0.80f, 1.0f });
    add("Assets/Fox.glb",       "Fox",       0.025f, { 0.88f, 0.52f, 0.18f, 1.0f });

    const XMFLOAT4 fbxColors[] = { { 0.55f, 0.75f, 0.45f, 1.0f }, { 0.80f, 0.60f, 0.75f, 1.0f },
                                   { 0.85f, 0.80f, 0.45f, 1.0f }, { 0.50f, 0.70f, 0.80f, 1.0f } };
    std::error_code ec;
    size_t fbxCount = 0;
    for (const auto& entry : std::filesystem::directory_iterator("Assets", ec)) {
        if (!entry.is_regular_file()) continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char ch) { return char(std::tolower(ch)); });
        if (ext != ".fbx") continue;
        add(entry.path().generic_string(), entry.path().stem().string(),
            0.0f /* auto */, fbxColors[fbxCount++ % 4]);
    }

    ID3D12CommandQueue* queue = ctx.renderer->GetCommandQueue();
    for (Character& c : m_chars) {
        std::string ext = std::filesystem::path(c.File).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char ch) { return char(std::tolower(ch)); });
        const bool parsed = (ext == ".fbx") ? LoadFBX(c.File.c_str(), c.Data)
                                            : LoadGLTF(c.File.c_str(), c.Data);
        c.Loaded = parsed
                && c.Mesh.Upload(ctx.device, ctx.renderer->GetGeometryHeap(),
                                 c.Data.Vertices, c.Data.Indices);
        if (!c.Loaded)
            LogError("AnimationScene: failed to load " + c.File);
        if (c.Loaded && c.Scale <= 0.0f)
            c.Scale = AutoScale(c.Data);

        // Base-color texture extracted from the glTF; when a character has a
        // real texture its tint becomes the material's factor. Untextured (or
        // failed) characters get 1x1 white so one PSO/shader serves all.
        c.HasTexture = c.Loaded && c.Data.BaseColorImage.IsValid()
                    && c.Tex.CreateFromImage(ctx.device, queue, c.Data.BaseColorImage, true);
        if (c.HasTexture) {
            c.Color = { c.Data.BaseColorFactor[0], c.Data.BaseColorFactor[1],
                        c.Data.BaseColorFactor[2], c.Data.BaseColorFactor[3] };
        } else {
            Image white;
            white.width = white.height = 1;
            white.pixels = { 255, 255, 255, 255 };
            c.Tex.CreateFromImage(ctx.device, queue, white, true);
        }
    }

    m_jobs.Initialize(); // (cores - 1) workers
    m_ready = BuildPipelines(ctx) && m_arena.Init(ctx.device, kArenaBytes)
           && !m_chars.empty() && m_srvs.Create(ctx.device, uint32_t(m_chars.size()));
    if (m_ready)
        for (uint32_t i = 0; i < uint32_t(m_chars.size()); ++i)
            if (m_chars[i].Tex.IsValid())
                m_srvs.Write(ctx.device, i, m_chars[i].Tex);
    m_clipIndex = 0;
    RebuildInstances();
}

void AnimationScene::OnUnload() {
    m_jobs.Shutdown();
    m_instances.clear();
    for (Character& c : m_chars) {
        c.Mesh.Reset();
        c.Tex.Reset();
    }
    m_chars.clear();
    m_srvs.Reset();
    m_skinPSO.Reset();
    m_gridPSO.Reset();
    m_bonePSO.Reset();
    m_skinRootSig.Reset();
    m_lineRootSig.Reset();
    m_arena.Shutdown();
    m_shaders.Shutdown();
    m_ready = false;
}

void AnimationScene::EvaluateInstance(Instance& inst) {
    const Anim::Skeleton& sk = m_chars[size_t(m_active)].Data.Skeleton;
    inst.Player.Sample(sk, inst.Pose);
    if (inst.Fade < 1.0f) {
        inst.PrevPlayer.Sample(sk, inst.PrevPose);
        const float f = inst.Fade;
        const float a = f * f * (3.0f - 2.0f * f);      // smoothstep ease
        Anim::BlendPoses(inst.PrevPose, inst.Pose, a, inst.Pose);
    }
    Anim::ComputeGlobals(sk, inst.Pose, inst.Globals);
    Anim::ComputePalette(sk, inst.Globals, inst.Palette);
}

void AnimationScene::OnUpdate(const DemoContext& ctx) {
    if (m_chars.empty()) return;
    Character& c = m_chars[size_t(m_active)];
    if (!c.Loaded || m_instances.empty()) return;

    if (m_playing) {
        const float dt = ctx.dt * m_speed;
        for (Instance& inst : m_instances) {
            inst.Player.Update(dt);
            if (inst.Fade < 1.0f) {
                inst.PrevPlayer.Update(dt);
                inst.Fade = std::min(1.0f, inst.Fade + ctx.dt / std::max(m_fadeDuration, 1e-3f));
            }
        }
    }

    // The full CPU side of skinning for every instance, timed. Each instance
    // writes only its own buffers, so the parallel path needs no locks.
    const auto t0 = std::chrono::steady_clock::now();
    const uint32_t count = uint32_t(m_instances.size());
    if (m_useJobs && count > 1) {
        m_jobs.Dispatch(count, 1, [this](uint32_t i) { EvaluateInstance(m_instances[i]); });
        m_jobs.Wait();
    } else {
        for (Instance& inst : m_instances)
            EvaluateInstance(inst);
    }
    const float ms = std::chrono::duration<float, std::milli>(
                         std::chrono::steady_clock::now() - t0).count();
    float& slot = (m_useJobs && count > 1) ? m_msJobs : m_msSerial;
    slot += (ms - slot) * 0.05f;   // smoothed for display
}

void AnimationScene::OnRender(const DemoContext& ctx) {
    if (!m_ready || m_chars.empty()) return;
    Character& c = m_chars[size_t(m_active)];
    if (!c.Loaded || m_instances.empty()) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;

    m_arena.BeginFrame(ctx.renderer->GetFrameIndex());
    const XMMATRIX vp = ctx.camera->GetViewProjection();

    // --- characters ---
    if (m_showMesh) {
        FrameCB fcb;
        XMStoreFloat4(&fcb.LightDir, XMVector3Normalize(XMVectorSet(0.45f, -1.0f, 0.35f, 0)));
        fcb.CameraPos = { ctx.cameraPos[0], ctx.cameraPos[1], ctx.cameraPos[2], 1.0f };
        cmd->SetGraphicsRootSignature(m_skinRootSig.Get());
        cmd->SetPipelineState(m_skinPSO.Get());
        cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        m_srvs.BindTable(cmd, 3, uint32_t(m_active));   // this character's albedo
        if (ctx.objectCB->BindCbv(cmd, 1, fcb)) {
            for (const Instance& inst : m_instances) {
                if (inst.Palette.empty()) continue;

                ObjCB ocb;
                const XMMATRIX model = InstanceMatrix(inst);
                XMStoreFloat4x4(&ocb.MVP, model * vp);
                XMStoreFloat4x4(&ocb.Model, model);
                ocb.BaseColor = c.Color;

                if (!ctx.objectCB->BindCbv(cmd, 0, ocb)) break;
                if (!m_arena.BindCbvRaw(cmd, 2, inst.Palette.data(),
                                        inst.Palette.size() * sizeof(Math::Mat4)))
                    break;   // arena full — skip remaining instances
                c.Mesh.Draw(cmd);
            }
        }
    }

    // --- lines: allocate vertices from the arena, draw as a line list ---
    auto drawLines = [&](const std::vector<LineVertex>& verts, const XMMATRIX& xform,
                         GraphicsPipeline& pso) {
        if (verts.empty()) return;
        const D3D12_VERTEX_BUFFER_VIEW vbv = m_arena.PushVertices(
            verts.data(), verts.size() * sizeof(LineVertex), sizeof(LineVertex));
        if (!vbv.BufferLocation) return;

        cmd->SetGraphicsRootSignature(m_lineRootSig.Get());
        cmd->SetPipelineState(pso.Get());
        cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
        LineCB lcb;
        XMStoreFloat4x4(&lcb.Transform, xform);
        if (!ctx.objectCB->BindCbv(cmd, 0, lcb)) return;
        cmd->IASetVertexBuffers(0, 1, &vbv);
        cmd->DrawInstanced(UINT(verts.size()), 1, 0, 0);
    };

    if (m_showGrid) {
        const int half = std::max(5, int(std::ceil(kSpacing * float(m_gridN) * 0.5f)) + 2);
        std::vector<LineVertex> grid;
        grid.reserve(size_t(2 * half + 1) * 4);
        for (int i = -half; i <= half; ++i) {
            const float g = (i == 0) ? 0.38f : 0.22f;
            grid.push_back({ { float(i), 0, float(-half) }, { g, g, g } });
            grid.push_back({ { float(i), 0, float( half) }, { g, g, g } });
            grid.push_back({ { float(-half), 0, float(i) }, { g, g, g } });
            grid.push_back({ { float( half), 0, float(i) }, { g, g, g } });
        }
        drawLines(grid, vp, m_gridPSO);
    }

    if (m_showSkeleton) {
        const Anim::Skeleton& sk = c.Data.Skeleton;
        std::vector<LineVertex> bones;
        bones.reserve(size_t(sk.JointCount()) * 2);
        for (const Instance& inst : m_instances) {
            if (inst.Globals.empty()) continue;
            bones.clear();
            for (uint32_t j = 0; j < sk.JointCount(); ++j) {
                const uint32_t p = sk.Parents[j];
                if (p == Anim::kInvalidJoint) continue;
                float a[4], b[4];
                _mm_storeu_ps(a, inst.Globals[p].r[3]);   // joint origin = global row 3
                _mm_storeu_ps(b, inst.Globals[j].r[3]);
                bones.push_back({ { a[0], a[1], a[2] }, { 1.0f, 0.55f, 0.08f } });
                bones.push_back({ { b[0], b[1], b[2] }, { 1.0f, 0.85f, 0.30f } });
            }
            drawLines(bones, InstanceMatrix(inst) * vp, m_bonePSO);   // model space
        }
    }
}

void AnimationScene::OnImGui() {
    if (m_chars.empty()) return;
    Character& c = m_chars[size_t(m_active)];

    // Character / clip selection (2 glTF samples + discovered FBX files).
    std::vector<const char*> charNames;
    charNames.reserve(m_chars.size());
    for (const Character& ch : m_chars) charNames.push_back(ch.Label.c_str());
    int active = m_active;
    if (ImGui::Combo("Character", &active, charNames.data(), int(charNames.size())) &&
        active != m_active) {
        m_active    = active;
        m_clipIndex = 0;
        RebuildInstances();   // new skeleton — no crossfade across characters
    }

    if (!c.Loaded) {
        ImGui::TextColored({ 1, 0.4f, 0.4f, 1 }, "Failed to load %s", c.File.c_str());
        return;
    }

    if (!c.Data.Clips.empty()) {
        std::vector<const char*> clipNames;
        for (const auto& clip : c.Data.Clips) clipNames.push_back(clip.Name.c_str());
        int clip = m_clipIndex;
        if (ImGui::Combo("Clip", &clip, clipNames.data(), int(clipNames.size())) && clip != m_clipIndex)
            SelectClip(clip);   // crossfades

        ImGui::SliderFloat("Crossfade", &m_fadeDuration, 0.0f, 1.0f, "%.2fs");

        ImGui::Checkbox("Play", &m_playing);
        ImGui::SameLine();
        if (ImGui::Checkbox("Loop", &m_loop)) {
            for (Instance& inst : m_instances)
                inst.Player.SetLooping(m_loop);
        }
        ImGui::SliderFloat("Speed", &m_speed, 0.0f, 2.0f, "%.2fx");

        if (m_instances.size() == 1) {
            Anim::AnimationPlayer& hero = m_instances[0].Player;
            const float duration = hero.Clip() ? hero.Clip()->Duration : 0.0f;
            float t = hero.Time();
            if (ImGui::SliderFloat("Time", &t, 0.0f, duration, "%.2fs"))
                hero.SetTime(t);
        } else {
            ImGui::TextDisabled("(scrubbing available with a 1x1 grid)");
        }
    }

    ImGui::Separator();
    int n = m_gridN;
    if (ImGui::SliderInt("Crowd", &n, 1, kMaxGrid, "%d x %d") && n != m_gridN) {
        m_gridN = n;
        RebuildInstances();
    }
    ImGui::Checkbox("Parallel pose evaluation (JobSystem)", &m_useJobs);
    ImGui::TextDisabled("pose pipeline, %zu char(s): serial %.3f ms | jobs %.3f ms (%u workers)",
                        m_instances.size(),
                        m_msSerial, m_msJobs, m_jobs.ThreadCount());

    ImGui::Separator();
    ImGui::Checkbox("Mesh", &m_showMesh);         ImGui::SameLine();
    ImGui::Checkbox("Skeleton", &m_showSkeleton); ImGui::SameLine();
    ImGui::Checkbox("Floor grid", &m_showGrid);
    ImGui::SliderAngle("Facing", &m_charYaw, -180.0f, 180.0f);

    ImGui::Separator();
    ImGui::TextDisabled("%u verts, %u tris, %u joints, %zu clip(s) per char",
                        c.Mesh.VertexCount(), c.Mesh.IndexCount() / 3,
                        c.Data.Skeleton.JointCount(), c.Data.Clips.size());
    if (c.HasTexture)
        ImGui::TextDisabled("Albedo: %ux%u, extracted from the glTF (WIC memory decode)",
                            c.Tex.Width(), c.Tex.Height());
    else
        ImGui::TextDisabled("Albedo: none in file — flat tint");
    ImGui::TextDisabled("Palette: %u x float4x4 root CBV (b2) per instance",
                        c.Data.Skeleton.JointCount());
}
