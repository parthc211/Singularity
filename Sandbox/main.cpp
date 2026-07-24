#include "Core/Application.h"
#include "Core/Camera.h"
#include "Core/InputSystem.h"
#include "Renderer/ShaderLibrary.h"
#include "Renderer/Mesh.h"
#include "Renderer/MeshLoader.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Scene/RenderSystem.h"
#include "Scene/SceneManager.h"
#include "Scene/DemoScene.h"
#include "Scenes/CubeGridScene.h"
#include "Scenes/SingleCubeScene.h"
#include "Scenes/GpuHeapScene.h"
#include "Scenes/DeferredScene.h"
#include "Scenes/SimdMathScene.h"
#include "Scenes/AllocatorScene.h"
#include "Scenes/TessellationScene.h"
#include "Scenes/ShadowScene.h"
#include "Scenes/BloomScene.h"
#include "Scenes/SsaoScene.h"
#include "Scenes/CsmScene.h"
#include "Scenes/JobScene.h"
#include "Scenes/PhysicsScene.h"
#include "Scenes/JointScene.h"

#include "imgui.h"

#include <memory>

#include <DirectXMath.h>
#include <cmath>
#include <cstdint>
#include <vector>

// Unit-radius UV sphere (clockwise winding, like cube.obj — the pipeline culls
// counter-clockwise back faces). Position doubles as the normal.
static void BuildUvSphere(std::vector<SGE::MeshVertex>& verts,
                          std::vector<uint32_t>&        indices,
                          uint32_t stacks = 16, uint32_t slices = 24)
{
    for (uint32_t r = 0; r <= stacks; ++r) {
        const float phi = DirectX::XM_PI * float(r) / float(stacks);
        for (uint32_t s = 0; s <= slices; ++s) {
            const float theta = DirectX::XM_2PI * float(s) / float(slices);
            const float x = std::sin(phi) * std::cos(theta);
            const float y = std::cos(phi);
            const float z = std::sin(phi) * std::sin(theta);
            SGE::MeshVertex v;
            v.position[0] = x; v.position[1] = y; v.position[2] = z;
            v.normal[0]   = x; v.normal[1]   = y; v.normal[2]   = z;
            v.texCoord[0] = float(s) / float(slices);
            v.texCoord[1] = float(r) / float(stacks);
            verts.push_back(v);
        }
    }
    const uint32_t stride = slices + 1;
    for (uint32_t r = 0; r < stacks; ++r) {
        for (uint32_t s = 0; s < slices; ++s) {
            const uint32_t a = r * stride + s;          // ring r, slice s
            const uint32_t b = a + 1;                   // ring r, slice s+1
            const uint32_t c = a + stride;              // ring r+1, slice s
            const uint32_t d = c + 1;                   // ring r+1, slice s+1
            indices.insert(indices.end(), { a, b, d }); // CW seen from outside
            indices.insert(indices.end(), { a, d, c });
        }
    }
}

class SandboxApp : public SGE::Application
{
protected:
    void OnStartup() override
    {
        ID3D12Device* device = GetRenderer().GetDevice();

        // Shaders
        m_shaderLib.Initialize(L"Shaders");
        auto vs = m_shaderLib.GetOrCompile(L"Triangle.hlsl", "VSMain", "vs_6_0");
        auto ps = m_shaderLib.GetOrCompile(L"Triangle.hlsl", "PSMain", "ps_6_0");

        // Root signature: one root CBV at b0 (root parameter slot 0). Visible to
        // ALL stages — the VS reads gMVP/gModel and the PS reads gBaseColor, so a
        // VERTEX-only visibility would make the tint unreadable in the PS (and the
        // debug layer would flag it). The slot index (0) is what RenderSystem binds.
        D3D12_ROOT_PARAMETER cbvParam          = {};
        cbvParam.ParameterType                 = D3D12_ROOT_PARAMETER_TYPE_CBV;
        cbvParam.Descriptor.ShaderRegister     = 0;
        cbvParam.Descriptor.RegisterSpace      = 0;
        cbvParam.ShaderVisibility              = D3D12_SHADER_VISIBILITY_ALL;
        m_rootSig.Create(device, &cbvParam, 1);

        SGE::GraphicsPipelineDesc pipelineDesc;
        pipelineDesc.rootSignature = m_rootSig.Get();
        pipelineDesc.vs            = vs;
        pipelineDesc.ps            = ps;
        m_pipeline.Create(device, pipelineDesc);

        // Per-object constant arena. Each draw sub-allocates one 256-aligned
        // ObjectConstants block from this; the buffer is split into FrameCount
        // regions internally so frame N+1 never stomps data the GPU is still
        // reading from frame N. 64 KB/frame ≈ 256 objects — ample for the demo.
        m_objectCB.Init(device, 64 * 1024);

        // Load mesh from OBJ. Owned here (by the app); MeshComponent only points
        // at it, so the mesh must outlive every entity that references it.
        std::vector<SGE::MeshVertex> verts;
        std::vector<uint32_t>        indices;
        if (SGE::LoadOBJ("Assets/cube.obj", verts, indices))
            m_mesh.Upload(device, GetRenderer().GetGeometryHeap(), verts, indices);

        // Procedural unit sphere for the physics scenes (scaled per body).
        verts.clear();
        indices.clear();
        BuildUvSphere(verts, indices);
        m_sphereMesh.Upload(device, GetRenderer().GetGeometryHeap(), verts, indices);

        // Register the demo scenes. Each owns its own ECS World; they share this
        // app's cube mesh, upload arena, RenderSystem, root sig and PSO. Activate
        // the first one — OnLoad here only builds CPU-side entities, so passing a
        // context whose command list isn't recording yet is fine.
        m_scenes.Add(std::make_unique<CubeGridScene>(&m_mesh));
        m_scenes.Add(std::make_unique<SingleCubeScene>(&m_mesh));
        m_scenes.Add(std::make_unique<GpuHeapScene>());
        m_scenes.Add(std::make_unique<DeferredScene>(&m_mesh));
        m_scenes.Add(std::make_unique<SimdMathScene>());
        m_scenes.Add(std::make_unique<AllocatorScene>());
        m_scenes.Add(std::make_unique<TessellationScene>());
        m_scenes.Add(std::make_unique<ShadowScene>(&m_mesh));
        m_scenes.Add(std::make_unique<BloomScene>(&m_mesh));
        m_scenes.Add(std::make_unique<SsaoScene>(&m_mesh));
        m_scenes.Add(std::make_unique<CsmScene>(&m_mesh));
        m_scenes.Add(std::make_unique<JobScene>(&m_mesh));
        m_scenes.Add(std::make_unique<PhysicsScene>(&m_mesh, &m_sphereMesh));
        m_scenes.Add(std::make_unique<JointScene>(&m_mesh, &m_sphereMesh));
        m_scenes.SetActiveIndex(0, BuildContext(0.0f));

        // Camera — projection; view is rebuilt each frame from fly-cam state
        float aspect = float(GetWindow().GetWidth()) / float(GetWindow().GetHeight());
        m_camera.Perspective(DirectX::XMConvertToRadians(60.0f), aspect, 0.1f, 100.0f);

        // Hot-reload: rebuild PSO when shader changes on disk
        m_shaderLib.OnReload(L"Triangle.hlsl", [this]()
        {
            auto& renderer = GetRenderer();
            renderer.WaitForGPU();
            auto vs = m_shaderLib.GetOrCompile(L"Triangle.hlsl", "VSMain", "vs_6_0");
            auto ps = m_shaderLib.GetOrCompile(L"Triangle.hlsl", "PSMain", "ps_6_0");
            m_pipeline.UpdateShaders(vs, ps);
            m_pipeline.Rebuild(renderer.GetDevice());
        });
    }

    void OnRender() override
    {
        using namespace DirectX;
        m_shaderLib.FlushReloads();

        auto&  input = GetInput();
        float  dt    = GetDeltaTime();

        // When the active scene changes, let it suggest an initial camera pose
        // (applied once, before this frame's movement, so the user can still fly).
        if (m_scenes.ActiveIndex() != m_lastSceneIndex) {
            m_lastSceneIndex = m_scenes.ActiveIndex();
            if (auto* sc = m_scenes.Active()) {
                float p[3] = { m_camPos.x, m_camPos.y, m_camPos.z };
                float yaw = m_camYaw, pitch = m_camPitch;
                if (sc->PreferredCamera(p, yaw, pitch)) {
                    m_camPos = { p[0], p[1], p[2] };
                    m_camYaw = yaw; m_camPitch = pitch;
                }
            }
        }

        // When the cursor is over an ImGui window (or typing in it), let ImGui
        // have the input instead of driving the camera with it.
        const bool uiMouse    = IsUICapturingMouse();
        const bool uiKeyboard = IsUICapturingKeyboard();

        // Hold right mouse button to enter FPS look mode; release to free cursor.
        if (!uiMouse && input.IsMouseButtonDown(SGE::MouseButton::Right)) {
            if (!input.IsCursorLocked())
                input.LockCursor(GetWindow().GetHandle());
            XMFLOAT2 delta = input.GetMouseDelta();
            m_camYaw   += delta.x * 0.002f;
            m_camPitch -= delta.y * 0.002f;
            constexpr float kLimit = XM_PIDIV2 - 0.01f;
            if (m_camPitch >  kLimit) m_camPitch =  kLimit;
            if (m_camPitch < -kLimit) m_camPitch = -kLimit;
        } else if (input.IsCursorLocked()) {
            input.UnlockCursor();
        }

        // Controller right stick look
        XMFLOAT2 rStick = input.GetRightStick(0);
        m_camYaw   += rStick.x * 2.0f * dt;
        m_camPitch -= rStick.y * 2.0f * dt;
        {
            constexpr float kLimit = XM_PIDIV2 - 0.01f;
            if (m_camPitch >  kLimit) m_camPitch =  kLimit;
            if (m_camPitch < -kLimit) m_camPitch = -kLimit;
        }

        // Camera basis vectors from yaw/pitch
        float cp = cosf(m_camPitch), sp = sinf(m_camPitch);
        float cy = cosf(m_camYaw),   sy = sinf(m_camYaw);
        XMVECTOR forward = XMVectorSet(sy * cp, sp, cy * cp, 0.0f);
        XMVECTOR right   = XMVectorSet(cy, 0.0f, -sy, 0.0f);
        XMVECTOR worldUp = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
        XMVECTOR pos     = XMLoadFloat3(&m_camPos);

        float moveSpeed = m_camSpeed * dt;

        if (input.IsKeyJustPressed(SGE::Key::Home)) {
            m_camPos   = { 0.0f, 2.0f, -8.0f };
            m_camYaw   = 0.0f;
            m_camPitch = 0.0f;
            pos        = XMLoadFloat3(&m_camPos);
        }

        if (!uiKeyboard) {
            if (input.IsKeyDown(SGE::Key::W)) pos = XMVectorAdd(pos, XMVectorScale(forward,  moveSpeed));
            if (input.IsKeyDown(SGE::Key::S)) pos = XMVectorAdd(pos, XMVectorScale(forward, -moveSpeed));
            if (input.IsKeyDown(SGE::Key::D)) pos = XMVectorAdd(pos, XMVectorScale(right,    moveSpeed));
            if (input.IsKeyDown(SGE::Key::A)) pos = XMVectorAdd(pos, XMVectorScale(right,   -moveSpeed));
            if (input.IsKeyDown(SGE::Key::E)) pos = XMVectorAdd(pos, XMVectorScale(worldUp,  moveSpeed));
            if (input.IsKeyDown(SGE::Key::Q)) pos = XMVectorAdd(pos, XMVectorScale(worldUp, -moveSpeed));
        }

        XMFLOAT2 lStick = input.GetLeftStick(0);
        pos = XMVectorAdd(pos, XMVectorScale(forward, lStick.y * moveSpeed));
        pos = XMVectorAdd(pos, XMVectorScale(right,   lStick.x * moveSpeed));

        XMStoreFloat3(&m_camPos, pos);

        // Recompute view from updated camera position and look direction
        XMVECTOR eye    = XMLoadFloat3(&m_camPos);
        XMVECTOR target = XMVectorAdd(eye, forward);
        XMFLOAT3 eyeF, targetF;
        XMStoreFloat3(&eyeF,    eye);
        XMStoreFloat3(&targetF, target);
        m_camera.LookAt(eyeF, targetF);

        // Set pipeline-wide state once (shared across every object and scene),
        // then reset this frame's arena region. RenderSystem deliberately does NOT
        // touch the root signature / PSO / topology, so we set them here.
        uint32_t frame = GetRenderer().GetFrameIndex();
        auto*    cmd   = GetRenderer().GetCommandList();
        cmd->SetGraphicsRootSignature(m_rootSig.Get());
        cmd->SetPipelineState(m_pipeline.Get());
        cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        m_objectCB.BeginFrame(frame);

        // Update then draw the active demo scene through the shared render path.
        SGE::DemoContext ctx = BuildContext(dt);
        m_scenes.Update(ctx);
        m_scenes.Render(ctx);
    }

    void OnImGui() override
    {
        ImGui::SetNextWindowPos({ 10, 10 }, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize({ 320, 0 }, ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Singularity"))
        {
            ImGui::Text("%.1f FPS  (%.2f ms)", ImGui::GetIO().Framerate,
                        1000.0f / ImGui::GetIO().Framerate);
            ImGui::Separator();
            ImGui::Text("Camera  %.2f, %.2f, %.2f", m_camPos.x, m_camPos.y, m_camPos.z);
            ImGui::SliderFloat("Move speed", &m_camSpeed, 1.0f, 30.0f, "%.1f");
            ImGui::Separator();
            ImGui::TextDisabled("Hold RMB to look | WASD/QE to fly | Home to reset");
            ImGui::Checkbox("ImGui demo window", &m_showDemo);
        }
        ImGui::End();

        // Scene-switcher window + the active scene's own controls/explainer.
        m_scenes.OnImGui(BuildContext(GetDeltaTime()));

        if (m_showDemo)
            ImGui::ShowDemoWindow(&m_showDemo);
    }

    void OnResize(uint32_t w, uint32_t h) override
    {
        if (w == 0 || h == 0) return;
        m_camera.Perspective(DirectX::XMConvertToRadians(60.0f),
                             float(w) / float(h), 0.1f, 100.0f);
    }

    void OnShutdown() override
    {
        // Constraint 6: the GPU may still be reading the upload arena / mesh from
        // the last in-flight frame. Drain it before releasing any GPU resource.
        // (Renderer::Shutdown also waits, but it runs *after* this hook.)
        GetRenderer().WaitForGPU();

        GetInput().UnlockCursor();
        m_shaderLib.Shutdown();
        m_objectCB.Shutdown();
        m_mesh.Reset();
        m_sphereMesh.Reset();
        m_pipeline.Reset();
        m_rootSig.Reset();
    }

private:
    // Builds the per-frame context handed to scenes. Pointers reference members
    // owned by this app; valid for the duration of the scene call.
    SGE::DemoContext BuildContext(float dt)
    {
        SGE::DemoContext ctx;
        ctx.device            = GetRenderer().GetDevice();
        ctx.cmd               = GetRenderer().GetCommandList();
        ctx.input             = &GetInput();
        ctx.camera            = &m_camera;
        ctx.objectCB          = &m_objectCB;
        ctx.renderSystem      = &m_renderSystem;
        ctx.renderer          = &GetRenderer();
        ctx.rootParamIndexCBV = 0; // b0 root CBV slot from OnStartup
        ctx.dt                = dt;
        ctx.cameraPos[0]      = m_camPos.x;
        ctx.cameraPos[1]      = m_camPos.y;
        ctx.cameraPos[2]      = m_camPos.z;
        return ctx;
    }

    SGE::ShaderLibrary       m_shaderLib;
    SGE::RootSignature       m_rootSig;
    SGE::GraphicsPipeline    m_pipeline;
    SGE::DynamicUploadBuffer m_objectCB;     // per-object constant arena (shared)
    SGE::Mesh                m_mesh;          // shared cube geometry (owned here)
    SGE::Mesh                m_sphereMesh;    // procedural unit sphere (physics scenes)
    SGE::Camera              m_camera;
    SGE::RenderSystem        m_renderSystem;  // draws Transform+Mesh entities
    SGE::SceneManager        m_scenes;        // owns + switches the demo scenes

    DirectX::XMFLOAT3 m_camPos    = { 0.0f, 2.0f, -8.0f };
    float             m_camYaw    = 0.0f;
    float             m_camPitch  = 0.0f;
    float             m_camSpeed  = 5.0f;
    bool              m_showDemo  = false;
    int               m_lastSceneIndex = -1; // detects scene switches to apply PreferredCamera
};

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int)
{
    SandboxApp app;

    if (!app.Initialize(hInstance, L"Singularity — Scene Switcher", 1280, 720))
        return 1;

    app.Run();
    app.Shutdown();
    return 0;
}
