#pragma once
#include <cstdint>

struct ID3D12Device;
struct ID3D12GraphicsCommandList;

namespace SGE {

class Camera;
class InputSystem;
class DynamicUploadBuffer;
class RenderSystem;
class Renderer;

// Everything a demo scene needs from the host app to update and draw itself,
// passed by value each frame. All pointers are non-owning and valid only for
// the duration of the call.
struct DemoContext {
    ID3D12Device*              device            = nullptr;
    ID3D12GraphicsCommandList* cmd               = nullptr;
    InputSystem*               input             = nullptr;
    const Camera*              camera            = nullptr;
    DynamicUploadBuffer*       objectCB          = nullptr; // per-object constant arena
    RenderSystem*              renderSystem      = nullptr;
    Renderer*                  renderer          = nullptr; // back buffer/depth/dims, WaitForGPU
    uint32_t                   rootParamIndexCBV = 0;       // b0 root-CBV slot
    float                      dt                = 0.0f;    // seconds since last frame
    float                      cameraPos[3]      = { 0, 0, 0 }; // world-space eye (for specular)
};

// A switchable, self-explaining demo. The SceneManager owns a list of these and
// activates one at a time. Each scene owns its own ECS World / state; shared GPU
// infrastructure (PSO, root sig, upload arena, camera, meshes) is injected via
// the DemoContext or the scene's constructor.
//
// OnImGui doubles as the showcase's per-demo explainer panel — this is the whole
// point of the switcher: every future graphics feature arrives as a selectable,
// documented scene rather than something bolted onto a single hard-coded app.
class DemoScene {
public:
    virtual ~DemoScene() = default;

    virtual const char* Name() const = 0;
    virtual const char* Description() const { return ""; }

    virtual void OnLoad(const DemoContext&)   {} // build entities / resources
    virtual void OnUnload()                   {} // release them when switched away
    virtual void OnUpdate(const DemoContext&) {} // per-frame logic (no draw recording)
    virtual void OnRender(const DemoContext&) {} // record draw calls
    virtual void OnImGui()                    {} // scene-specific controls + explainer

    // Optionally suggest an initial fly-camera pose applied when this scene
    // becomes active (e.g. terrain wants an elevated overview). Fill pos[0..2]
    // (world xyz), yaw, pitch (radians) and return true; return false to keep
    // the current camera.
    virtual bool PreferredCamera(float /*pos*/[3], float& /*yaw*/, float& /*pitch*/) const { return false; }
};

} // namespace SGE
