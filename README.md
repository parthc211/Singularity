# Singularity Engine

A from-scratch **DirectX 12** rendering & systems engine written in modern **C++20**, built as both a learning vehicle for explicit GPU APIs and a reusable, showcase-oriented foundation. Every core system — the SIMD math library, the CPU and GPU allocators, the renderer core, the ECS — is hand-written rather than pulled from a library, and each graphics technique ships as a self-contained, switchable demo scene with live ImGui controls.

> **Namespace:** `SGE::` · **Toolchain:** CMake + MSVC (Visual Studio 2022, x64) · **Shader model:** SM 6.0 via runtime DXC compilation

---

## Highlights

The Sandbox app is a single executable hosting **16 interactive demos**, selectable at runtime from an ImGui dropdown:

| Demo | What it shows |
|---|---|
| **Cube Grid / Single Cube** | Forward rendering, ECS-driven draws, per-object constant arena |
| **GPU Heap Allocator** | Custom `ID3D12Heap` + first-fit free-list handing out *placed* resources, with a live heap map |
| **Deferred Rendering** | 3-target MRT G-buffer + fullscreen lighting pass, up to 64 moving point lights |
| **SIMD Math** | Hand-written SSE/AVX `Vec4`/`Mat4`/`Quat`, validated 1:1 against DirectXMath, with benchmarks |
| **CPU Allocators** | Arena / Stack / Pool / Free-list allocators with live memory maps and vs-`malloc` timings |
| **Tessellation** | Hull/domain-shader terrain, distance-based crack-free LOD, procedural fBm displacement |
| **Shadow Mapping** | Directional shadow map with hardware PCF (comparison sampler) |
| **HDR + Bloom** | HDR render target → bright-pass → separable Gaussian blur → ACES tonemap |
| **SSAO** | Screen-space ambient occlusion over the deferred G-buffer |
| **Texture / Normal Mapping** | sRGB albedo + tangent-space normal maps via a per-vertex TBN basis (tangents baked by the OBJ loader), CPU-mipped WIC textures with a procedural brick fallback |
| **Cascaded Shadow Maps** | 4-cascade CSM with bounding-sphere fit + texel snapping (stable, shimmer-free) |
| **Job System** | Work-stealing thread pool recording DX12 command lists in parallel, with a single/multi A/B toggle |
| **Physics: Stacks & Rain** | Hand-written rigid-body solver — stable 10-box stacks, body rain, live tunables, contact overlay |
| **Physics: Joints** | Ball / hinge / distance joints: hanging chain, trapdoor, wrecking ball vs box stack |
| **Skeletal Animation** | Hand-written glTF 2.0 loader (JSON parser, GLB, skins, clips) + keyframe sampling, pose blending and GPU skinning via a bone-palette CBV, with an x-ray skeleton overlay |

## Engine systems

- **Renderer (DX12):** device/swap-chain/command-context bootstrap, root signatures, PSO management (MRT + tessellation capable), depth buffers, reusable render-target helpers (`GBuffer`, `ShadowMap`, `CascadedShadowMap`, `RenderTexture`), and sampled textures (`Texture2D`: WIC decoding, sRGB-aware CPU mip generation, staged DEFAULT-heap upload).
- **Shaders:** compiled at runtime with **DXC** (SM 6.0); input layouts auto-reflected from the vertex shader; a `ShaderLibrary` hot-reloads shaders on file change.
- **Memory:** a GPU placed-resource allocator (`GpuHeap`) that real mesh vertex/index buffers sub-allocate from, plus four hand-written CPU allocators.
- **Math:** SIMD `Vec4`/`Mat4`/`Quat` (SSE + an AVX SoA path), conventions matching DirectXMath for free interop.
- **Jobs:** a work-stealing thread pool (`JobSystem`) — per-worker deques, LIFO owner pop / FIFO stealing, a helping `Wait()` — used by threaded command recording and the physics step.
- **Physics:** a from-scratch 3D rigid-body engine (`Physics::PhysicsWorld`) built entirely on the engine's own SIMD math: sphere/box/plane colliders, SAT box-box narrowphase with Sutherland–Hodgman clipping, a warm-started sequential-impulse solver (accumulated & clamped impulses, friction cones, restitution, Baumgarte + speculative contacts), persistent manifolds, a spatial-hash broadphase, and ball/hinge/distance joints with energy-neutral NGS position correction — stepped at fixed 120 Hz substeps, single-threaded or JobSystem-parallel with bit-identical trajectories.
- **Animation:** a skeletal-animation stack built on the engine's SIMD math — `Skeleton` (flat parent-before-child joint arrays, inverse binds), sparse keyframe `AnimationClip`s, cursor-cached sampling (slerp), pose blending (nlerp), single-pass pose propagation and bone-palette generation, GPU-skinned in the vertex shader (4 weights, 256-joint palette CBV).
- **Assets:** a hand-written **glTF 2.0 loader** (own JSON parser, GLB container, accessors, skins, animations, base64/external buffers) with right-handed→left-handed conversion at import, feeding a format-agnostic `SkeletalMeshData`; plus the OBJ loader with tangent generation and WIC image loading.
- **Scene:** a sparse-set **ECS** (`World` / `Entity` / `SparseSet`) with a `RenderSystem`, plus a `DemoScene` framework and a `SceneManager` with safe deferred scene switching.
- **UI:** Dear ImGui (Win32 + DX12 backends) for per-demo controls and the scene switcher.

## Build & run

```powershell
# Configure (first time, or after CMakeLists changes)
cmake -S . -B build -G "Visual Studio 17 2022" -A x64

# Build
cmake --build build --config Debug

# Run
build\bin\Debug\Sandbox.exe
```

`dxil.dll` and `dxcompiler.dll` are copied next to the executable post-build (required for SM 6 PSO validation), along with the `Shaders/` and `Assets/` directories.

> **Debug builds** enable the DX12 debug layer + GPU-Based Validation (`SGE_DEBUG`); the first frame is slow as a result. For representative allocator/SIMD benchmark numbers, build **Release** — Debug disables inlining and under-represents SIMD.

## Project layout

```
Singularity/
├── CMakeLists.txt      # DXC/DXIL discovery, ImGui (FetchContent), subdirs
├── Engine/             # static lib "SingularityEngine"
│   └── src/
│       ├── Core/       # Logger, Window, Application, InputSystem, Camera, Transform
│       ├── Jobs/       # work-stealing JobSystem
│       ├── Math/       # SimdMath, ScalarMath, benchmarks
│       ├── Memory/     # Arena / Stack / Pool / FreeList allocators
│       ├── Physics/    # rigid bodies, narrowphase, solver, broadphase, joints
│       ├── Renderer/   # Renderer, ShaderLibrary, Mesh, + DX12/ and DXC/ backends
│       ├── Scene/      # ECS (World/Entity/SparseSet), RenderSystem, SceneManager
│       └── UI/         # ImGuiLayer
└── Sandbox/            # Win32 executable
    ├── main.cpp        # shared GPU infra + scene registry + fly-camera
    ├── Scenes/         # the 15 demo scenes
    ├── Shaders/        # HLSL (forward, deferred, tessellation, shadows, post)
    └── Assets/         # cube.obj
```

## Roadmap

**Done:** tooling · SIMD math · CPU allocators · DX12 bootstrap · GPU-heap allocator · ECS · deferred rendering · tessellation · shadow mapping · cascaded shadow maps · HDR/bloom · SSAO · ImGui scene switcher · job system (work-stealing pool → parallel command-list recording) · rigid-body physics (impulse solver, stacking, joints) · texture & normal mapping (WIC loader, `Texture2D`, baked tangents) · skeletal animation (hand-written glTF 2.0 loader, keyframe sampling, GPU skinning).

**Sample assets:** `CesiumMan.glb` (© Cesium, [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)) and `Fox.glb` (© PixelMannen/tomkranis, CC-BY 4.0) from the [Khronos glTF-Sample-Assets](https://github.com/KhronosGroup/glTF-Sample-Assets) repository; `SimpleSkin` from the glTF 2.0 specification tutorials (CC-BY 4.0).

**Next:** a renderer abstraction layer · portfolio polish.

**Physics stretch ideas (documented, not built):** sleeping/islands · split-impulse contacts · capsule colliders · render-state interpolation between fixed steps · gyroscopic term in the integrator.

---

*Built as a learning project to understand DirectX 12 internals — memory management, SIMD, deferred rendering, tessellation, shadows, and rigid-body simulation — from the ground up.*
