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
| **Texture / Normal / PBR Mapping** | The material pipeline: sRGB albedo, tangent-space normal maps (TBN from baked tangents), parallax occlusion mapping with contact shadows (height in the normal map's alpha), and an ORM map driving a Cook-Torrance GGX BRDF; CPU-mipped WIC textures with a procedural brick fallback |
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
- **Physics:** a from-scratch 3D rigid-body engine (`Physics::PhysicsWorld`) built entirely on the engine's own SIMD math: sphere/capsule/box/plane colliders, SAT box-box narrowphase with Sutherland–Hodgman clipping, a warm-started sequential-impulse solver (accumulated & clamped impulses, friction cones, restitution, split-impulse position correction with a Baumgarte A/B, speculative contacts), persistent manifolds, a spatial-hash broadphase, island-based sleeping (displacement-anchor quietness, whole-island wake), and ball/hinge/distance joints with energy-neutral NGS position correction — stepped at fixed 120 Hz substeps, single-threaded or JobSystem-parallel with bit-identical trajectories.
- **Animation:** a skeletal-animation stack built on the engine's SIMD math — `Skeleton` (flat parent-before-child joint arrays, inverse binds), sparse keyframe `AnimationClip`s, cursor-cached sampling (slerp), pose blending (nlerp), single-pass pose propagation and bone-palette generation, GPU-skinned in the vertex shader (4 weights, 256-joint palette CBV).
- **Assets:** a hand-written **glTF 2.0 loader** (own JSON parser, GLB container, accessors, skins, animations, embedded textures, base64/external buffers) with right-handed→left-handed conversion at import, feeding a format-agnostic `SkeletalMeshData`; an **FBX front-end** over the vendored single-file [ufbx](https://github.com/ufbx/ufbx) library (MIT/Unlicense — plumbing, like ImGui) targeting the same intermediate, with 30 Hz clip baking from ufbx's world matrices; plus the OBJ loader with tangent generation and WIC image loading.
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
├── CMakeLists.txt      # DXC/DXIL discovery, imgui target, subdirs
├── Engine/             # static lib "SingularityEngine"
│   ├── third_party/    # vendored deps: imgui v1.91.5 (MIT), ufbx (MIT/Unlicense)
│   └── src/
│       ├── Animation/  # Skeleton, AnimationClip, sampling/blending/palette, AnimationPlayer
│       ├── Assets/     # hand-written JSON parser + glTF 2.0 loader (SkeletalMeshData)
│       ├── Core/       # Logger, Window, Application, InputSystem, Camera, Transform
│       ├── Jobs/       # work-stealing JobSystem
│       ├── Math/       # SimdMath, ScalarMath, benchmarks
│       ├── Memory/     # Arena / Stack / Pool / FreeList allocators
│       ├── Physics/    # rigid bodies, narrowphase, solver, broadphase, joints
│       ├── Renderer/   # Renderer, ShaderLibrary, Mesh/SkinnedMesh, ImageLoader, + DX12/ and DXC/
│       ├── Scene/      # ECS (World/Entity/SparseSet), RenderSystem, SceneManager
│       └── UI/         # ImGuiLayer
└── Sandbox/            # Win32 executable
    ├── main.cpp        # shared GPU infra + scene registry + fly-camera
    ├── Scenes/         # the 16 demo scenes
    ├── Shaders/        # HLSL (forward, deferred, tessellation, shadows, post, skinning, lines)
    └── Assets/         # cube.obj + Khronos glTF sample characters (see Roadmap)
```

## Roadmap

**Done:** tooling · SIMD math · CPU allocators · DX12 bootstrap · GPU-heap allocator · ECS · deferred rendering · tessellation · shadow mapping · cascaded shadow maps · HDR/bloom · SSAO · ImGui scene switcher · job system (work-stealing pool → parallel command-list recording) · rigid-body physics (impulse solver, stacking, joints) · texture & normal mapping (WIC loader, `Texture2D`, baked tangents) · skeletal animation (hand-written glTF 2.0 loader, keyframe sampling, GPU skinning) · renderer abstraction layer (`RootSignatureBuilder`, `SrvHeap`, one-call constant/vertex binding, back-buffer pass restore — all 16 scenes migrated).

**Sample assets:** `CesiumMan.glb` (© Cesium, [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)) and `Fox.glb` (© PixelMannen/tomkranis, CC-BY 4.0) from the [Khronos glTF-Sample-Assets](https://github.com/KhronosGroup/glTF-Sample-Assets) repository; `SimpleSkin` from the glTF 2.0 specification tutorials (CC-BY 4.0); `KenneyCharacter.fbx` (Kenney assets, CC0) and `maya_game_sausage_wiggle.fbx` from the [ufbx test suite](https://github.com/ufbx/ufbx).

### TODO

**Portfolio polish (next up):**
- [ ] Screenshots/GIFs of the flagship demos in this README (deferred lighting, CSM, physics stacks, skeletal animation crowd)
- [ ] Release-build benchmark numbers published here (SIMD vs DirectXMath, allocators vs `malloc`, serial-vs-JobSystem timings for physics / animation / command recording)
- [ ] Demo reel — a short video walking all 16 scenes

**Engine architecture (highest-leverage — turns the demo collection into a unified engine):**
- [ ] Render graph (frame graph) — passes declare resource reads/writes; the graph derives all barriers, culls unused passes, and aliases transient memory. Existing `GBuffer`/`ShadowMap`/`RenderTexture` become graph-allocated handles instead of per-scene objects
- [ ] Unified scene + visibility system — one world composing all techniques, backed by a BVH/octree with frustum + Hi-Z occlusion culling (renderer decides what's worth drawing instead of drawing everything)
- [ ] Reflection + serialization — compile-time C++ field reflection (à la `UPROPERTY`); the substrate for scene save/load, an auto-generated inspector, and scripting bindings

**Rendering — completing the modern rasterizer:**
- [ ] IBL (image-based lighting) — irradiance (SH) + prefiltered env cubemap + BRDF LUT (split-sum); the missing ambient half of the existing GGX BRDF (metals currently use an F0 placeholder)
- [ ] Clustered / tiled forward+ shading — froxel light culling for thousands of lights and lit transparency (removes the deferred 64-light cap)
- [ ] TAA + motion vectors — jittered temporal accumulation with history reprojection; motion vectors also drive motion blur, SSR, and temporal upsampling
- [ ] Composable post-processing stack — auto-exposure/eye adaptation, DoF, motion blur, 3D-LUT color grading, vignette/chromatic-aberration/film-grain; existing bloom becomes one node
- [ ] SSR + reflection probes — screen-space depth ray-march with baked cubemap probe fallback for off-screen/edge misses
- [ ] Transparency / OIT — weighted-blended OIT or a sorted forward transparent pass (everything is opaque today)

**Rendering — frontiers:**
- [ ] DXR (hardware ray tracing) — BLAS/TLAS + shader binding table + ray-gen/hit/miss; ship RT shadows → reflections/AO → reference path tracer
- [ ] GPU-driven rendering — `ExecuteIndirect` with GPU frustum/occlusion culling and draw-arg compaction (depends on bindless)
- [ ] Mesh shaders — meshlet clusters + amplification/mesh shader front-end with per-meshlet culling and LOD (Nanite-adjacent)
- [ ] Dynamic GI — DDGI irradiance probes (or voxel cone tracing / screen-space GI) for indirect bounce
- [ ] Volumetric fog / lighting — froxel in-scatter with shadow-map visibility, ray-marched into a screen texture (light shafts, god rays)
- [ ] Deferred decals — project albedo/normal/roughness into the G-buffer via box volumes (bullet holes, grunge, road markings)
- [ ] GPU particle system / VFX — compute-simulated particles (spawn/update/sort) rendered as instanced billboards; foundation of a VFX system
- [ ] Atmosphere & ocean — physically-based sky (Hillaire aerial perspective) and FFT ocean (Tessendorf) as set-pieces

**Low-level systems:**
- [ ] Bindless / descriptor indexing — SM 6.6 `ResourceDescriptorHeap`; materials index resources by integer (prerequisite for GPU-driven rendering and DXR)
- [ ] Multi-queue — async compute queue (SSAO/particles/light-culling overlapping graphics) + copy queue for streaming uploads, fence-synced
- [ ] RHI abstraction — pull DX12 behind an `IRenderDevice`/`ICommandList` seam (design for a future Vulkan backend even if stubbed)
- [ ] Full GPU allocator + residency — extend `GpuHeap` to multi-heap, budget-aware, `MakeResident`/`Evict` residency management + defrag (à la D3D12MA)
- [ ] CPU task graph — dependency DAG over the existing work-stealing `JobSystem` (animation → physics → culling → recording); fiber-based scheduling as the stretch
- [ ] Profiler & instrumentation — CPU scope timers + GPU timestamp queries per pass + PIX markers, surfaced as a frame overlay (pairs with the render graph)
- [ ] CVar / config / VFS — runtime console variables, config loading, and a virtual file system with pak packaging + async I/O
- [ ] Offline asset cooking — bake glTF/FBX to a near-`memcpy` binary format at build time; asset GUIDs + dependency database + hot-reload (replaces runtime parsing)
- [ ] Memory tracking — tagged allocations, per-subsystem budgets, leak detection over the existing custom allocators

**Core engine capabilities:**
- [ ] Editor — dockable UI: scene hierarchy, reflection-driven inspector, ImGuizmo transform gizmos, asset browser, drag-drop placement (depends on reflection + serialization)
- [ ] Transform hierarchy / scene graph — parent-child transforms with dirty-flag world-matrix propagation (parent-before-child, like the skeleton pass)
- [ ] Scene serialization + prefabs — save/load worlds to disk and instance reusable entity templates with per-instance overrides
- [ ] Animation graph — state machines + blend trees over the existing clip/blend runtime, plus IK (two-bone / FABRIK) and skeleton retargeting
- [ ] Audio — XAudio2/WASAPI mixer with a submix/bus graph and 3D spatialization (currently absent entirely)
- [ ] Input action mapping — named action/axis bindings over the existing raw keyboard/mouse/XInput, remappable from config
- [ ] Scripting / gameplay layer — hot-reloadable native gameplay modules or embedded Lua/C# driving reflected components
- [ ] Navigation — Recast-style navmesh bake + A* pathfinding with local avoidance

**Rendering & assets:**
- [x] glTF-embedded texture extraction — decode GLB PNG buffers through the WIC loader so characters render textured
- [x] `ufbx` front-end feeding the same `SkeletalMeshData` — any `.fbx` dropped into `Assets/` (e.g. a Mixamo download) appears in the Skeletal Animation character list
- [x] Parallax occlusion mapping — height-field ray-march in tangent space (height in the normal map's alpha) with contact shadows, in the Texture / Normal Mapping demo
- [x] PBR material maps — ORM texture (AO/roughness/metalness, glTF packing) driving a Cook-Torrance GGX BRDF in the Texture / Normal Mapping demo; image-based lighting remains a future step (metals use a small F0 ambient placeholder)
- [x] Exact CUBICSPLINE keyframe interpolation and sparse-accessor support in the glTF loader (STEP is now exact too; channels carry per-key Hermite tangents, sampled natively by the animation runtime)

**Animation:**
- [x] Additive blend layers and per-bone masks — a second clip's delta (vs its first frame) composed onto the base pose, weight-scaled and maskable to a joint subtree, in the Skeletal Animation demo
- [x] Animation events — named timeline markers on clips; `AnimationPlayer` reports crossings with exact loop-wrap/seek/clamp semantics, editable live in the Skeletal Animation demo
- [x] Root-motion extraction — horizontal travel stripped from the motion joint's track (vertical kept, cubic tangents adjusted) into a wrap-aware `RootMotion` delta the scene applies to instance transforms; bundled clips are authored in place, Mixamo drop-ins travel for real
- [x] GPU pose evaluation (compute) for very large crowds — clips baked to a structured buffer; one compute dispatch samples, nlerps, propagates the hierarchy and builds all palettes per instance; the crowd renders as a single instanced draw with `SV_InstanceID`-indexed palettes (up to 64×64 = 4096 characters, zero CPU pose cost)

**Physics:**
- [x] Sleeping/islands — union-find islands over contacts+joints; whole-island sleep on a displacement-from-anchor quietness criterion (immune to solver jitter), pair collection and solving skipped for sleeping islands, automatic wake on contact; deterministic serial/parallel, sleeping bodies render dimmed in the physics demo
- [x] Split-impulse contacts — penetration corrected by a pseudo-velocity position pass instead of Baumgarte bias (zero injected energy: a 0.2m overlap resolves at 0.00 m/s vs Baumgarte's 4.6 m/s launch, measured in PhysicsTests); Baumgarte kept as a live A/B toggle
- [x] Capsule colliders — segment+radius shape with four new narrowphase pairs (vs sphere/capsule/box/plane; two-contact manifolds for lying capsules, convex ternary-search closest-point vs boxes), correct solid-capsule inertia, procedural capsule mesh in the physics demo
- [x] Render-state interpolation between fixed steps — poses blended between the last two 120 Hz substeps by the accumulator fraction ("Fix Your Timestep"); purely cosmetic (determinism untouched), sleeping bodies bit-exact, A/B toggle in the physics demo
- [x] Gyroscopic term in the integrator — Euler's `ω × (Iω)` coupling solved implicitly (one Newton step in body space; explicit is unstable): anisotropic bodies precess and Dzhanibekov-flip (5 flips / 10 s measured, energy ratio 1.0000, zero flips without — all asserted in PhysicsTests), with a live A/B toggle and a "Dzhanibekov flip" spawner in the demo

---

*Built as a learning project to understand DirectX 12 internals — memory management, SIMD, deferred rendering, tessellation, shadows, rigid-body simulation, and skeletal animation — from the ground up.*
