# Singularity Engine

A from-scratch **DirectX 12** rendering & systems engine written in modern **C++20**, built as both a learning vehicle for explicit GPU APIs and a reusable, showcase-oriented foundation. Every core system — the SIMD math library, the CPU and GPU allocators, the renderer core, the ECS — is hand-written rather than pulled from a library, and each graphics technique ships as a self-contained, switchable demo scene with live ImGui controls.

> **Namespace:** `SGE::` · **Toolchain:** CMake + MSVC (Visual Studio 2022, x64) · **Shader model:** SM 6.0–6.6 via runtime DXC compilation

---

## Highlights

The Sandbox app is a single executable hosting **27 interactive demos**, selectable at runtime from an ImGui dropdown:

| Demo | What it shows |
|---|---|
| **Cube Grid / Single Cube** | Forward rendering, ECS-driven draws, per-object constant arena |
| **GPU Heap Allocator** | Custom `ID3D12Heap` + first-fit free-list handing out *placed* resources, with a live heap map |
| **Deferred Rendering** | 3-target MRT G-buffer + fullscreen lighting pass, up to 64 moving point lights (barriers scheduled by the render graph) |
| **Clustered Shading** | Froxel (view-frustum voxel) light culling: compute builds per-froxel AABBs, a second compute pass atomic-appends hundreds of moving lights into per-froxel lists, and the forward pass loops only each pixel's froxel lights — with a lights-per-froxel heatmap view |
| **Render Graph** | Declarative multi-pass frame: passes declare per-resource states, the graph derives/batches all barriers and reference-count culls unused passes — with a live panel of the compiled schedule, per-pass barriers, and culling |
| **Bindless** | SM 6.6 `ResourceDescriptorHeap[]`: a grid of cubes, each a different texture, drawn through one root signature with no per-draw descriptor tables — every texture in one heap, each cube passing only an integer index. Foundation for GPU-driven rendering and ray tracing |
| **SIMD Math** | Hand-written SSE/AVX `Vec4`/`Mat4`/`Quat`, validated 1:1 against DirectXMath, with benchmarks |
| **CPU Allocators** | Arena / Stack / Pool / Free-list allocators with live memory maps and vs-`malloc` timings |
| **Tessellation** | Hull/domain-shader terrain, distance-based crack-free LOD, procedural fBm displacement |
| **Shadow Mapping** | Directional shadow map with hardware PCF (comparison sampler) |
| **HDR + Bloom** | HDR render target → bright-pass → separable Gaussian blur → ACES tonemap |
| **Post-Processing** | A composable post stack over an HDR target, run through the render graph: compute eye-adaptation → bloom → a composite doing ACES tonemap, colour grade, vignette, chromatic aberration and film grain — every effect a live toggle |
| **Temporal AA** | Sub-pixel jittered projection + a motion-vector target, resolved by reprojecting the previous frame's result and neighbourhood-clamping history to kill ghosting — accumulated jitter = supersampled edges, with an A/B toggle vs. raw aliasing |
| **SSAO** | Screen-space ambient occlusion over the deferred G-buffer |
| **Screen-Space Reflections** | View-space G-buffer (colour+reflectivity / normal / position) + a resolve pass that reflects each view ray and ray-marches the depth buffer, compositing hits by reflectivity × Fresnel × edge fade — a reflective floor mirroring the objects, with the on-screen-only limitation visible |
| **Order-Independent Transparency** | Weighted-blended OIT: overlapping translucent spheres accumulated into additive-colour + multiplicative-revealage targets (independent per-target blend) with depth-weighted weighting, composited over the opaque scene — correct at any draw order, with a naive-alpha A/B toggle |
| **Ray Tracing (DXR)** | Hardware ray tracing of the shared cube + sphere meshes (BLAS per mesh, TLAS of instances) + a ray-tracing pipeline state object + shader binding table driving `DispatchRays` — primary rays, hard shadow rays, and recursive reflection rays that trace the *real* scene (so they show off-screen geometry SSR can't); the hit shader fetches vertices bindlessly |
| **GPU-Driven Rendering** | 14k instances of the shared cube mesh frustum-culled by a compute pass that atomic-compacts surviving `DrawIndexedInstanced` commands into an indirect-args buffer, drawn by a single `ExecuteIndirect` — zero per-object CPU draws, with a live drawn/total read-back |
| **Mesh Shaders** | A dense sphere partitioned into meshlets (≤64 verts / ≤124 tris); one `DispatchMesh` runs an amplification shader that frustum- and normal-cone-culls whole clusters, then dispatches survivors as mesh-shader groups that emit triangles directly — no input assembler, no index buffer. A toggle tints each meshlet so the partition is visible |
| **Texture / Normal / PBR Mapping** | The material pipeline: sRGB albedo, tangent-space normal maps (TBN from baked tangents), parallax occlusion mapping with contact shadows (height in the normal map's alpha), and an ORM map driving a Cook-Torrance GGX BRDF; CPU-mipped WIC textures with a procedural brick fallback |
| **Image-Based Lighting** | Compute-baked IBL from a procedural HDR sky: environment cube → irradiance cube (diffuse) + prefiltered-mip cube (specular) + BRDF LUT (split-sum). A roughness×metalness sphere grid lit entirely from the environment, under a skybox |
| **Cascaded Shadow Maps** | 4-cascade CSM with bounding-sphere fit + texel snapping (stable, shimmer-free) |
| **Job System** | Work-stealing thread pool recording DX12 command lists in parallel, with a single/multi A/B toggle |
| **Physics: Stacks & Rain** | Hand-written rigid-body solver — stable 10-box stacks, body rain, live tunables, contact overlay |
| **Physics: Joints** | Ball / hinge / distance joints: hanging chain, trapdoor, wrecking ball vs box stack |
| **Skeletal Animation** | Hand-written glTF 2.0 loader (JSON parser, GLB, skins, clips) + keyframe sampling, pose blending and GPU skinning via a bone-palette CBV, with an x-ray skeleton overlay |

## Engine systems

- **Renderer (DX12):** device/swap-chain/command-context bootstrap, root signatures, PSO management (MRT + tessellation capable), depth buffers, reusable render-target helpers (`GBuffer`, `ShadowMap`, `CascadedShadowMap`, `RenderTexture`), and sampled textures (`Texture2D`: WIC decoding, sRGB-aware CPU mip generation, staged DEFAULT-heap upload).
- **Geometry resource:** `Mesh` uploads a vertex/index pair once into GPU-local (DEFAULT-heap) memory via a staged copy, ending in `GENERIC_READ`, and exposes its buffer views, GPU addresses, counts, stride and formats — so one asset (e.g. `cube.obj`) feeds the forward/deferred raster path, GPU-driven `ExecuteIndirect`, and DXR acceleration-structure builds + bindless hit-shader vertex fetch, instead of each path hand-authoring its own geometry.
- **Bindless:** SM 6.6 `ResourceDescriptorHeap[]` support — `GraphicsDevice` detects it, `BindlessHeap` is a shader-visible CBV/SRV/UAV heap with a persistent index allocator, and a directly-indexed root signature lets shaders fetch any resource by integer with no per-draw descriptor tables (the basis for GPU-driven rendering and ray tracing).
- **GPU-driven rendering:** a cull-and-draw-indirect path (`GpuDrivenScene`) — a compute pass tests each instance's bounding sphere against the six frustum planes and atomic-appends a `DrawIndexedInstanced` command (plus a per-draw root constant) into an indirect buffer, which one `ExecuteIndirect` renders through a command signature; a counter read-back reports how many survived. It draws instances of the shared `cube.obj` mesh — the same geometry resource the forward path uses — bound straight from the mesh's vertex/index views.
- **Ray tracing (DXR):** a hardware ray tracer (`RtScene`) — the shared cube + sphere meshes are built into bottom-level acceleration structures straight from their VRAM vertex/index buffers, then instanced by a top-level AS with per-instance transforms and materials; a ray-tracing pipeline state object (ray-gen / closest-hit / two miss shaders, compiled as a `lib_6_6` DXIL library) and a shader binding table drive `DispatchRays`, tracing primary, hard-shadow and recursive reflection rays against the real acceleration structure. The single closest-hit shader fetches the hit mesh's vertices bindlessly (`ResourceDescriptorHeap[]`, indexed per-instance), so one hit group shades every instance. `GraphicsDevice` detects raytracing support and the DXC path gained `lib_6_*` compilation.
- **Mesh shaders:** the amplification→mesh→pixel geometry front end (`MeshShaderScene`) — a CPU meshlet builder (`BuildMeshlets`) partitions a mesh into clusters (≤64 verts / ≤124 tris) with a bounding sphere + normal cone each, and a stream-based mesh-shader PSO (`MeshPipeline`, via `ID3D12Device2::CreatePipelineState`) runs one `DispatchMesh`: an amplification shader frustum- and normal-cone-culls whole meshlets and dispatches only the survivors, which a mesh shader expands into triangles with no input assembler. `GraphicsDevice` detects mesh-shader support and the DXC path compiles `as_6_5`/`ms_6_5`.
- **Order-independent transparency:** weighted-blended OIT (`OitScene`) — transparent geometry accumulated into an additive colour buffer and a multiplicative revealage buffer via independent per-target blend states, depth-weighted so nearer surfaces dominate, then resolved over the opaque image; needs no sorting. Added blend-state and depth-test-without-write support to `GraphicsPipeline`.
- **Screen-space reflections:** a view-space G-buffer + ray-march resolve (`SsrScene`) — reflects each pixel's view ray, marches it through the stored depth/position with an accelerating step and binary refinement, and composites the hit colour by reflectivity, Fresnel and a screen-edge fade.
- **Clustered shading:** a froxel light-culling path (`ClusteredScene`) — a compute pass builds view-space froxel AABBs, a second atomic-appends overlapping lights into per-froxel index lists (bound as structured buffers via root descriptors), and a clustered forward pass reconstructs each pixel's froxel and loops only its lights, so cost tracks local light density rather than total count.
- **Temporal AA:** a jittered-projection + motion-vector (`TaaScene`) pipeline — geometry writes colour and screen-space velocity (from un-jittered clip), and a resolve pass reprojects and neighbourhood-clamps a ping-ponged history buffer, so sub-pixel jitter accumulates into supersampled, stable edges.
- **Post-processing:** a composable chain expressed as render-graph passes over an HDR target — a compute eye-adaptation pass (geometric-mean scene luminance adapted over time into a persistent 1×1 target), bloom (bright-pass + separable Gaussian ping-pong), and a composite applying ACES tonemap, analytic colour grading, vignette, chromatic aberration and film grain.
- **Render graph:** a per-frame declarative pass scheduler (`RenderGraph`) — passes declare the state they need each resource in, and the graph derives every barrier by diffing a cross-frame state cache (batched per pass) and reference-count culls passes whose outputs go unread, so barriers stop being hand-written per scene. Scheduling + culling are implemented (transient-resource memory aliasing is the planned Phase 2).
- **Shaders:** compiled at runtime with **DXC** (SM 6.0 for most stages; mesh/amplification shaders at SM 6.5, the bindless + DXR libraries at SM 6.6); input layouts auto-reflected from the vertex shader; a `ShaderLibrary` hot-reloads shaders on file change.
- **Memory:** a GPU placed-resource allocator (`GpuHeap`) that skinned-mesh and tessellation vertex/index buffers sub-allocate from (static `Mesh` buffers get their own staged DEFAULT-heap home), plus four hand-written CPU allocators.
- **Math:** SIMD `Vec4`/`Mat4`/`Quat` (SSE + an AVX SoA path), conventions matching DirectXMath for free interop.
- **Jobs:** a work-stealing thread pool (`JobSystem`) — per-worker deques, LIFO owner pop / FIFO stealing, a helping `Wait()` — used by threaded command recording and the physics step.
- **IBL:** `IblEnvironment` bakes image-based lighting from a procedural HDR sky via compute — environment cube → cosine-convolved irradiance cube (diffuse) + GGX-prefiltered mip cube (specular) + a split-sum BRDF LUT — on a self-contained blocking command list, feeding a real ambient term into the PBR BRDF.
- **Profiling:** `GpuProfiler` times regions with DX12 timestamp queries (per-frame-slot readback resolved ~`FrameCount` frames later, after the fence wait, so no stall); the render graph auto-wraps every pass, so per-pass GPU cost is free. `CpuProfiler` adds nested RAII CPU scope timers. Both feed a live ImGui overlay (a `Scene ▸ pass` tree with millisecond timings).
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
│       ├── Profiling/  # GpuProfiler (timestamp queries), CpuProfiler
│       ├── Renderer/   # Renderer, ShaderLibrary, Mesh/SkinnedMesh/Meshlet, ImageLoader, + DX12/, DXC/, RenderGraph/, IBL/
│       ├── Scene/      # ECS (World/Entity/SparseSet), RenderSystem, SceneManager
│       └── UI/         # ImGuiLayer
└── Sandbox/            # Win32 executable
    ├── main.cpp        # shared GPU infra + scene registry + fly-camera
    ├── Scenes/         # the 27 demo scenes
    ├── Shaders/        # HLSL (forward, deferred, tessellation, shadows, post, skinning, lines)
    └── Assets/         # cube.obj + Khronos glTF sample characters (see Roadmap)
```

## Roadmap

**Done:** tooling · SIMD math · CPU allocators · DX12 bootstrap · GPU-heap allocator · ECS · deferred rendering · tessellation · shadow mapping · cascaded shadow maps · HDR/bloom · SSAO · ImGui scene switcher · job system (work-stealing pool → parallel command-list recording) · rigid-body physics (impulse solver, stacking, joints) · texture & normal mapping (WIC loader, `Texture2D`, baked tangents) · skeletal animation (hand-written glTF 2.0 loader, keyframe sampling, GPU skinning) · renderer abstraction layer (`RootSignatureBuilder`, `SrvHeap`, one-call constant/vertex binding, back-buffer pass restore — all scenes migrated).

**Sample assets:** `CesiumMan.glb` (© Cesium, [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)) and `Fox.glb` (© PixelMannen/tomkranis, CC-BY 4.0) from the [Khronos glTF-Sample-Assets](https://github.com/KhronosGroup/glTF-Sample-Assets) repository; `SimpleSkin` from the glTF 2.0 specification tutorials (CC-BY 4.0); `KenneyCharacter.fbx` (Kenney assets, CC0) and `maya_game_sausage_wiggle.fbx` from the [ufbx test suite](https://github.com/ufbx/ufbx).

### TODO

**Portfolio polish (next up):**
- [ ] Screenshots/GIFs of the flagship demos in this README (deferred lighting, CSM, physics stacks, skeletal animation crowd)
- [ ] Release-build benchmark numbers published here (SIMD vs DirectXMath, allocators vs `malloc`, serial-vs-JobSystem timings for physics / animation / command recording)
- [ ] Demo reel — a short video walking all 27 scenes

**Engine architecture (highest-leverage — turns the demo collection into a unified engine):**
- [~] Render graph (frame graph) — **scheduling + culling done**: passes declare per-resource read/write states (`RenderGraph::Import`/`AddPass`/`Execute`), the graph derives every barrier by diffing a cross-frame state cache (batched per pass) and reference-count culls passes whose outputs go unread. The `Render Graph` demo prints the live compiled schedule/barriers/culling; `DeferredScene` is migrated off manual `GBuffer::TransitionToX` calls. **Phase 2 (TODO):** graph-owned *transient* resources + lifetime-based memory aliasing (the `Import` API is the seam it slots into)
- [ ] Unified scene + visibility system — one world composing all techniques, backed by a BVH/octree with frustum + Hi-Z occlusion culling (renderer decides what's worth drawing instead of drawing everything)
- [ ] Reflection + serialization — compile-time C++ field reflection (à la `UPROPERTY`); the substrate for scene save/load, an auto-generated inspector, and scripting bindings

**Rendering — completing the modern rasterizer:**
- [~] IBL (image-based lighting) — **done:** compute-baked irradiance cube + prefiltered-mip env cube + BRDF LUT (split-sum) from a procedural HDR sky, driving a real ambient term in the `Image-Based Lighting` demo (replaces the F0 placeholder). `IblEnvironment` (`Engine/src/Renderer/IBL`) owns the bake; cube UAV writes added `RootSignatureBuilder::UavTable`/`Constants`. **TODO:** load real `.hdr`/`.exr` equirectangular environments (procedural sky is featureless, so reflections read as gradients); SH irradiance as a compact alternative; wire IBL ambient into the Normal-Map/PBR demo too
- [~] Clustered / tiled forward+ shading — **done:** a froxel grid (screen tiles × exponential depth slices), a compute AABB-build pass, a compute light-culling pass (sphere-vs-AABB, atomic-appended per-froxel index lists) and a clustered forward shade pass looping only each pixel's froxel lights, in the `Clustered Shading` demo (hundreds of lights + heatmap view). **TODO:** depth prepass to skip occluded shading; apply it to transparent geometry (its real advantage over deferred) once OIT lands
- [~] TAA + motion vectors — **done:** Halton-jittered projection + a screen-space motion-vector target (from un-jittered clip positions), resolved by reprojecting history and clamping to the 3×3 neighbourhood AABB, ping-ponged across frames, in the `Temporal AA` demo (A/B toggle vs. no-AA). **TODO:** per-object motion vectors for moving geometry (prev-model tracking), then reuse the velocity buffer for motion blur + a DoF node in the post stack, and temporal upsampling
- [~] Composable post-processing stack — **done:** a render-graph chain over an HDR target in the `Post-Processing` demo — compute **eye-adaptation** (auto-exposure), bloom (bright-pass + separable blur), then a composite doing ACES tonemap, analytic colour grade (contrast/saturation/temperature), vignette, chromatic aberration and film grain, each a live toggle. **TODO:** depth of field, motion blur (needs motion vectors), 3D-LUT grading
- [~] SSR + reflection probes — **done:** screen-space reflections in the `Screen-Space Reflections` demo — a view-space G-buffer fed to a resolve pass that reflects each view ray, marches it through the depth buffer (accelerating step + binary refinement), and composites the hit by reflectivity × Fresnel × edge fade. **TODO:** a baked cubemap reflection-probe fallback for off-screen/edge misses; reuse the IBL prefiltered cube for rough reflections
- [x] Transparency / OIT — **weighted-blended OIT** (McGuire/Bavoil) in the `Order-Independent Transparency` demo: an opaque pass, then transparent geometry accumulated into additive-colour + multiplicative-revealage targets via independent per-target blend (depth-weighted), resolved over the opaque image — correct in any draw order, with a naive-alpha A/B toggle. Added `GraphicsPipeline` blend-state + depth-test-without-write support

**Rendering — frontiers:**
- [~] DXR (hardware ray tracing) — **done:** the full pipeline in the `Ray Tracing (DXR)` demo — BLAS/TLAS build, a ray-tracing pipeline state object + shader binding table, and `DispatchRays` with primary + hard-shadow + recursive reflection rays (reflections trace the real scene, fixing SSR's off-screen limitation); `GraphicsDevice` detects raytracing tier, DXC gained a `lib_6_6` path (the hit shader fetches vertices bindlessly). **TODO:** RT ambient occlusion; a hybrid pass (RT shadows/reflections over the rasterized G-buffer); a reference path tracer with accumulation
- [~] GPU-driven rendering — **done:** the `GPU-Driven Rendering` demo — a compute pass frustum-culls 14k instances and atomic-compacts the surviving draws into an indirect-args buffer, issued by one `ExecuteIndirect` (via a command signature carrying a per-draw root constant), with a read-back drawn/total HUD. **TODO:** Hi-Z occlusion culling on top of frustum; bindless per-instance materials/meshes so a single indirect batch spans many meshes
- [~] Mesh shaders — **done:** the `Mesh Shaders` demo — a CPU meshlet builder + a stream-based mesh-shader PSO run one `DispatchMesh` whose amplification shader frustum- and normal-cone-culls whole meshlets before a mesh shader expands the survivors (no input assembler / index buffer). **TODO:** per-meshlet LOD selection + a GPU-culled meshlet count read-back HUD
- [ ] Dynamic GI — DDGI irradiance probes (or voxel cone tracing / screen-space GI) for indirect bounce
- [ ] Volumetric fog / lighting — froxel in-scatter with shadow-map visibility, ray-marched into a screen texture (light shafts, god rays)
- [ ] Deferred decals — project albedo/normal/roughness into the G-buffer via box volumes (bullet holes, grunge, road markings)
- [ ] GPU particle system / VFX — compute-simulated particles (spawn/update/sort) rendered as instanced billboards; foundation of a VFX system
- [ ] Atmosphere & ocean — physically-based sky (Hillaire aerial perspective) and FFT ocean (Tessendorf) as set-pieces

**Low-level systems:**
- [~] Bindless / descriptor indexing — **done:** SM 6.6 `ResourceDescriptorHeap[]` infrastructure — `GraphicsDevice` feature detection (SM 6.6 + binding tier 3), a persistent `BindlessHeap` index allocator, and `RootSignatureBuilder::DirectlyIndexed`; the `Bindless` demo draws a differently-textured cube grid through one root signature with zero per-draw tables. **TODO:** migrate existing scenes to a single global bindless heap; index material/constant buffers (not just textures) — the GPU-driven/DXR prerequisite is in place
- [ ] Multi-queue — async compute queue (SSAO/particles/light-culling overlapping graphics) + copy queue for streaming uploads, fence-synced
- [ ] RHI abstraction — pull DX12 behind an `IRenderDevice`/`ICommandList` seam (design for a future Vulkan backend even if stubbed)
- [ ] Full GPU allocator + residency — extend `GpuHeap` to multi-heap, budget-aware, `MakeResident`/`Evict` residency management + defrag (à la D3D12MA)
- [ ] CPU task graph — dependency DAG over the existing work-stealing `JobSystem` (animation → physics → culling → recording); fiber-based scheduling as the stretch
- [~] Profiler & instrumentation — **done:** GPU timestamp-query timing per render-graph pass (`GpuProfiler`, multi-frame-safe async read-back) + nested CPU scope timers (`CpuProfiler`) + a live ImGui overlay (Scene ▸ per-pass tree, ms each) and a per-pass GPU-ms column in the Render Graph demo. **TODO:** PIX marker regions (`PIXBeginEvent`) for capture-tool correlation
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
