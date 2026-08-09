#pragma once
#include "Scene/DemoScene.h"
#include "Animation/Animation.h"
#include "Assets/GltfLoader.h"
#include "Jobs/JobSystem.h"
#include "Renderer/SkinnedMesh.h"
#include "Renderer/ShaderLibrary.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/DX12/SrvHeap.h"
#include "Renderer/DX12/Texture2D.h"

#include <DirectXMath.h>
#include <string>
#include <vector>

// Skeletal animation showcase: glTF characters (hand-written loader) skinned
// on the GPU. Every frame the CPU samples the active clip, propagates the
// joint hierarchy and builds a bone palette per character instance; the
// vertex shader blends 4 weighted palette matrices per vertex.
//
// Clip switches crossfade (both clips sampled, poses nlerp-blended over the
// fade). An NxN stress grid evaluates every instance's pose pipeline either
// serially or JobSystem-parallel — same A/B pattern as the physics and
// threaded-command-list demos, with both timings on screen.
class AnimationScene : public SGE::DemoScene {
public:
    const char* Name()        const override { return "Skeletal Animation"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override {
        pos[0] = 0.0f; pos[1] = 1.7f; pos[2] = -4.5f;
        yaw = 0.0f; pitch = -0.12f;
        return true;
    }

private:
    struct Character {
        std::string           File;
        std::string           Label;
        float                 Scale  = 0.0f;   // display scale; <= 0 = auto from skinned extents
        DirectX::XMFLOAT4     Color{ 1, 1, 1, 1 };  // tint; replaced by the material factor when textured
        SGE::SkeletalMeshData Data;
        SGE::SkinnedMesh      Mesh;
        SGE::Texture2D        Tex;              // glTF base-color texture, or 1x1 white
        bool                  HasTexture = false;
        bool                  Loaded     = false;
    };

    // One animated character in the grid: its own playback state (current +
    // fading-out previous clip) and its own pose/palette buffers, so parallel
    // evaluation writes strictly per-instance memory.
    struct Instance {
        SGE::Anim::AnimationPlayer        Player;
        SGE::Anim::AnimationPlayer        PrevPlayer;   // fading out
        float                             Fade = 1.0f;  // 1 = no crossfade running
        float                             TimeOffset = 0.0f;
        float                             X = 0.0f, Z = 0.0f;
        std::vector<SGE::Anim::JointPose> Pose, PrevPose;
        std::vector<SGE::Math::Mat4>      Globals, Palette;
    };

    bool BuildPipelines(const SGE::DemoContext& ctx);
    void RebuildInstances();               // grid size / character changed
    void SelectClip(int clip);             // same character: crossfades
    void EvaluateInstance(Instance& inst); // sample -> blend -> globals -> palette
    DirectX::XMMATRIX InstanceMatrix(const Instance& inst) const;

    std::vector<Character> m_chars;   // 2 glTF samples + every *.fbx found in Assets/
    int                    m_active    = 0;
    int                    m_clipIndex = 0;

    std::vector<Instance> m_instances;
    int                   m_gridN = 1;    // grid is N x N instances

    SGE::JobSystem           m_jobs;
    SGE::ShaderLibrary       m_shaders;
    SGE::RootSignature       m_skinRootSig;
    SGE::RootSignature       m_lineRootSig;
    SGE::GraphicsPipeline    m_skinPSO;
    SGE::GraphicsPipeline    m_gridPSO;   // depth-tested (ground grid)
    SGE::GraphicsPipeline    m_bonePSO;   // depth-off x-ray (skeleton overlay)
    SGE::DynamicUploadBuffer m_arena;     // per-frame bone palettes + line vertices
    SGE::SrvHeap             m_srvs;      // slot per character: base-color texture

    bool  m_playing      = true;
    float m_speed        = 1.0f;
    bool  m_loop         = true;
    float m_fadeDuration = 0.3f;   // seconds; 0 = hard cut
    bool  m_useJobs      = false;
    bool  m_showMesh     = true;
    bool  m_showSkeleton = true;
    bool  m_showGrid     = true;
    float m_charYaw      = 0.0f;   // radians
    float m_msSerial     = 0.0f;   // smoothed pose-pipeline ms, serial path
    float m_msJobs       = 0.0f;   // smoothed pose-pipeline ms, JobSystem path
    bool  m_ready        = false;
};
