#include "Renderer/Renderer.h"
#include "Core/Logger.h"

#include <vector>

namespace SGE {

bool Renderer::Initialize(HWND hwnd, uint32_t width, uint32_t height)
{
    try {
        if (!m_graphicsDevice.Initialize()) return false;

        if (!m_swapChain.Initialize(
                m_graphicsDevice.GetFactory(),
                m_graphicsDevice.GetCommandQueue(),
                m_graphicsDevice.GetDevice(),
                hwnd, width, height))
            return false;

        if (!m_commandContext.Initialize(m_graphicsDevice.GetDevice()))
            return false;

        SGE_THROW_IF_FAILED(m_graphicsDevice.GetDevice()->CreateFence(
            0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_fence)
        ));

        m_fenceEvent = CreateEventW(nullptr, FALSE, FALSE, nullptr);
        if (!m_fenceEvent)
            throw std::runtime_error("Failed to create fence event.");

        if (!m_depthBuffer.Create(m_graphicsDevice.GetDevice(), width, height))
            return false;

        // 64 MB UPLOAD heap for static geometry (mesh VB/IB). Placed resources are
        // carved from this by the GpuHeap free-list allocator instead of each mesh
        // owning a committed resource.
        if (!m_geometryHeap.Init(m_graphicsDevice.GetDevice(), 64ull * 1024 * 1024,
                                 D3D12_HEAP_TYPE_UPLOAD))
            return false;

        m_frameIndex = m_swapChain.GetCurrentBackBufferIndex();
        m_width      = width;
        m_height     = height;

        LogInfo("Renderer initialized.");
        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void Renderer::BeginFrame()
{
    m_frameIndex = m_swapChain.GetCurrentBackBufferIndex();

    // If the GPU hasn't finished this frame's previous work, wait for it.
    // This is what makes double-buffering safe: frame N waits for the GPU's
    // frame N-2 submission before reusing its command allocator.
    if (m_fence->GetCompletedValue() < m_fenceValues[m_frameIndex]) {
        SGE_THROW_IF_FAILED(m_fence->SetEventOnCompletion(m_fenceValues[m_frameIndex], m_fenceEvent));
        WaitForSingleObject(m_fenceEvent, INFINITE);
    }

    m_commandContext.Reset(m_frameIndex);

    // Back buffer starts in PRESENT state; transition to RENDER_TARGET before clearing/drawing.
    m_commandContext.ResourceBarrier(
        m_swapChain.GetCurrentBackBuffer(),
        D3D12_RESOURCE_STATE_PRESENT,
        D3D12_RESOURCE_STATE_RENDER_TARGET
    );

    constexpr float clearColor[] = { 0.392f, 0.584f, 0.929f, 1.0f }; // cornflower blue
    m_commandContext.ClearRenderTarget(m_swapChain.GetCurrentRTV(), clearColor);

    // Bind the back buffer + depth buffer, clear depth to 1.0.
    // Viewport/scissor are not sticky — must be re-issued every reset.
    D3D12_CPU_DESCRIPTOR_HANDLE rtv = m_swapChain.GetCurrentRTV();
    D3D12_CPU_DESCRIPTOR_HANDLE dsv = m_depthBuffer.GetDSV();
    m_commandContext.GetCommandList()->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);
    m_commandContext.GetCommandList()->OMSetRenderTargets(1, &rtv, FALSE, &dsv);

    D3D12_VIEWPORT vp = { 0.0f, 0.0f, float(m_width), float(m_height), 0.0f, 1.0f };
    D3D12_RECT scissor = { 0, 0, LONG(m_width), LONG(m_height) };
    m_commandContext.GetCommandList()->RSSetViewports(1, &vp);
    m_commandContext.GetCommandList()->RSSetScissorRects(1, &scissor);
}

void Renderer::EndFrame()
{
    // Must transition back to PRESENT before calling Present().
    m_commandContext.ResourceBarrier(
        m_swapChain.GetCurrentBackBuffer(),
        D3D12_RESOURCE_STATE_RENDER_TARGET,
        D3D12_RESOURCE_STATE_PRESENT
    );

    m_commandContext.Close();

    ID3D12CommandList* lists[] = { m_commandContext.GetCommandList() };
    m_graphicsDevice.GetCommandQueue()->ExecuteCommandLists(1, lists);

    m_swapChain.Present();

    // Tag this frame with a new fence value so BeginFrame can wait for it next time.
    m_fenceValues[m_frameIndex] = ++m_fenceCounter;
    SGE_THROW_IF_FAILED(m_graphicsDevice.GetCommandQueue()->Signal(m_fence.Get(), m_fenceValues[m_frameIndex]));
}

void Renderer::ExecuteFrameLists(ID3D12CommandList* const* lists, uint32_t count)
{
    // Flush the shared frame list so far (clear + RT setup from BeginFrame) so that,
    // on the single queue, the clear runs BEFORE the caller's parallel draw lists.
    m_commandContext.Close();

    std::vector<ID3D12CommandList*> all;
    all.reserve(count + 1);
    all.push_back(m_commandContext.GetCommandList());
    for (uint32_t i = 0; i < count; ++i)
        all.push_back(lists[i]);

    m_graphicsDevice.GetCommandQueue()->ExecuteCommandLists(UINT(all.size()), all.data());

    // Re-open the shared list to record the rest of the frame (ImGui, then
    // EndFrame's present barrier). A fresh list inherits no state, so re-bind the
    // back buffer + depth and the viewport/scissor for whatever draws next.
    m_commandContext.ReopenList(m_frameIndex);

    D3D12_CPU_DESCRIPTOR_HANDLE rtv = m_swapChain.GetCurrentRTV();
    D3D12_CPU_DESCRIPTOR_HANDLE dsv = m_depthBuffer.GetDSV();
    m_commandContext.GetCommandList()->OMSetRenderTargets(1, &rtv, FALSE, &dsv);

    D3D12_VIEWPORT vp = { 0.0f, 0.0f, float(m_width), float(m_height), 0.0f, 1.0f };
    D3D12_RECT scissor = { 0, 0, LONG(m_width), LONG(m_height) };
    m_commandContext.GetCommandList()->RSSetViewports(1, &vp);
    m_commandContext.GetCommandList()->RSSetScissorRects(1, &scissor);
}

void Renderer::WaitForGPU()
{
    uint64_t signalValue = ++m_fenceCounter;
    SGE_THROW_IF_FAILED(m_graphicsDevice.GetCommandQueue()->Signal(m_fence.Get(), signalValue));
    SGE_THROW_IF_FAILED(m_fence->SetEventOnCompletion(signalValue, m_fenceEvent));
    WaitForSingleObject(m_fenceEvent, INFINITE);
}

void Renderer::OnResize(uint32_t width, uint32_t height)
{
    if (width == 0 || height == 0) return;

    m_width  = width;
    m_height = height;
    WaitForGPU();
    m_swapChain.Resize(m_graphicsDevice.GetDevice(), width, height);
    m_depthBuffer.Resize(m_graphicsDevice.GetDevice(), width, height);
}

ID3D12Device* Renderer::GetDevice() const
{
    return m_graphicsDevice.GetDevice();
}

ID3D12GraphicsCommandList* Renderer::GetCommandList() const
{
    return m_commandContext.GetCommandList();
}

void Renderer::Shutdown()
{
    WaitForGPU();

    if (m_fenceEvent) {
        CloseHandle(m_fenceEvent);
        m_fenceEvent = nullptr;
    }

    m_fence.Reset();
    m_commandContext.Shutdown();
    m_depthBuffer.Reset();
    // Meshes already freed their placed resources in the app's OnShutdown (after
    // WaitForGPU), so the heap has no outstanding placed resources to dangle.
    m_geometryHeap.Shutdown();
    m_swapChain.Shutdown();
    m_graphicsDevice.Shutdown();

    LogInfo("Renderer shut down.");
}

} // namespace SGE
