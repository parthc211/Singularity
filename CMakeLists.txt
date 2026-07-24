cmake_minimum_required(VERSION 3.20)
project(Singularity VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(MSVC)
    add_compile_options(/utf-8)
endif()

# Locate dxcompiler.dll. Prefer the SDK version CMake selected for compilation;
# fall back to all installed SDK versions sorted newest-first.
if(MSVC)
    get_filename_component(MSVC_BIN_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)

    # Primary: exact SDK CMake is targeting (CMAKE_SYSTEM_VERSION = "10.0.22621.0" etc.)
    set(_DXC_SEARCH_PATHS "")
    foreach(_root
        "C:/Program Files (x86)/Windows Kits/10/bin"
        "C:/Program Files/Windows Kits/10/bin"
    )
        if(EXISTS "${_root}/${CMAKE_SYSTEM_VERSION}/x64")
            list(APPEND _DXC_SEARCH_PATHS "${_root}/${CMAKE_SYSTEM_VERSION}/x64")
        endif()
    endforeach()

    # Fallback: every installed SDK version, newest-first
    file(GLOB _WDK_ALL
        "C:/Program Files (x86)/Windows Kits/10/bin/*/x64"
        "C:/Program Files/Windows Kits/10/bin/*/x64"
    )
    list(SORT _WDK_ALL COMPARE NATURAL ORDER DESCENDING)
    list(APPEND _DXC_SEARCH_PATHS ${_WDK_ALL} "${MSVC_BIN_DIR}")

    # Unset caches so re-configuring always re-evaluates the list above.
    unset(DXC_DLL  CACHE)
    unset(DXIL_DLL CACHE)
    find_file(DXC_DLL  NAMES dxcompiler.dll PATHS ${_DXC_SEARCH_PATHS} NO_DEFAULT_PATH)
    find_file(DXIL_DLL NAMES dxil.dll       PATHS ${_DXC_SEARCH_PATHS} NO_DEFAULT_PATH)

    if(DXC_DLL)
        message(STATUS "DXC:  ${DXC_DLL}")
    else()
        message(WARNING "dxcompiler.dll not found. Runtime shader compilation will fail.")
    endif()

    if(DXIL_DLL)
        message(STATUS "DXIL: ${DXIL_DLL}")
    else()
        message(WARNING "dxil.dll not found. SM6 PSO creation will fail.")
    endif()
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ---------------------------------------------------------------------------
# Dear ImGui (vendored via FetchContent, pinned tag).
# ImGui ships no CMakeLists, so we download the sources and define our own
# static library target from the core + the Win32 platform + DX12 renderer
# backends. v1.91.5 is pinned deliberately: it uses the simple single-SRV
# ImGui_ImplDX12_Init signature (later 1.92+ requires a descriptor-heap
# allocator callback for dynamic textures — more machinery than we need).
# ImGui is "boring plumbing" (UI), so using the library is in-bounds per the
# project's no-third-party-for-showcase-systems rule.
# ---------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.5
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_dx12.cpp
)
target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC d3d12.lib dxgi.lib dwmapi.lib)

add_subdirectory(Engine)
add_subdirectory(Sandbox)
