add_executable(Sandbox WIN32
    main.cpp
    Scenes/CubeGridScene.h
    Scenes/CubeGridScene.cpp
    Scenes/SingleCubeScene.h
    Scenes/SingleCubeScene.cpp
    Scenes/GpuHeapScene.h
    Scenes/GpuHeapScene.cpp
    Scenes/DeferredScene.h
    Scenes/DeferredScene.cpp
    Scenes/SimdMathScene.h
    Scenes/SimdMathScene.cpp
    Scenes/AllocatorScene.h
    Scenes/AllocatorScene.cpp
    Scenes/TessellationScene.h
    Scenes/TessellationScene.cpp
    Scenes/ShadowScene.h
    Scenes/ShadowScene.cpp
    Scenes/BloomScene.h
    Scenes/BloomScene.cpp
    Scenes/SsaoScene.h
    Scenes/SsaoScene.cpp
    Scenes/CsmScene.h
    Scenes/CsmScene.cpp
    Scenes/JobScene.h
    Scenes/JobScene.cpp
    Scenes/PhysicsScene.h
    Scenes/PhysicsScene.cpp
    Scenes/JointScene.h
    Scenes/JointScene.cpp
)

# So scene sources can include "Scenes/..." relative to the Sandbox root.
target_include_directories(Sandbox PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(Sandbox PRIVATE SingularityEngine)

target_compile_definitions(Sandbox PRIVATE
    UNICODE
    _UNICODE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
)

set_property(TARGET Sandbox PROPERTY VS_DEBUGGER_WORKING_DIRECTORY $<TARGET_FILE_DIR:Sandbox>)

add_custom_command(TARGET Sandbox POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/Shaders
        $<TARGET_FILE_DIR:Sandbox>/Shaders
    COMMENT "Copying shaders to output directory"
)

add_custom_command(TARGET Sandbox POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/Assets
        $<TARGET_FILE_DIR:Sandbox>/Assets
    COMMENT "Copying assets to output directory"
)

if(DXC_DLL)
    add_custom_command(TARGET Sandbox POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${DXC_DLL}" $<TARGET_FILE_DIR:Sandbox>/dxcompiler.dll
        COMMENT "Copying dxcompiler.dll"
    )
endif()

if(DXIL_DLL)
    add_custom_command(TARGET Sandbox POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${DXIL_DLL}" $<TARGET_FILE_DIR:Sandbox>/dxil.dll
        COMMENT "Copying dxil.dll"
    )
endif()
