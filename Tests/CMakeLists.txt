# Headless test suite (console executables, run via ctest):
#   LoaderTests       — glTF + FBX importers vs sample assets + a synthetic
#                       glTF with hand-derived ground truth (60+ checks:
#                       winding via signed volume, sparse accessors, exact
#                       CUBICSPLINE/STEP, embedded textures, skeletons, skins)
#   AnimationTests    — the pose pipeline against hand-computed kinematics
#                       (sampling, cursors, blending, globals, palette, player)
#   DeterminismTests  — crossfade behavior + bit-identical serial vs
#                       JobSystem-parallel pose evaluation
#
# Tests run with WORKING_DIRECTORY = Sandbox/ so asset paths ("Assets/...")
# are the same ones the app uses. Run: ctest --test-dir build -C Debug
add_executable(LoaderTests LoaderTests.cpp)
add_executable(AnimationTests AnimationTests.cpp)
add_executable(DeterminismTests DeterminismTests.cpp)
add_executable(PhysicsTests PhysicsTests.cpp)

foreach(t LoaderTests AnimationTests DeterminismTests PhysicsTests)
    target_link_libraries(${t} PRIVATE SingularityEngine)
    set_property(TARGET ${t} PROPERTY FOLDER "Tests")
    add_test(NAME ${t}
             COMMAND ${t}
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/Sandbox)
endforeach()
