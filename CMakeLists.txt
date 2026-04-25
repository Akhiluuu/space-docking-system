cmake_minimum_required(VERSION 3.20)
project(DtProjectScenarios CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimization Flags
if(MSVC)
    add_compile_options($<$<NOT:$<CONFIG:Debug>>:/O2> /arch:AVX2 /fp:fast)
else()
    add_compile_options(-O3 -march=native -ffast-math)
endif()

include(FetchContent)

# Fetch Eigen
FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(Eigen3)

# Fetch TinyXML2
FetchContent_Declare(
    tinyxml2
    GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
    GIT_TAG        9.0.0
)
FetchContent_MakeAvailable(tinyxml2)

# Fetch GLM
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        0.9.9.8
)
FetchContent_MakeAvailable(glm)

# Fetch GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.8
)
FetchContent_MakeAvailable(glfw)

# Fetch Glad
FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG        v2.0.4
)
FetchContent_GetProperties(glad)
if(NOT glad_POPULATED)
    FetchContent_Populate(glad)
endif()

find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(GLAD_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/glad_gen")
file(MAKE_DIRECTORY ${GLAD_OUT_DIR})
set(GLAD_SOURCES 
    "${GLAD_OUT_DIR}/src/gl.c"
    "${GLAD_OUT_DIR}/include/glad/gl.h"
)
add_custom_command(
    OUTPUT ${GLAD_SOURCES}
    COMMAND ${Python3_EXECUTABLE} -m glad --out-path ${GLAD_OUT_DIR} --api gl:core=3.3 --reproducible c
    WORKING_DIRECTORY ${glad_SOURCE_DIR}
)
add_library(glad STATIC ${GLAD_SOURCES})
target_include_directories(glad PUBLIC "${GLAD_OUT_DIR}/include")

# Fetch Assimp
set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    assimp
    GIT_REPOSITORY https://github.com/assimp/assimp.git
    GIT_TAG        v5.2.5
)
FetchContent_MakeAvailable(assimp)

# Fetch Dear ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.90
)
FetchContent_MakeAvailable(imgui)

add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC 
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw glad)

# Mars Stewart Simulation Executable
add_executable(MarsStewartSimulation
    src/scenarios_precombined.cpp
    src/BulletSimulation.cpp
    src/DockingSimulation.cpp
    src/DockingPhysicsEngine.cpp
    src/ScenarioLoader.cpp
    src/Scenario.cpp
)

target_include_directories(MarsStewartSimulation PRIVATE
    include
    ${eigen3_SOURCE_DIR}
    ${glm_SOURCE_DIR}
    ${assimp_SOURCE_DIR}/include
    ${bullet_SOURCE_DIR}/src
)

# Main Executable
add_executable(DtProjectScenarios
    src/leg_joint_viewer.cpp
    src/ScenarioLoader.cpp
    src/DockingSimulation.cpp
    src/DockingPhysicsEngine.cpp
    src/Scenario.cpp
)

target_include_directories(DtProjectScenarios PRIVATE
    include
    ${eigen3_SOURCE_DIR}
    ${glm_SOURCE_DIR}
    ${assimp_SOURCE_DIR}/include
)

target_link_libraries(DtProjectScenarios PRIVATE
    Eigen3::Eigen
    tinyxml2
    glfw
    glad
    assimp::assimp
    imgui_lib
)

if(MSVC)
    target_link_libraries(DtProjectScenarios PRIVATE winmm)
endif()
