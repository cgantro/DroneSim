include(FetchContent)

function(aerosimlab_add_common_dependencies)
  FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(glm)
endfunction()

function(aerosimlab_add_ecs_dependencies)
  set(FLECS_SHARED OFF CACHE BOOL "Build Flecs as a shared library" FORCE)
  set(FLECS_STATIC ON CACHE BOOL "Build Flecs as a static library" FORCE)
  set(FLECS_TESTS OFF CACHE BOOL "Disable Flecs tests" FORCE)
  FetchContent_Declare(
    flecs
    GIT_REPOSITORY https://github.com/SanderMertens/flecs.git
    GIT_TAG v4.1.6
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(flecs)
endfunction()

function(aerosimlab_add_jolt_dependency)
  set(USE_ASSERTS OFF CACHE BOOL "Disable Jolt assertions in project builds" FORCE)
  set(OVERRIDE_CXX_FLAGS OFF CACHE BOOL "Keep the project's compiler flags" FORCE)
  set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "Keep Jolt separate from project LTO" FORCE)
  set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "Use the project's warning policy" FORCE)
  set(ENABLE_INSTALL OFF CACHE BOOL "Do not install a project-private dependency" FORCE)
  set(FLOATING_POINT_EXCEPTIONS_ENABLED OFF CACHE BOOL "Use the project's floating-point policy" FORCE)
  set(JPH_BUILD_SHARED_LIBS OFF CACHE BOOL "Build Jolt as a static library" FORCE)
  set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "Use the project's dynamic MSVC runtime" FORCE)
  set(ENABLE_OBJECT_STREAM OFF CACHE BOOL "Disable unused Jolt object serialization" FORCE)
  set(JPH_USE_CPU_COMPUTE OFF CACHE BOOL "Disable unused Jolt compute shaders" FORCE)
  set(JPH_USE_DX12 OFF CACHE BOOL "Disable unused Jolt DirectX compute" FORCE)
  set(JPH_USE_VK OFF CACHE BOOL "Disable unused Jolt Vulkan compute" FORCE)
  set(JPH_USE_MTL OFF CACHE BOOL "Disable unused Jolt Metal compute" FORCE)
  set(DEBUG_RENDERER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "Disable unused Jolt debug renderer" FORCE)
  set(PROFILER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "Disable unused Jolt profiler" FORCE)

  FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG e77f175595e64cb44218cc9d9d56fc365ad0e36a
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR Build
  )
  FetchContent_MakeAvailable(JoltPhysics)
endfunction()

function(aerosimlab_add_scenario_dependencies)
  set(JSON_BuildTests OFF CACHE INTERNAL "Disable dependency tests")
  set(JSON_Install OFF CACHE INTERNAL "Do not install a project-private dependency")
  FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.12.0
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(nlohmann_json)
endfunction()

function(aerosimlab_add_graphics_dependencies)
  FetchContent_Declare(
    stb_image
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG 003501f6d7a8668ce8b6d875e34dd97ad0d5d5b5
    GIT_SHALLOW FALSE
  )
  FetchContent_MakeAvailable(stb_image)
  add_library(aerosimlab_stb_image INTERFACE)
  target_include_directories(aerosimlab_stb_image SYSTEM INTERFACE
    "${stb_image_SOURCE_DIR}"
  )

  if(EMSCRIPTEN)
    add_library(glfw INTERFACE)
    add_library(OpenGL::GL INTERFACE IMPORTED GLOBAL)
    return()
  endif()

  set(GLFW_BUILD_DOCS OFF CACHE BOOL "Disable GLFW documentation" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "Disable GLFW tests" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Disable GLFW examples" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "Disable GLFW installation" FORCE)
  FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(glfw)
  find_package(OpenGL REQUIRED)
endfunction()
