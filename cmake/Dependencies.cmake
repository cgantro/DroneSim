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
