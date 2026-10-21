include(FetchContent)

function(grasplink_add_common_dependencies)
  FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(glm)
endfunction()

function(grasplink_add_robotics_dependencies)
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "Disable Eigen documentation" FORCE)
  set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Disable Eigen pkg-config files" FORCE)
  FetchContent_Declare(
    eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
  )
  FetchContent_GetProperties(eigen3)
  if(NOT eigen3_POPULATED)
    FetchContent_Populate(eigen3)
  endif()
  add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${eigen3_SOURCE_DIR}"
  )
endfunction()

function(grasplink_add_scene_dependencies)
  FetchContent_Declare(
    flecs
    GIT_REPOSITORY https://github.com/SanderMertens/flecs.git
    GIT_TAG v4.1.5
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(flecs)
endfunction()

function(grasplink_add_model_dependencies)
  set(TINYGLTF_BUILD_LOADER_EXAMPLE OFF CACHE BOOL "" FORCE)
  set(TINYGLTF_BUILD_GL_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(TINYGLTF_BUILD_VALIDATOR_EXAMPLE OFF CACHE BOOL "" FORCE)
  set(TINYGLTF_BUILD_BUILDER_EXAMPLE OFF CACHE BOOL "" FORCE)
  set(TINYGLTF_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    tinygltf
    GIT_REPOSITORY https://github.com/syoyo/tinygltf.git
    GIT_TAG v2.9.7
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(tinygltf)

  set(tinyxml2_BUILD_TESTING OFF CACHE BOOL "Disable TinyXML-2 tests" FORCE)
  FetchContent_Declare(
    tinyxml2
    GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
    GIT_TAG 10.0.0
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(tinyxml2)
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

function(grasplink_add_graphics_dependencies)
  if(EMSCRIPTEN)
    add_library(glfw INTERFACE)
    add_library(OpenGL::GL INTERFACE IMPORTED GLOBAL)
    return()
  endif()

  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(glfw)
  find_package(OpenGL REQUIRED)
endfunction()

function(grasplink_add_gui_dependencies)
  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.9b
    GIT_SHALLOW TRUE
  )
  FetchContent_GetProperties(imgui)
  if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
  endif()
endfunction()

function(grasplink_add_physics_dependencies)
  set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "Use the static MSVC runtime library" FORCE)

  FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG v5.6.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR Build
  )
  FetchContent_MakeAvailable(JoltPhysics)
endfunction()
