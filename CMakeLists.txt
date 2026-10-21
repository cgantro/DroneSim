cmake_minimum_required(VERSION 3.20)

project(AeroSimLab VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8 "$<$<COMPILE_LANGUAGE:CXX>:/EHsc>")
endif()

include(CTest)
include(cmake/Dependencies.cmake)

option(AEROSIMLAB_BUILD_GRAPHICS "드론 화면을 함께 빌드합니다" ON)
aerosimlab_add_common_dependencies()
aerosimlab_add_scenario_dependencies()

add_library(aerosimlab_simulation_core STATIC
  modules/simulation_core/src/DroneModel.cpp
  modules/simulation_core/src/ScenarioRunner.cpp
  modules/simulation_core/src/SimulationCore.cpp
)
target_include_directories(aerosimlab_simulation_core PUBLIC modules/simulation_core/include)
target_link_libraries(aerosimlab_simulation_core
  PUBLIC glm::glm
  PRIVATE nlohmann_json::nlohmann_json
)

add_executable(aerosimlab_headless apps/headless/main.cpp)
target_link_libraries(aerosimlab_headless PRIVATE
  aerosimlab_simulation_core
  nlohmann_json::nlohmann_json
)

if(AEROSIMLAB_BUILD_GRAPHICS)
  aerosimlab_add_graphics_dependencies()

  if(NOT EMSCRIPTEN)
    add_library(aerosimlab_glad STATIC third_party/glad/src/glad.c)
    target_include_directories(aerosimlab_glad PUBLIC third_party/glad/include)
  endif()

  set(AEROSIMLAB_GRAPHICS_SOURCES
    modules/graphics/src/graphics/Camera.cpp
    modules/graphics/src/graphics/IndexBuffer.cpp
    modules/graphics/src/graphics/Material.cpp
    modules/graphics/src/graphics/Mesh.cpp
    modules/graphics/src/graphics/Renderer.cpp
    modules/graphics/src/graphics/Shader.cpp
    modules/graphics/src/graphics/VertexArray.cpp
    modules/graphics/src/graphics/VertexBuffer.cpp
    modules/graphics/src/graphics/Window.cpp
  )
  if(NOT EMSCRIPTEN)
    list(APPEND AEROSIMLAB_GRAPHICS_SOURCES
      modules/graphics/src/graphics/MultisampleFramebuffer.cpp
    )
  endif()

  add_library(aerosimlab_graphics STATIC ${AEROSIMLAB_GRAPHICS_SOURCES})
  target_include_directories(aerosimlab_graphics PUBLIC
    modules/graphics/include
  )
  target_link_libraries(aerosimlab_graphics PUBLIC glfw glm::glm)
  if(EMSCRIPTEN)
    target_link_libraries(aerosimlab_graphics PUBLIC OpenGL::GL)
    target_link_options(aerosimlab_graphics INTERFACE "-sUSE_GLFW=3" "-sFULL_ES3=1")
  else()
    target_link_libraries(aerosimlab_graphics PUBLIC aerosimlab_glad OpenGL::GL)
  endif()
  target_compile_definitions(aerosimlab_graphics PUBLIC GLFW_INCLUDE_NONE)

  add_executable(aerosimlab_simulator
    apps/simulator/main.cpp
    apps/simulator/AeroSimLabApp.cpp
  )
  target_include_directories(aerosimlab_simulator PRIVATE apps/simulator)
  target_link_libraries(aerosimlab_simulator PRIVATE
    aerosimlab_graphics
    aerosimlab_simulation_core
  )
  if(EMSCRIPTEN)
    set_target_properties(aerosimlab_simulator PROPERTIES SUFFIX ".html")
    target_link_options(aerosimlab_simulator PRIVATE "-sALLOW_MEMORY_GROWTH=1")
  else()
    add_custom_command(TARGET aerosimlab_simulator POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/assets/shaders"
        "$<TARGET_FILE_DIR:aerosimlab_simulator>/shaders"
      VERBATIM
    )
  endif()
endif()

if(BUILD_TESTING)
  set(BUILD_GMOCK OFF CACHE BOOL "Disable Google Mock" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "Disable GoogleTest installation" FORCE)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.18.0
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(googletest)

  include(GoogleTest)
  add_executable(aerosimlab_simulation_tests
    tests/simulation_core/SimulationCoreTests.cpp
    tests/simulation_core/ScenarioRunnerTests.cpp
  )
  target_link_libraries(aerosimlab_simulation_tests PRIVATE
    aerosimlab_simulation_core
    GTest::gtest_main
  )
  gtest_discover_tests(aerosimlab_simulation_tests
    TEST_PREFIX "Simulation."
    DISCOVERY_TIMEOUT 60
  )

  add_test(NAME HeadlessScenarioSmoke
    COMMAND aerosimlab_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/normal-flight.json")
  set_tests_properties(HeadlessScenarioSmoke PROPERTIES LABELS "INTEGRATION;simulation")

  if(AEROSIMLAB_BUILD_GRAPHICS)
    add_test(NAME ViewerSmoke COMMAND aerosimlab_simulator --smoke-test)
    set_tests_properties(ViewerSmoke PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:aerosimlab_simulator>"
      TIMEOUT 60
      RUN_SERIAL TRUE
      LABELS "INTEGRATION;graphics")
  endif()
endif()
