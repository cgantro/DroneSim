cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    CACHE STRING "vcpkg CMake toolchain")
endif()

project(AeroSimLab VERSION 0.1.0 LANGUAGES C CXX)

set(AEROSIMLAB_SOURCE_REVISION "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE AEROSIMLAB_SOURCE_REVISION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(NOT AEROSIMLAB_SOURCE_REVISION)
    set(AEROSIMLAB_SOURCE_REVISION "unknown")
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8 "$<$<COMPILE_LANGUAGE:CXX>:/EHsc>")
endif()

include(CTest)
include(cmake/Dependencies.cmake)

if(NOT EMSCRIPTEN)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc)
endif()

option(AEROSIMLAB_BUILD_GRAPHICS "드론 화면을 함께 빌드합니다" ON)
aerosimlab_add_common_dependencies()
aerosimlab_add_scenario_dependencies()

add_library(aerosimlab_simulation_core STATIC
  modules/simulation_core/src/DroneModel.cpp
  modules/simulation_core/src/EoCameraModel.cpp
  modules/simulation_core/src/ScenarioRunner.cpp
  modules/simulation_core/src/SimulationCore.cpp
)
target_include_directories(aerosimlab_simulation_core PUBLIC modules/simulation_core/include)
target_link_libraries(aerosimlab_simulation_core
  PUBLIC glm::glm
  PRIVATE nlohmann_json::nlohmann_json
)

add_library(aerosimlab_communication STATIC
  modules/communication/src/Protocol.cpp
)
if(NOT EMSCRIPTEN)
  target_sources(aerosimlab_communication PRIVATE modules/communication/src/UdpSocket.cpp)
endif()
target_include_directories(aerosimlab_communication PUBLIC modules/communication/include)
target_link_libraries(aerosimlab_communication PUBLIC
  aerosimlab_simulation_core
  nlohmann_json::nlohmann_json
)
if(WIN32)
  target_link_libraries(aerosimlab_communication PUBLIC ws2_32)
endif()

add_library(aerosimlab_manual_control STATIC
  modules/manual_control/src/GamepadCommandMapper.cpp
)
target_include_directories(aerosimlab_manual_control PUBLIC modules/manual_control/include)
target_link_libraries(aerosimlab_manual_control PUBLIC aerosimlab_simulation_core)

add_library(aerosimlab_sensor STATIC modules/sensor/src/RgbFrame.cpp)
target_include_directories(aerosimlab_sensor PUBLIC modules/sensor/include)
if(NOT EMSCRIPTEN)
  add_library(aerosimlab_vision STATIC
    modules/vision/src/RgbFrameView.cpp
    modules/vision/src/RgbFrameQuality.cpp
  )
  target_include_directories(aerosimlab_vision PUBLIC modules/vision/include)
  target_link_libraries(aerosimlab_vision PUBLIC aerosimlab_sensor ${OpenCV_LIBS})
endif()

if(NOT EMSCRIPTEN)
  add_library(aerosimlab_ground_control_client STATIC
    apps/ground_control/GroundControlClient.cpp
  )
  target_include_directories(aerosimlab_ground_control_client PUBLIC apps/ground_control)
  target_link_libraries(aerosimlab_ground_control_client PUBLIC aerosimlab_communication)

  if(WIN32)
    add_executable(aerosimlab_ground_control WIN32
      apps/ground_control/WindowsGroundControlApp.cpp
    )
    target_link_libraries(aerosimlab_ground_control PRIVATE
      aerosimlab_ground_control_client
      user32
      gdi32
    )

    add_executable(aerosimlab_ground_control_cli apps/ground_control/main.cpp)
    target_link_libraries(aerosimlab_ground_control_cli PRIVATE aerosimlab_ground_control_client)
  else()
    add_executable(aerosimlab_ground_control apps/ground_control/main.cpp)
    target_link_libraries(aerosimlab_ground_control PRIVATE aerosimlab_ground_control_client)
  endif()
endif()

add_executable(aerosimlab_headless apps/headless/main.cpp)
target_link_libraries(aerosimlab_headless PRIVATE
  aerosimlab_simulation_core
  nlohmann_json::nlohmann_json
)
if(EMSCRIPTEN)
  target_link_options(aerosimlab_headless PRIVATE
    "--preload-file" "${CMAKE_SOURCE_DIR}/scenarios@scenarios"
  )
endif()

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

  if(WIN32)
    add_executable(aerosimlab_simulator WIN32
      apps/simulator/main.cpp
      apps/simulator/AeroSimLabApp.cpp
    )
    target_link_libraries(aerosimlab_simulator PRIVATE shell32)
  else()
    add_executable(aerosimlab_simulator
      apps/simulator/main.cpp
      apps/simulator/AeroSimLabApp.cpp
    )
  endif()
  target_include_directories(aerosimlab_simulator PRIVATE apps/simulator)
  target_link_libraries(aerosimlab_simulator PRIVATE
    aerosimlab_graphics
    aerosimlab_simulation_core
    aerosimlab_manual_control
    aerosimlab_sensor
    aerosimlab_communication
    nlohmann_json::nlohmann_json
  )
  if(NOT EMSCRIPTEN)
    target_link_libraries(aerosimlab_simulator PRIVATE aerosimlab_vision)
  endif()
  target_compile_definitions(aerosimlab_simulator PRIVATE
    AEROSIMLAB_SOURCE_REVISION="${AEROSIMLAB_SOURCE_REVISION}"
  )
  if(WIN32)
    target_link_libraries(aerosimlab_simulator PRIVATE Psapi)
  endif()
  if(EMSCRIPTEN)
    set_target_properties(aerosimlab_simulator PROPERTIES SUFFIX ".html")
    target_link_options(aerosimlab_simulator PRIVATE
      "-sALLOW_MEMORY_GROWTH=1"
      "--preload-file" "${CMAKE_SOURCE_DIR}/assets/shaders@shaders"
    )
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
    PROPERTIES LABELS "UNIT;simulation"
  )

  add_executable(aerosimlab_communication_tests
    tests/communication/ProtocolTests.cpp
  )
  if(NOT EMSCRIPTEN)
    target_sources(aerosimlab_communication_tests PRIVATE
      tests/communication/GroundControlClientTests.cpp
    )
    target_link_libraries(aerosimlab_communication_tests PRIVATE
      aerosimlab_ground_control_client
    )
  endif()
  target_link_libraries(aerosimlab_communication_tests PRIVATE
    aerosimlab_communication
    GTest::gtest_main
  )
  gtest_discover_tests(aerosimlab_communication_tests
    TEST_PREFIX "Communication."
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS "UNIT;communication"
  )

  add_executable(aerosimlab_manual_control_tests
    tests/manual_control/GamepadCommandMapperTests.cpp
  )
  target_link_libraries(aerosimlab_manual_control_tests PRIVATE
    aerosimlab_manual_control
    GTest::gtest_main
  )
  gtest_discover_tests(aerosimlab_manual_control_tests
    TEST_PREFIX "ManualControl."
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS "UNIT;manual_control"
  )

  add_executable(aerosimlab_sensor_tests tests/sensor/RgbFrameTests.cpp)
  target_link_libraries(aerosimlab_sensor_tests PRIVATE aerosimlab_sensor GTest::gtest_main)
  gtest_discover_tests(aerosimlab_sensor_tests
    TEST_PREFIX "Sensor."
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS "UNIT;sensor"
  )

  if(NOT EMSCRIPTEN)
    add_executable(aerosimlab_vision_tests
      tests/vision/RgbFrameViewTests.cpp
      tests/vision/RgbFrameQualityTests.cpp
    )
    target_link_libraries(aerosimlab_vision_tests PRIVATE aerosimlab_vision GTest::gtest_main)
    gtest_discover_tests(aerosimlab_vision_tests
      TEST_PREFIX "Vision."
      DISCOVERY_TIMEOUT 60
      PROPERTIES LABELS "UNIT;vision"
    )
  endif()

  add_test(NAME HeadlessScenarioSmoke
    COMMAND aerosimlab_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/normal-flight.json")
  set_tests_properties(HeadlessScenarioSmoke PROPERTIES LABELS "INTEGRATION;simulation")

  add_test(NAME HeadlessFaultScenarioSmoke
    COMMAND aerosimlab_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/fault-injection.json")
  set_tests_properties(HeadlessFaultScenarioSmoke PROPERTIES LABELS "INTEGRATION;simulation")

  if(AEROSIMLAB_BUILD_GRAPHICS)
    add_test(NAME ViewerSmoke COMMAND aerosimlab_simulator --smoke-test)
    set_tests_properties(ViewerSmoke PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:aerosimlab_simulator>"
      TIMEOUT 60
      RUN_SERIAL TRUE
      LABELS "INTEGRATION;graphics")
  endif()
endif()
