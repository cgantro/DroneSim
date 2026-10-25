cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    CACHE STRING "vcpkg CMake toolchain")
endif()

project(AeroSimLab VERSION 0.1.0 LANGUAGES C CXX)

set(AEROSIMLAB_SOURCE_REVISION "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
  # 새 커밋의 리비전이 다음 빌드의 측정 결과에도 반영되도록 Git 참조를 감시한다.
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" rev-parse --git-path HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE AEROSIMLAB_GIT_HEAD_FILE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(AEROSIMLAB_GIT_HEAD_FILE)
    get_filename_component(AEROSIMLAB_GIT_HEAD_FILE
      "${AEROSIMLAB_GIT_HEAD_FILE}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
    if(EXISTS "${AEROSIMLAB_GIT_HEAD_FILE}")
      set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${AEROSIMLAB_GIT_HEAD_FILE}")
    endif()
  endif()

  execute_process(
    COMMAND "${GIT_EXECUTABLE}" symbolic-ref --quiet HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE AEROSIMLAB_GIT_BRANCH_REFERENCE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(AEROSIMLAB_GIT_BRANCH_REFERENCE)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" rev-parse --git-path "${AEROSIMLAB_GIT_BRANCH_REFERENCE}"
      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
      OUTPUT_VARIABLE AEROSIMLAB_GIT_BRANCH_FILE
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    if(AEROSIMLAB_GIT_BRANCH_FILE)
      get_filename_component(AEROSIMLAB_GIT_BRANCH_FILE
        "${AEROSIMLAB_GIT_BRANCH_FILE}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
      if(EXISTS "${AEROSIMLAB_GIT_BRANCH_FILE}")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
          "${AEROSIMLAB_GIT_BRANCH_FILE}")
      endif()
    endif()
  endif()

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
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" status --porcelain
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE AEROSIMLAB_WORKTREE_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(AEROSIMLAB_WORKTREE_STATUS)
    string(APPEND AEROSIMLAB_SOURCE_REVISION "-dirty")
  endif()
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8 "$<$<COMPILE_LANGUAGE:CXX>:/EHsc>")
  add_compile_options("$<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CXX>>:/Zi>")
  add_link_options("$<$<CONFIG:Release>:/DEBUG>")
endif()

include(CTest)
include(cmake/Dependencies.cmake)
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

if(NOT EMSCRIPTEN)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc)
endif()

option(AEROSIMLAB_BUILD_GRAPHICS "드론 화면을 함께 빌드합니다" ON)
aerosimlab_add_common_dependencies()
aerosimlab_add_ecs_dependencies()
aerosimlab_add_scenario_dependencies()
if(NOT EMSCRIPTEN)
  aerosimlab_add_airsim_dependency()
endif()

add_library(aerosimlab_terrain STATIC
  modules/terrain/src/TerrainGrid.cpp
)
target_include_directories(aerosimlab_terrain PUBLIC modules/terrain/include)
target_link_libraries(aerosimlab_terrain PUBLIC glm::glm)

add_library(aerosimlab_simulation_core STATIC
  modules/simulation_core/src/DroneModel.cpp
  modules/simulation_core/src/CommandRecording.cpp
  modules/simulation_core/src/EoCameraModel.cpp
  modules/simulation_core/src/ScenarioRunner.cpp
  modules/simulation_core/src/ScenarioDefinition.cpp
  modules/simulation_core/src/SimulationCore.cpp
  modules/simulation_core/src/QuadrotorModel.cpp
  modules/simulation_core/src/QuadrotorFlightController.cpp
)
if(NOT EMSCRIPTEN)
  target_sources(aerosimlab_simulation_core PRIVATE
    modules/simulation_core/src/CollisionGeometry.cpp
    modules/simulation_core/src/PhysicsWorld.cpp
  )
endif()
target_include_directories(aerosimlab_simulation_core PUBLIC modules/simulation_core/include)
target_link_libraries(aerosimlab_simulation_core
  PUBLIC glm::glm aerosimlab_terrain
  PRIVATE nlohmann_json::nlohmann_json flecs::flecs_static
)
if(NOT EMSCRIPTEN)
  target_link_libraries(aerosimlab_simulation_core PRIVATE aerosimlab_airsim)
endif()

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

    FetchContent_Declare(
      aerosimlab_imgui_source
      GIT_REPOSITORY https://github.com/ocornut/imgui.git
      GIT_TAG d88f238d1d363ef387382b2e29ad12cf96f826ac
      GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(aerosimlab_imgui_source)
    if(NOT aerosimlab_imgui_source_POPULATED)
      FetchContent_Populate(aerosimlab_imgui_source)
    endif()
    add_library(aerosimlab_imgui STATIC
      "${aerosimlab_imgui_source_SOURCE_DIR}/imgui.cpp"
      "${aerosimlab_imgui_source_SOURCE_DIR}/imgui_draw.cpp"
      "${aerosimlab_imgui_source_SOURCE_DIR}/imgui_tables.cpp"
      "${aerosimlab_imgui_source_SOURCE_DIR}/imgui_widgets.cpp"
      "${aerosimlab_imgui_source_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
      "${aerosimlab_imgui_source_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
    )
    target_include_directories(aerosimlab_imgui PUBLIC
      "${aerosimlab_imgui_source_SOURCE_DIR}"
      "${aerosimlab_imgui_source_SOURCE_DIR}/backends"
    )
    target_compile_definitions(aerosimlab_imgui PRIVATE GLFW_INCLUDE_NONE)
    target_link_libraries(aerosimlab_imgui PUBLIC glfw OpenGL::GL)
  endif()

  set(AEROSIMLAB_GRAPHICS_SOURCES
    modules/graphics/src/graphics/Camera.cpp
    modules/graphics/src/graphics/IndexBuffer.cpp
    modules/graphics/src/graphics/Material.cpp
    modules/graphics/src/graphics/Mesh.cpp
    modules/graphics/src/graphics/Renderer.cpp
    modules/graphics/src/graphics/Shader.cpp
    modules/graphics/src/graphics/Texture2D.cpp
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
  target_link_libraries(aerosimlab_graphics PUBLIC glfw glm::glm
    PRIVATE nlohmann_json::nlohmann_json)
  if(EMSCRIPTEN)
    target_link_libraries(aerosimlab_graphics PUBLIC OpenGL::GL)
    target_link_options(aerosimlab_graphics INTERFACE "-sUSE_GLFW=3" "-sFULL_ES3=1")
  else()
    target_link_libraries(aerosimlab_graphics PUBLIC aerosimlab_glad OpenGL::GL)
  endif()
  target_compile_definitions(aerosimlab_graphics PUBLIC GLFW_INCLUDE_NONE)

  add_library(aerosimlab_simulator_app STATIC
    apps/simulator/AeroSimLabApp.cpp
    apps/simulator/AeroSimLabAppBenchmark.cpp
    apps/simulator/AeroSimLabAppGamepad.cpp
    apps/simulator/AeroSimLabAppInput.cpp
    apps/simulator/FleetDroneSpawn.cpp
    apps/simulator/SimulationBenchmark.cpp
  )
  if(NOT EMSCRIPTEN)
    target_sources(aerosimlab_simulator_app PRIVATE
      apps/simulator/AeroSimLabAppEoCapture.cpp
      apps/simulator/CooperativeMissionBenchmark.cpp
      apps/simulator/DroneScalingBenchmark.cpp
      apps/simulator/SimulatorHud.cpp
    )
  endif()
  target_include_directories(aerosimlab_simulator_app PUBLIC apps/simulator)
  target_link_libraries(aerosimlab_simulator_app PUBLIC
    aerosimlab_graphics
    aerosimlab_simulation_core
    aerosimlab_manual_control
    aerosimlab_sensor
    aerosimlab_communication
    nlohmann_json::nlohmann_json
  )
  if(NOT EMSCRIPTEN)
    target_link_libraries(aerosimlab_simulator_app PRIVATE aerosimlab_vision aerosimlab_imgui)
  endif()
  target_compile_definitions(aerosimlab_simulator_app PRIVATE
    AEROSIMLAB_SOURCE_REVISION="${AEROSIMLAB_SOURCE_REVISION}"
    AEROSIMLAB_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  )
  if(WIN32)
    target_link_libraries(aerosimlab_simulator_app PRIVATE Psapi)
  endif()

  if(WIN32)
    add_executable(aerosimlab_simulator WIN32 apps/simulator/main.cpp)
    target_link_libraries(aerosimlab_simulator PRIVATE aerosimlab_simulator_app shell32)
  else()
    add_executable(aerosimlab_simulator apps/simulator/main.cpp)
    target_link_libraries(aerosimlab_simulator PRIVATE aerosimlab_simulator_app)
  endif()
  if(EMSCRIPTEN)
    set_target_properties(aerosimlab_simulator PROPERTIES SUFFIX ".html")
    target_link_options(aerosimlab_simulator PRIVATE
      "--shell-file=${CMAKE_SOURCE_DIR}/apps/simulator/web/web_shell_single_thread.html"
      "-sALLOW_MEMORY_GROWTH=1"
      "--preload-file=${CMAKE_SOURCE_DIR}/assets/shaders@shaders"
    )
  else()
    add_custom_command(TARGET aerosimlab_simulator POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/assets/shaders"
        "$<TARGET_FILE_DIR:aerosimlab_simulator>/shaders"
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/assets/models/drones"
        "$<TARGET_FILE_DIR:aerosimlab_simulator>/models/drones"
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
    tests/simulation_core/CommandRecordingTests.cpp
    tests/simulation_core/SimulationCoreTests.cpp
    tests/simulation_core/ScenarioRunnerTests.cpp
    tests/simulation_core/QuadrotorModelTests.cpp
    tests/terrain/TerrainGridTests.cpp
  )
  target_compile_definitions(aerosimlab_simulation_tests PRIVATE
    AEROSIMLAB_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  )
  if(NOT EMSCRIPTEN)
    target_include_directories(aerosimlab_simulation_tests PRIVATE
      modules/simulation_core/src
    )
    target_sources(aerosimlab_simulation_tests PRIVATE tests/simulation_core/PhysicsWorldTests.cpp)
  endif()
  target_link_libraries(aerosimlab_simulation_tests PRIVATE
    aerosimlab_simulation_core
    nlohmann_json::nlohmann_json
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
    COMMAND aerosimlab_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/flight/rigid-body-flight.json")
  set_tests_properties(HeadlessScenarioSmoke PROPERTIES
    LABELS "INTEGRATION;simulation"
    PASS_REGULAR_EXPRESSION "\\\"tick\\\": 600;\\\"angular_velocity_radians_per_second\\\"")

  add_test(NAME HeadlessFaultScenarioSmoke
    COMMAND aerosimlab_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/fault-injection/fault-injection.json")
  set_tests_properties(HeadlessFaultScenarioSmoke PROPERTIES LABELS "INTEGRATION;simulation")

  add_test(NAME HeadlessMultiDroneSmoke
    COMMAND aerosimlab_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/multi-drone/independent-flight.json")
  set_tests_properties(HeadlessMultiDroneSmoke PROPERTIES
    LABELS "INTEGRATION;simulation"
    PASS_REGULAR_EXPRESSION "lead.*scout")

  add_test(NAME HeadlessFormationOutageSmoke
    COMMAND aerosimlab_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/multi-drone/leader-following-outage.json")
  set_tests_properties(HeadlessFormationOutageSmoke PROPERTIES
    LABELS "INTEGRATION;simulation"
    PASS_REGULAR_EXPRESSION "위치 공유가 끊겼습니다")

  add_test(NAME HeadlessCooperativeMissionSmoke
    COMMAND aerosimlab_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/multi-drone/four-drone-survey-outage.json")
  set_tests_properties(HeadlessCooperativeMissionSmoke PROPERTIES
    LABELS "INTEGRATION;simulation"
    PASS_REGULAR_EXPRESSION "\"mission\"")

  if(AEROSIMLAB_BUILD_GRAPHICS)
    add_test(NAME ViewerSmoke COMMAND aerosimlab_simulator --smoke-test)
    set_tests_properties(ViewerSmoke PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:aerosimlab_simulator>"
      TIMEOUT 60
      RUN_SERIAL TRUE
      LABELS "INTEGRATION;graphics")

    if(NOT EMSCRIPTEN)
      add_executable(aerosimlab_simulator_integration_tests
        tests/simulator/SimulatorSessionTests.cpp
        tests/graphics/CameraTests.cpp
      )
      target_link_libraries(aerosimlab_simulator_integration_tests PRIVATE
        aerosimlab_simulator_app
        aerosimlab_ground_control_client
        GTest::gtest_main
      )
      gtest_discover_tests(aerosimlab_simulator_integration_tests
        TEST_PREFIX "Simulator."
        DISCOVERY_TIMEOUT 60
        PROPERTIES
          LABELS "INTEGRATION;simulator"
          WORKING_DIRECTORY "$<TARGET_FILE_DIR:aerosimlab_simulator>"
          TIMEOUT 40
          RUN_SERIAL TRUE)
      add_dependencies(aerosimlab_simulator_integration_tests aerosimlab_simulator)
    endif()
  endif()
endif()
