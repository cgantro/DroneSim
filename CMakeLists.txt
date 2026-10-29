cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    CACHE STRING "vcpkg CMake toolchain")
endif()

project(AeroSimLab VERSION 0.1.0 LANGUAGES C CXX)

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

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
include(CTest)
include(cmake/Dependencies.cmake)
option(AEROSIMLAB_BUILD_GRAPHICS "드론 화면을 함께 빌드합니다" ON)
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

if(NOT EMSCRIPTEN AND (AEROSIMLAB_BUILD_GRAPHICS OR BUILD_TESTING))
  set(_aerosimlab_opencv_components core imgproc)
  find_package(OpenCV REQUIRED COMPONENTS ${_aerosimlab_opencv_components})
endif()

if(NOT EMSCRIPTEN AND BUILD_TESTING)
  find_package(Threads REQUIRED)
endif()

aerosimlab_add_common_dependencies()
aerosimlab_add_ecs_dependencies()
aerosimlab_add_scenario_dependencies()
if(NOT EMSCRIPTEN)
  aerosimlab_add_airsim_dependency()
endif()

add_library(aerosimlab_terrain STATIC
  modules/terrain/src/MapBridges.cpp
  modules/terrain/src/MapBuildings.cpp
  modules/terrain/src/MapFlightRoute.cpp
  modules/terrain/src/MapRoads.cpp
  modules/terrain/src/MapTrees.cpp
  modules/terrain/src/MapWaterBodies.cpp
  modules/terrain/src/TerrainGrid.cpp
  modules/terrain/src/TrainingRangeTerrain.cpp
)
target_include_directories(aerosimlab_terrain PUBLIC modules/terrain/include)
target_link_libraries(aerosimlab_terrain PUBLIC glm::glm PRIVATE nlohmann_json::nlohmann_json)

add_library(aerosimlab_flight_planning STATIC
  modules/flight_planning/src/FormationPlanner.cpp
)
target_include_directories(aerosimlab_flight_planning PUBLIC modules/flight_planning/include)
target_link_libraries(aerosimlab_flight_planning PUBLIC glm::glm aerosimlab_terrain)

add_library(aerosimlab_simulation_core STATIC
  modules/simulation_core/src/BatteryModel.cpp
  modules/simulation_core/src/DroneModel.cpp
  modules/simulation_core/src/EoCameraModel.cpp
  modules/simulation_core/src/CollisionGeometry.cpp
  modules/simulation_core/src/SimulationCore.cpp
  modules/simulation_core/src/QuadrotorModel.cpp
  modules/simulation_core/src/QuadrotorFlightController.cpp
)
if(NOT EMSCRIPTEN)
  target_sources(aerosimlab_simulation_core PRIVATE
    modules/simulation_core/src/PhysicsWorld.cpp
  )
endif()
target_include_directories(aerosimlab_simulation_core PUBLIC modules/simulation_core/include)
target_link_libraries(aerosimlab_simulation_core
  PUBLIC glm::glm aerosimlab_terrain
  PRIVATE flecs::flecs_static
)
if(NOT EMSCRIPTEN)
  target_link_libraries(aerosimlab_simulation_core PRIVATE aerosimlab_airsim)
endif()

add_library(aerosimlab_scenario STATIC
  modules/scenario/src/ScenarioRunner.cpp
  modules/scenario/src/ScenarioDefinition.cpp
)
target_include_directories(aerosimlab_scenario PUBLIC modules/scenario/include)
target_link_libraries(aerosimlab_scenario
  PUBLIC aerosimlab_simulation_core
  PRIVATE nlohmann_json::nlohmann_json
)

add_library(aerosimlab_communication STATIC
  modules/communication/src/Protocol.cpp
  modules/communication/src/VirtualMeshNetwork.cpp
)
if(NOT EMSCRIPTEN)
  target_sources(aerosimlab_communication PRIVATE
    modules/communication/src/UdpSocket.cpp
  )
  if(WIN32)
    target_sources(aerosimlab_communication PRIVATE
      modules/communication/src/UdpSocketWindows.cpp
    )
  else()
    target_sources(aerosimlab_communication PRIVATE
      modules/communication/src/UdpSocketPosix.cpp
    )
  endif()
endif()
target_include_directories(aerosimlab_communication PUBLIC modules/communication/include)
target_link_libraries(aerosimlab_communication PUBLIC aerosimlab_simulation_core)
target_link_libraries(aerosimlab_communication PRIVATE nlohmann_json::nlohmann_json)
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
if(NOT EMSCRIPTEN AND (AEROSIMLAB_BUILD_GRAPHICS OR BUILD_TESTING))
  add_library(aerosimlab_vision STATIC
    modules/vision/src/RgbFrameView.cpp
    modules/vision/src/RgbFrameQuality.cpp
  )
  target_include_directories(aerosimlab_vision PUBLIC modules/vision/include)
  target_link_libraries(aerosimlab_vision PUBLIC
    aerosimlab_sensor opencv_core opencv_imgproc)
endif()

if(NOT EMSCRIPTEN)
  add_library(aerosimlab_ground_control_client STATIC
    apps/ground_control/GroundControlClient.cpp
  )
  target_include_directories(aerosimlab_ground_control_client PUBLIC apps/ground_control)
  target_link_libraries(aerosimlab_ground_control_client PUBLIC aerosimlab_communication)
endif()

add_executable(aerosimlab_headless apps/headless/main.cpp)
target_link_libraries(aerosimlab_headless PRIVATE
  aerosimlab_scenario
  nlohmann_json::nlohmann_json
)
if(NOT EMSCRIPTEN)
  add_executable(aerosimlab_terrain_grid_converter
    tools/terrain/convert_ascii_grid.cpp
  )
  target_link_libraries(aerosimlab_terrain_grid_converter PRIVATE aerosimlab_terrain)
endif()
if(EMSCRIPTEN)
  target_link_options(aerosimlab_headless PRIVATE
    "--preload-file" "${CMAKE_SOURCE_DIR}/scenarios@scenarios"
    "-sSTACK_SIZE=1mb"
  )
endif()

if(AEROSIMLAB_BUILD_GRAPHICS)
  find_package(Git QUIET)
  aerosimlab_add_graphics_dependencies()

  if(NOT EMSCRIPTEN)
    add_library(aerosimlab_glad STATIC third_party/glad/src/glad.c)
    target_include_directories(aerosimlab_glad PUBLIC third_party/glad/include)

    FetchContent_Declare(
      aerosimlab_imgui_source
      GIT_REPOSITORY https://github.com/ocornut/imgui.git
      GIT_TAG v1.93.0-docking
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
    target_link_libraries(aerosimlab_imgui PRIVATE glfw OpenGL::GL)
  endif()

  set(AEROSIMLAB_GRAPHICS_SOURCES
    modules/graphics/src/graphics/Camera.cpp
    modules/graphics/src/graphics/IndexBuffer.cpp
    modules/graphics/src/graphics/GlbModel.cpp
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
  target_link_libraries(aerosimlab_graphics PUBLIC glm::glm
    PRIVATE glfw nlohmann_json::nlohmann_json aerosimlab_stb_image)
  if(EMSCRIPTEN)
    target_link_libraries(aerosimlab_graphics PUBLIC OpenGL::GL)
    target_link_options(aerosimlab_graphics INTERFACE "-sUSE_GLFW=3" "-sFULL_ES3=1")
  else()
    target_link_libraries(aerosimlab_graphics PUBLIC aerosimlab_glad OpenGL::GL)
  endif()
  target_compile_definitions(aerosimlab_graphics PRIVATE GLFW_INCLUDE_NONE)

  add_library(aerosimlab_simulator_app STATIC
    apps/simulator/AeroSimLabApp.cpp
    apps/simulator/AeroSimLabAppLifecycle.cpp
    apps/simulator/AeroSimLabAppBenchmark.cpp
    apps/simulator/BuildRevision.cpp
    apps/simulator/AeroSimLabAppGamepad.cpp
    apps/simulator/AeroSimLabAppInput.cpp
    apps/simulator/AeroSimLabAppMission.cpp
    apps/simulator/FleetMissionProgress.cpp
    apps/simulator/FleetDroneSpawn.cpp
    apps/simulator/TrainingRangeScene.cpp
    apps/simulator/SimulationBenchmark.cpp
  )
  if(NOT EMSCRIPTEN)
    target_sources(aerosimlab_simulator_app PRIVATE
      apps/simulator/AeroSimLabAppEoCapture.cpp
      apps/simulator/SimulatorHud.cpp
    )
  endif()
  target_include_directories(aerosimlab_simulator_app PUBLIC apps/simulator)
  set(_aerosimlab_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
  set(_aerosimlab_build_revision_header
    "${_aerosimlab_generated_dir}/AeroSimLabBuildRevision.h")
  set(_aerosimlab_revision_paths
    CMakeLists.txt
    cmake
    apps/simulator
    modules
    assets
    ":(exclude)assets/models/source"
    ":(exclude)assets/**/README.md"
    ":(exclude)assets/**/LICENSE")
  string(REPLACE ";" "," _aerosimlab_revision_paths_argument
    "${_aerosimlab_revision_paths}")
  set(_aerosimlab_build_revision_dependencies
    "${CMAKE_SOURCE_DIR}/cmake/GenerateBuildRevision.cmake")
  if(GIT_EXECUTABLE AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" rev-parse --git-path HEAD
      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
      OUTPUT_VARIABLE _aerosimlab_git_head
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    get_filename_component(_aerosimlab_git_head "${_aerosimlab_git_head}"
      ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
    if(_aerosimlab_git_head AND EXISTS "${_aerosimlab_git_head}")
      list(APPEND _aerosimlab_build_revision_dependencies "${_aerosimlab_git_head}")
      file(READ "${_aerosimlab_git_head}" _aerosimlab_git_head_contents)
      string(REGEX MATCH "^ref: (.+)$" _aerosimlab_git_ref_match
        "${_aerosimlab_git_head_contents}")
      if(CMAKE_MATCH_1)
        execute_process(
          COMMAND "${GIT_EXECUTABLE}" rev-parse --git-path "${CMAKE_MATCH_1}"
          WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
          OUTPUT_VARIABLE _aerosimlab_git_ref
          OUTPUT_STRIP_TRAILING_WHITESPACE
          ERROR_QUIET
        )
        get_filename_component(_aerosimlab_git_ref "${_aerosimlab_git_ref}"
          ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
        if(_aerosimlab_git_ref AND EXISTS "${_aerosimlab_git_ref}")
          list(APPEND _aerosimlab_build_revision_dependencies "${_aerosimlab_git_ref}")
        endif()
      endif()
    endif()
    # dirty 표시는 측정 결과의 코드 상태를 구분한다. 매 빌드 git status를
    # 실행하는 대신, 현재 Git 작업 폴더의 파일과 부모 폴더를 의존성으로 둔다.
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" ls-files --cached --others --exclude-standard
        -- ${_aerosimlab_revision_paths}
      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
      OUTPUT_VARIABLE _aerosimlab_worktree_files
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    string(REPLACE "\r\n" "\n" _aerosimlab_worktree_files
      "${_aerosimlab_worktree_files}")
    string(REPLACE "\n" ";" _aerosimlab_worktree_files
      "${_aerosimlab_worktree_files}")
    foreach(_aerosimlab_worktree_file IN LISTS _aerosimlab_worktree_files)
      if(EXISTS "${CMAKE_SOURCE_DIR}/${_aerosimlab_worktree_file}")
        list(APPEND _aerosimlab_build_revision_dependencies
          "${CMAKE_SOURCE_DIR}/${_aerosimlab_worktree_file}")
        get_filename_component(_aerosimlab_worktree_directory
          "${_aerosimlab_worktree_file}" DIRECTORY)
        while(_aerosimlab_worktree_directory)
          list(APPEND _aerosimlab_build_revision_dependencies
            "${CMAKE_SOURCE_DIR}/${_aerosimlab_worktree_directory}")
          get_filename_component(_aerosimlab_worktree_directory
            "${_aerosimlab_worktree_directory}" DIRECTORY)
        endwhile()
      endif()
    endforeach()
    # 실행 코드와 실제 자산만 추적해 문서·시험 변경으로 앱을 다시 만들지 않는다.
    list(REMOVE_DUPLICATES _aerosimlab_build_revision_dependencies)
  endif()
  add_custom_command(
    OUTPUT "${_aerosimlab_build_revision_header}"
    COMMAND "${CMAKE_COMMAND}"
      "-DSOURCE_DIR=${CMAKE_SOURCE_DIR}"
      "-DOUTPUT=${_aerosimlab_build_revision_header}"
      "-DGIT_EXECUTABLE=${GIT_EXECUTABLE}"
      "-DREVISION_PATHS=${_aerosimlab_revision_paths_argument}"
      -P "${CMAKE_SOURCE_DIR}/cmake/GenerateBuildRevision.cmake"
    DEPENDS ${_aerosimlab_build_revision_dependencies}
    VERBATIM
  )
  add_custom_target(aerosimlab_build_revision
    DEPENDS "${_aerosimlab_build_revision_header}")
  add_dependencies(aerosimlab_simulator_app aerosimlab_build_revision)
  target_include_directories(aerosimlab_simulator_app PRIVATE "${_aerosimlab_generated_dir}")
  set_source_files_properties(apps/simulator/BuildRevision.cpp PROPERTIES
    OBJECT_DEPENDS "${_aerosimlab_build_revision_header}")
  target_link_libraries(aerosimlab_simulator_app PUBLIC
    aerosimlab_graphics
    aerosimlab_simulation_core
    aerosimlab_flight_planning
    aerosimlab_manual_control
    aerosimlab_sensor
    aerosimlab_communication
    nlohmann_json::nlohmann_json
  )
  if(NOT EMSCRIPTEN)
    target_link_libraries(aerosimlab_simulator_app PRIVATE
      aerosimlab_vision aerosimlab_imgui glfw)
    target_compile_definitions(aerosimlab_simulator_app PRIVATE GLFW_INCLUDE_NONE)
  endif()
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
  set_target_properties(aerosimlab_simulator PROPERTIES OUTPUT_NAME "DroneSim")

  set(training_range_models
    "models/environment/industrial/building-a.glb"
    "models/environment/industrial/building-c.glb"
    "models/environment/industrial/building-f.glb"
    "models/environment/industrial/building-j.glb"
    "models/environment/industrial/building-m.glb"
    "models/environment/industrial/building-q.glb"
    "models/environment/industrial/building-t.glb"
    "models/environment/industrial/shipping-container-a.glb"
    "models/environment/industrial/shipping-container-b.glb"
    "models/environment/industrial/shipping-container-c.glb"
    "models/environment/industrial/detail-tank.glb"
    "models/environment/industrial/detail-tank-large.glb"
    "models/environment/industrial/chimney-large.glb"
    "models/environment/industrial/water-tower.glb"
    "models/environment/industrial/windmill.glb"
    "models/environment/industrial/Textures/colormap.png"
    "models/environment/nature/tree_oak.glb"
    "models/environment/nature/tree_pineTallA_detailed.glb"
    "models/environment/nature/rock_largeA.glb"
    "models/environment/nature/rock_largeC.glb"
    "models/environment/nature/rock_tallB.glb"
  )
  if(EMSCRIPTEN)
    set_target_properties(aerosimlab_simulator PROPERTIES SUFFIX ".html")
    # Flecs 월드 초기화에 필요한 스택을 확보한다. Emscripten 기본 64KB로는 부족하다.
    target_link_options(aerosimlab_simulator PRIVATE
      "--shell-file=${CMAKE_SOURCE_DIR}/apps/simulator/web/web_shell_single_thread.html"
      "-sALLOW_MEMORY_GROWTH=1"
      "-sSTACK_SIZE=1mb"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/shaders@shaders"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/models/drones/x500/visual.glb@models/drones/x500/visual.glb"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/models/drones/x500/action-camera.glb@models/drones/x500/action-camera.glb"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/models/drones/x500/carbon-weave.ppm@models/drones/x500/carbon-weave.ppm"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/models/drones/x500/LICENSE@models/drones/x500/LICENSE"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/models/environment/industrial/LICENSE.txt@models/environment/industrial/LICENSE.txt"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/models/environment/nature/LICENSE.txt@models/environment/nature/LICENSE.txt"
    )
    foreach(model IN LISTS training_range_models)
      target_link_options(aerosimlab_simulator PRIVATE
        "--embed-file=${CMAKE_SOURCE_DIR}/assets/${model}@${model}"
      )
    endforeach()
  else()
    set(simulator_runtime_assets
      "${CMAKE_SOURCE_DIR}/assets/shaders/Scene.glsl"
      "${CMAKE_SOURCE_DIR}/assets/models/drones/x500/visual.glb"
      "${CMAKE_SOURCE_DIR}/assets/models/drones/x500/action-camera.glb"
      "${CMAKE_SOURCE_DIR}/assets/models/drones/x500/carbon-weave.ppm"
      "${CMAKE_SOURCE_DIR}/assets/models/drones/x500/LICENSE"
      "${CMAKE_SOURCE_DIR}/assets/models/environment/industrial/LICENSE.txt"
      "${CMAKE_SOURCE_DIR}/assets/models/environment/nature/LICENSE.txt")
    foreach(model IN LISTS training_range_models)
      list(APPEND simulator_runtime_assets "${CMAKE_SOURCE_DIR}/assets/${model}")
    endforeach()

    set(simulator_asset_copy_commands)
    set(simulator_runtime_outputs)
    foreach(asset_path IN LISTS simulator_runtime_assets)
      if(NOT EXISTS "${asset_path}")
        message(FATAL_ERROR "시뮬레이터 실행 자료를 찾지 못했습니다: ${asset_path}")
      endif()
      file(RELATIVE_PATH runtime_path
        "${CMAKE_SOURCE_DIR}/assets" "${asset_path}")
      get_filename_component(runtime_directory "${runtime_path}" DIRECTORY)
      set(runtime_output "${CMAKE_CURRENT_BINARY_DIR}/${runtime_path}")
      list(APPEND simulator_runtime_outputs "${runtime_output}")
      list(APPEND simulator_asset_copy_commands
        COMMAND ${CMAKE_COMMAND} -E make_directory
          "${CMAKE_CURRENT_BINARY_DIR}/${runtime_directory}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${asset_path}"
          "${runtime_output}")
    endforeach()

    # 실행 자료 복사는 링크 단계와 분리해, 지도·질감만 바뀔 때 실행 파일을 다시 만들지 않는다.
    set(simulator_runtime_stamp
      "${CMAKE_CURRENT_BINARY_DIR}/simulator_runtime_assets.stamp")
    add_custom_command(OUTPUT "${simulator_runtime_stamp}"
      BYPRODUCTS ${simulator_runtime_outputs}
      ${simulator_asset_copy_commands}
      COMMAND ${CMAKE_COMMAND} -E touch "${simulator_runtime_stamp}"
      DEPENDS ${simulator_runtime_assets}
      VERBATIM)
    add_custom_target(aerosimlab_simulator_runtime_assets
      DEPENDS "${simulator_runtime_stamp}")
    add_dependencies(aerosimlab_simulator aerosimlab_simulator_runtime_assets)
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
    tests/flight_planning/FormationPlannerTests.cpp
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
    aerosimlab_flight_planning
    nlohmann_json::nlohmann_json
    GTest::gtest_main
  )
  gtest_discover_tests(aerosimlab_simulation_tests
    TEST_PREFIX "Simulation."
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS "UNIT;simulation"
  )

  add_executable(aerosimlab_scenario_tests
    tests/scenario/ScenarioRunnerTests.cpp
  )
  target_compile_definitions(aerosimlab_scenario_tests PRIVATE
    AEROSIMLAB_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  )
  target_link_libraries(aerosimlab_scenario_tests PRIVATE
    aerosimlab_scenario
    nlohmann_json::nlohmann_json
    GTest::gtest_main
  )
  gtest_discover_tests(aerosimlab_scenario_tests
    TEST_PREFIX "Scenario."
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS "UNIT;simulation"
  )

  add_executable(aerosimlab_communication_tests
    tests/communication/ProtocolTests.cpp
    tests/communication/VirtualMeshNetworkTests.cpp
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
  if(NOT EMSCRIPTEN)
    target_link_libraries(aerosimlab_communication_tests PRIVATE Threads::Threads)
  endif()
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

  add_test(NAME HeadlessCommunicationOutageSmoke
    COMMAND aerosimlab_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/multi-drone/communication-outage.json")
  set_tests_properties(HeadlessCommunicationOutageSmoke PROPERTIES
    LABELS "INTEGRATION;simulation"
    PASS_REGULAR_EXPRESSION "communication_status.*stale")

  add_test(NAME HeadlessMultiDroneSmoke
    COMMAND aerosimlab_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/multi-drone/independent-flight.json")
  set_tests_properties(HeadlessMultiDroneSmoke PROPERTIES
    LABELS "INTEGRATION;simulation"
    PASS_REGULAR_EXPRESSION "lead.*scout")

  add_test(NAME HeadlessCooperativeMissionSmoke
    COMMAND aerosimlab_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/multi-drone/two-point-survey.json")
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
      target_compile_definitions(aerosimlab_simulator_integration_tests PRIVATE
        AEROSIMLAB_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
      )
      target_link_libraries(aerosimlab_simulator_integration_tests PRIVATE
        aerosimlab_simulator_app
        aerosimlab_ground_control_client
        nlohmann_json::nlohmann_json
        GTest::gtest_main
        Threads::Threads
      )
      gtest_discover_tests(aerosimlab_simulator_integration_tests
        TEST_PREFIX "Simulator."
        DISCOVERY_TIMEOUT 60
        PROPERTIES
          LABELS "INTEGRATION;simulator"
          WORKING_DIRECTORY "$<TARGET_FILE_DIR:aerosimlab_simulator>"
          TIMEOUT 40
          RUN_SERIAL TRUE)
      # 통합 시험은 실행 파일이 아니라 화면 자료만 필요하므로, 실행 파일을 다시 연결하지 않는다.
      add_dependencies(aerosimlab_simulator_integration_tests
        aerosimlab_simulator_runtime_assets)
    endif()
  endif()
endif()
