cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    CACHE STRING "vcpkg CMake toolchain")
endif()

project(DroneSim VERSION 0.1.0 LANGUAGES C CXX)

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
option(DRONESIM_BUILD_GRAPHICS "드론 화면을 함께 빌드합니다" ON)
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

dronesim_add_common_dependencies()
dronesim_add_ecs_dependencies()
dronesim_add_scenario_dependencies()
dronesim_add_jolt_dependency()

add_library(dronesim_terrain STATIC
  modules/terrain/src/TerrainGrid.cpp
)
target_include_directories(dronesim_terrain PUBLIC modules/terrain/include)
target_link_libraries(dronesim_terrain PUBLIC glm::glm)

add_library(dronesim_flight_planning STATIC
  modules/flight_planning/src/BuildingObstacle.cpp
  modules/flight_planning/src/FormationPlanner.cpp
  modules/flight_planning/src/RoutePlanner.cpp
)
target_include_directories(dronesim_flight_planning PUBLIC modules/flight_planning/include)
target_link_libraries(dronesim_flight_planning PUBLIC glm::glm dronesim_terrain)

add_library(dronesim_simulation_core STATIC
  modules/simulation_core/src/BatteryModel.cpp
  modules/simulation_core/src/DroneModel.cpp
  modules/simulation_core/src/EoCameraModel.cpp
  modules/simulation_core/src/CollisionGeometry.cpp
  modules/simulation_core/src/SimulationCore.cpp
  modules/simulation_core/src/QuadrotorModel.cpp
  modules/simulation_core/src/QuadrotorFlightController.cpp
)
target_sources(dronesim_simulation_core PRIVATE
  modules/simulation_core/src/JoltPhysicsWorld.cpp
  modules/simulation_core/src/SimulationPhysicsJolt.cpp
)
target_link_libraries(dronesim_simulation_core PRIVATE Jolt::Jolt)
target_include_directories(dronesim_simulation_core PUBLIC modules/simulation_core/include)
target_link_libraries(dronesim_simulation_core
  PUBLIC glm::glm dronesim_terrain
  PRIVATE flecs::flecs_static
)

add_library(dronesim_scenario STATIC
  modules/scenario/src/ScenarioRunner.cpp
  modules/scenario/src/ScenarioParser.cpp
  modules/scenario/src/ScenarioValidation.cpp
)
target_include_directories(dronesim_scenario PUBLIC modules/scenario/include)
target_link_libraries(dronesim_scenario
  PUBLIC dronesim_simulation_core
  PRIVATE nlohmann_json::nlohmann_json
)

add_library(dronesim_communication STATIC
  modules/communication/src/DroneSocketTransport.cpp
  modules/communication/src/VirtualMeshNetwork.cpp
)
target_include_directories(dronesim_communication PUBLIC modules/communication/include)
target_link_libraries(dronesim_communication
  PUBLIC dronesim_simulation_core
  PRIVATE nlohmann_json::nlohmann_json
)
if(EMSCRIPTEN)
  target_sources(dronesim_communication PRIVATE
    modules/communication/src/DroneSocketInProcess.cpp
  )
elseif(WIN32)
  target_sources(dronesim_communication PRIVATE
    modules/communication/src/DroneSocketWindows.cpp
  )
  target_link_libraries(dronesim_communication PUBLIC ws2_32)
else()
  target_sources(dronesim_communication PRIVATE
    modules/communication/src/DroneSocketPosix.cpp
  )
endif()

add_library(dronesim_manual_control STATIC
  modules/manual_control/src/GamepadCommandMapper.cpp
)
target_include_directories(dronesim_manual_control PUBLIC modules/manual_control/include)
target_link_libraries(dronesim_manual_control PUBLIC dronesim_simulation_core)

add_library(dronesim_mission STATIC
  modules/mission/src/FleetMissionProgress.cpp
  modules/mission/src/FleetTelemetry.cpp
)
target_include_directories(dronesim_mission PUBLIC modules/mission/include)
target_link_libraries(dronesim_mission PUBLIC
  dronesim_flight_planning
  dronesim_simulation_core
)

add_executable(dronesim_headless apps/headless/main.cpp)
target_link_libraries(dronesim_headless PRIVATE
  dronesim_scenario
  nlohmann_json::nlohmann_json
)
if(NOT EMSCRIPTEN)
  add_executable(dronesim_terrain_grid_converter
    tools/terrain/convert_ascii_grid.cpp
  )
  target_link_libraries(dronesim_terrain_grid_converter PRIVATE dronesim_terrain)
endif()
if(EMSCRIPTEN)
  target_link_options(dronesim_headless PRIVATE
    "--preload-file" "${CMAKE_SOURCE_DIR}/scenarios@scenarios"
    "-sSTACK_SIZE=1mb"
  )
endif()

if(DRONESIM_BUILD_GRAPHICS)
  dronesim_add_graphics_dependencies()

  target_sources(dronesim_manual_control PRIVATE
    modules/manual_control/src/GamepadController.cpp
  )
  if(NOT EMSCRIPTEN)
    target_link_libraries(dronesim_manual_control PRIVATE glfw)
    target_compile_definitions(dronesim_manual_control PRIVATE GLFW_INCLUDE_NONE)
  endif()

  if(NOT EMSCRIPTEN)
    add_library(dronesim_glad STATIC third_party/glad/src/glad.c)
    target_include_directories(dronesim_glad PUBLIC third_party/glad/include)

    FetchContent_Declare(
      dronesim_imgui_source
      GIT_REPOSITORY https://github.com/ocornut/imgui.git
      GIT_TAG v1.93.0-docking
      GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(dronesim_imgui_source)
    if(NOT dronesim_imgui_source_POPULATED)
      FetchContent_Populate(dronesim_imgui_source)
    endif()
    add_library(dronesim_imgui STATIC
      "${dronesim_imgui_source_SOURCE_DIR}/imgui.cpp"
      "${dronesim_imgui_source_SOURCE_DIR}/imgui_draw.cpp"
      "${dronesim_imgui_source_SOURCE_DIR}/imgui_tables.cpp"
      "${dronesim_imgui_source_SOURCE_DIR}/imgui_widgets.cpp"
      "${dronesim_imgui_source_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
      "${dronesim_imgui_source_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
    )
    target_include_directories(dronesim_imgui PUBLIC
      "${dronesim_imgui_source_SOURCE_DIR}"
      "${dronesim_imgui_source_SOURCE_DIR}/backends"
    )
    target_compile_definitions(dronesim_imgui PRIVATE GLFW_INCLUDE_NONE)
    target_link_libraries(dronesim_imgui PRIVATE glfw OpenGL::GL)
  endif()

  set(DRONESIM_GRAPHICS_SOURCES
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
    list(APPEND DRONESIM_GRAPHICS_SOURCES
      modules/graphics/src/graphics/MultisampleFramebuffer.cpp
    )
  endif()

  add_library(dronesim_graphics STATIC ${DRONESIM_GRAPHICS_SOURCES})
  target_include_directories(dronesim_graphics PUBLIC
    modules/graphics/include
  )
  target_link_libraries(dronesim_graphics PUBLIC glm::glm
    PRIVATE glfw nlohmann_json::nlohmann_json dronesim_stb_image)
  if(EMSCRIPTEN)
    target_link_libraries(dronesim_graphics PUBLIC OpenGL::GL)
    target_link_options(dronesim_graphics INTERFACE "-sUSE_GLFW=3" "-sFULL_ES3=1")
  else()
    target_link_libraries(dronesim_graphics PUBLIC dronesim_glad OpenGL::GL)
  endif()
  target_compile_definitions(dronesim_graphics PRIVATE GLFW_INCLUDE_NONE)

  if(NOT EMSCRIPTEN)
    add_library(dronesim_gui STATIC
      modules/gui/src/SimulatorHud.cpp
      modules/gui/src/SimulatorView.cpp
    )
    target_include_directories(dronesim_gui PUBLIC modules/gui/include)
    target_link_libraries(dronesim_gui
      PUBLIC dronesim_communication dronesim_simulation_core dronesim_flight_planning
      PRIVATE dronesim_graphics dronesim_imgui glfw
    )
    target_compile_definitions(dronesim_gui PRIVATE GLFW_INCLUDE_NONE)
  endif()

  add_library(dronesim_performance STATIC
    modules/performance/src/BenchmarkSession.cpp
    modules/performance/src/SimulationBenchmark.cpp
  )
  target_include_directories(dronesim_performance PUBLIC
    modules/performance/include
  )
  target_link_libraries(dronesim_performance PUBLIC
    dronesim_graphics
    dronesim_simulation_core
    nlohmann_json::nlohmann_json
  )

  add_library(dronesim_scene STATIC
    modules/scene/src/TrainingRangeScene.cpp
  )
  target_include_directories(dronesim_scene PUBLIC modules/scene/include)
  target_link_libraries(dronesim_scene PUBLIC
    dronesim_graphics
    dronesim_simulation_core
    dronesim_flight_planning
  )

  add_library(dronesim_simulator_app STATIC
    apps/simulator/DroneSimApp.cpp
    apps/simulator/DroneSimAppLifecycle.cpp
    apps/simulator/DroneSimAppGamepad.cpp
    apps/simulator/DroneSimAppMission.cpp
  )
  target_include_directories(dronesim_simulator_app PUBLIC
    apps/simulator/include
    modules/gui/include
  )
  target_link_libraries(dronesim_simulator_app PUBLIC
    dronesim_graphics
    dronesim_performance
    dronesim_simulation_core
    dronesim_flight_planning
    dronesim_manual_control
    dronesim_mission
    dronesim_scene
    dronesim_communication
    nlohmann_json::nlohmann_json
  )
  if(NOT EMSCRIPTEN)
    target_link_libraries(dronesim_simulator_app PUBLIC
      dronesim_gui
      PRIVATE glfw
    )
    target_compile_definitions(dronesim_simulator_app PRIVATE GLFW_INCLUDE_NONE)
  endif()
  if(WIN32)
    target_link_libraries(dronesim_simulator_app PRIVATE Psapi)
  endif()

  if(WIN32)
    add_executable(dronesim_simulator WIN32 apps/simulator/main.cpp)
    target_link_libraries(dronesim_simulator PRIVATE dronesim_simulator_app shell32)
  else()
    add_executable(dronesim_simulator apps/simulator/main.cpp)
    target_link_libraries(dronesim_simulator PRIVATE dronesim_simulator_app)
  endif()
  set_target_properties(dronesim_simulator PROPERTIES OUTPUT_NAME "DroneSim")

  # 맵 원본은 자산 배포 조건 때문에 저장소에 넣지 않고 로컬에서 배치한다.
  set(city_map_asset "maps/ruined_city.glb")
  if(EMSCRIPTEN)
    set_target_properties(dronesim_simulator PROPERTIES SUFFIX ".html")
    # Flecs 월드 초기화에 필요한 스택을 확보한다. Emscripten 기본 64KB로는 부족하다.
    target_link_options(dronesim_simulator PRIVATE
      "--shell-file=${CMAKE_SOURCE_DIR}/apps/simulator/web/web_shell_single_thread.html"
      "-sALLOW_MEMORY_GROWTH=1"
      "-sSTACK_SIZE=1mb"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/shaders@shaders"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/models/drones/x500/visual.glb@models/drones/x500/visual.glb"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/models/drones/x500/cgo3-gimbal.glb@models/drones/x500/cgo3-gimbal.glb"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/models/drones/x500/carbon-weave.ppm@models/drones/x500/carbon-weave.ppm"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/models/drones/x500/LICENSE@models/drones/x500/LICENSE"
    )
    foreach(model IN LISTS city_map_asset)
      target_link_options(dronesim_simulator PRIVATE
        "--embed-file=${CMAKE_SOURCE_DIR}/assets/${model}@${model}"
      )
    endforeach()
  else()
    set(simulator_runtime_assets
      "${CMAKE_SOURCE_DIR}/assets/shaders/Scene.glsl"
      "${CMAKE_SOURCE_DIR}/assets/models/drones/x500/visual.glb"
      "${CMAKE_SOURCE_DIR}/assets/models/drones/x500/cgo3-gimbal.glb"
      "${CMAKE_SOURCE_DIR}/assets/models/drones/x500/carbon-weave.ppm"
      "${CMAKE_SOURCE_DIR}/assets/models/drones/x500/LICENSE")
    foreach(model IN LISTS city_map_asset)
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
    add_custom_target(dronesim_simulator_runtime_assets
      DEPENDS "${simulator_runtime_stamp}")
    add_dependencies(dronesim_simulator dronesim_simulator_runtime_assets)
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
  add_executable(dronesim_simulation_tests
    tests/simulation_core/SimulationCoreTests.cpp
    tests/flight_planning/FormationPlannerTests.cpp
    tests/simulation_core/QuadrotorModelTests.cpp
    tests/terrain/TerrainGridTests.cpp
  )
  target_compile_definitions(dronesim_simulation_tests PRIVATE
    DRONESIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  )
  if(NOT EMSCRIPTEN)
    target_sources(dronesim_simulation_tests PRIVATE
      tests/simulation_core/JoltPhysicsWorldTests.cpp
    )
  endif()
  target_link_libraries(dronesim_simulation_tests PRIVATE
    dronesim_simulation_core
    dronesim_flight_planning
    nlohmann_json::nlohmann_json
    GTest::gtest_main
  )
  gtest_discover_tests(dronesim_simulation_tests
    TEST_PREFIX "Simulation."
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS "UNIT;simulation"
  )

  add_executable(dronesim_scenario_tests
    tests/scenario/ScenarioRunnerTests.cpp
  )
  target_compile_definitions(dronesim_scenario_tests PRIVATE
    DRONESIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  )
  target_link_libraries(dronesim_scenario_tests PRIVATE
    dronesim_scenario
    nlohmann_json::nlohmann_json
    GTest::gtest_main
  )
  gtest_discover_tests(dronesim_scenario_tests
    TEST_PREFIX "Scenario."
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS "UNIT;simulation"
  )

  add_executable(dronesim_communication_tests
    tests/communication/VirtualMeshNetworkTests.cpp
  )
  target_link_libraries(dronesim_communication_tests PRIVATE
    dronesim_communication
    GTest::gtest_main
  )
  gtest_discover_tests(dronesim_communication_tests
    TEST_PREFIX "Communication."
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS "UNIT;communication"
  )

  add_executable(dronesim_manual_control_tests
    tests/manual_control/GamepadCommandMapperTests.cpp
  )
  target_link_libraries(dronesim_manual_control_tests PRIVATE
    dronesim_manual_control
    GTest::gtest_main
  )
  gtest_discover_tests(dronesim_manual_control_tests
    TEST_PREFIX "ManualControl."
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS "UNIT;manual_control"
  )

  add_test(NAME HeadlessScenarioSmoke
    COMMAND dronesim_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/flight/normal-flight.json")
  set_tests_properties(HeadlessScenarioSmoke PROPERTIES
    LABELS "INTEGRATION;simulation")

  add_test(NAME HeadlessCommunicationOutageSmoke
    COMMAND dronesim_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/multi-drone/communication-outage.json")
  set_tests_properties(HeadlessCommunicationOutageSmoke PROPERTIES
    LABELS "INTEGRATION;simulation")

  add_test(NAME HeadlessMultiDroneSmoke
    COMMAND dronesim_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/multi-drone/independent-flight.json")
  set_tests_properties(HeadlessMultiDroneSmoke PROPERTIES
    LABELS "INTEGRATION;simulation")

  add_test(NAME HeadlessCooperativeMissionSmoke
    COMMAND dronesim_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/multi-drone/two-point-survey.json")
  set_tests_properties(HeadlessCooperativeMissionSmoke PROPERTIES
    LABELS "INTEGRATION;simulation")

  add_test(NAME HeadlessFourDroneSurveySmoke
    COMMAND dronesim_headless --scenario "${CMAKE_SOURCE_DIR}/scenarios/multi-drone/four-drone-survey.json")
  set_tests_properties(HeadlessFourDroneSurveySmoke PROPERTIES
    LABELS "INTEGRATION;simulation")

  if(DRONESIM_BUILD_GRAPHICS)
    add_test(NAME ViewerSmoke COMMAND dronesim_simulator --smoke-test)
    set_tests_properties(ViewerSmoke PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:dronesim_simulator>"
      TIMEOUT 60
      RUN_SERIAL TRUE
      LABELS "INTEGRATION;graphics")

    if(NOT EMSCRIPTEN)
      add_executable(dronesim_simulator_integration_tests
        tests/simulator/SimulatorSessionTests.cpp
        tests/graphics/CameraTests.cpp
      )
      target_compile_definitions(dronesim_simulator_integration_tests PRIVATE
        DRONESIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
      )
      target_link_libraries(dronesim_simulator_integration_tests PRIVATE
        dronesim_simulator_app
        nlohmann_json::nlohmann_json
        GTest::gtest_main
      )
      gtest_discover_tests(dronesim_simulator_integration_tests
        TEST_PREFIX "Simulator."
        DISCOVERY_TIMEOUT 60
        PROPERTIES
          LABELS "INTEGRATION;simulator"
          WORKING_DIRECTORY "$<TARGET_FILE_DIR:dronesim_simulator>"
          TIMEOUT 40
          RUN_SERIAL TRUE)
      # 통합 시험은 실행 파일이 아니라 화면 자료만 필요하므로, 실행 파일을 다시 연결하지 않는다.
      add_dependencies(dronesim_simulator_integration_tests
        dronesim_simulator_runtime_assets)
    endif()
  endif()
endif()
