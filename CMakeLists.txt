cmake_minimum_required(VERSION 3.20)

project(GraspLink VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8 "$<$<COMPILE_LANGUAGE:CXX>:/EHsc>")
endif()

if(EMSCRIPTEN)
  set(BUILD_TESTING OFF CACHE BOOL "Disable native tests for the browser build" FORCE)
  set(GRASPLINK_ENABLE_TRACY OFF CACHE BOOL "Disable Tracy for the browser build" FORCE)
  set(GRASPLINK_BUILD_BENCHMARKS OFF CACHE BOOL "Disable native benchmarks for the browser build" FORCE)
  option(GRASPLINK_WEB_PTHREADS "Enable pthreads in the Emscripten build" ON)
  set(GRASPLINK_WEB_SHELL "${CMAKE_SOURCE_DIR}/apps/simulator/web_shell.html")
  if(NOT GRASPLINK_WEB_PTHREADS)
    set(GRASPLINK_WEB_SHELL "${CMAKE_SOURCE_DIR}/apps/simulator/web_shell_single_thread.html")
  endif()
  if(GRASPLINK_WEB_PTHREADS)
    add_compile_options(-pthread)
  endif()
  add_link_options(
    "-sALLOW_MEMORY_GROWTH=1"
    "-sUSE_GLFW=3"
    "-sFULL_ES3=1"
    "-sINITIAL_MEMORY=67108864"
    "-sSINGLE_FILE=1"
    "-sSINGLE_FILE_BINARY_ENCODE=0"
    "SHELL:-sSTACK_SIZE=8388608"
  )
  if(GRASPLINK_WEB_PTHREADS)
    add_link_options(-pthread "-sPTHREAD_POOL_SIZE=8")
  endif()
endif()

include(cmake/Dependencies.cmake)
grasplink_add_common_dependencies()
include(CTest)

option(GRASPLINK_BUILD_GRAPHICS "Build the OpenGL viewer" ON)
option(GRASPLINK_ENABLE_TRACY "Enable Tracy runtime profiling" ON)
option(GRASPLINK_BUILD_BENCHMARKS "Build FK, IK, and path-planning benchmarks" ${BUILD_TESTING})

if(GRASPLINK_BUILD_GRAPHICS AND GRASPLINK_ENABLE_TRACY)
  set(TRACY_ENABLE ON CACHE BOOL "Enable Tracy runtime profiling" FORCE)
  set(TRACY_ON_DEMAND ON CACHE BOOL "Capture Tracy data only when a profiler connects" FORCE)
  FetchContent_Declare(
    tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG v0.14.1
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(tracy)
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
endif()

if(GRASPLINK_BUILD_BENCHMARKS)
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Disable Google Benchmark tests" FORCE)
  set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "Disable Google Benchmark GoogleTest tests" FORCE)
  set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "Disable Google Benchmark installation" FORCE)
  FetchContent_Declare(
    googlebenchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.9.5
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(googlebenchmark)
endif()


# ============================================================
# Diagnostics
# ============================================================
#
# Logger는 domain/graphics 모듈과 독립된 비동기 log와 metric 기록을 제공한다.
#
add_library(grasplink_diagnostics STATIC
  modules/diagnostics/src/Logger.cpp
)

target_include_directories(grasplink_diagnostics PUBLIC
  modules/diagnostics/include
)

find_package(Threads REQUIRED)
target_link_libraries(grasplink_diagnostics PUBLIC Threads::Threads)


# ============================================================
# Robotics
# ============================================================
#
# 이 라이브러리는 로봇과 그리퍼 모델 사양, 하드웨어·시뮬레이션 backend의 공통 계약을 제공한다.
# Viewer, Flecs, GLM에 의존하지 않으므로 화면이나 ECS가 없는 로봇 코드에서도 사용할 수 있다.
#
add_library(grasplink_robotics STATIC
  modules/robotics/src/backends/simulation/SimRobotController.cpp
  modules/robotics/src/backends/simulation/SimRobotControllerMotion.cpp
  modules/robotics/src/planning/LinearPathPlanner.cpp
  modules/robotics/src/planning/JointPathPlanner.cpp
  modules/robotics/src/planning/StateValidity.cpp
  modules/robotics/src/backends/simulation/SimGripperController.cpp
  modules/robotics/src/kinematics/RobotKinematics.cpp
  modules/robotics/src/kinematics/RobotInverseKinematics.cpp
  modules/robotics/src/kinematics/GripperKinematics.cpp
)

target_include_directories(grasplink_robotics PUBLIC
  modules/robotics/include
)
target_link_libraries(grasplink_robotics PUBLIC Eigen3::Eigen)
if(GRASPLINK_BUILD_GRAPHICS AND GRASPLINK_ENABLE_TRACY)
  target_compile_definitions(grasplink_robotics PRIVATE GRASPLINK_ENABLE_TRACY=1)
  target_link_libraries(grasplink_robotics PRIVATE Tracy::TracyClient)
else()
  target_compile_definitions(grasplink_robotics PRIVATE GRASPLINK_ENABLE_TRACY=0)
endif()

# ============================================================
# Physics
# ============================================================
#
# Jolt Physics 의존성을 먼저 구성한다. 아래 물리 라이브러리가 링크할 Jolt target이 이 단계에서 만들어진다.
#
# 따라서 이 설정은 add_library(grasplink_physics ...)보다 앞에 있어야 이후 target_link_libraries(... Jolt)가 링크할 대상을 찾을 수 있다.
#
grasplink_add_physics_dependencies()


# 물리 라이브러리 target을 먼저 만든다.
#
# target_sources(), target_include_directories(), target_link_libraries()는 이 add_library 호출 뒤에 실행해야 설정이 해당 라이브러리에 적용된다.
#
add_library(grasplink_physics STATIC
  modules/physics/src/PhysicsWorld.cpp
)

target_include_directories(grasplink_physics PUBLIC
  modules/physics/include
)

target_link_libraries(grasplink_physics PUBLIC
  Jolt
  glm::glm
)

if(BUILD_TESTING)
  include(GoogleTest)

  function(grasplink_add_test target name source)
    cmake_parse_arguments(PARSE_ARGV 3 TEST "GTEST_DISCOVERY;GTEST_GROUPED;INTEGRATION;RUN_SERIAL" "TIMEOUT;WORKING_DIRECTORY;CATEGORY;FILTER" "")
    add_executable(${target} ${source})
    # Keep shared test fixtures available after splitting test sources by module.
    target_include_directories(${target} PRIVATE "${CMAKE_SOURCE_DIR}/tests")
    if(TEST_INTEGRATION)
      set(test_kind_label INTEGRATION)
    else()
      set(test_kind_label UNIT)
    endif()
    set(test_labels ${test_kind_label})
    if(NOT "${TEST_CATEGORY}" STREQUAL "")
      list(APPEND test_labels "${TEST_CATEGORY}")
    endif()
    if(TEST_GTEST_DISCOVERY OR TEST_GTEST_GROUPED)
      target_link_libraries(${target} PRIVATE ${TEST_UNPARSED_ARGUMENTS} GTest::gtest_main)
      set(discovery_options)
      set(test_properties)
      if(NOT "${TEST_TIMEOUT}" STREQUAL "")
        list(APPEND test_properties TIMEOUT "${TEST_TIMEOUT}")
      endif()
      if(NOT "${TEST_WORKING_DIRECTORY}" STREQUAL "")
        list(APPEND test_properties WORKING_DIRECTORY "${TEST_WORKING_DIRECTORY}")
      endif()
      if(TEST_RUN_SERIAL)
        list(APPEND test_properties RUN_SERIAL TRUE)
      endif()
      list(APPEND test_properties LABELS "${test_labels}")
      if(TEST_GTEST_DISCOVERY)
        if(NOT "${TEST_FILTER}" STREQUAL "")
          list(APPEND discovery_options TEST_FILTER "${TEST_FILTER}")
        endif()
        list(APPEND discovery_options PROPERTIES ${test_properties})
        gtest_discover_tests(${target} TEST_PREFIX "${name}." DISCOVERY_TIMEOUT 60 ${discovery_options})
      else()
        add_test(NAME ${name} COMMAND ${target})
        set_tests_properties(${name} PROPERTIES LABELS "${test_labels}")
        if(TEST_TIMEOUT)
          set_tests_properties(${name} PROPERTIES TIMEOUT "${TEST_TIMEOUT}")
        endif()
        if(NOT "${TEST_WORKING_DIRECTORY}" STREQUAL "")
          set_tests_properties(${name} PROPERTIES WORKING_DIRECTORY "${TEST_WORKING_DIRECTORY}")
        endif()
        if(TEST_RUN_SERIAL)
          set_tests_properties(${name} PROPERTIES RUN_SERIAL TRUE)
        endif()
      endif()
    else()
      target_link_libraries(${target} PRIVATE ${TEST_UNPARSED_ARGUMENTS})
      add_test(NAME ${name} COMMAND ${target})
      set_tests_properties(${name} PROPERTIES LABELS "${test_labels}")
      if(TEST_TIMEOUT)
        set_tests_properties(${name} PROPERTIES TIMEOUT "${TEST_TIMEOUT}")
      endif()
      if(NOT "${TEST_WORKING_DIRECTORY}" STREQUAL "")
        set_tests_properties(${name} PROPERTIES WORKING_DIRECTORY "${TEST_WORKING_DIRECTORY}")
      endif()
    endif()
  endfunction()

  grasplink_add_test(grasplink_diagnostics_tests Logger tests/diagnostics/LoggerTests.cpp grasplink_diagnostics GTEST_DISCOVERY CATEGORY diagnostics)
  grasplink_add_test(grasplink_control_tests ControlRuntime tests/robotics/ControlRuntimeTests.cpp grasplink_robotics GTEST_DISCOVERY CATEGORY robotics FILTER "ControlRuntimeTests.FixedLoopAndControllerContracts")
  grasplink_add_test(grasplink_kinematics_tests RobotKinematics tests/robotics/RobotKinematicsTests.cpp grasplink_robotics GTEST_DISCOVERY CATEGORY robotics)
  grasplink_add_test(grasplink_gripper_kinematics_tests GripperKinematics tests/robotics/GripperKinematicsTests.cpp grasplink_robotics GTEST_DISCOVERY CATEGORY robotics)
  grasplink_add_test(grasplink_inverse_kinematics_tests RobotInverseKinematics tests/robotics/RobotInverseKinematicsTests.cpp grasplink_robotics GTEST_GROUPED CATEGORY robotics)
  grasplink_add_test(grasplink_robot_motion_tests RobotMotion tests/robotics/RobotMotionTests.cpp grasplink_robotics GTEST_DISCOVERY CATEGORY robotics FILTER "RobotMotion.MovePoseAndFailurePreservation:RobotMotion.JointStateValidityAndSelfCollisionDiagnostics")
endif()

# ============================================================
# Graphics / Viewer
# ============================================================

grasplink_add_scene_dependencies()
grasplink_add_model_dependencies()

add_library(grasplink_scene STATIC
  modules/scene/src/Entity.cpp
  modules/scene/src/scene/Scene.cpp
  modules/scene/src/systems/TransformSystemModule.cpp
)
target_include_directories(grasplink_scene PUBLIC
  modules/scene/include
)
target_link_libraries(grasplink_scene PUBLIC flecs::flecs_static glm::glm)
target_compile_definitions(grasplink_scene PRIVATE GLM_ENABLE_EXPERIMENTAL)

add_library(grasplink_model STATIC
  modules/model/src/GltfLoader.cpp
  modules/model/src/RobotModelLoader.cpp
)
target_include_directories(grasplink_model PUBLIC modules/model/include)
target_link_libraries(grasplink_model PUBLIC glm::glm PRIVATE tinygltf tinyxml2)
target_compile_definitions(grasplink_model PRIVATE GLM_ENABLE_EXPERIMENTAL)

add_library(grasplink_simulation STATIC
  modules/simulation/src/SimulationSceneBuilder.cpp
  modules/simulation/src/robotics/RobotTransformAdapter.cpp
  modules/simulation/src/robotics/GripperTransformAdapter.cpp
  modules/simulation/src/robotics/RobotEnvironmentCollisionGuard.cpp
  modules/simulation/src/robotics/RobotModelAdapter.cpp
  modules/simulation/src/robotics/RobotModelTransforms.cpp
  modules/simulation/src/robotics/CollisionGeometry.cpp
  modules/simulation/src/robotics/GripperColliders.cpp
  modules/simulation/src/robotics/GripperGraspAdapter.cpp
  modules/simulation/src/robotics/RobotCollisionGeometryBuilder.cpp
  modules/simulation/src/robotics/RobotPhysicsAdapter.cpp
  modules/simulation/src/systems/PhysicsSystemModule.cpp
)
target_include_directories(grasplink_simulation PUBLIC modules/simulation/include)
target_link_libraries(grasplink_simulation PUBLIC
  grasplink_scene
  grasplink_model
  grasplink_physics
  grasplink_robotics
  PRIVATE grasplink_diagnostics
)
target_compile_definitions(grasplink_simulation PRIVATE GLM_ENABLE_EXPERIMENTAL)
if(GRASPLINK_BUILD_GRAPHICS AND GRASPLINK_ENABLE_TRACY)
  target_compile_definitions(grasplink_simulation PRIVATE GRASPLINK_ENABLE_TRACY=1)
  target_link_libraries(grasplink_simulation PRIVATE Tracy::TracyClient)
else()
  target_compile_definitions(grasplink_simulation PRIVATE GRASPLINK_ENABLE_TRACY=0)
endif()

add_library(grasplink_application STATIC
  modules/application/src/PickPlaceMission.cpp
  modules/application/src/PickPlaceScenarioSampler.cpp
)
target_include_directories(grasplink_application PUBLIC modules/application/include)
target_link_libraries(grasplink_application PUBLIC grasplink_robotics glm::glm)
target_compile_definitions(grasplink_application PRIVATE GLM_ENABLE_EXPERIMENTAL)

if(BUILD_TESTING)
  grasplink_add_test(grasplink_robot_model_tests RobotModelLoader tests/model/RobotModelLoaderTests.cpp grasplink_model GTEST_DISCOVERY CATEGORY model)
  grasplink_add_test(grasplink_gltf_loader_tests GltfLoader tests/model/GltfLoaderTests.cpp grasplink_model GTEST_DISCOVERY CATEGORY model)
  target_compile_definitions(grasplink_robot_model_tests PRIVATE GRASPLINK_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

  grasplink_add_test(grasplink_pick_place_tests PickPlaceMission tests/application/PickPlaceMissionContractTests.cpp grasplink_application GTEST_DISCOVERY CATEGORY application)
  grasplink_add_test(grasplink_physics_ecs_tests PhysicsEcsIntegration tests/integration/PhysicsEcsIntegrationTests.cpp grasplink_simulation GTEST_DISCOVERY INTEGRATION CATEGORY integration FILTER "PhysicsEcsIntegration.SceneHierarchyPhysicsContactsAndLifetime")
  grasplink_add_test(grasplink_collision_tests RobotCollisionGeometry tests/integration/RobotCollisionGeometryTests.cpp grasplink_simulation GTEST_DISCOVERY INTEGRATION CATEGORY integration WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" FILTER "RobotCollisionGeometry.SyntheticModelBuildsIndependentFkProxies:RobotCollisionGeometry.AllowedCollisionMatrixExcludesOnlyConnectedPairs:RobotCollisionGeometry.Hcr12aBaseAndArmProxiesPassOverlapValidation")
endif()

if(GRASPLINK_BUILD_GRAPHICS)

  # Viewer가 사용하는 OpenGL, GLFW, Flecs, TinyGLTF 등의 외부 의존성을 구성한다.
  grasplink_add_graphics_dependencies()

  if(NOT EMSCRIPTEN)
    add_library(grasplink_glad STATIC third_party/glad/src/glad.c)
    target_include_directories(grasplink_glad PUBLIC third_party/glad/include)
  endif()

  set(GRASPLINK_GRAPHICS_SOURCES
    modules/viewer/src/graphics/Camera.cpp
    modules/viewer/src/graphics/OrbitCameraController.cpp
    modules/viewer/src/graphics/IndexBuffer.cpp
    modules/viewer/src/graphics/Material.cpp
    modules/viewer/src/graphics/Mesh.cpp
    modules/viewer/src/graphics/Renderer.cpp
    modules/viewer/src/graphics/Shader.cpp
    modules/viewer/src/graphics/Texture.cpp
    modules/viewer/src/graphics/VertexArray.cpp
    modules/viewer/src/graphics/VertexBuffer.cpp
    modules/viewer/src/graphics/Window.cpp
  )
  if(NOT EMSCRIPTEN)
    list(APPEND GRASPLINK_GRAPHICS_SOURCES
      modules/viewer/src/graphics/MultisampleFramebuffer.cpp
    )
  endif()

  add_library(grasplink_graphics STATIC ${GRASPLINK_GRAPHICS_SOURCES})
  target_include_directories(grasplink_graphics PUBLIC
    modules/viewer/include
  )
  target_link_libraries(grasplink_graphics PUBLIC
    glfw glm::glm grasplink_model
  )
  if(EMSCRIPTEN)
    target_link_libraries(grasplink_graphics PUBLIC OpenGL::GL)
  else()
    target_link_libraries(grasplink_graphics PUBLIC grasplink_glad OpenGL::GL)
  endif()
  target_compile_definitions(grasplink_graphics PUBLIC GLFW_INCLUDE_NONE)

  # ImGui UI backend는 공통 그래픽 코드와 분리해 GUI 모듈 의존성으로 구성한다.
  grasplink_add_gui_dependencies()
  FetchContent_GetProperties(imgui SOURCE_DIR imgui_SOURCE_DIR)
  add_library(grasplink_imgui_backend STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  )
  target_include_directories(grasplink_imgui_backend PUBLIC ${imgui_SOURCE_DIR})
  target_link_libraries(grasplink_imgui_backend PUBLIC glfw OpenGL::GL)


  # ----------------------------------------------------------
  # Viewer
  # ----------------------------------------------------------

  set(GRASPLINK_RENDERING_SOURCES
    modules/viewer/src/systems/RenderSystemModule.cpp
    modules/viewer/src/assets/AssetManager.cpp
    modules/viewer/src/assets/PrefabFactory.cpp
  )

  add_library(grasplink_rendering STATIC
    ${GRASPLINK_RENDERING_SOURCES}
  )

  target_include_directories(grasplink_rendering PUBLIC
    modules/viewer/include
  )


  # 공개 헤더에 쓰이는 자료형만 사용자에게 전달하고 GPU 구현은 내부에 둔다.
  target_link_libraries(grasplink_rendering
    PUBLIC
    grasplink_scene
    grasplink_model
    PRIVATE
    grasplink_graphics
  )

  target_compile_definitions(grasplink_rendering PRIVATE
    GLM_ENABLE_EXPERIMENTAL
  )

  add_library(grasplink_gui STATIC
    modules/gui/src/GuiModule.cpp
    modules/gui/src/panels/GripperPanel.cpp
    modules/gui/src/panels/RobotPanel.cpp
    modules/gui/src/panels/PhysicsDebugPanel.cpp
    modules/gui/src/overlays/ColliderOverlay.cpp
  )

  target_include_directories(grasplink_gui PUBLIC
    modules/gui/include
  )

  # GUI 공개 헤더는 로봇 자료와 Flecs World를 노출한다. 화면·시뮬레이션 구현은 GUI 안에서만 쓴다.
  target_link_libraries(grasplink_gui
    PUBLIC
    grasplink_robotics
    flecs::flecs_static
    PRIVATE
    grasplink_simulation
    grasplink_graphics
    grasplink_imgui_backend
  )

  target_compile_definitions(grasplink_gui PRIVATE
    GLM_ENABLE_EXPERIMENTAL
  )


  # ----------------------------------------------------------
  # Simulator Application
  # ----------------------------------------------------------

  set(GRASPLINK_APP_SOURCES
    apps/simulator/main.cpp
    apps/simulator/ViewerApp.cpp
    apps/simulator/PickPlaceScenario.cpp
  )

  add_executable(grasplink_simulator
    ${GRASPLINK_APP_SOURCES}
  )

  if(EMSCRIPTEN)
    set_target_properties(grasplink_simulator PROPERTIES
      OUTPUT_NAME index
      SUFFIX .html
    )
    target_link_options(grasplink_simulator PRIVATE
      "--shell-file=${GRASPLINK_WEB_SHELL}"
    )
    set_property(TARGET grasplink_simulator APPEND PROPERTY LINK_DEPENDS
      "${GRASPLINK_WEB_SHELL}"
      "${CMAKE_SOURCE_DIR}/cmake/EmbedEmscriptenPackage.py"
    )
    if(GRASPLINK_WEB_PTHREADS)
      set_property(TARGET grasplink_simulator APPEND PROPERTY LINK_DEPENDS
        "${CMAKE_SOURCE_DIR}/third_party/coi-serviceworker.min.js"
      )
    endif()
  target_link_options(grasplink_simulator PRIVATE
      "--preload-file=${CMAKE_SOURCE_DIR}/assets/shaders@/shaders"
      "--preload-file=${CMAKE_SOURCE_DIR}/assets/robots@/robots"
      "--preload-file=${CMAKE_SOURCE_DIR}/assets/plane.glb@/plane.glb"
    )
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_custom_command(TARGET grasplink_simulator POST_BUILD
      COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_SOURCE_DIR}/cmake/EmbedEmscriptenPackage.py"
        "$<TARGET_FILE:grasplink_simulator>"
        "$<TARGET_FILE_DIR:grasplink_simulator>/index.data"
      VERBATIM
    )
    if(GRASPLINK_WEB_PTHREADS)
      add_custom_command(TARGET grasplink_simulator POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${CMAKE_SOURCE_DIR}/third_party/coi-serviceworker.min.js"
          "$<TARGET_FILE_DIR:grasplink_simulator>/coi-serviceworker.min.js"
        VERBATIM
      )
    endif()
  endif()

  target_include_directories(grasplink_simulator PRIVATE
    apps/simulator
  )

  target_link_libraries(grasplink_simulator PRIVATE
    grasplink_gui
    grasplink_graphics
    grasplink_imgui_backend
    grasplink_application
    grasplink_simulation
    grasplink_rendering
    grasplink_robotics
    grasplink_physics
    grasplink_diagnostics
  )

  # BUILD_TESTING은 Emscripten 빌드에서 꺼지므로 재사용 브라우저 모듈은 시뮬레이터 target과 함께 정의한다.
  if(EMSCRIPTEN AND GRASPLINK_WEB_PTHREADS)
    add_executable(grasplink_embed ${GRASPLINK_APP_SOURCES})
    set_target_properties(grasplink_embed PROPERTIES
      OUTPUT_NAME grasplink-embed
      SUFFIX .js
    )
    target_include_directories(grasplink_embed PRIVATE apps/simulator)
    target_link_libraries(grasplink_embed PRIVATE
      grasplink_gui
      grasplink_graphics
      grasplink_imgui_backend
      grasplink_application
      grasplink_simulation
      grasplink_rendering
      grasplink_robotics
      grasplink_physics
      grasplink_diagnostics
    )
    target_compile_definitions(grasplink_embed PRIVATE GRASPLINK_ENABLE_TRACY=0)
    target_link_options(grasplink_embed PRIVATE
      "-sMODULARIZE=1"
      "-sEXPORT_NAME=createGraspLink"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/shaders@/shaders"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/robots@/robots"
      "--embed-file=${CMAKE_SOURCE_DIR}/assets/plane.glb@/plane.glb"
    )
  endif()

  target_compile_definitions(grasplink_simulator PRIVATE
    GRASPLINK_ENABLE_TRACY=$<BOOL:${GRASPLINK_ENABLE_TRACY}>
  )
  if(GRASPLINK_ENABLE_TRACY)
    target_link_libraries(grasplink_simulator PRIVATE Tracy::TracyClient)
  endif()


  # ----------------------------------------------------------
  # Runtime Assets
  # ----------------------------------------------------------
  #
  # 실행 파일이 자산을 찾을 수 있도록 Shader와 GLB 파일을 실행 파일 디렉터리에 복사한다.
  #
  if(NOT EMSCRIPTEN)
  add_custom_command(
    TARGET grasplink_simulator
    POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/assets/shaders
      $<TARGET_FILE_DIR:grasplink_simulator>/shaders

    COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/assets/robots
      $<TARGET_FILE_DIR:grasplink_simulator>/robots

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_SOURCE_DIR}/assets/plane.glb
      $<TARGET_FILE_DIR:grasplink_simulator>/plane.glb
  )
  endif()

  if(BUILD_TESTING)
    if(NOT EMSCRIPTEN)
      grasplink_add_test(grasplink_texture_state_tests GraphicsResource tests/viewer/TextureStateTests.cpp grasplink_graphics grasplink_rendering GTEST_DISCOVERY INTEGRATION RUN_SERIAL CATEGORY graphics TIMEOUT 60 WORKING_DIRECTORY "$<TARGET_FILE_DIR:grasplink_simulator>")
    endif()

    foreach(axis_count IN ITEMS 4 7)
      add_test(NAME ViewerSmoke${axis_count}Axis
        COMMAND grasplink_simulator --smoke-test --robot "${CMAKE_SOURCE_DIR}/tests/assets/robots/axis${axis_count}")
      set_tests_properties(ViewerSmoke${axis_count}Axis PROPERTIES
        WORKING_DIRECTORY "$<TARGET_FILE_DIR:grasplink_simulator>" TIMEOUT 60 RUN_SERIAL TRUE LABELS "INTEGRATION;e2e")
    endforeach()

    add_test(NAME ViewerSmoke6Axis
      COMMAND grasplink_simulator --smoke-test --robot "${CMAKE_SOURCE_DIR}/assets/robots/HCR12A")
    set_tests_properties(ViewerSmoke6Axis PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:grasplink_simulator>" TIMEOUT 60 RUN_SERIAL TRUE LABELS "INTEGRATION;e2e")

  endif()

endif()

if(GRASPLINK_BUILD_BENCHMARKS)
  add_executable(grasplink_robotics_benchmarks
    benchmarks/RoboticsBenchmarks.cpp
    benchmarks/BenchmarkSupport.cpp
    benchmarks/ModelBenchmarks.cpp
    benchmarks/KinematicsBenchmarks.cpp
    benchmarks/LinearPathBenchmarks.cpp
    benchmarks/LinearPathStressBenchmarks.cpp
    benchmarks/PosePlanBenchmarks.cpp
    benchmarks/MotionBenchmarks.cpp
  )
  target_link_libraries(grasplink_robotics_benchmarks PRIVATE
    grasplink_application
    grasplink_simulation
    grasplink_robotics
    grasplink_model
    grasplink_physics
    benchmark::benchmark
  )
  target_compile_definitions(grasplink_robotics_benchmarks PRIVATE GRASPLINK_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
endif()
