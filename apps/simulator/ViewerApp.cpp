#include "ViewerApp.h"

#include "simulation/robotics/RobotEnvironmentCollisionGuard.h"
#include "simulation/robotics/RobotModelAdapter.h"

#include "graphics/Camera.h"
#include "PickPlaceScenario.h"
#include "application/PickPlaceConfig.h"
#include "scene/Entity.h"
#include "graphics/OrbitCameraController.h"
#include "rendering/RenderContext.h"
#include "graphics/Renderer.h"
#include "rendering/systems/RenderSystemModule.h"
#include "graphics/Shader.h"
#include "scene/TransformSystemModule.h"
#include "graphics/Window.h"

#include "rendering/assets/AssetManager.h"
#include "model/GltfLoader.h"
#include "model/RobotModel.h"
#include "rendering/assets/PrefabFactory.h"

#include "physics/PhysicsWorld.h"
#include "application/PickPlaceMission.h"
#include "gui/GuiModule.h"
#include "gui/panels/GripperPanel.h"
#include "gui/panels/RobotPanel.h"
#include "gui/panels/PhysicsDebugPanel.h"
#include "gui/overlays/ColliderOverlay.h"
#include "application/PickPlaceScenarioSampler.h"
#include "simulation/robotics/GripperColliders.h"
#include "simulation/robotics/GripperGraspAdapter.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/systems/PhysicsSystemModule.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/backends/simulation/SimGripperController.h"
#include "robotics/models/robotiq/TwoF85.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/kinematics/GripperKinematics.h"

#include "scene/Scene.h"

#include "simulation/robotics/RobotTransformAdapter.h"
#include "simulation/robotics/GripperTransformAdapter.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <glm/gtc/quaternion.hpp>
#include <imgui.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#if GRASPLINK_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#else
#define ZoneScopedN(name) ((void)0)
#define FrameMark ((void)0)
#endif

namespace grasplink::simulator
{
using grasplink::graphics::Camera;
using grasplink::graphics::OrbitCameraController;
using grasplink::graphics::Renderer;
using grasplink::graphics::Shader;
using grasplink::graphics::Window;
using grasplink::model::GltfLoader;
using grasplink::model::ModelResource;
using grasplink::rendering::AssetManager;
using grasplink::rendering::RenderContext;
using grasplink::rendering::RenderSystemModule;
using grasplink::rendering::prefab_factory::CreateModel;
using grasplink::scene::Entity;
using grasplink::scene::Scene;
using grasplink::scene::TransformSystemModule;

namespace
{

// Viewer 창은 시작할 때 이 픽셀 크기로 요청한다. 실제 렌더 대상 크기는 DPI 배율에 따라 framebuffer에서 다시 읽는다.
namespace viewer_window
{
inline constexpr int width = 1280;
inline constexpr int height = 720;
inline constexpr char title[] = "Simulation | GraspLink";
}

// 로봇 제어기와 물리 시뮬레이션은 화면 프레임률과 관계없이 250 Hz, 즉 4 ms 간격으로 갱신한다.
namespace viewer_control
{
inline constexpr double frequencyHz = 250.0;
inline constexpr double fixedDeltaSeconds = 1.0 / frequencyHz;
}

// 창 이동이나 디버거 정지 뒤 긴 시간이 쌓여도 한 프레임에서 최대 100 ms만 시뮬레이션에 누적한다.
namespace viewer_frame
{
inline constexpr double maximumDeltaSeconds = 0.1;
inline constexpr double smokeTestDeltaSeconds = 1.0 / 60.0;
inline constexpr std::size_t smokeTestFrameCount = 8;
}



// 카메라의 시작 위치와 바라볼 지점은 Scene의 World 좌표 [m]로 지정한다.
namespace viewer_camera
{
inline const glm::vec3 position{2.0F, 1.35F, 1.15F};
inline const glm::vec3 target{0.05F, 0.50F, 0.40F};
}

namespace viewer_layout
{
    // 사이드바 폭을 화면 크기에 맞추고 지나치게 좁거나 넓지 않게 한다.
    inline constexpr float sidebarPreferredFraction = 0.36F;
    inline constexpr float sidebarMaximumFraction = 0.48F;
    inline constexpr float sidebarMinimumWidth = 260.0F;
    inline constexpr float sidebarMaximumWidth = 460.0F;
}

using SimRobotController = grasplink::robotics::backends::simulation::SimRobotController;
using RobotTransformAdapter = grasplink::simulation::robotics::RobotTransformAdapter;
using PhysicsWorld = grasplink::physics::PhysicsWorld;

// World 행렬에는 Entity의 크기 변경도 들어가므로 축 길이를 제거한 뒤 회전만 quaternion으로 바꾼다.
glm::quat RotationFromWorldMatrix(const glm::mat4& worldMatrix)
{
    glm::mat3 axes(worldMatrix);
    for (int axis = 0; axis < 3; ++axis)
        axes[axis] = glm::normalize(axes[axis]);
    return glm::normalize(glm::quat_cast(axes));
}

grasplink::robotics::CartesianPose ToRobotBasePose(
    const Entity& robotRoot, const glm::mat4& worldMatrix, float heightOffset)
{
    const glm::mat4 baseWorld = robotRoot.GetWorldMatrix();
    const glm::vec3 position(glm::inverse(baseWorld) * worldMatrix[3]);
    const glm::quat baseInverseRotation = glm::inverse(RotationFromWorldMatrix(baseWorld));
    const glm::quat orientation = glm::normalize(
        baseInverseRotation * RotationFromWorldMatrix(worldMatrix));
    return {{position.x, position.y + heightOffset, position.z},
        {orientation.x, orientation.y, orientation.z, orientation.w}};
}

} // namespace


ViewerApp::ViewerApp(ViewerOptions options)
    : m_Options(options),
      m_ControlLoop(viewer_control::fixedDeltaSeconds,
          viewer_frame::maximumDeltaSeconds)
{
}


ViewerApp::~ViewerApp()
{
    Shutdown();
}


int ViewerApp::Run()
{
    try
    {
        if (!Init())
        {
            m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "viewer.init", "Viewer 초기화에 실패했습니다.");
            Shutdown();
            return -1;
        }

        MainLoop();
        Shutdown();
        return 0;
    }
    catch (const std::exception& error)
    {
        try
        {
            m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "viewer.run", error.what());
        }
        catch (...)
        {
        }
        Shutdown();
        throw;
    }
    catch (...)
    {
        try
        {
            m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "viewer.run", "알 수 없는 예외가 발생했습니다.");
        }
        catch (...)
        {
        }
        Shutdown();
        throw;
    }
}


bool ViewerApp::Init()
{
    // 먼저 OpenGL Context를 만들고, 그 Context를 사용하는 Scene과 GPU 모델, 로봇 계산기, Physics Body 순서로 연결한다.
    // 중간 단계가 실패해도 Run()의 종료 경로가 Shutdown()을 호출해 그때까지 만들어진 자원을 정리한다.
    if (!InitViewer())
        return false;

    Entity robotRoot;
    Entity floorEntity;
    InitScene(robotRoot, floorEntity);
    m_RobotRoot = robotRoot;

    if (!InitRobot(robotRoot))
        return false;

    if (!InitGripper(robotRoot))
        return false;

    InitPhysics(robotRoot, floorEntity);
    m_RobotModel = {};
    m_AssetManager.reset();
    m_RobotShader.reset();

    return true;
}


bool ViewerApp::InitViewer()
{
    // Window가 OpenGL Context를 소유한다. Renderer와 AssetManager의 GPU 자원은 이 Context 안에서 만든다.
    m_Window = std::make_unique<Window>(
        Window::Properties{viewer_window::width, viewer_window::height,
            viewer_window::title,
            !m_Options.smokeTest, !m_Options.smokeTest});

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

    // Renderer는 논리 창 크기가 아니라 DPI 배율이 반영된 실제 framebuffer 크기로 초기화해야 픽셀과 화면이 맞는다.
    m_Renderer = std::make_unique<Renderer>();
    m_Renderer->Init(framebufferWidth, framebufferHeight);

    const float aspectRatio = static_cast<float>(viewer_window::width) /
        static_cast<float>(viewer_window::height);
    m_Camera = std::make_unique<Camera>(
        viewer_camera::position, viewer_camera::target, aspectRatio);
    m_GuiModule = std::make_unique<grasplink::gui::GuiModule>(*m_Window);
    m_ColliderOverlay = std::make_unique<grasplink::gui::ColliderOverlay>(m_World);

    m_CameraController = std::make_unique<OrbitCameraController>(*m_Camera, *m_Window);

    // Flecs World에 Renderer와 Camera의 포인터를 전달하지만 이 자료는 World가 소유하지 않는다.
    // Shutdown에서는 이 포인터를 읽는 system과 World를 두 객체보다 먼저 정리한다.
    m_World.set<RenderContext>({m_Renderer.get(), m_Camera.get()});

    // Transform은 fixed step의 물리 전후와 렌더 직전에 계산한다. RenderSystem은 World 진행 시 렌더 대상을 처리한다.
    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();

    m_Scene = std::make_unique<Scene>(m_World);

    m_AssetManager = std::make_unique<AssetManager>();

    return true;
}


void ViewerApp::InitScene(Entity& robotRoot, Entity& floorEntity)
{
    m_RobotShader = Shader::Create("shaders/Robot.glsl");
    auto gridShader = Shader::Create("shaders/Grid.glsl");

    // GLB를 읽어 CPU 모델 자료를 만든 다음 Mesh와 Material을 OpenGL 자원으로 업로드한다.
    auto loadedRobot = grasplink::model::RobotModelLoader::Load(m_Options.robotDirectory);
    m_RobotDescription = std::move(loadedRobot.robot);
    m_RobotSpecificationAdapter = std::make_unique<grasplink::simulation::robotics::RobotModelAdapter>(
        m_RobotDescription);
    m_RobotModel = std::move(loadedRobot.visual);
    m_AssetManager->UploadModel(m_RobotModel);

    // GLB의 node 부모 관계를 Flecs Entity 계층으로 복사해 모델에 정의된 관절과 링크를 찾는다.
    robotRoot = CreateModel(
        *m_Scene,
        m_RobotModel,
        *m_AssetManager,
        m_RobotShader);

    // 바닥 Mesh를 만들고, InitPhysics()에서 같은 Entity에 움직이지 않는 Static Collider를 연결한다.
    ModelResource planeModel = GltfLoader::LoadGLB("plane.glb");
    m_AssetManager->UploadModel(planeModel);

    floorEntity = CreateModel(
        *m_Scene,
        planeModel,
        *m_AssetManager,
        gridShader);

}


bool ViewerApp::InitRobot(const Entity& robotRoot)
{
    // 로봇 모델 사양에서 관절 수, 회전축, 허용 범위와 최대 속도를 가져온다.
    const auto& robotSpec = m_RobotSpecificationAdapter->Specification();

    // 모델 계층이 보관한 Local 변환을 합친 뒤 TCP를 정한다. World 변환의 역행렬을 사용하므로 로봇 전체 배치는 TCP 보정에 포함되지 않는다.
    TransformSystemModule::UpdateWorldTransforms(m_World);
    auto simController = std::make_unique<SimRobotController>(robotSpec);

    const auto connectResult = simController->Connect();

    if (!connectResult)
    {
        m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "robot.connect", connectResult.message);
        return false;
    }

    m_RobotController = std::move(simController);

    // 관절 각도로 각 링크의 위치와 회전을 계산하는 정기구학(Forward Kinematics) 결과를 화면 Entity와 충돌용 단순 형상에도 적용해 두 자세가 어긋나지 않게 한다.
    m_RobotKinematics = std::make_unique<grasplink::robotics::kinematics::RobotKinematics>(robotSpec);
    m_RobotTransformAdapter = std::make_unique<RobotTransformAdapter>(robotRoot, robotSpec);

    return true;
}


bool ViewerApp::InitGripper(const Entity& robotRoot)
{
    const Entity gripperRoot = robotRoot.FindChildByNameRecursive("Gripper");
    const auto& specification = grasplink::robotics::models::robotiq::kTwoF85;
    if (!gripperRoot)
        return true;
    for (std::size_t joint = 0; joint < specification.jointCount; ++joint)
        if (!robotRoot.FindChildByNameRecursive(std::string{specification.joints[joint].name}))
            return true;

    m_GripperController = std::make_unique<grasplink::robotics::backends::simulation::SimGripperController>(specification);
    m_PickPlaceMission = std::make_unique<grasplink::application::PickPlaceMission>();
    auto result = m_GripperController->Connect();
    if (result)
        result = m_GripperController->Activate();
    if (!result)
    {
        m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "gripper.initialize", result.message);
        return false;
    }
    m_GripperKinematics = std::make_unique<grasplink::robotics::kinematics::GripperKinematics>(specification);
    m_GripperTransformAdapter = std::make_unique<grasplink::simulation::robotics::GripperTransformAdapter>(
        gripperRoot, specification);

    // 데모와 smoke test에서는 테스트할 수 있도록 그리퍼를 실제로 닫는다. 일반 Viewer는 열린 기준 자세에서 시작한다.
    if (m_Options.smokeTest)
    {
        const auto result = m_GripperController->Command({255, 255, 128});
        if (!result)
        {
            m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "gripper.smoke_test_command", result.message);
            return false;
        }
    }
    return true;
}


void ViewerApp::ApplyControllerPoses()
{
    const auto& robotPose = m_RobotKinematics->Update(m_RobotController->GetStateView());
    m_RobotTransformAdapter->Apply(robotPose);
    m_RobotPhysicsAdapter->Apply(robotPose);
    // 한 고정 tick에서 팔의 관절 자세와 그리퍼의 부모 기준 Local 회전을 함께 적용한다. 이어지는 World 변환 갱신으로 화면과 충돌 프록시가 같은 계층 자세를 얻는다.
    if (m_GripperController)
    {
        const auto& gripperPose = m_GripperKinematics->Update(m_GripperController->GetState());
        m_GripperTransformAdapter->Apply(gripperPose);
    }
}


void ViewerApp::InitPhysics(const Entity& robotRoot, Entity& floorEntity)
{
    m_PhysicsWorld = std::make_unique<PhysicsWorld>();
    pick_place::ConfigureFloor(floorEntity);
    m_RobotPhysicsAdapter = std::make_unique<grasplink::simulation::RobotPhysicsAdapter>(
        *m_Scene, robotRoot, m_RobotDescription);
    InitGripperScene(robotRoot);

    // Physics Body를 만들기 전에 첫 FK 자세와 계층 World 행렬을 계산해 화면 Entity와 Kinematic 목표를 같은 위치에 맞춘다.
    ApplyControllerPoses();
    TransformSystemModule::UpdateWorldTransforms(m_World);
    m_PhysicsSystemModule = std::make_unique<grasplink::simulation::PhysicsSystemModule>(
        m_World, *m_PhysicsWorld, &m_Logger);
    InitGripperPhysics(robotRoot);

    std::optional<grasplink::simulation::robotics::RobotEnvironmentCollisionGuard::GripperSupport>
        gripperSupport;
    if (m_GripperController)
    {
        gripperSupport.emplace(grasplink::simulation::robotics::RobotEnvironmentCollisionGuard::GripperSupport{
            robotRoot.FindChildByNameRecursive("Gripper"), *m_GripperController,
            *m_GripperKinematics, *m_GripperTransformAdapter});
    }

    m_RobotCollisionGuard = std::make_unique<grasplink::simulation::robotics::RobotEnvironmentCollisionGuard>(
        *m_RobotController, *m_RobotKinematics, *m_RobotTransformAdapter, *m_RobotPhysicsAdapter,
        m_World, *m_PhysicsWorld, *m_PhysicsSystemModule, robotRoot, std::move(gripperSupport));
}

void ViewerApp::InitGripperScene(const Entity& robotRoot)
{
    if (!m_GripperController)
        return;

    // 현재 2F-85 예제는 GLB 메시에서 충돌용 외피를 만들고, 원본 관절 Entity의 자식으로 붙인다.
    // 그래서 관절 회전과 Scene 변환 갱신 뒤에 충돌 외피도 같은 자세를 따른다.
    grasplink::simulation::ConfigureTwoF85Colliders(*m_Scene, robotRoot, m_RobotModel);
    m_GraspBox = pick_place::CreateGraspBox(*m_Scene, m_RobotShader);
    m_PlacementArea = pick_place::CreatePlacementArea(*m_Scene, m_RobotShader);
    m_ScenarioSampler = std::make_unique<grasplink::application::PickPlaceScenarioSampler>();
    pick_place::RandomizePickPlaceScene(m_GraspBox, m_PlacementArea, *m_ScenarioSampler);
}

void ViewerApp::InitGripperPhysics(const Entity& robotRoot)
{
    if (!m_GripperController)
        return;

    m_GripperGraspAdapter = std::make_unique<grasplink::simulation::GripperGraspAdapter>(
        *m_PhysicsWorld, *m_PhysicsSystemModule, *m_GripperController);
    if (!m_GripperGraspAdapter->Bind(robotRoot))
        throw std::runtime_error("그리퍼 충돌 관절을 물리 계산에 연결하지 못했습니다.");
}


void ViewerApp::AdvanceSimulationTick(double fixedDeltaSeconds)
{
    if (m_GripperGraspAdapter)
        m_GripperGraspAdapter->BeforePhysicsStep();
    m_RobotCollisionGuard->CaptureSafeJointPose();
    {
        ZoneScopedN("RobotUpdate");
        m_RobotController->Update(fixedDeltaSeconds);
        if (m_GripperController)
            m_GripperController->Update(fixedDeltaSeconds);
        ApplyControllerPoses();
    }
    TransformSystemModule::UpdateWorldTransforms(m_World);
    m_RobotCollisionGuard->RestoreSafePoseIfOverlapping();
    {
        ZoneScopedN("PhysicsStep");
        m_PhysicsSystemModule->Step(fixedDeltaSeconds);
    }
    if (m_GripperGraspAdapter)
        m_GripperGraspAdapter->AfterPhysicsStep();
    TransformSystemModule::UpdateWorldTransforms(m_World);
}

void ViewerApp::AdvancePlanningBudget()
{
    // 호출자는 계획이 진행 중일 때만 들어온다. 한 프레임에서 계획이 화면 갱신 시간을 모두 쓰지 않도록 제한한다.
    using Clock = std::chrono::steady_clock;
    constexpr auto planningBudget = std::chrono::microseconds{1500};
    const auto deadline = Clock::now() + planningBudget;
    do
    {
        m_RobotController->AdvanceMotionPlanning(1);
    } while (m_RobotController->IsMotionPlanning() && Clock::now() < deadline);
}

bool ViewerApp::UpdateMission(const grasplink::robotics::CartesianPose& boxPose,
    const grasplink::robotics::CartesianPose& placementPose)
{
    m_PickPlaceMission->Update(m_RobotController->GetStateView(), *m_RobotController,
        *m_GripperController, m_GripperGraspAdapter->GetState().grasped, boxPose, placementPose);
    if (!m_PickPlaceMission->ConsumeSuccessEvent())
        return false;

    RandomizeScenario();
    if (m_GripperGraspAdapter)
        m_GripperGraspAdapter->Release();
    m_PickPlaceMission->PrepareNextTask();
    return true;
}

void ViewerApp::RandomizeScenario()
{
    pick_place::RandomizePickPlaceScene(m_GraspBox, m_PlacementArea, *m_ScenarioSampler);
    TransformSystemModule::UpdateWorldTransforms(m_World);
    const glm::mat4 boxTransform = m_GraspBox.GetWorldMatrix();
    const auto boxHandle = m_PhysicsSystemModule->GetBodyHandle(m_GraspBox.GetHandle());
    const glm::quat boxRotation = RotationFromWorldMatrix(boxTransform);
    m_PhysicsWorld->SetBodyTransform(boxHandle, grasplink::physics::Transform{
        glm::vec3(boxTransform[3]), boxRotation});
}

void ViewerApp::MainLoop()
{
    using Clock = std::chrono::steady_clock;

    m_LastFrameTime = Clock::now();
    m_RenderedFrames = 0;

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg([](void* userData)
    {
        auto* app = static_cast<ViewerApp*>(userData);
        if (app->m_Window->ShouldClose())
        {
            app->Shutdown();
            emscripten_cancel_main_loop();
            return;
        }
        try
        {
            app->MainLoopFrame();
        }
        catch (const std::exception& error)
        {
            std::cerr << "[Fatal Error] " << error.what() << std::endl;
            app->Shutdown();
            emscripten_cancel_main_loop();
        }
    }, this, 0, 1);
#else
    while (!m_Window->ShouldClose())
    {
        MainLoopFrame();
        if (m_Options.smokeTest &&
            m_RenderedFrames >= viewer_frame::smokeTestFrameCount)
            break;
    }
#endif
}

grasplink::gui::RobotPanelActions ViewerApp::DrawControlPanel(float sceneDisplayWidth,
    float sidebarWidth, float displayHeight, const grasplink::gui::RobotPanelView& view,
    const grasplink::simulation::GripperGraspState& graspState)
{
    ImGui::SetNextWindowPos(ImVec2(sceneDisplayWidth, 0.0F), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(sidebarWidth, displayHeight), ImGuiCond_Always);
    constexpr ImGuiWindowFlags controlWindowFlags = ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoSavedSettings;
    ImGui::Begin(m_RobotDescription.name.c_str(), nullptr, controlWindowFlags);
    ImGui::TextDisabled("GRASPLINK     SIMULATION CONSOLE");
    ImGui::Spacing();

    grasplink::gui::RobotPanelActions actions;
    if (ImGui::BeginTabBar("Control sections"))
    {
        if (ImGui::BeginTabItem("Robot"))
        {
            actions = grasplink::gui::DrawRobotPanelContents(view);
            ImGui::EndTabItem();
        }
        if (m_GripperController && ImGui::BeginTabItem("Gripper"))
        {
            m_GripperPanel.DrawContents(*m_GripperController, &graspState,
                !m_RobotController->IsMotionPlanning());
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Physics"))
        {
            m_PhysicsDebugPanel.DrawContents();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
    return actions;
}

void ViewerApp::MainLoopFrame()
{
    using Clock = std::chrono::steady_clock;

    ZoneScopedN("Frame");
    FrameMark;
    const auto currentFrameTime = Clock::now();
    const double frameDeltaSeconds = m_Options.smokeTest ? viewer_frame::smokeTestDeltaSeconds
        : std::chrono::duration<double>(currentFrameTime - m_LastFrameTime).count();

    m_LastFrameTime = currentFrameTime;

    // 디버거 정지 등으로 한 프레임이 길어져도 제어와 물리 누적기에 최대 100 ms만 전달한다.
    const double clampedFrameDeltaSeconds = std::min(frameDeltaSeconds, viewer_frame::maximumDeltaSeconds);
    const float renderDeltaSeconds = static_cast<float>(clampedFrameDeltaSeconds);
    // 창 이벤트를 처리하고, ImGui가 마우스를 사용하지 않을 때만 카메라 입력을 전달한다.
    m_Window->PollEvents();
    if (!m_GuiModule->WantsMouse())
        m_CameraController->OnUpdate();

    const bool planningFrame = m_RobotController->IsMotionPlanning();
    if (planningFrame)
        AdvancePlanningBudget();

    // 경로 계획 중에는 Jolt와 로봇 상태를 고정해 계획에 사용한 충돌 환경을 유지한다.
    // 일반 tick에서는 Jolt에 Kinematic 목표를 보내고 Dynamic 결과를 ECS에 반영한 뒤 World 행렬을 다시 계산한다.
    if (!planningFrame)
    {
        m_ControlLoop.Advance(frameDeltaSeconds, [this](double fixedDeltaSeconds)
        {
            ZoneScopedN("FixedTick");
            AdvanceSimulationTick(fixedDeltaSeconds);
        });
    }

    grasplink::robotics::CartesianPose boxPose{};
    grasplink::robotics::CartesianPose placementPose{};
    if (m_PickPlaceMission)
    {
        boxPose = ToRobotBasePose(m_RobotRoot, m_GraspBox.GetWorldMatrix(), 0.0F);
        placementPose = ToRobotBasePose(m_RobotRoot, m_PlacementArea.GetWorldMatrix(),
            grasplink::application::pick_place::config::boxSideMeters * 0.5F);
        if (UpdateMission(boxPose, placementPose))
        {
            boxPose = ToRobotBasePose(m_RobotRoot, m_GraspBox.GetWorldMatrix(), 0.0F);
            placementPose = ToRobotBasePose(m_RobotRoot, m_PlacementArea.GetWorldMatrix(),
                grasplink::application::pick_place::config::boxSideMeters * 0.5F);
        }
    }
    int framebufferWidth = 0;
    int framebufferHeight = 0;

    m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

    // 최소화된 창이나 아직 크기가 정해지지 않은 canvas는 렌더링만 건너뛴다.
    if (framebufferWidth <= 0 || framebufferHeight <= 0)
        return;

    m_Renderer->Resize(framebufferWidth, framebufferHeight);
    m_GuiModule->BeginFrame();

    const ImGuiIO& guiIo = ImGui::GetIO();
    const float displayWidth = std::max(guiIo.DisplaySize.x, 1.0F);
    const float displayHeight = std::max(guiIo.DisplaySize.y, 1.0F);
    const float sidebarWidth = std::min(displayWidth * viewer_layout::sidebarMaximumFraction,
        std::clamp(displayWidth * viewer_layout::sidebarPreferredFraction,
            viewer_layout::sidebarMinimumWidth,
            viewer_layout::sidebarMaximumWidth));
    const float sceneDisplayWidth = std::max(displayWidth - sidebarWidth, 1.0F);
    const float horizontalScale = static_cast<float>(framebufferWidth) / displayWidth;
    const int sceneWidth = std::clamp(static_cast<int>(std::lround(sceneDisplayWidth * horizontalScale)),
        1, framebufferWidth);
    const float aspectRatio = static_cast<float>(sceneWidth) / static_cast<float>(framebufferHeight);
    m_Camera->SetAspectRatio(aspectRatio);
    m_Renderer->SetSceneViewport(0, 0, sceneWidth, framebufferHeight);

    {
        ZoneScopedN("Render");
        m_Renderer->BeginFrame();

        TransformSystemModule::UpdateWorldTransforms(m_World);
        m_World.progress(renderDeltaSeconds);
        const auto graspState = m_GripperGraspAdapter
            ? m_GripperGraspAdapter->GetState()
            : grasplink::simulation::GripperGraspState{};
        const auto& robotState = m_RobotController->GetStateView();
        std::optional<grasplink::gui::RobotPanelMissionView> missionView;
        if (m_PickPlaceMission)
        {
            const auto mission = m_PickPlaceMission->Snapshot();
            missionView.emplace(grasplink::gui::RobotPanelMissionView{mission.stageLabel, mission.lastMessage,
                mission.completedCount, grasplink::application::pick_place::config::linearVelocityMetersPerSecond,
                grasplink::application::pick_place::config::angularVelocityRadiansPerSecond, mission.paused,
                mission.missionSucceeded, mission.autoRepeat, mission.hasResult,
                mission.lastRequestAccepted, mission.canStart});
        }
        const grasplink::gui::RobotPanelView robotPanelView{robotState,
            m_RobotController->GetSpecification(), boxPose, placementPose, missionView};
        const auto panelActions = DrawControlPanel(sceneDisplayWidth, sidebarWidth, displayHeight,
            robotPanelView, graspState);
        if (m_PickPlaceMission)
            m_PickPlaceMission->ApplyActions(grasplink::application::PickPlaceMissionActions{
                panelActions.start, panelActions.resume, panelActions.stop},
                m_RobotController->GetStateView(), *m_RobotController, graspState.grasped, boxPose);
        m_ColliderOverlay->Draw(*m_Camera, m_PhysicsDebugPanel.IsColliderVisible(),
            ImVec2(sceneDisplayWidth, displayHeight));
        m_Renderer->EndFrame();
        m_GuiModule->EndFrame();
        m_Window->SwapBuffers();
    }
    ++m_RenderedFrames;
}

void ViewerApp::Shutdown()
{
    // GUI와 GPU 객체를 모두 정리한 다음 Window를 파괴해 OpenGL Context를 마지막에 닫는다.
    m_ColliderOverlay.reset();
    m_PhysicsDebugPanel = {};
    m_GripperPanel = {};
    // 파지 제약은 연결된 Body와 PhysicsWorld를 빌려 쓰므로 Scene, Controller, 물리 시스템이 살아 있을 때 먼저 해제한다.
    m_GripperGraspAdapter.reset();
    // Controller가 검증 함수를 보관하므로 guard 소멸자가 참조를 해제할 때까지 Controller가 살아 있어야 한다.
    m_RobotCollisionGuard.reset();
    // Scene Entity handle을 빌린 Adapter를 Scene보다 먼저 파괴해 소멸 처리 중 이미 삭제된 handle을 참조하지 않게 한다.
    m_RobotTransformAdapter.reset();
    m_GripperTransformAdapter.reset();
    m_RobotPhysicsAdapter.reset();
    m_RobotKinematics.reset();
    m_GripperKinematics.reset();

    if (m_RobotController)
    {
        m_RobotController->Disconnect();
    }

    m_RobotController.reset();
    if (m_GripperController)
        m_GripperController->Disconnect();
    m_GripperController.reset();

    // Scene을 파괴하면 ECS 삭제 observer가 Jolt Body를 제거한다.
    // Scene을 지우는 callback이 Jolt 물체를 정리하므로 PhysicsWorld가 살아 있을 때 Scene을 먼저 지운다.
    m_RobotRoot = {};
    m_GraspBox = {};
    m_PlacementArea = {};
    m_Scene.reset();
    m_PhysicsSystemModule.reset();
    m_GuiModule.reset();

    // 모든 Scene Body가 제거된 뒤 observer와 ECS World를 해제하고, 마지막으로 Jolt PhysicsWorld를 파괴한다.
    m_World.reset();
    m_PhysicsWorld.reset();

    m_AssetManager.reset();
    m_RobotShader.reset();
    m_ScenarioSampler.reset();

    // 사이드바 폭을 화면 크기에 맞추고 지나치게 좁거나 넓지 않게 한다.
    m_CameraController.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();

    m_Logger.Shutdown();
}

} // namespace grasplink::simulator
