#include "AeroSimLabApp.h"

#include "graphics/Camera.h"
#include "graphics/Material.h"
#include "graphics/Mesh.h"
#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "graphics/Window.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>

namespace aerosimlab::simulator
{
namespace
{
constexpr int windowWidth = 1280;
constexpr int windowHeight = 720;
constexpr double maximumFrameSeconds = 0.1;
constexpr double fixedStepSeconds = 1.0 / 60.0;
constexpr std::size_t smokeTestFrameLimit = 8;
constexpr std::size_t demoLengthTicks = 900;

glm::vec3 ToRenderPosition(const glm::dvec3& positionMeters)
{
    // 모의는 Z축을 위로 쓰고 화면은 Y축을 위로 쓴다. Y축 방향은 화면의 -Z로 옮겨 오른손 좌표계를 유지한다.
    return {static_cast<float>(positionMeters.x), static_cast<float>(positionMeters.z),
        static_cast<float>(-positionMeters.y)};
}

glm::mat4 CubeTransform(const glm::vec3& position, const glm::vec3& size)
{
    return glm::translate(glm::mat4{1.0F}, position) * glm::scale(glm::mat4{1.0F}, size);
}
}

AeroSimLabApp::AeroSimLabApp(bool smokeTest)
    : smokeTest_(smokeTest)
{
}

AeroSimLabApp::~AeroSimLabApp() = default;

int AeroSimLabApp::Run()
{
    InitializeGraphics();
    auto previousFrame = std::chrono::steady_clock::now();

    while (!window_->ShouldClose())
    {
        window_->PollEvents();
        const auto now = std::chrono::steady_clock::now();
        const double frameSeconds = smokeTest_
            ? fixedStepSeconds
            : std::chrono::duration<double>(now - previousFrame).count();
        previousFrame = now;

        AdvanceSimulation(std::min(frameSeconds, maximumFrameSeconds));
        RenderFrame();
        window_->SwapBuffers();
        ++renderedFrames_;

        if (smokeTest_ && renderedFrames_ >= smokeTestFrameLimit)
            break;
    }
    return 0;
}

void AeroSimLabApp::InitializeGraphics()
{
    window_ = std::make_unique<aerosimlab::graphics::Window>(
        aerosimlab::graphics::Window::Properties{windowWidth, windowHeight,
            "AeroSimLab | 드론 관찰 · EO 영상", !smokeTest_, !smokeTest_});

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    window_->GetFramebufferSize(framebufferWidth, framebufferHeight);

    renderer_ = std::make_unique<aerosimlab::graphics::Renderer>();
    renderer_->Init(framebufferWidth, framebufferHeight);
    observerCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{7.0F, 5.0F, 7.0F}, glm::vec3{0.0F}, 1.0F);
    eoCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, 1.0F, 70.0F, 0.05F, 200.0F);

    cube_ = std::shared_ptr<aerosimlab::graphics::Mesh>(
        aerosimlab::graphics::Mesh::CreateCube(1.0F));
    shader_ = aerosimlab::graphics::Shader::Create("shaders/Scene.glsl");
    droneMaterial_ = std::make_shared<aerosimlab::graphics::Material>(
        glm::vec4{0.12F, 0.48F, 0.32F, 1.0F}, 0.0F, 0.75F);
    propellerMaterial_ = std::make_shared<aerosimlab::graphics::Material>(
        glm::vec4{0.08F, 0.09F, 0.10F, 1.0F}, 0.0F, 0.9F);
    groundMaterial_ = std::make_shared<aerosimlab::graphics::Material>(
        glm::vec4{0.35F, 0.39F, 0.34F, 1.0F}, 0.0F, 1.0F);
    buildingMaterial_ = std::make_shared<aerosimlab::graphics::Material>(
        glm::vec4{0.63F, 0.59F, 0.48F, 1.0F}, 0.0F, 0.9F);
}

void AeroSimLabApp::AdvanceSimulation(double elapsedSeconds)
{
    accumulatedSeconds_ += elapsedSeconds;
    while (accumulatedSeconds_ >= fixedStepSeconds)
    {
        ApplyDemoCommands();
        simulation_.AdvanceOneTick();
        accumulatedSeconds_ -= fixedStepSeconds;
    }
}

void AeroSimLabApp::ApplyDemoCommands()
{
    using Kind = aerosimlab::simulation::DroneCommand::Kind;
    const std::size_t tick = static_cast<std::size_t>(simulation_.TickCount()) % demoLengthTicks;
    aerosimlab::simulation::DroneCommand command;
    bool hasCommand = true;

    if (tick == 0)
        command.kind = Kind::TakeOff;
    else if (tick == 120)
    {
        command.kind = Kind::SetWaypoint;
        command.vector = {8.0, 0.0, 2.0};
    }
    else if (tick == 300)
    {
        command.kind = Kind::SetYawRate;
        command.yawRateRadiansPerSecond = 0.35;
    }
    else if (tick == 360)
        command.kind = Kind::Stop;
    else if (tick == 600)
        command.kind = Kind::Land;
    else
        hasCommand = false;

    if (hasCommand)
        (void)simulation_.Apply(command);
}

void AeroSimLabApp::RenderFrame()
{
    int width = 0;
    int height = 0;
    window_->GetFramebufferSize(width, height);
    if (width <= 0 || height <= 0)
        return;

    renderer_->Resize(width, height);
    const int observerWidth = width / 2;
    const int eoWidth = width - observerWidth;
    const auto& state = simulation_.State();
    const glm::vec3 dronePosition = ToRenderPosition(state.positionMeters);
    const glm::mat4 heading = glm::rotate(glm::mat4{1.0F},
        static_cast<float>(state.yawRadians), glm::vec3{0.0F, 1.0F, 0.0F});

    observerCamera_->SetPosition(dronePosition + glm::vec3{7.0F, 5.0F, 7.0F});
    observerCamera_->SetTarget(dronePosition + glm::vec3{0.0F, 0.2F, 0.0F});
    observerCamera_->SetAspectRatio(static_cast<float>(observerWidth) / static_cast<float>(height));

    const glm::vec3 cameraPosition = dronePosition + glm::vec3{
        heading * glm::vec4{0.35F, 0.12F, 0.0F, 0.0F}};
    const glm::vec3 forward = glm::vec3{heading * glm::vec4{1.0F, 0.0F, 0.0F, 0.0F}};
    eoCamera_->SetPosition(cameraPosition);
    eoCamera_->SetTarget(cameraPosition + forward * 25.0F + glm::vec3{0.0F, -2.5F, 0.0F});
    eoCamera_->SetAspectRatio(static_cast<float>(eoWidth) / static_cast<float>(height));

    renderer_->BeginFrame();
    renderer_->SetSceneViewport(0, 0, observerWidth, height);
    DrawWorld(observerCamera_->GetViewMatrix(), observerCamera_->GetProjectionMatrix(),
        observerCamera_->GetPosition());
    renderer_->SetSceneViewport(observerWidth, 0, eoWidth, height);
    DrawWorld(eoCamera_->GetViewMatrix(), eoCamera_->GetProjectionMatrix(),
        eoCamera_->GetPosition());
    renderer_->SetSceneViewport(0, 0, width, height);
    renderer_->EndFrame();
}

void AeroSimLabApp::DrawWorld(const glm::mat4& view, const glm::mat4& projection,
    const glm::vec3& cameraPosition)
{
    DrawCube(CubeTransform({0.0F, -0.2F, 0.0F}, {80.0F, 0.2F, 80.0F}), cameraPosition,
        groundMaterial_, view, projection);
    DrawCube(CubeTransform({6.0F, 1.5F, -3.0F}, {2.0F, 3.0F, 2.0F}), cameraPosition,
        buildingMaterial_, view, projection);
    DrawCube(CubeTransform({-5.0F, 2.0F, -7.0F}, {3.0F, 4.0F, 2.0F}), cameraPosition,
        buildingMaterial_, view, projection);
    DrawCube(CubeTransform({12.0F, 2.5F, 5.0F}, {2.5F, 5.0F, 2.5F}), cameraPosition,
        buildingMaterial_, view, projection);

    const glm::vec3 position = ToRenderPosition(simulation_.State().positionMeters);
    const glm::mat4 heading = glm::translate(glm::mat4{1.0F}, position) *
        glm::rotate(glm::mat4{1.0F}, static_cast<float>(simulation_.State().yawRadians),
            glm::vec3{0.0F, 1.0F, 0.0F});
    const auto drawPart = [&](const glm::vec3& offset, const glm::vec3& size,
                              const std::shared_ptr<aerosimlab::graphics::Material>& material)
    {
        DrawCube(heading * glm::translate(glm::mat4{1.0F}, offset) *
            glm::scale(glm::mat4{1.0F}, size), cameraPosition, material, view, projection);
    };

    drawPart({0.0F, 0.0F, 0.0F}, {0.55F, 0.16F, 0.42F}, droneMaterial_);
    drawPart({0.0F, 0.0F, -0.35F}, {0.50F, 0.05F, 0.05F}, droneMaterial_);
    drawPart({0.0F, 0.0F, 0.35F}, {0.50F, 0.05F, 0.05F}, droneMaterial_);
    drawPart({-0.32F, 0.08F, -0.32F}, {0.18F, 0.04F, 0.18F}, propellerMaterial_);
    drawPart({0.32F, 0.08F, -0.32F}, {0.18F, 0.04F, 0.18F}, propellerMaterial_);
    drawPart({-0.32F, 0.08F, 0.32F}, {0.18F, 0.04F, 0.18F}, propellerMaterial_);
    drawPart({0.32F, 0.08F, 0.32F}, {0.18F, 0.04F, 0.18F}, propellerMaterial_);
}

void AeroSimLabApp::DrawCube(const glm::mat4& transform, const glm::vec3& cameraPosition,
    const std::shared_ptr<aerosimlab::graphics::Material>& material,
    const glm::mat4& view, const glm::mat4& projection)
{
    renderer_->Draw(transform, *cube_, *material, *shader_, view, projection, cameraPosition);
}
}
