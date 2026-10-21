#pragma once

#include "simulation_core/SimulationCore.h"

#include <glm/glm.hpp>

#include <memory>

namespace aerosimlab::graphics
{
class Camera;
class Material;
class Mesh;
class Renderer;
class Shader;
class Window;
}

namespace aerosimlab::simulator
{
/**
 * @brief 드론 모의를 진행하고 관찰 화면과 탑재 카메라 화면을 나란히 그린다.
 * @details 드론 위치는 미터, 모의 간격은 1/60초다. 왼쪽 화면은 드론 주변을 보는 관찰자 시점이고 오른쪽은 기체 앞쪽에 고정한 RGB 카메라 시점이다.
 * 화면 갱신 속도와 모의 속도를 분리해, 렌더링이 느려져도 같은 고정 간격으로 드론 상태를 계산한다.
 */
class AeroSimLabApp
{
public:
    explicit AeroSimLabApp(bool smokeTest);
    ~AeroSimLabApp();

    /** @brief 그래픽 자원을 준비하고 창이 닫힐 때까지 모의를 실행한다. */
    int Run();

private:
    void InitializeGraphics();
    void AdvanceSimulation(double elapsedSeconds);
    void ApplyDemoCommands();
    void RenderFrame();
    void DrawWorld(const glm::mat4& view, const glm::mat4& projection,
        const glm::vec3& cameraPosition);
    void DrawCube(const glm::mat4& transform, const glm::vec3& cameraPosition,
        const std::shared_ptr<aerosimlab::graphics::Material>& material,
        const glm::mat4& view, const glm::mat4& projection);

    bool smokeTest_ = false;
    std::size_t renderedFrames_ = 0;
    double accumulatedSeconds_ = 0.0;
    std::unique_ptr<aerosimlab::graphics::Window> window_;
    std::unique_ptr<aerosimlab::graphics::Renderer> renderer_;
    std::unique_ptr<aerosimlab::graphics::Camera> observerCamera_;
    std::unique_ptr<aerosimlab::graphics::Camera> eoCamera_;
    std::shared_ptr<aerosimlab::graphics::Mesh> cube_;
    std::shared_ptr<aerosimlab::graphics::Shader> shader_;
    std::shared_ptr<aerosimlab::graphics::Material> droneMaterial_;
    std::shared_ptr<aerosimlab::graphics::Material> propellerMaterial_;
    std::shared_ptr<aerosimlab::graphics::Material> groundMaterial_;
    std::shared_ptr<aerosimlab::graphics::Material> buildingMaterial_;
    aerosimlab::simulation::SimulationCore simulation_;
};
}
