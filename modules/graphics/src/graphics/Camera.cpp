#include "graphics/Camera.h"

#include <glm/gtc/matrix_transform.hpp>

#include <stdexcept>

namespace aerosimlab::graphics
{

Camera::Camera(
    const glm::vec3& position,
    const glm::vec3& target,
    float aspectRatio,
    float fov,
    float nearPlane,
    float farPlane)
    : m_Position(position),
      m_Target(target),
      m_AspectRatio(aspectRatio),
      m_Fov(fov),
      m_NearPlane(nearPlane),
      m_FarPlane(farPlane)
{
    // 화면 가로세로 비율이 0이면 투영 화면 폭을 계산할 수 없다. 앞·뒤 자르기 거리가 유효한 순서가 아니면 원근 깊이도 정할 수 없다.
    if (aspectRatio <= 0.0F)
        throw std::runtime_error("Camera aspect ratio must be greater than 0");

    if (nearPlane <= 0.0F || farPlane <= nearPlane)
        throw std::runtime_error("Invalid camera clipping planes");
}

glm::mat4 Camera::GetViewMatrix() const
{
    // View 변환은 장면 좌표를 카메라가 원점에서 바라보는 좌표로 바꾸는 표다. lookAt은 Target 방향을 -Z, Up에서 계산한 오른쪽을 +X로 삼는다.
    return glm::lookAt(m_Position, m_Target, m_Up);
}

glm::mat4 Camera::GetProjectionMatrix() const
{

    // Projection 깊이는 카메라에서 잰 실제 거리와 비례하지 않고 앞뒤 순서를 나타낸다. 앞쪽 잘라내기 평면을 지나치게 가까이 두면 먼 표면 사이를 구분할 정밀도가 낮아진다.
    // FOV는 세로 시야각이며 화면 가로세로 비율로 가로 시야각이 정해진다. GLM에 전달하는 각도 단위는 rad다.
    return glm::perspective(
        glm::radians(m_Fov),
        m_AspectRatio,
        m_NearPlane,
        m_FarPlane);
}

void Camera::SetAspectRatio(float aspectRatio)
{
    if (aspectRatio <= 0.0F) return;
    m_AspectRatio = aspectRatio;
}

void Camera::SetPosition(const glm::vec3& position)
{
    m_Position = position;
}

void Camera::SetTarget(const glm::vec3& target)
{
    m_Target = target;
}

glm::vec3 Camera::GetPosition() const
{
    return m_Position;
}

glm::vec3 Camera::GetTarget() const
{
    return m_Target;
}

} // namespace aerosimlab::graphics
