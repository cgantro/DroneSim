#pragma once

#include <glm/glm.hpp>

namespace aerosimlab::graphics
{

/**
 * @brief 장면을 어디에서 어느 점 쪽으로 볼지와 화면에 들어올 거리 범위를 보관한다.
 * @details
 * 카메라는 장면을 보는 위치와 방향을 정하는 가상의 관찰자다. 앱이 위치와 바라볼 점을 바꾸고 Renderer가 이 값으로 행렬을 만든다.
 * 행렬은 여러 좌표 변환을 숫자 하나의 표로 묶어 점에 적용하는 방법이다. position과 target은 장면 전체에서 쓰는 좌표이며 +Y가 위다.
 * Local 좌표는 각 Mesh의 기준점에서 잰 위치고 World 좌표는 부모 변환까지 적용한 장면 전체 위치다. 이 클래스는 World 좌표의 카메라 위치와 목표점을 보관한다.
 * View 변환은 장면의 점을 카메라 위치와 방향 기준으로 바꾼다. Projection 변환은 가까운 물체가 크게, 먼 물체가 작게 보이도록 원근을 적용한다.
 * Projection 뒤의 Clip 좌표는 GPU가 화면 밖의 점을 자르고 깊이를 계산하는 중간 표현이다. 보이는 범위 안의 Clip 좌표를 w로 나눈 뒤에만 x/y/z가 각각 -1~1 범위에 든다.
 * 여기서 깊이는 카메라 앞뒤 순서를 비교할 값이지 실제 거리와 일정한 비율로 증가하는 값이 아니다. 앞뒤 잘라내기 평면 사이의 표면만 화면에 그린다.
 * 거리와 앞·뒤 평면은 [m], 위아래 시야각은 [deg]다. 수학 함수에 넘길 때 각도를 [rad]로 바꾼다.
 */
class Camera
{
public:

    /**
     * @brief 카메라 위치, 바라볼 점, 화면에 보일 깊이를 지정한다.
     * @param position 장면 좌표에서 카메라 위치 [m]
     * @param target 장면 좌표에서 카메라가 바라볼 점 [m]
     * @param aspectRatio 실제 화면 너비를 높이로 나눈 비율. 0보다 커야 함
     * @param fov 위아래 시야각 [deg]
     * @param nearPlane 카메라 앞에서 가까운 잘라내기 평면까지 거리 [m]. 0보다 커야 함
     * @param farPlane 카메라 앞에서 먼 잘라내기 평면까지 거리 [m]. nearPlane보다 커야 함
     * @throws std::runtime_error 화면 비율 또는 가까운·먼 평면의 조건이 잘못된 경우
     * @details
     * 카메라에서 target으로 향하는 방향을 카메라 앞(-Z)으로 삼고 장면 위쪽을 참고해 좌우(+X)와 위쪽(+Y)을 정한다.
     * 시야각과 화면 비율이 화면의 네 모서리를 정한다. 가까운 평면보다 앞이나 먼 평면보다 뒤에 있는 점은 그리지 않는다.
     * 화면 비율과 평면 간격만 검사한다. 시야각과 숫자가 유한한지는 검사하지 않으므로 호출자가 유효한 값을 전달해야 한다.
     */
    Camera(
        const glm::vec3& position,
        const glm::vec3& target,
        float aspectRatio,
        float fov = 45.0F,
        float nearPlane = 0.1F,
        float farPlane = 100.0F);

    /**
     * @brief World 좌표의 점을 카메라가 원점에서 -Z 방향을 보는 좌표로 바꾸는 행렬을 반환한다.
     * @return 장면 좌표 → 카메라 좌표 변환 행렬. 행렬은 점의 좌표를 위치와 방향 기준으로 바꾸는 숫자 표다.
     * @details
     * 카메라 위치가 원점이 되고 시선이 -Z를 향하도록 장면 좌표를 변환한다.
     * m_Position과 m_Target은 장면 좌표의 점이고 m_Up은 장면에서 위쪽을 나타내는 방향이다.
     */
    glm::mat4 GetViewMatrix() const;

    /**
     * @brief 카메라 기준 좌표에 원근을 적용해 화면에 보일 모양과 앞뒤 범위를 정하는 행렬을 반환한다.
     * @return 카메라 좌표 → Clip 좌표 변환 행렬. 보이는 범위 안의 점은 Clip 좌표를 w로 나눈 뒤 깊이가 -1~1이며 실제 거리값은 아니다.
     * @details
     * 위아래 시야각과 화면 비율이 가로·세로 시야를 정하고 가까운·먼 평면 사이만 그린다. 원근 때문에 멀리 있는 물체는 작아진다.
     * 깊이 숫자는 실제 거리 간격과 비례하지 않는다. 가까운 평면을 너무 가까이 두면 깊이 정밀도가 앞쪽에 몰려 멀리 있는 표면의 앞뒤를 구분하기 어려워진다.
     */
    glm::mat4 GetProjectionMatrix() const;

    /** @brief 화면 너비/높이 비율을 갱신한다. @param aspectRatio 화면 너비를 높이로 나눈 값. 양수만 반영한다. */
    void SetAspectRatio(float aspectRatio);

    /** @brief 장면 좌표에서 카메라 위치 [m]를 갱신한다. @param position 새 위치 */
    void SetPosition(const glm::vec3& position);

    /** @brief 장면 좌표에서 바라볼 점 [m]을 갱신한다. @param target 새 목표점 */
    void SetTarget(const glm::vec3& target);

    /** @brief 카메라가 화면 위쪽으로 사용할 장면 좌표 방향을 갱신한다. @param up 새 위쪽 방향 */
    void SetUp(const glm::vec3& up);

    /** @return 장면 좌표에서 카메라 위치 [m] */
    glm::vec3 GetPosition() const;

    /** @return 장면 좌표에서 카메라가 바라보는 점 [m] */
    glm::vec3 GetTarget() const;

private:
    glm::vec3 m_Position;

    glm::vec3 m_Target;

    glm::vec3 m_Up{0.0F, 1.0F, 0.0F};

    float m_AspectRatio;

    float m_Fov;

    float m_NearPlane;

    float m_FarPlane;
};

} // namespace aerosimlab::graphics
