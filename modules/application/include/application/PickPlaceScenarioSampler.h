#pragma once

#include <array>
#include <cstdint>
#include <random>

namespace grasplink::application
{

/**
 * @brief 반복 pick-and-place 임무에 사용할 상자와 배치 위치 및 방향을 생성한다.
 * @details 각 sampler는 난수 상태와 직전 상자·배치 위치를 따로 보관한다. 같은 seed와 같은 호출 순서를 사용하면 같은 표본을 다시 만들 수 있고, 서로 다른 sampler는 난수 상태를 공유하지 않는다.
 */
class PickPlaceScenarioSampler final
{
public:
    /** @brief 같은 호출 순서에서 재현할 난수 순서로 초기화하고 위치 이력을 지운다. */
    void Seed(std::uint32_t seed);

    /** @brief 작업 반경 안에서 상자 중심 위치 [m]를 고른다. */
    [[nodiscard]] std::array<float, 3> SampleBoxPosition();

    /** @brief 작업 반경 안에서 배치 영역 중심 위치 [m]를 고른다. */
    [[nodiscard]] std::array<float, 3> SamplePlacementPosition();

    /** @brief 상자나 배치 영역에 사용할 Y축 회전각 [rad]을 고른다. */
    [[nodiscard]] float SamplePlanarRotation();

private:
    void EnsureSeeded();
    [[nodiscard]] std::array<float, 3> SamplePosition(float heightMeters,
        float minimumRadiusMeters, float maximumRadiusMeters, float minimumSeparationMeters,
        std::array<float, 3>& previousPosition, bool& hasPreviousPosition);

    std::mt19937 generator_;
    std::array<float, 3> previousBoxPosition_{};
    std::array<float, 3> previousPlacementPosition_{};
    bool hasPreviousBoxPosition_ = false;
    bool hasPreviousPlacementPosition_ = false;
    bool initialized_ = false;
};

}
