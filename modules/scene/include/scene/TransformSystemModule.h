#pragma once

#include <flecs.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace grasplink::scene
{

/**
 * @brief 장면 물체의 부모 기준 위치·회전·크기를 Scene 전체 기준 변환으로 계산한다.
 * @details Local 값은 바로 위 부모를 기준으로 하고 World 값은 모든 부모 변환을 누적한 Scene 기준이다. 로봇 팔 링크의 Local 회전을 바꾸면 이 모듈은 부모 관절의 회전까지 반영한 World 행렬을 만든다.
 * 위치(Translation), 회전(Rotation), 크기(Scale)를 묶어 TRS라 부른다. 이 타입을 Flecs World에 등록해도 자동 실행되지 않으므로 호출자가 물리 계산 전과 화면 그리기 전에 `UpdateWorldTransforms`를 불러야 한다.
 * World 행렬은 마지막 호출 결과를 저장하므로 Local 값을 바꿔도 자동 갱신되지 않는다.
 */
class TransformSystemModule
{
public:
    /** @brief 변환 계산 모듈을 World에 등록한다. 자동 실행은 없으므로 호출자가 필요한 시점에 계산한다. */
    explicit TransformSystemModule(flecs::world& world);

    /**
     * @brief 물체의 부모 기준 위치·방향·크기를 하나의 행렬로 합친다.
     * @param position 부모 원점에서 떨어진 위치 [m].
     * @param rotation 부모 기준 방향을 나타내는 quaternion. 성분은 유한해야 하고 모두 0일 수 없다.
     * @param scale 부모 기준 축별 크기 배율. 단위 없음.
     * @return 점에 크기 배율, 회전, 위치 이동을 차례로 적용하는 행렬이다.
     * @throws std::invalid_argument 위치나 크기가 유한하지 않거나 회전을 나타낼 수 없는 quaternion을 전달한 경우.
     */
    static glm::mat4 ComposeLocalMatrix(
        const glm::vec3& position,
        const glm::quat& rotation,
        const glm::vec3& scale);

    /**
     * @brief 각 물체의 Local 값을 부모 값과 합쳐 Scene 전체에서 사용할 World 행렬을 갱신한다.
     * @details Local은 바로 위 부모 기준이고 World는 Scene root부터 현재 물체까지 누적한 값이다. 시작 물체에는 위치·회전·크기와 두 종류의 행렬 저장 공간이 모두 있어야 한다.
     * 중간 부모에 위치·회전·크기 중 하나라도 없으면 일부 값만 적용하지 않고 그 부모의 자체 변환을 생략한다. 예를 들어 이름만 있는 grouping 물체는 위치를 옮기지 않고 조상 행렬만 자식에게 전달한다.
     * 방문한 중간 부모와 SceneRoot에는 계산한 World 행렬을 저장하지만 새 Local 값은 만들지 않는다. SceneRootTag는 Scene 소속 경계이며 위치 변환 값이 아니다.
     * 한 호출 안에서 여러 자식이 같은 부모를 쓰면 그 부모 행렬을 한 번만 계산해 재사용한다. 계산용 목록, 행렬 캐시와 Flecs 조회는 World에 보관해 다음 호출에서도 재사용하며, 각 호출마다 바뀐 Local 값을 다시 읽는다. 같은 World의 변환 갱신은 동시에 실행하지 않는다. 이 함수는 행렬만 계산하며 물체를 움직이지 않는다.
     */
    static void UpdateWorldTransforms(flecs::world& world);
};

}
