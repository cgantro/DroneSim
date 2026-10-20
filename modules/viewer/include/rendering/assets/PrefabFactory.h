#pragma once

#include <memory>

namespace grasplink::graphics { class Shader; }
namespace grasplink::rendering { class AssetManager; }

namespace grasplink::scene
{
class Entity;
class Scene;
}

namespace grasplink::model
{
struct ModelResource;
}

/**
 * @brief GLB 모델에서 읽은 부품 Entity를 만드는 함수를 제공한다.
 * @details
 * GltfLoader가 읽은 부품의 부모 기준 위치 [m], 회전, 크기를 Entity에 복사하고 GPU 형상과 표면 재질을 그리기 정보에 연결한다.
 * 형상이 없는 부품도 관절 중심이나 자식 위치의 기준이므로 보존한다. AssetManager::UploadModel을 먼저 호출해 GPU 자료를 준비한다.
 * 부모 부품이 배열에서 자식보다 뒤에 있어도 되도록 Entity를 전부 만든 후 부모 번호로 연결한다.
 * SetParent는 부모 관계만 지정하고 각 부품의 부모 기준 변환은 유지한다. 장면 좌표 변환은 TransformSystemModule이 다음 갱신에서 부모부터 계산한다.
 *
 * 형상 묶음이 하나면 부품 Entity 자체에 그리기 정보를 붙인다. 여러 묶음이면 묶음마다 변환이 기본값인 자식 Entity를 만들어 부모 부품의 위치와 회전을 물려받게 한다.
 * MeshFilter 범위는 이어 붙인 꼭짓점 연결 번호 배열에서 시작할 번호와 사용할 번호 개수를 뜻한다.
 *
 * Entity와 부모·자식 관계의 수명은 Scene이 관리한다. 반환 wrapper는 수명을 늘리지 않으므로 Scene이나 Flecs World가 파괴되면 무효다.
 * 그리기 정보는 형상·재질·Shader의 공유 참조를 보유한다. GPU 객체의 마지막 참조를 놓을 때 OpenGL 실행 환경(context)이 현재 스레드에서 활성화되어 있어야 한다.
 * 부모 관계, Mesh 범위, GPU Mesh와 Material은 Entity를 만들기 전에 검사한다. Scene 자체의 Entity 생성이 실패하면 먼저 만든 Entity가 남을 수 있다.
 */
namespace grasplink::rendering
{
namespace prefab_factory
{
/**
 * @brief 모델 부품 Entity를 만들고 최상위 부품을 가리키는 wrapper를 반환한다.
 * @param scene Entity를 추가할 활성 장면. 장면이 Entity와 관계의 수명을 관리한다.
 * @param model GLB에서 읽은 CPU 자료. 각 GPU Mesh는 AssetManager의 자원 번호로 찾는다.
 * @param assets 자원 번호로 GPU 형상과 재질을 찾는 관리자.
 * @param shader 만들어지는 모든 표면이 사용할 화면 그리기 프로그램.
 * @return 선택된 최상위 부품을 가리키는 Entity wrapper.
 * @throws std::runtime_error 그리기 프로그램, 부품 관계, GPU 형상 또는 재질 참조가 잘못됐을 때 발생한다.
 * 모델 관계·GPU 형상·재질 참조가 잘못된 경우 Entity를 만들기 전에 실패한다. Scene 생성 자체가 실패하면 앞서 만든 Entity가 남을 수 있다.
 * @details 각 부품의 위치는 부모 기준 [m], 회전은 길이 1인 quaternion, 크기는 배율이다. GPU 자료는 번호로 찾아 공유한다.
 * 재질 번호가 -1인 표면만 기본 재질을 사용한다. 여러 표면 묶음은 각각 자식 Entity가 맡아 연결 번호의 일부를 그린다.
 */
::grasplink::scene::Entity CreateModel(
    ::grasplink::scene::Scene& scene,
    const ::grasplink::model::ModelResource& model,
    const AssetManager& assets,
    const std::shared_ptr<::grasplink::graphics::Shader>& shader);
}

} // namespace grasplink::rendering
