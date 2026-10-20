#pragma once

#include "model/ModelResource.h"

#include <memory>
#include <unordered_map>

namespace grasplink::graphics
{
class Material;
class Mesh;
class Texture;
}

namespace grasplink::rendering
{

/**
 * @brief 파일에서 읽은 모델 데이터를 그래픽 카드에 올리고 같은 자원을 여러 사용자가 함께 쓰게 한다.
 * @details
 * GltfLoader는 GLB 파일에서 삼각형 정점·픽셀·재질 데이터를 CPU 메모리로 읽는다. UploadModel은 픽셀과 정점 byte를 GPU로 복사해 Texture와 Mesh를 만든다.
 * PrefabFactory는 이 이미지와 형상을 장면 부품 및 Material(표면 색·빛 반응 정보)에 연결한다.
 * ResourceID 캐시는 같은 식별자를 가진 이미지·재질·형상을 다시 만들어 올리지 않고 재사용한다.
 * 형상은 이 관리자의 캐시와 Entity의 MeshFilter가 공유한다. CPU ModelResource가 해제되어도 Entity가 그리는 동안 GPU 데이터는 유지된다.
 * 마지막 공유 참조를 놓으면 GPU 객체 삭제가 발생한다. 장면의 Entity와 이 관리자의 참조를 정리한 뒤 OpenGL context를 종료해야 한다.
 * OpenGL context는 GPU 명령을 실행할 상태와 자원 연결을 제공하는 환경이다. 단지 창이 존재하는 것만으로 충분하지 않고 업로드와 마지막 참조 해제 때 현재 스레드에서 활성화되어 있어야 한다.
 * 이 캐시는 동시에 여러 스레드에서 호출될 때 서로 보호하지 않는다.
 */
class AssetManager final
{
public:
    /** @brief 이미지 없이 기본 표면 색상 계수만 사용하는 재질을 준비한다. */
    AssetManager();

    /**
     * @brief 모델의 픽셀·꼭짓점 데이터를 그래픽 카드에 복사해 캐시에 저장한다.
     * @param model GltfLoader가 읽은 CPU 모델 데이터.
     * @throws std::runtime_error 이미지 크기·채널이 잘못됐거나 재질이 요구한 기본색 이미지가 없거나, 정점·삼각형 번호가 잘못됐으면 발생한다. 형상 크기는 32-bit 번호 범위 안이어야 한다.
     * @details Texture는 GPU가 표면 색을 읽을 이미지이고 Mesh는 삼각형 정점과 연결 번호를 담는 형상이다. 이미지, 재질, 형상 순서로 올려 뒤 단계가 앞 단계 자원을 찾는다.
     * 캐시에 같은 ResourceID가 있으면 다시 만들지 않고 공유한다. 픽셀 자료가 없는 이미지는 업로드하지 않으며 그런 이미지를 요구하는 재질은 오류다.
     * 재질 번호가 없는 표면만 기본 재질을 쓴다. 지정한 이미지가 없을 때는 기본 재질로 대체하지 않는다.
     * 이미지 채널은 단일 밝기 1개, RGB 색 3개, RGBA 색과 alpha 4개만 지원한다. GLB의 모든 이미지 Texture를 현재 sRGB 형식으로 업로드한다.
     * sRGB는 모니터용 RGB 색을 GPU 조명 계산용 선형 색으로 읽게 하는 저장 형식이다. Material에는 기본색 이미지 참조만 연결하며 금속성·거칠기, 표면 방향, 빛 가림, 발광 이미지 참조는 유효한 ID로 설정하지 않는다.
     * GPU 업로드에는 OpenGL 명령 실행 환경(context)이 현재 스레드에서 활성화되어 있어야 한다.
     */
    void UploadModel(const ::grasplink::model::ModelResource& model);

    /** @brief 형상 식별자에 해당하는 GPU Mesh를 찾는다. @return 업로드되지 않았으면 빈 공유 참조. */
    std::shared_ptr<::grasplink::graphics::Mesh> GetMesh(::grasplink::model::ResourceID id) const;

    /** @brief 자원 식별자에 해당하는 Material(표면 색·빛 반응 정보)을 찾는다. @return 없으면 빈 공유 참조. */
    std::shared_ptr<::grasplink::graphics::Material> GetMaterial(::grasplink::model::ResourceID id) const;

    /** @brief 파일에서 재질 번호를 주지 않은 표면에 쓸 기본 재질을 반환한다. */
    std::shared_ptr<::grasplink::graphics::Material> GetDefaultMaterial() const;

private:
    std::unordered_map<::grasplink::model::ResourceID, std::shared_ptr<::grasplink::graphics::Mesh>> meshes_;
    std::unordered_map<::grasplink::model::ResourceID, std::shared_ptr<::grasplink::graphics::Material>> materials_;
    std::unordered_map<::grasplink::model::ResourceID, std::shared_ptr<::grasplink::graphics::Texture>> textures_;

    std::shared_ptr<::grasplink::graphics::Material> defaultMaterial_;
};

} // namespace grasplink::rendering
