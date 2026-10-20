#pragma once

#include "model/ResourceID.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace grasplink::model
{

/**
 * @brief 모델 표면의 꼭짓점 한 개와 그릴 때 필요한 표면 정보를 담는다.
 * @details
 * Mesh는 여러 삼각형의 꼭짓점과 연결 번호를 모은 모델 형상이다. position은 Mesh 자체 기준(Local) 위치고 normal(법선)은 그 꼭짓점 표면에 수직인 방향이다.
 * texCoord(UV)는 표면에서 색상 Texture의 어느 픽셀을 읽을지 나타내는 이미지 좌표다. GltfLoader는 실수형 위치·법선·UV만 읽는다.
 */
struct Vertex
{
    /// Mesh 기준 좌표의 위치. 모델 변환이 적용되기 전 값이다.
    glm::vec3 position{0.0f};
    /// Mesh 기준 표면의 수직 방향.
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    /// 색상 이미지 안에서 읽을 위치. 각 성분은 이미지 크기에 대한 비율이다.
    glm::vec2 texCoord{0.0f};
};

/**
 * @brief GLB 이미지 파일을 픽셀 바이트로 풀어 둔 CPU 데이터다.
 * @details
 * GltfLoader는 색의 빨강·초록·파랑 및 선택적 alpha 채널마다 8 bit를 쓰는 RGB 또는 RGBA 이미지로 읽는다. AssetManager가 이 바이트를 GPU 이미지로 복사하고 캐시에 보관한다. 따라서 pixels는 GPU 업로드 전까지 사용하는 원본이다.
 */
struct TextureData
{
    /// 모델 파일 경로와 glTF texture 항목 번호를 조합한 식별자.
    ResourceID uniqueID;

    /// 이미지 너비, 단위: pixel.
    int width = 0;
    /// 이미지 높이, 단위: pixel.
    int height = 0;
    /// 픽셀당 채널 수. 현재 3(RGB) 또는 4(RGBA).
    int channels = 0;

    /// 위에서 아래 순서의 픽셀 바이트. 행 사이에 빈 패딩이 없다.
    std::vector<unsigned char> pixels;
};

/**
 * @brief 표면 색·반사 계수와 기본색 이미지 연결 정보를 담는다.
 */
struct MaterialData
{
    /// 모델 파일 경로와 재질 번호를 조합한 식별자.
    ResourceID uniqueID;
    /// GLB에 기록된 표시 이름. 비어 있을 수 있다.
    std::string name;

    /// 표면 기본 RGB 색과 투명도에 곱할 값.
    glm::vec4 baseColorFactor{1.0f,1.0f,1.0f,1.0f};
    /// 금속성 계수, glTF 범위 0~1.
    float metallicFactor = 1.0f;
    /// 거칠기 계수, glTF 범위 0~1.
    float roughnessFactor = 1.0f;

    /// 기본색 이미지 연결 정보. 해당 픽셀과 GPU 객체는 AssetManager가 관리한다.
    ResourceID baseColorTexture;

};

/**
 * @brief 이어 붙인 꼭짓점 연결 배열 중 한 표면 묶음의 범위와 재질을 가리킨다.
 * @details
 * indexStart는 첫 연결 번호의 위치이고 indexCount는 사용할 연결 번호 개수다. 둘 다 4-byte 정수 원소 단위다. Renderer는 GPU 호출 전에 시작 위치를 byte 단위로 바꾼다. 재질 번호 -1은 기본 재질을 뜻한다.
 */
struct SubMeshInfo{
    /// MeshData::indices 배열에서 이 묶음의 첫 연결 번호 위치.
    std::uint32_t indexStart = 0;
    /// 이 묶음이 사용할 꼭짓점 연결 번호의 개수.
    std::uint32_t indexCount = 0;

    /// 연결된 번호가 없으면 화면 기본 재질을 사용한다.
    std::optional<std::size_t> defaultMaterialIndex;
};

/**
 * @brief 한 모델 Mesh의 CPU 데이터와 표면별 범위를 모은다.
 * @details
 * AssetManager가 vertices와 indices를 그래픽 카드 메모리에 올린다. ModelResource에는 GPU 객체가 아닌 GLB에서 읽은 CPU 자료만 남는다.
 * PrefabFactory는 AssetManager에서 GPU Mesh를 찾아 Entity의 MeshFilter에 공유 참조로 연결한다. CPU 모델이 해제되어도 Entity가 참조하는 동안 그릴 형상은 유지된다. 마지막 GPU 참조를 해제할 때 OpenGL context가 현재 스레드에서 활성화되어 있어야 한다.
 */
struct MeshData
{
    /// 모델 파일 경로와 형상 번호를 조합한 식별자.
    ResourceID uniqueID;

    /// 디버깅과 오류 메시지에 사용하는 Mesh 이름.
    std::string name;

    /// GPU 업로드 전 정점 속성 배열.
    std::vector<Vertex> vertices;

    /// 삼각형을 만들 꼭짓점 번호의 연속 배열. 번호는 vertices의 위치를 가리킨다.
    std::vector<std::uint32_t> indices;

    /// 원래 Primitive별 draw 범위와 기본 재질 번호.
    std::vector<SubMeshInfo> subMeshes;

};

/**
 * @brief 모델 안 한 부품의 위치·회전·크기와 부모·자식, 그릴 Mesh 번호를 저장한다.
 * @details
 * 위치(translation), 회전(rotation), 크기(scale)는 부모 부품 기준 값이다. 위치 단위는 m이고 크기는 배율이다.
 * Quaternion은 회전축과 회전량을 네 숫자로 나타내는 방식이며 yaw/pitch/roll처럼 세 축 각도를 차례로 적용하는 방식과 다르다.
 * 이 회전은 길이가 1인 quaternion으로 저장한다. 파일이 회전을 행렬로 적었든 네 숫자로 적었든 같은 quaternion 형태로 저장한다.
 * GltfLoader는 부모가 하나인 나무 모양 관계만 허용하고 PrefabFactory가 이를 Entity 관계로 옮긴다.
 * Mesh가 없는 부품도 관절 중심점이나 자식 위치의 기준이 될 수 있어 보존한다.
 */
struct NodeData
{
    /// 이름이 비어 있으면 GltfLoader가 Node 번호를 붙여 생성한다.
    std::string name;

    /// 부모 부품 기준 위치 [m].
    glm::vec3 translation{
        0.0F
    };

    /// 부모 부품 기준 회전. 길이가 1인 quaternion이며 GLM 생성자 인자 순서는 (w,x,y,z)다.
    glm::quat rotation{
        1.0F, 0.0F, 0.0F, 0.0F
    };

    /// 부모 부품 기준 크기 배율. (1,1,1)은 원래 크기다.
    glm::vec3 scale{
        1.0F
    };

    /// 이 부품에 연결된 화면 형상 번호. 형상이 없으면 비어 있다.
    std::optional<std::size_t> meshIndex;

    /// 부모 부품의 번호. 최상위 부품이면 비어 있다.
    std::optional<std::size_t> parentIndex;

    /// 이 부품 바로 아래에 연결할 자식 번호 목록.
};

/**
 * @brief GLB 파일에서 읽은 이미지·재질·형상·부품 관계를 한데 모은 결과다.
 * @details
 * GltfLoader는 우선 CPU 배열을 만든다. AssetManager가 이를 그래픽 카드에 올린다.
 * GPU 자원은 AssetManager와 이를 사용하는 Entity가 소유하며 CPU 모델 수명과 분리된다.
 * 현재는 선택한 장면이 최상위 부품 하나에서 시작하는 계층만 허용하며, 장면 밖의 별도 부품 묶음은 거부한다.
 */
struct ModelResource
{
    /// 파일에 나온 순서대로 저장한 재질 입력값.
    std::vector<MaterialData> materials;

    /// 파일에 나온 순서대로 저장한 형상 데이터.
    std::vector<MeshData> meshes;

    /// 부품의 위치·부모·자식 관계 데이터.
    std::vector<NodeData> nodes;

    /// 파일에서 픽셀로 풀어 둔 이미지 데이터.
    std::vector<TextureData> textures;

    /// 화면에 표시할 계층의 최상위 부품 번호.
    std::optional<std::size_t> rootNodeIndex;
};

} // namespace grasplink::model
