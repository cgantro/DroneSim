#pragma once

#include "model/ModelResource.h"

#include <filesystem>

namespace grasplink::model
{

/**
 * @brief GLB 파일을 읽어 모델의 CPU 자료를 만든다.
 * @details GLB에는 형상, 재질, 이미지와 부품의 부모·자식 관계가 들어 있다.
 * 이 함수는 화면에 올리기 전 자료를 만든다. GPU 업로드는 AssetManager가 맡는다.
 * 형상이 없는 부품도 다른 부품의 위치 기준이 될 수 있어 결과에 보존한다.
 * 정수로 압축한 정점 속성, 일부 값만 바꾸는 Sparse 형식, 삼각형이 아닌 표면은 지원하지 않는다.
 */
class GltfLoader final
{
public:
    /**
     * @brief GLB에서 형상·재질·이미지와 부품 연결을 읽어 CPU 자료구조로 만든다.
     * @param path 읽을 GLB 파일 경로.
     * @return 파일 경로를 포함한 자원 번호, CPU 배열, 선택 장면의 시작 부품 번호.
     * @throws std::runtime_error 파일을 읽지 못했거나 바이트 범위가 잘못됐거나 무한대·NaN 또는 지원하지 않는 파일 형식을 만났을 때 발생한다. 자세한 내용은 예외 메시지에 있다.
     * @throws std::out_of_range GLB 안의 정점 자료 번호가 유효 범위를 벗어났을 때 발생한다.
     * @details
     * 정점 속성은 GLB가 기록한 간격에 맞춰 읽고, 참조한 모든 데이터가 실제 파일 범위 안에 있는지 확인한다.
     * 여러 표면은 하나의 형상으로 합치되 표면마다 재질과 그릴 범위를 따로 기록한다.
     * 부품 변환은 위치·회전·크기로 저장한다. 이 세 값으로 표현할 수 없는 축 기울임과 원근 변환은 거부한다.
     * 회전 quaternion은 GLB의 [x,y,z,w] 순서에서 GLM의 (w,x,y,z) 생성자 순서로 바꿔 보관한다.
     * 이 함수는 GPU 객체를 만들지 않는다.
     */
    static ModelResource LoadGLB(const std::filesystem::path& path);

private:
    GltfLoader() = delete;
};

} // namespace grasplink::model
