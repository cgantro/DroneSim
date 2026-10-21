#pragma once

#include "model/ModelResource.h"

#include <filesystem>

namespace grasplink::model
{

/**
 * @brief GLB 파일에서 화면에 필요한 모델 자료를 읽는다.
 * @param path 읽을 GLB 파일 경로.
 * @return 파일 경로, 자원 번호, CPU 배열, 선택 장면의 시작 부품 번호.
 * @throws std::runtime_error 파일이 잘못됐거나 지원하지 않는 형식을 만났을 때 발생한다.
 * @throws std::out_of_range GLB 안의 정점 자료 번호가 범위를 벗어났을 때 발생한다.
 * @details 형상이 없는 부품도 다른 부품의 위치 기준일 수 있어 결과에 남긴다.
 * 정점은 파일에 기록된 간격으로 읽고, 참조한 자료가 파일 범위 안에 있는지 확인한다.
 * 여러 표면은 한 형상으로 합치되 재질과 그릴 범위는 표면마다 따로 기록한다.
 * 부품 변환은 위치·회전·크기로 저장한다. 이 값으로 표현할 수 없는 축 기울임과 원근 변환은 지원하지 않는다.
 * 회전 quaternion은 GLB의 [x,y,z,w] 순서에서 GLM의 생성자 순서인 (w,x,y,z)로 바꾼다.
 * GPU 업로드는 AssetManager가 맡으며, 정수로 압축한 정점 속성과 Sparse 형식은 지원하지 않는다.
 */
[[nodiscard]] ModelResource LoadGlb(const std::filesystem::path& path);

} // namespace grasplink::model
