#pragma once

#include <cstdint>

namespace aerosimlab::graphics
{

/**
 * @brief VertexBuffer Object(VBO)는 꼭짓점의 위치·법선·이미지 좌표 byte를 GPU에 보관한다.
 * @details
 * Vertex(정점)는 삼각형을 이루는 한 꼭짓점의 위치와 표면 방향, 이미지에서 색을 읽을 위치를 담는다. 생성자는 이 CPU byte를 GPU 메모리에 복사한다.
 * 복사 뒤 CPU 원본은 보관하지 않는다. UnBind는 현재 선택만 해제하고 GPU byte는 유지하며, 객체 파괴 때 GPU 사본을 삭제한다.
 *
 * VAO(VertexArray Object)는 정점 자료를 해석하는 설정을 기억한다. stride는 다음 Vertex까지의 byte 간격이고 offset은 한 Vertex 안에서 속성이 시작하는 위치다.
 */
class VertexBuffer
{
public:
    /**
     * @brief 연속된 꼭짓점 데이터를 GPU 메모리에 복사한다.
     * @param data 복사할 원본 byte 배열 주소. 업로드가 끝나면 원본을 보관할 필요가 없다.
     * @param size 복사할 전체 크기 [byte]. 정점 개수가 아니다.
     * @details GPU에 처음 올린 뒤 드물게 바꿀 데이터라는 힌트를 전달한다. 주소와 크기는 검사하지 않으므로 호출자가 유효한 범위를 준다.
     * GPU 작업을 실행하는 OpenGL context가 현재 스레드에서 활성화되어야 한다.
     */
    VertexBuffer(const void* data, uint32_t size);

    /** @brief GPU에 보관한 꼭짓점 데이터를 해제한다. 현재 스레드에서 OpenGL 실행 환경(context)이 활성화되어야 한다. */
    ~VertexBuffer();

    VertexBuffer(const VertexBuffer&) = delete;
    VertexBuffer& operator=(const VertexBuffer&) = delete;

    /** @brief GPU의 현재 정점 자료 대상으로 이 VBO를 선택한다. 실제로 어떤 속성이 있는지는 VAO가 따로 기억한다. */
    void Bind() const;

    /** @brief 현재 꼭짓점 배열 선택을 해제한다. 복사한 데이터는 유지된다. */
    void UnBind() const;

private:
    // GPU가 부여한 꼭짓점 배열의 이름.
    uint32_t m_RendererID = 0;
};

} // namespace aerosimlab::graphics
