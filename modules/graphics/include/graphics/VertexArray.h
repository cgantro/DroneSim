#pragma once

#include <cstdint>

namespace aerosimlab::graphics
{

/**
 * @brief VertexArray Object(VAO)는 GPU 정점 자료의 읽는 방법과 꼭짓점 연결 배열을 기억한다.
 * @details
 * 정점(vertex)은 삼각형의 꼭짓점 위치와 법선·이미지 좌표 같은 속성 자료다. VAO는 이 값이나 연결 번호 자체를 보관하지 않고 각 속성의 자료형·다음 정점까지의 byte 간격·정점 안에서 속성이 시작하는 위치를 기억한다.
 * VertexBuffer Object(VBO)는 위치·법선·이미지 좌표가 들어 있는 실제 정점 byte 자료를 GPU에 보관한다.
 * Element Buffer Object(EBO)는 삼각형마다 어느 정점 번호를 이을지 보관한다. 실제 정점 자료는 VertexBuffer, 연결 번호는 IndexBuffer에 있다.
 *
 * VAO를 선택한 상태에서 정점 속성과 연결 번호 배열을 지정하면 그 설정을 VAO가 기억한다. 선택된 상태에서 연결 번호 배열을 0으로 바꾸면 이 VAO가 기억한 연결도 끊긴다.
 * 그릴 때 같은 VAO를 선택하면 정점 자료를 읽는 방법과 삼각형 연결 번호를 함께 사용한다.
 */
class VertexArray
{
public:
    /** @brief GPU 정점 읽기 설정을 기억할 VAO를 만든다. 현재 스레드에 OpenGL 명령을 실행할 context가 활성화되어야 한다. */
    VertexArray();

    /** @brief 저장한 GPU 정점 읽기 설정을 해제한다. 현재 스레드에서 OpenGL 실행 환경(context)이 활성화되어야 한다. */
    ~VertexArray();

    VertexArray(const VertexArray&) = delete;
    VertexArray& operator=(const VertexArray&) = delete;

    /** @brief 이 객체가 기억한 정점 자료 읽기 방법과 연결 번호 배열을 선택한다. */
    void Bind() const;

    /** @brief 정점 읽기 설정의 현재 선택을 해제한다. 저장한 읽기 방법과 GPU 자료는 남는다. */
    void UnBind() const;

private:
    // GPU가 부여한 정점 읽기 설정의 이름.
    uint32_t m_RendererID = 0;
};

} // namespace aerosimlab::graphics
