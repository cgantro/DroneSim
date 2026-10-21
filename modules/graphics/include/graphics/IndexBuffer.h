#pragma once

#include <cstdint>

namespace aerosimlab::graphics
{

/**
 * @brief Element Buffer Object(EBO)는 삼각형마다 이을 꼭짓점 번호를 GPU에 보관한다.
 * @details
 * index는 정점 배열에서 어느 Vertex(꼭짓점 자료)를 선택할지 가리키는 번호다. 번호 세 개가 삼각형 하나를 만든다.
 * 이 객체는 32-bit 번호의 GPU 복사본을 소유하고 CPU 원본은 소유하지 않는다. 업로드 byte 수는 번호 개수에 4를 곱해 구한다.
 * UnBind는 현재 VAO와의 연결만 해제하며 GPU 복사본은 남긴다.
 *
 * EBO 연결은 현재 선택된 VertexArray Object(VAO, 정점 자료 읽기 설정)에 기록된다. VAO가 선택된 채 연결을 0으로 바꾸면 그 VAO에 저장된 연결도 지워진다.
 * 그릴 때 같은 VAO를 선택하면 정점 형식과 이 번호 배열을 함께 사용한다.
 */
class IndexBuffer
{
public:
    /**
     * @brief 32-bit 정수로 된 삼각형 꼭짓점 번호 배열을 GPU에 복사한다.
     * @param indices 복사할 정점 번호 배열 주소.
     * @param count 배열에 든 번호 개수. 저장 공간의 byte 수와는 다르다.
     * @throws std::runtime_error indices가 null이거나 count가 0인 경우.
     * @details GPU에는 처음 한 번 올린 뒤 거의 바꾸지 않을 것이라는 사용 정보를 전달한다. 각 번호가 실제 정점을 가리키는지는 확인하지 않으므로 호출자가 올바른 번호를 제공해야 한다.
     * GPU 명령 실행 환경인 OpenGL context가 현재 스레드에서 활성화되어 있어야 한다.
     */
    IndexBuffer(const std::uint32_t* indices, std::uint32_t count);

    /** @brief 소유한 GPU 꼭짓점 번호 배열을 해제한다. 현재 스레드에서 OpenGL 실행 환경(context)이 활성화되어야 한다. */
    ~IndexBuffer();

    IndexBuffer(const IndexBuffer&) = delete;
    IndexBuffer& operator=(const IndexBuffer&) = delete;

    /** @brief 이 EBO의 번호 배열을 현재 VAO가 기억할 연결 번호로 지정한다. */
    void Bind() const;

    /** @brief 현재 정점 읽기 설정에서 번호 배열을 분리한다. 그 설정이 선택돼 있으면 저장된 연결도 지워진다. */
    void UnBind() const;

    /** @brief GPU에 복사한 꼭짓점 번호의 개수를 반환한다. */
    std::uint32_t GetCount() const { return m_Count; }

private:
    // GPU가 부여한 번호 배열의 이름.
    std::uint32_t m_RendererID = 0;

    // 그리기에 사용할 번호 개수. GPU로 복사한 byte 수는 번호 개수×4다.
    std::uint32_t m_Count = 0;
};

} // namespace aerosimlab::graphics
