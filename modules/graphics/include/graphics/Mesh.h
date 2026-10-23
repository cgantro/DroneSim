#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <glm/glm.hpp>

namespace aerosimlab::graphics
{

/** @brief 한 정점의 3D 위치와 표면 방향을 보관한다. 위치는 모델 중심 기준이다. */
struct Vertex
{
    glm::vec3 position{0.0F};
    glm::vec3 normal{0.0F, 1.0F, 0.0F};
    glm::vec2 textureCoordinate{0.0F};
};

class VertexArray;
class VertexBuffer;
class IndexBuffer;

/**
 * @brief Mesh는 삼각형 모델의 꼭짓점 자료와 삼각형마다 이을 꼭짓점 번호를 GPU에 올려 그릴 형상을 만든다.
 * @details
 * Vertex(정점)는 삼각형 한 모서리의 위치와 표면 방향을 담는다. Mesh는 여러 정점을 연속된 Vertex 구조체 배열로 저장한다.
 * VertexArray Object(VAO)는 각 속성의 byte 형식과 정점 사이 간격을 기억한다. VertexBuffer Object(VBO)는 위치·방향 자료를 GPU에 보관한다.
 * IndexBuffer는 삼각형마다 연결할 정점 번호를 보관한다. 세 번호가 삼각형 하나를 만들고 여러 삼각형이 같은 정점을 함께 쓸 수 있어 저장량이 줄어든다.
 * 인접 면의 표면 방향이 다르면 같은 위치라도 정점을 따로 저장한다. Shader는 정점을 장면 좌표로 바꾸고 표면 방향으로 빛을 계산한다.
 * 생성 중 CPU 배열을 GPU로 복사하므로 그 뒤 원본을 해제해도 된다. GPU 자원을 만들거나 마지막으로 해제할 때 GPU 명령 실행 환경(context)이 현재 스레드에서 활성화되어 있어야 한다.
 * 화면은 Material 기본색을 바탕으로 색을 계산한다. UV 좌표가 있는 Mesh는 이미지 질감도 사용할 수 있다.
 */
class Mesh final
{
public:
    /**
     * @brief CPU 꼭짓점과 연결 번호를 GPU에 복사하고 각 값의 읽는 방법을 설정한다.
     * @param vertices Vertex 배열 주소. 생성 중 복사하므로 호출자가 이후에도 보관할 필요는 없다.
     * @param vertexCount vertices의 Vertex 원소 개수이며 byte 수가 아니다.
     * @param indices 삼각형을 구성하는 정점 번호 배열 주소.
     * @param indexCount indices의 uint32 원소 개수이며 byte 수가 아니다.
     * @throws std::invalid_argument 배열 주소가 null이거나 해당 원소 개수가 0인 경우.
     * @throws std::runtime_error 정점 배열의 byte 크기가 OpenGL 버퍼의 표현 범위를 넘는 경우.
     * @details 각 연결 번호는 vertices 배열 안의 꼭짓점 번호여야 한다. 그리기 시작 위치는 번호 개수로 지정하지만 GPU에는 번호 하나의 byte 크기(4 byte)를 곱한 주소를 전달한다.
     * 이 생성자는 번호가 유효한지, 번호 개수가 삼각형을 이루도록 3의 배수인지 검사하지 않으므로 호출자가 맞는 배열을 제공해야 한다.
     */
    Mesh(
        const Vertex* vertices,
        std::uint32_t vertexCount,
        const std::uint32_t* indices,
        std::uint32_t indexCount);

    /** @brief 소유한 GPU 형상 데이터를 해제한다. 현재 스레드에서 OpenGL 실행 환경(context)이 활성화되어야 한다. */
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    /**
     * @brief 이 Mesh의 VAO와 삼각형 연결 번호가 든 EBO를 현재 그리기 설정으로 선택한다.
     * @details VAO는 정점 위치·법선·이미지 좌표를 어디서 어떻게 읽을지 기억하는 객체다. EBO는 삼각형을 이룰 정점 번호를 저장한다.
     * 그리기가 끝나면 UnBind로 현재 선택을 해제한다. GPU 배열은 남아 다음 그리기에서 다시 쓴다.
     */
    void Bind() const;

    /** @brief 현재 선택된 정점 읽기 설정을 해제한다. GPU 데이터는 유지된다. */
    void UnBind() const;

    /** @brief GPU의 연결 번호 배열에 저장된 꼭짓점 번호 개수를 반환한다. */
    std::uint32_t GetIndexCount() const;

    /**
     * @brief Mesh 기준 좌표의 (0,0,0)을 중심으로 정육면체를 만든다.
     * @param sideLengthMeters 한 변 길이 [m]. 유한한 양수여야 한다.
     * @return 각 면의 표면 방향을 유지하도록 면마다 정점을 나눈 Mesh. 삼각형은 12개다.
     * @throws std::invalid_argument 변 길이가 유한하지 않거나 0 이하인 경우.
     */
    static std::unique_ptr<Mesh> CreateCube(float sideLengthMeters = 1.0F);

    /**
     * @brief 위치·법선·UV 좌표·삼각형 번호가 하나의 메시로 저장된 GLB 모델을 GPU에 올린다.
     * @param path 읽을 GLB 파일 경로다. 위치와 방향은 모델 기준이며 변환이 미리 적용되어 있어야 한다.
     * @throws std::runtime_error 파일이 손상됐거나 지원하지 않는 GLB 구조인 경우.
     */
    static std::unique_ptr<Mesh> LoadGlb(const std::string& path);

private:
    // 화면 그리기 때 읽을 속성 형식과 꼭짓점 연결 번호 배열을 기억한다. 실제 데이터는 아래 GPU 배열에 있다.
    std::unique_ptr<VertexArray> vertexArray_;

    // 정점 구조체의 byte 복사본을 GPU에 둔다. 다음 정점까지의 간격은 Vertex 구조체 크기다.
    std::unique_ptr<VertexBuffer> vertexBuffer_;

    // GPU에는 32-bit 꼭짓점 번호가 있다. 그리기 명령은 번호 세 개씩 묶어 삼각형을 만든다.
    std::unique_ptr<IndexBuffer> indexBuffer_;
};

} // namespace aerosimlab::graphics
