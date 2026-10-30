#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include <glm/glm.hpp>

namespace dronesim::graphics
{

/**
 * @brief 정점 위치를 화면에 그릴 계산 프로그램 두 개를 준비해 GPU에서 실행하게 한다.
 * @details
 * Shader는 GPU에서 실행할 프로그램이고 GLSL은 그 프로그램을 적는 언어다. Vertex 단계는 각 정점(삼각형 꼭짓점)의 화면 위치를 계산한다.
 * Fragment 단계는 삼각형이 덮은 각 픽셀 조각의 표면 색을 계산한다. 두 단계 프로그램을 연결한 결과를 OpenGL Program이라 부른다.
 * 파일 입력은 `#type vertex`, `#type fragment` 표시를 기준으로 두 코드로 나눈다. 각 코드를 검사·컴파일한 뒤 연결해 하나의 OpenGL Program으로 만든다.
 * 연결이 끝나면 임시 코드 단계 객체를 버리고 실행 Program만 소유한다. 사용자는 먼저 이 Program을 선택하고 입력값을 지정한다.
 * uniform은 그리기 명령마다 GPU Program에 전달하는 입력값이며 예를 들어 색과 좌표 변환 행렬이 있다.
 * 같은 입력 이름의 위치는 캐시한다. 이미지 입력에는 이미지 자체가 아니라 OpenGL 이미지 슬롯 번호를 전달한다.
 * GPU 프로그램을 만들고 삭제할 때는 GPU 작업을 실행하는 OpenGL 실행 환경(context)이 현재 스레드에서 활성화되어야 한다.
 */
class Shader
{
public:
    /**
     * @brief `#type` 표시로 나뉜 정점 코드와 픽셀 코드 파일을 읽어 실행 프로그램을 만든다.
     * @param filepath 코드를 읽을 파일 경로. 파일명과 확장자는 코드 단계를 정하지 않는다.
     * @throws std::runtime_error 파일 열기, 단계 이름 확인, 각 코드 검사·변환 또는 두 코드 연결이 실패하면 발생한다.
     */
    explicit Shader(const std::string& filepath);

    /**
     * @brief 정점 계산 코드와 픽셀 계산 코드로 실행 프로그램을 만든다.
     * @param name 오류 메시지와 식별에 사용할 이름
     * @param vertexSrc 각 꼭짓점을 화면 좌표로 바꾸는 GLSL 코드
     * @param fragmentSrc 화면 픽셀의 색을 계산하는 GLSL 코드
     * @throws std::runtime_error 각 코드 검사·변환 또는 두 코드 연결이 실패하면 발생한다.
     */
    Shader(
        const std::string& name,
        const std::string& vertexSrc,
        const std::string& fragmentSrc);

    /** @brief GPU에서 소유한 실행 프로그램을 삭제한다. 현재 스레드에서 OpenGL 실행 환경(context)이 활성화되어야 한다. */
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    /**
     * @brief 이후 입력값 전달과 그리기 명령에 사용할 GPU Program을 선택한다.
     * @details Program은 정점 위치와 픽셀 색을 계산하도록 연결된 Shader 단계들이다. 선택된 동안 입력값을 바꾸고 그리기를 마쳐야 한다.
     */
    void Bind() const;

    /** @brief 현재 그리기 프로그램 선택을 해제한다. */
    void UnBind() const;

    /** @brief 정수 입력값을 프로그램에 전달한다. 이미지 입력 이름에는 연결할 이미지 슬롯 번호를 쓴다. */
    void SetInt(const std::string& name, int value);
    /** @brief 정수 입력값 여러 개를 연속해서 전달한다. count는 정수의 개수다. */
    void SetIntArray(const std::string& name, int* value, uint32_t count);
    /** @brief 실수 입력값 하나를 전달한다. */
    void SetFloat(const std::string& name, float value);
    /** @brief 실수 두 개로 된 좌표나 값 묶음을 전달한다. */
    void SetFloat2(const std::string& name, const glm::vec2& value);
    /** @brief 실수 세 개로 된 위치, 방향, 색상 등을 전달한다. */
    void SetFloat3(const std::string& name, const glm::vec3& value);
    /** @brief 실수 네 개로 된 색상이나 값 묶음을 전달한다. */
    void SetFloat4(const std::string& name, const glm::vec4& value);
    /** @brief 행과 열을 바꾸지 않고 3×3 실수 행렬을 전달한다. */
    void SetMat3(const std::string& name, const glm::mat3& matrix);
    /** @brief 행과 열을 바꾸지 않고 4×4 실수 행렬을 전달한다. */
    void SetMat4(const std::string& name, const glm::mat4& matrix);

    /** @brief 만들 때 지정한 이 그리기 프로그램의 이름을 반환한다. */
    const std::string& GetName() const { return m_Name; }

private:
    std::string ReadFile(const std::string& filepath);

    // 파일의 #type 표시를 기준으로 꼭짓점 계산 코드와 픽셀 계산 코드를 나눈다.
    std::unordered_map<unsigned int, std::string> PreProcess(const std::string& source);

    // 코드 검사나 연결이 실패하면 임시 코드와 실행 프로그램을 정리한 뒤 오류를 알린다.
    void Compile(const std::unordered_map<unsigned int, std::string>& shaderSources);

    int GetUniformLocation(const std::string& name) const;

private:
    // 코드 두 부분의 연결이 끝난 실행 프로그램 번호. 이 객체가 GPU 소유권을 가진다.
    uint32_t m_RendererID = 0;

    std::string m_Name;

    mutable std::unordered_map<std::string, int> m_UniformLocationCache;
};

} // namespace dronesim::graphics
