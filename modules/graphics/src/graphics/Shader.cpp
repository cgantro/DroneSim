#include "graphics/Shader.h"

#include "graphics/GlApi.h"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace aerosimlab::graphics
{
namespace
{
/** @brief 파일에 적힌 단계 이름을 OpenGL 셰이더 종류로 바꾼다. */
GLenum ShaderTypeFromString(const std::string& type)
{
    if (type == "vertex")
        return GL_VERTEX_SHADER;
    if (type == "fragment" || type == "pixel")
        return GL_FRAGMENT_SHADER;

    throw std::runtime_error("알 수 없는 셰이더 단계입니다: " + type);
}

/** @brief 링크가 끝날 때까지 임시 셰이더와 프로그램의 소유권을 맡는다. */
class PendingShaderProgram final
{
public:
    explicit PendingShaderProgram(std::size_t shaderCount)
    {
        shaderIDs_.reserve(shaderCount);
        program_ = glCreateProgram();
        if (program_ == 0)
            throw std::runtime_error("화면 계산 프로그램을 만들지 못했습니다.");
    }

    ~PendingShaderProgram()
    {
        for (const GLuint shader : shaderIDs_)
            glDeleteShader(shader);
        if (program_ != 0)
            glDeleteProgram(program_);
    }

    PendingShaderProgram(const PendingShaderProgram&) = delete;
    PendingShaderProgram& operator=(const PendingShaderProgram&) = delete;

    [[nodiscard]] GLuint Program() const noexcept { return program_; }

    [[nodiscard]] GLuint CreateShader(GLenum type)
    {
        const GLuint shader = glCreateShader(type);
        if (shader != 0)
            shaderIDs_.push_back(shader);
        return shader;
    }

    void Attach(GLuint shader) const
    {
        glAttachShader(program_, shader);
    }

    [[nodiscard]] GLuint Release()
    {
        for (const GLuint shader : shaderIDs_)
        {
            glDetachShader(program_, shader);
            glDeleteShader(shader);
        }
        shaderIDs_.clear();
        return std::exchange(program_, 0U);
    }

private:
    GLuint program_ = 0;
    std::vector<GLuint> shaderIDs_;
};

std::string ShaderInfoLog(GLuint shader)
{
    GLint length = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    if (length <= 1)
        return {};

    std::string log(static_cast<std::size_t>(length), '\0');
    GLsizei written = 0;
    glGetShaderInfoLog(shader, length, &written, log.data());
    if (written <= 0)
        return {};

    log.resize(static_cast<std::size_t>(written));
    return log;
}

std::string ProgramInfoLog(GLuint program)
{
    GLint length = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
    if (length <= 1)
        return {};

    std::string log(static_cast<std::size_t>(length), '\0');
    GLsizei written = 0;
    glGetProgramInfoLog(program, length, &written, log.data());
    if (written <= 0)
        return {};

    log.resize(static_cast<std::size_t>(written));
    return log;
}
}

Shader::Shader(const std::string& filepath)
{
    const std::string source = ReadFile(filepath);
    Compile(PreProcess(source));
    m_Name = std::filesystem::path(filepath).stem().string();
}

Shader::Shader(
    const std::string& name,
    const std::string& vertexSource,
    const std::string& fragmentSource)
    : m_Name(name)
{
    std::unordered_map<unsigned int, std::string> sources{
        {GL_VERTEX_SHADER, vertexSource},
        {GL_FRAGMENT_SHADER, fragmentSource}};
    Compile(sources);
}

Shader::~Shader()
{
    if (m_RendererID != 0)
        glDeleteProgram(m_RendererID);
}

std::string Shader::ReadFile(const std::string& filepath)
{
    std::ifstream file(filepath, std::ios::in | std::ios::binary);
    if (!file.is_open())
        throw std::runtime_error("화면 계산 코드 파일을 열지 못했습니다: " + filepath);

    std::stringstream source;
    source << file.rdbuf();
    return source.str();
}

std::unordered_map<unsigned int, std::string> Shader::PreProcess(const std::string& source)
{
    std::unordered_map<unsigned int, std::string> shaderSources;
    std::istringstream input(source);
    std::string line;
    GLenum currentType = 0;
    std::stringstream currentSource;

    // #type 표식 뒤에 나오는 줄을 해당 셰이더 단계의 코드로 모은다.
    while (std::getline(input, line))
    {
        // Windows 줄바꿈 끝의 '\r'을 지워 표식 이름을 정확히 읽는다.
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        if (line.rfind("#type ", 0) == 0)
        {
            if (currentType != 0)
            {
                shaderSources[currentType] = currentSource.str();
                currentSource.str("");
                currentSource.clear();
            }

            currentType = ShaderTypeFromString(line.substr(6));
            continue;
        }

        if (currentType != 0)
            currentSource << line << '\n';
    }

    // 파일 끝에 도달하면 마지막 단계의 코드도 저장한다.
    if (currentType != 0)
        shaderSources[currentType] = currentSource.str();
    if (shaderSources.empty())
        throw std::runtime_error("화면 계산 코드가 없습니다.");

    return shaderSources;
}

void Shader::Compile(const std::unordered_map<unsigned int, std::string>& shaderSources)
{
    PendingShaderProgram pending(shaderSources.size());

    for (const auto& [type, originalSource] : shaderSources)
    {
#ifdef __EMSCRIPTEN__
        std::string source = originalSource;
        const std::string desktopVersion = "#version 330 core";
        const auto versionPosition = source.find(desktopVersion);
        if (versionPosition != std::string::npos)
            source.replace(versionPosition, desktopVersion.size(), "#version 300 es");

        if (type == GL_FRAGMENT_SHADER)
        {
            const auto versionEnd = source.find('\n');
            if (versionEnd != std::string::npos)
                source.insert(versionEnd + 1, "precision highp float;\n");
        }
#else
        const std::string& source = originalSource;
#endif

        const GLuint shader = pending.CreateShader(type);
        if (shader == 0)
            throw std::runtime_error("화면 계산 단계를 만들지 못했습니다.");

        const GLchar* sourcePointer = source.c_str();
        glShaderSource(shader, 1, &sourcePointer, nullptr);
        glCompileShader(shader);

        GLint compiled = GL_FALSE;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if (compiled == GL_FALSE)
            throw std::runtime_error("화면 계산 코드 컴파일에 실패했습니다.\n"
                + ShaderInfoLog(shader));

        pending.Attach(shader);
    }

    // 단계별 코드를 하나의 GPU 프로그램으로 연결한다.
    glLinkProgram(pending.Program());
    GLint linked = GL_FALSE;
    glGetProgramiv(pending.Program(), GL_LINK_STATUS, &linked);
    if (linked == GL_FALSE)
        throw std::runtime_error("화면 계산 단계를 연결하지 못했습니다.\n"
            + ProgramInfoLog(pending.Program()));

    // 연결에 성공하면 실행 프로그램만 이 객체가 소유하고, 임시 단계 코드는 해제한다.
    m_RendererID = static_cast<std::uint32_t>(pending.Release());
}

void Shader::Bind() const
{
    glUseProgram(m_RendererID);
}

void Shader::UnBind() const
{
    glUseProgram(0);
}

int Shader::GetUniformLocation(std::string_view name) const
{
    const auto found = m_UniformLocationCache.find(name);
    if (found != m_UniformLocationCache.end())
        return found->second;

    // 입력 위치는 프로그램 연결 뒤 바뀌지 않아, 이름별로 한 번만 조회한다.
    const std::string ownedName(name);
    const int location = glGetUniformLocation(m_RendererID, ownedName.c_str());
    if (location == -1)
        std::cout << "경고: 입력값을 셰이더에서 찾지 못했습니다: " << name << '\n';

    m_UniformLocationCache.emplace(ownedName, location);
    return location;
}

// 각 입력 함수는 현재 선택한 GPU 프로그램에 값을 전달한다. 행렬은 전치하지 않고 보낸다.
void Shader::SetInt(std::string_view name, int value)
{
    glUniform1i(GetUniformLocation(name), value);
}

void Shader::SetIntArray(std::string_view name, int* values, uint32_t count)
{
    glUniform1iv(GetUniformLocation(name), static_cast<GLsizei>(count), values);
}

void Shader::SetFloat(std::string_view name, float value)
{
    glUniform1f(GetUniformLocation(name), value);
}

void Shader::SetFloat2(std::string_view name, const glm::vec2& value)
{
    glUniform2fv(GetUniformLocation(name), 1, glm::value_ptr(value));
}

void Shader::SetFloat3(std::string_view name, const glm::vec3& value)
{
    glUniform3fv(GetUniformLocation(name), 1, glm::value_ptr(value));
}

void Shader::SetFloat4(std::string_view name, const glm::vec4& value)
{
    glUniform4fv(GetUniformLocation(name), 1, glm::value_ptr(value));
}

void Shader::SetMat3(std::string_view name, const glm::mat3& matrix)
{
    glUniformMatrix3fv(GetUniformLocation(name), 1, GL_FALSE, glm::value_ptr(matrix));
}

void Shader::SetMat4(std::string_view name, const glm::mat4& matrix)
{
    glUniformMatrix4fv(GetUniformLocation(name), 1, GL_FALSE, glm::value_ptr(matrix));
}
}
