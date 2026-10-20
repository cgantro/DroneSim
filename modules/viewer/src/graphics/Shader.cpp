#include "graphics/Shader.h"

#include "graphics/GlApi.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <utility>

namespace grasplink::graphics
{



// 파일의 단계 이름을 OpenGL이 쓰는 정점·픽셀 계산 종류로 바꾼다.
static GLenum ShaderTypeFromString(const std::string& type)
{
    if (type == "vertex")
        return GL_VERTEX_SHADER;
    if (type == "fragment" || type == "pixel")
        return GL_FRAGMENT_SHADER;
    throw std::runtime_error("Unknown Shader Type: " + type);
}

namespace
{
class PendingShaderProgram final
{
public:
    explicit PendingShaderProgram(std::size_t shaderCount)
    {
        shaderIDs_.reserve(shaderCount);
        program_ = glCreateProgram();
        if (program_ == 0)
            throw std::runtime_error("Failed to create Shader Program");
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

Shader::Shader(const std::string& filepath){
    const std::string src = ReadFile(filepath);

    // 파일 내용을 #type 표식이 나눈 구간별로 읽어 각 Shader 단계의 소스 문자열을 만든다.
    const auto shaderSources = PreProcess(src);

    Compile(shaderSources);

    m_Name = std::filesystem::path(filepath)
        .stem()
        .string();
}

Shader::Shader(
    const std::string& name,
    const std::string& vertexSrc,
    const std::string& fragmentSrc): m_Name(name){
    std::unordered_map<unsigned int, std::string> srcs;

    srcs[GL_VERTEX_SHADER] = vertexSrc;
    srcs[GL_FRAGMENT_SHADER] = fragmentSrc;

    Compile(srcs);
}

Shader::~Shader(){
    if(m_RendererID != 0){
        glDeleteProgram(m_RendererID);

        m_RendererID = 0;
    }
}

std::shared_ptr<Shader> Shader::Create(const std::string& filepath){
    return std::make_shared<Shader>(filepath);
}

std::shared_ptr<Shader> Shader::CreateFromSource(const std::string& name, const std::string& vertexSrc, const std::string& fragmentSrc)
{
    return std::make_shared<Shader>(name, vertexSrc, fragmentSrc);
}

std::string Shader::ReadFile(const std::string& filepath){
    // Shader 전처리는 호출자가 수행하므로 이 단계에서는 #type 표식까지 원문 그대로 유지한다.
    std::ifstream file(filepath, std::ios::in | std::ios::binary);

    if(!file.is_open()) throw std::runtime_error(
        "Could not open Shader File: " + filepath
    );

    std::stringstream stream;
    stream << file.rdbuf();

    return stream.str();
}

std::unordered_map<unsigned int, std::string> Shader::PreProcess(const std::string& source){
    std::unordered_map<unsigned int, std::string> shaderSources;

    std::istringstream stream(source);
    std::string line;

    GLenum currentType = 0;
    std::stringstream currentSource;

    // 각 표식은 다음 줄부터 특정 Shader 단계의 소스를 모으는 경계다. 첫 표식 앞에 있는 설명문은 단계 소스에 넣지 않는다.
    while(std::getline(stream,line)){
        // Windows 줄바꿈은 '\r\n'으로 끝날 수 있다. 줄 끝에 남은 '\r'을 제거해 표식 문자열을 정확히 비교한다.
        if(!line.empty() && line.back() == '\r') line.pop_back();

        // 새 #type 경계를 만나면 앞 단계의 소스를 저장하고, 이후 줄은 새로 지정된 단계에 모은다.
        if(line.rfind("#type ",0) == 0){
            if(currentType != 0){
                shaderSources[currentType] = currentSource.str();

                currentSource.str("");
                currentSource.clear();
            }

            const std::string type = line.substr(6);
            currentType = ShaderTypeFromString(type);
            continue;
        }
        if(currentType == 0) continue;
        currentSource << line << '\n';
    }
    // 파일 끝(EOF)에 도달하면 마지막으로 읽은 #type 구간의 소스도 저장한다.
    if(currentType != 0) shaderSources[currentType] = currentSource.str();
    
    if(shaderSources.empty()) throw std::runtime_error("No Shader Source found.");

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
            throw std::runtime_error("Failed to Create Shader object");

        const GLchar* sourceCstr = source.c_str();
        glShaderSource(shader, 1, &sourceCstr, nullptr);
        glCompileShader(shader);

        GLint isCompiled = GL_FALSE;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &isCompiled);
        if (isCompiled == GL_FALSE)
            throw std::runtime_error("Shader Compile Failed\n" + ShaderInfoLog(shader));

        pending.Attach(shader);
    }

    // 각 단계가 따로 컴파일되어도 서로 연결되지 않을 수 있다. 따라서 모든 단계를 Program으로 묶는 링크 작업을 별도로 확인한다.
    glLinkProgram(pending.Program());
    GLint isLinked = GL_FALSE;
    glGetProgramiv(pending.Program(), GL_LINK_STATUS, &isLinked);
    if (isLinked == GL_FALSE)
        throw std::runtime_error("Shader Link Failed:\n" + ProgramInfoLog(pending.Program()));

    // 링크가 성공하면 실행에 필요한 코드는 Program에 보존된다. 따라서 컴파일에 쓴 임시 Shader 단계 객체는 Program에서 분리해 해제한다.
    m_RendererID = static_cast<uint32_t>(pending.Release());
}

void Shader::Bind() const {glUseProgram(m_RendererID);} 
void Shader::UnBind() const {glUseProgram(0);}

int Shader::GetUniformLocation(const std::string& name)const{
    auto it = m_UniformLocationCache.find(name);
    if(it != m_UniformLocationCache.end()) return it->second;

    // uniform의 위치는 링크된 Program 안에서 고정된다. 찾은 위치와 존재하지 않음을 뜻하는 -1을 모두 이름별로 저장해 반복 조회를 피한다.
    int location = glGetUniformLocation(m_RendererID,name.c_str());
    if(location == -1) std::cout << "Warning: Uniform " << name << "'doesn't exist\n";

    m_UniformLocationCache[name] = location;

    return location;
}


// 각 Set 함수는 값을 대응하는 OpenGL uniform 자료형으로 전달한다. 값을 쓰기 전에 Program을 Bind해야 하며 행렬은 전치 없이 보낸다.
void Shader::SetInt(const std::string& name, int value){glUniform1i(GetUniformLocation(name),value);}
void Shader::SetIntArray(const std::string& name, int* values, uint32_t cnt){glUniform1iv(GetUniformLocation(name),static_cast<GLsizei>(cnt),values);}
void Shader::SetFloat(const std::string& name, float value){glUniform1f(GetUniformLocation(name),value);}
void Shader::SetFloat2(const std::string& name, const glm::vec2& value){glUniform2fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetFloat3(const std::string& name, const glm::vec3& value){glUniform3fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetFloat4(const std::string& name, const glm::vec4& value){glUniform4fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetMat3(const std::string& name, const glm::mat3& value){glUniformMatrix3fv(GetUniformLocation(name),1,GL_FALSE,glm::value_ptr(value));}
void Shader::SetMat4(const std::string& name, const glm::mat4& value){glUniformMatrix4fv(GetUniformLocation(name),1,GL_FALSE,glm::value_ptr(value));}

} // namespace grasplink::graphics
