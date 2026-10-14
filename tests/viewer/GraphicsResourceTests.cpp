#include "graphics/MultisampleFramebuffer.h"
#include "graphics/Shader.h"
#include "graphics/Window.h"
#include "TestSupport.h"

#include <glad/glad.h>

#include <gtest/gtest.h>

#include <cmath>

#include <array>
#include <iostream>
#include <string>
#include <vector>

namespace
{
using grasplink::graphics::MultisampleFramebuffer;
using grasplink::graphics::Shader;
using grasplink::graphics::Window;

struct GlHooks;
GlHooks* activeHooks = nullptr;

/**
 * @brief OpenGL 객체 생성/실패 지점을 감시하는 테스트용 GLAD 함수 집합.
 * @details GLAD는 C++ 코드가 OpenGL에 요청할 때 실제 그래픽 함수로 연결해 주는 표다. 이 시험용 객체는 원래 함수를 호출하면서 만들어진 framebuffer, texture, shader ID를 기록한다.
 * 일부 검사에서는 OpenGL의 완료 상태나 shader 입력만 바꿔 실패를 재현하고, 생성 도중 만들어진 GPU 객체가 모두 해제되는지 확인한다.
 */
struct TestCheckFailure {};

void CheckImpl(bool condition, const std::string& label, const char* file, int line)
{
    if (condition) return;
    ADD_FAILURE_AT(file, line) << label;
    throw TestCheckFailure{};
}

void CheckNearImpl(double actual, double expected, double tolerance, const std::string& label,
    const char* file, int line)
{
    if (std::isfinite(actual) && std::abs(actual - expected) <= tolerance) return;
    const std::string detail = label + ": actual=" + std::to_string(actual) +
        ", expected=" + std::to_string(expected) + ", tolerance=" + std::to_string(tolerance);
    ADD_FAILURE_AT(file, line) << detail;
    throw TestCheckFailure{};
}

#define Check(...) CheckImpl(__VA_ARGS__, __FILE__, __LINE__)
#define CheckNear(...) CheckNearImpl(__VA_ARGS__, __FILE__, __LINE__)

template<typename Exception, typename Function>
void CheckThrows(Function&& function, const std::string& label)
{
    try
    {
        function();
        Check(false, label);
    }
    catch (const TestCheckFailure&)
    {
        throw;
    }
    catch (const Exception&)
    {
    }
}

struct GlHooks
{
    PFNGLGENFRAMEBUFFERSPROC genFramebuffers = glad_glGenFramebuffers;
    PFNGLGENTEXTURESPROC genTextures = glad_glGenTextures;
    PFNGLGENRENDERBUFFERSPROC genRenderbuffers = glad_glGenRenderbuffers;
    PFNGLCHECKFRAMEBUFFERSTATUSPROC checkFramebufferStatus = glad_glCheckFramebufferStatus;
    PFNGLGETINTEGERVPROC getIntegerv = glad_glGetIntegerv;
    PFNGLCREATEPROGRAMPROC createProgram = glad_glCreateProgram;
    PFNGLCREATESHADERPROC createShader = glad_glCreateShader;
    PFNGLCOMPILESHADERPROC compileShader = glad_glCompileShader;
    std::vector<GLuint> framebuffers;
    std::vector<GLuint> textures;
    std::vector<GLuint> renderbuffers;
    std::vector<GLuint> programs;
    std::vector<GLuint> shaders;
    bool incompleteFramebuffer = false;
    bool unsupportedSamples = false;
    bool failCreateProgram = false;
    int failCreateShaderCall = 0;
    int failCompileShaderCall = 0;
    int createShaderCalls = 0;
    int compileShaderCalls = 0;

    GlHooks();
    ~GlHooks();
    GlHooks(const GlHooks&) = delete;
    GlHooks& operator=(const GlHooks&) = delete;

    void CheckReleased(const std::string& label) const
    {
        for (GLuint id : framebuffers) Check(glIsFramebuffer(id) == GL_FALSE, label + ": framebuffer released");
        for (GLuint id : textures) Check(glIsTexture(id) == GL_FALSE, label + ": texture released");
        for (GLuint id : renderbuffers) Check(glIsRenderbuffer(id) == GL_FALSE, label + ": renderbuffer released");
        for (GLuint id : shaders) Check(glIsShader(id) == GL_FALSE, label + ": shader stage released");
        for (GLuint id : programs) Check(glIsProgram(id) == GL_FALSE, label + ": program released");
    }

    void Reset()
    {
        CheckReleased("before next case");
        framebuffers.clear();
        textures.clear();
        renderbuffers.clear();
        programs.clear();
        shaders.clear();
        incompleteFramebuffer = false;
        unsupportedSamples = false;
        failCreateProgram = false;
        failCreateShaderCall = 0;
        failCompileShaderCall = 0;
        createShaderCalls = 0;
        compileShaderCalls = 0;
    }
};

void APIENTRY RecordFramebuffers(GLsizei count, GLuint* ids)
{
    activeHooks->genFramebuffers(count, ids);
    activeHooks->framebuffers.insert(activeHooks->framebuffers.end(), ids, ids + count);
}

void APIENTRY RecordTextures(GLsizei count, GLuint* ids)
{
    activeHooks->genTextures(count, ids);
    activeHooks->textures.insert(activeHooks->textures.end(), ids, ids + count);
}

void APIENTRY RecordRenderbuffers(GLsizei count, GLuint* ids)
{
    activeHooks->genRenderbuffers(count, ids);
    activeHooks->renderbuffers.insert(activeHooks->renderbuffers.end(), ids, ids + count);
}

GLenum APIENTRY CheckFramebuffer(GLenum target)
{
    return activeHooks->incompleteFramebuffer ? GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT
        : activeHooks->checkFramebufferStatus(target);
}

void APIENTRY GetInteger(GLenum name, GLint* value)
{
    if (name == GL_MAX_SAMPLES && activeHooks->unsupportedSamples) *value = 1;
    else activeHooks->getIntegerv(name, value);
}

GLuint APIENTRY RecordProgram()
{
    if (activeHooks->failCreateProgram) return 0;
    const GLuint id = activeHooks->createProgram();
    if (id != 0) activeHooks->programs.push_back(id);
    return id;
}

GLuint APIENTRY RecordShader(GLenum type)
{
    if (++activeHooks->createShaderCalls == activeHooks->failCreateShaderCall) return 0;
    const GLuint id = activeHooks->createShader(type);
    if (id != 0) activeHooks->shaders.push_back(id);
    return id;
}

void APIENTRY CompileShader(GLuint shader)
{
    if (++activeHooks->compileShaderCalls == activeHooks->failCompileShaderCall)
    {
        // 두 번째 Shader 단계의 GLSL 문법을 실제로 잘못 써 컴파일 실패를 만든다. 어떤 단계가 먼저 생성되는지에 테스트 결과가 좌우되지 않게 한다.
        const GLchar* invalidSource = "#version 330 core\nvoid main(){ invalid GLSL; }\n";
        glShaderSource(shader, 1, &invalidSource, nullptr);
    }
    activeHooks->compileShader(shader);
}

GlHooks::GlHooks()
{
    // 테스트가 끝나거나 예외가 발생해도 바꿔 둔 GLAD 함수 포인터를 원래 OpenGL 함수로 복구한다.
    activeHooks = this;
    glad_glGenFramebuffers = RecordFramebuffers;
    glad_glGenTextures = RecordTextures;
    glad_glGenRenderbuffers = RecordRenderbuffers;
    glad_glCheckFramebufferStatus = CheckFramebuffer;
    glad_glGetIntegerv = GetInteger;
    glad_glCreateProgram = RecordProgram;
    glad_glCreateShader = RecordShader;
    glad_glCompileShader = CompileShader;
}

GlHooks::~GlHooks()
{
    glad_glGenFramebuffers = genFramebuffers;
    glad_glGenTextures = genTextures;
    glad_glGenRenderbuffers = genRenderbuffers;
    glad_glCheckFramebufferStatus = checkFramebufferStatus;
    glad_glGetIntegerv = getIntegerv;
    glad_glCreateProgram = createProgram;
    glad_glCreateShader = createShader;
    glad_glCompileShader = compileShader;
    activeHooks = nullptr;
}

template<typename Function>
void CheckFailure(Function&& function, const std::string& expectedMessage)
{
    try { function(); }
    catch (const std::runtime_error& error)
    {
        Check(std::string(error.what()).find(expectedMessage) != std::string::npos,
            "unexpected failure: " + std::string(error.what()));
        return;
    }
    Check(false, "missing failure: " + expectedMessage);
}

void TestMultisampleFramebuffer(GlHooks& hooks)
{
    // MSAA는 경계 픽셀을 여러 번 계산해 거친 선을 부드럽게 한다. GPU가 지원하는 표본 수보다 크게 요청하지 않는지 확인한다.
    // framebuffer 크기를 바꾸면 그 크기에 맞는 색·깊이 저장소가 필요하다. 생성이나 변경이 실패해도 새로 만든 GPU 객체가 남지 않아야 한다.
    GLint maxSamples = 0;
    glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);
    Check(maxSamples >= 2, "test context supports MSAA");
    CheckThrows<std::invalid_argument>([] { MultisampleFramebuffer invalid(0, 32); }, "zero MSAA width rejected");
    CheckThrows<std::invalid_argument>([] { MultisampleFramebuffer invalid(32, -1); }, "negative MSAA height rejected");
    for (int samples : {-1, 0, 1})
        CheckFailure([&] { MultisampleFramebuffer invalid(32, 32, samples); }, "multisampling is not supported");
    hooks.unsupportedSamples = true;
    CheckFailure([] { MultisampleFramebuffer unsupported(32, 32, 4); }, "multisampling is not supported");
    Check(hooks.framebuffers.empty() && hooks.textures.empty() && hooks.renderbuffers.empty(),
        "invalid or unsupported MSAA allocates nothing");
    hooks.Reset();
    {
        MultisampleFramebuffer framebuffer(32, 48, maxSamples + 1);
        Check(framebuffer.GetSamples() == maxSamples, "MSAA samples clamp to GPU limit");
        framebuffer.Bind();
        Check(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE, "MSAA framebuffer complete");
        framebuffer.Resize(0, 72);
        Check(framebuffer.GetWidth() == 32 && framebuffer.GetHeight() == 48, "invalid resize keeps dimensions");
        framebuffer.Resize(32, 48);
        Check(hooks.framebuffers.size() == 1, "unchanged resize keeps storage");
        framebuffer.Resize(64, 72);
        Check(framebuffer.GetWidth() == 64 && framebuffer.GetHeight() == 72, "MSAA resized dimensions");
        framebuffer.Bind();
        Check(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE, "resized MSAA framebuffer complete");
        std::array<GLint, 4> viewport{};
        glGetIntegerv(GL_VIEWPORT, viewport.data());
        Check(viewport == std::array<GLint, 4>{0, 0, 64, 72}, "resized MSAA viewport");
        GLint width = 0;
        glGetTexLevelParameteriv(GL_TEXTURE_2D_MULTISAMPLE, 0, GL_TEXTURE_WIDTH, &width);
        Check(width == 64, "resized MSAA color storage");
        framebuffer.ResolveToDefault();
    }
    hooks.CheckReleased("MSAA destructor after resize");
    hooks.Reset();
    hooks.incompleteFramebuffer = true;
    CheckFailure([] { MultisampleFramebuffer failed(32, 32); }, "MSAA framebuffer is incomplete");
    Check(hooks.framebuffers.size() == 1 && hooks.textures.size() == 1 && hooks.renderbuffers.size() == 1,
        "failed MSAA constructor allocated all objects");
    hooks.CheckReleased("failed MSAA constructor");
    hooks.Reset();
    {
        MultisampleFramebuffer framebuffer(32, 32);
        hooks.incompleteFramebuffer = true;
        CheckFailure([&] { framebuffer.Resize(64, 80); }, "MSAA framebuffer is incomplete");
        Check(hooks.framebuffers.size() == 2 && hooks.textures.size() == 2 && hooks.renderbuffers.size() == 2,
            "failed resize attempted new storage");
        hooks.CheckReleased("failed resize releases original and new storage");
        hooks.Reset();
        framebuffer.Resize(64, 80);
        Check(hooks.framebuffers.size() == 1 && hooks.textures.size() == 1 && hooks.renderbuffers.size() == 1,
            "same-size retry recreates all MSAA storage");
        framebuffer.Bind();
        Check(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE,
            "MSAA recovers at same size after failed resize");
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    hooks.CheckReleased("recovered MSAA destructor");
    hooks.Reset();
}

void TestShader(GlHooks& hooks)
{
    // Shader 프로그램 또는 단계 생성 자체가 실패하거나 GLSL 컴파일·연결이 실패해도 임시 OpenGL 객체가 남지 않는지 확인한다.
    const std::string vertex = "#version 330 core\nout vec3 value; void main(){value=vec3(1);gl_Position=vec4(0,0,0,1);}";
    const std::string fragment = "#version 330 core\nin vec3 value; out vec4 color; void main(){color=vec4(value,1);}";
    {
        auto shader = Shader::CreateFromSource("resource-test", vertex, fragment);
        Check(shader->GetName() == "resource-test", "shader source name");
        Check(hooks.programs.size() == 1 && hooks.shaders.size() == 2, "shader success resource count");
        Check(glIsProgram(hooks.programs.front()) == GL_TRUE, "linked program retained");
        for (GLuint id : hooks.shaders) Check(glIsShader(id) == GL_FALSE, "linked stage detached and deleted");
        shader->Bind();
        GLint current = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &current);
        Check(current == static_cast<GLint>(hooks.programs.front()), "shader binds linked program");
        shader->UnBind();
    }
    hooks.CheckReleased("shader destructor");
    hooks.Reset();
    hooks.failCompileShaderCall = 2;
    CheckFailure([&] { Shader failed("compile-failure", vertex, fragment); }, "Shader Compile Failed");
    Check(hooks.shaders.size() == 2 && hooks.compileShaderCalls == 2, "second-stage real compile failure exercised");
    // 첫 Shader 단계는 프로그램에 이미 연결되어 있다. 프로그램을 삭제한 뒤 이 단계의 OpenGL 객체도 실제로 해제되는지 확인한다.
    hooks.CheckReleased("second-stage compile failure");
    hooks.Reset();
    const std::string mismatch = "#version 330 core\nin vec2 value; out vec4 color; void main(){color=vec4(value,0,1);}";
    CheckFailure([&] { Shader failed("link-failure", vertex, mismatch); }, "Shader Link Failed");
    Check(hooks.shaders.size() == 2, "real shader link failure exercised");
    hooks.CheckReleased("link failure");
    hooks.Reset();
    for (int failedCall : {1, 2})
    {
        hooks.failCreateShaderCall = failedCall;
        CheckFailure([&] { Shader failed("stage-create-failure", vertex, fragment); }, "Failed to Create Shader object");
        Check(hooks.createShaderCalls == failedCall && hooks.shaders.size() == static_cast<std::size_t>(failedCall - 1),
            "selected shader creation failure exercised");
        hooks.CheckReleased("stage creation failure");
        hooks.Reset();
    }
    hooks.failCreateProgram = true;
    CheckFailure([&] { Shader failed("program-create-failure", vertex, fragment); }, "Failed to create Shader Program");
    Check(hooks.programs.empty() && hooks.shaders.empty(), "program creation failure allocates no stages");
    hooks.Reset();
}
}

class GraphicsResourceTest : public testing::Test
{
protected:
    Window window{Window::Properties{128, 128, "GraphicsResourceTests", false, false}};
    GlHooks hooks;
};

TEST_F(GraphicsResourceTest, MultisampleFramebufferAllocationResizeAndFailureCleanup)
{
    TestMultisampleFramebuffer(hooks);
    Check(glGetError() == GL_NO_ERROR, "MSAA checks leave no OpenGL error");
}

TEST_F(GraphicsResourceTest, ShaderCompilationLinkingAndFailureCleanup)
{
    TestShader(hooks);
    Check(glGetError() == GL_NO_ERROR, "shader checks leave no OpenGL error");
}
