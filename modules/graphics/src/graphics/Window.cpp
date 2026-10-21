#include "graphics/Window.h"

#include <stdexcept>

#include "graphics/GlApi.h"
#include <GLFW/glfw3.h>

namespace aerosimlab::graphics
{

Window::Window(const Properties& properties)
{
    Init(properties);
}

Window::~Window()
{
    Shutdown();
}

void Window::Init(const Properties& properties)
{

    // GLFW 초기화는 프로세스 전체에서 공유된다. 초기화가 실패하면 아직 해제할 Window는 만들어지지 않았다.
    if (glfwInit() == GLFW_FALSE)
        throw std::runtime_error("Failed to Init GLFW");

#ifdef __EMSCRIPTEN__
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
    glfwWindowHint(GLFW_VISIBLE, properties.visible ? GLFW_TRUE : GLFW_FALSE);

    m_Handle = glfwCreateWindow(
        properties.width,
        properties.height,
        properties.title.c_str(),
        nullptr,
        nullptr);

    if (m_Handle == nullptr)
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW Window");
    }

    // GPU 자원은 이 OpenGL context에서만 사용할 수 있으므로 Window가 닫히기 전에 먼저 해제해야 한다.
    // 데스크톱 GLAD는 현재 context에서 함수 주소를 읽으므로 함수 적재 전에 context를 활성화한다.
    glfwMakeContextCurrent(m_Handle);

#ifndef __EMSCRIPTEN__
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        // 초기화 중간에 실패해도 이미 만든 창을 닫고 GLFW의 전역 상태를 함께 정리한다.
        Shutdown();
        throw std::runtime_error("Failed to Init GLAD");
    }
#endif

    glfwSetWindowUserPointer(m_Handle, this);
    glfwSetFramebufferSizeCallback(m_Handle, FramebufferSizeCallback);
    glfwSetScrollCallback(m_Handle, ScrollCallback);

    // 값 1은 모니터 주사 시점에 화면을 교환하고, 값 0은 주사 시점과 맞추지 않고 가능한 대로 교환한다.
    glfwSwapInterval(properties.vsync ? 1 : 0);

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    glfwGetFramebufferSize(m_Handle, &framebufferWidth, &framebufferHeight);

    // OpenGL viewport는 창의 논리 단위가 아니라 실제 framebuffer 픽셀 단위로 지정한다.
    glViewport(0, 0, framebufferWidth, framebufferHeight);
}

void Window::Shutdown()
{
    // context를 가진 창을 닫은 다음 GLFW가 프로세스 전체에 보유한 상태를 종료한다.
    if (m_Handle != nullptr)
    {
        glfwDestroyWindow(m_Handle);
        m_Handle = nullptr;
    }

    glfwTerminate();
}

bool Window::ShouldClose() const
{
    return glfwWindowShouldClose(m_Handle) == GLFW_TRUE;
}

void Window::PollEvents() const
{
    glfwPollEvents();
}

void Window::SwapBuffers() const
{
    glfwSwapBuffers(m_Handle);
}

void Window::SetTitle(const std::string& title) const
{
    glfwSetWindowTitle(m_Handle, title.c_str());
}

void Window::GetFramebufferSize(int& width, int& height) const
{
    glfwGetFramebufferSize(m_Handle, &width, &height);
}

bool Window::IsMouseButtonPressed(MouseButton button) const
{
    int glfwButton = GLFW_MOUSE_BUTTON_LEFT;

    switch (button)
    {
    case MouseButton::Left:
        glfwButton = GLFW_MOUSE_BUTTON_LEFT;
        break;
    case MouseButton::Right:
        glfwButton = GLFW_MOUSE_BUTTON_RIGHT;
        break;
    case MouseButton::Middle:
        glfwButton = GLFW_MOUSE_BUTTON_MIDDLE;
        break;
    }

    return glfwGetMouseButton(m_Handle, glfwButton) == GLFW_PRESS;
}

void Window::GetCursorPosition(double& x, double& y) const
{
    glfwGetCursorPos(m_Handle, &x, &y);
}

GLFWwindow* Window::GetNativeHandle() const
{
    return m_Handle;
}

double Window::ConsumeScrollOffset()
{
    const double result = m_ScrollOffset;
    m_ScrollOffset = 0.0;
    return result;
}

void Window::FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    (void)window;
    // HiDPI 배율이 달라져도 framebuffer 크기 변경으로 전달된다. 따라서 viewport를 새 픽셀 크기에 맞춘다.
    glViewport(0, 0, width, height);
}

void Window::ScrollCallback(
    GLFWwindow* window,
    double xOffset,
    double yOffset)
{
    (void)xOffset;

    Window* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
    if (self == nullptr) return;

    // 한 프레임에 발생한 여러 scroll callback의 세로 이동량을 합쳐 카메라 Controller가 한 번에 처리하게 한다.
    self->m_ScrollOffset += yOffset;
}

} // namespace aerosimlab::graphics
