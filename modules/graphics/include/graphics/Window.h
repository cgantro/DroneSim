#pragma once

#include <string>

struct GLFWwindow;

namespace aerosimlab::graphics
{

enum class MouseButton
{
    Left,
    Right,
    Middle
};

/**
 * @brief 키보드·마우스 입력을 받고 GPU가 그린 이미지를 화면에 표시하는 창을 소유한다.
 * @details OpenGL context는 GPU 명령을 실행하고 GPU 이미지·프로그램 상태를 연결하는 환경이다. 창을 만들 때 GLFW가 context를 만들고 현재 스레드에서 활성화한다.
 * context는 창이 존재하는 것만으로 현재 실행 중인 조건을 만족하지 않으므로 GPU 호출을 할 스레드에서 활성화되어야 한다. 데스크톱에서는 GLAD가 함수 주소를 읽고, WebAssembly에서는 GLES 3 함수를 직접 사용한다.
 * 이 객체는 창과 context 수명을 관리하므로 GPU 자원을 쓰는 객체를 먼저 정리하고 마지막에 파괴해야 한다. 애플리케이션은 한 번에 창 하나만 만든다.
 */
class Window
{
public:

    /** 창 크기와 화면 표시 방식을 정하는 값이다. */
    struct Properties
    {

        /** 요청하는 창 너비 [논리 픽셀]. */
        int width = 1280;
        /** 요청하는 창 높이 [논리 픽셀]. */
        int height = 720;
        std::string title = "GraspLink";
        /** true이면 완성된 화면을 모니터 주사 주기에 맞춰 표시한다. */
        bool vsync = true;
        /** false이면 보이는 창 없이 화면을 그릴 수 있다. */
        bool visible = true;
    };

public:

    /**
     * @brief 창과 그 창에서 GPU를 호출할 실행 환경을 만들고 OpenGL 함수 주소를 준비한다.
     * @param properties 창의 논리 크기, 제목, 표시 여부, 화면 교환 동기화 설정.
     * @throws std::runtime_error 창 시스템 시작, 창 생성 또는 OpenGL 함수 준비가 실패한 경우.
     */
    explicit Window(const Properties& properties);

    /** 창과 이 프로그램의 GLFW 사용을 종료한다. GPU 자원은 context가 사라지기 전에 먼저 해제해야 한다. */
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    /** @return 사용자가 창 닫기를 눌렀거나 프로그램이 종료를 요청했으면 true. */
    bool ShouldClose() const;

    /**
     * @brief 쌓인 키보드·마우스·창 크기 입력을 읽어 등록된 처리를 실행한다.
     * @details 실제 그릴 픽셀 크기가 바뀌면 이 호출 중 화면 표시 영역도 맞춘다. GetFramebufferSize로 픽셀 수를 다시 읽고 창을 최소화해 한 축이라도 0이면 화면 그리기를 건너뛴다.
     */
    void PollEvents() const;

    /** GPU가 뒤에서 그려 둔 다음 화면을 모니터에 표시하고, 이후 그릴 버퍼를 준비한다. */
    void SwapBuffers() const;

    /**
     * @brief GPU가 그림을 그릴 실제 픽셀 너비와 높이를 돌려준다.
     * @param width 실제 너비 [pixel]
     * @param height 실제 높이 [pixel]
     * @details 고해상도 화면에서는 창 크기를 나타내는 논리 픽셀보다 실제 그림 픽셀이 많을 수 있다.
     * 창을 최소화하면 한 축 또는 두 축이 0일 수 있으므로 그리기 전에 확인해야 한다.
     */
    void GetFramebufferSize(int& width, int& height) const;

    /** @param button 확인할 마우스 버튼 @return 해당 버튼을 현재 누르고 있으면 true. */
    bool IsMouseButtonPressed(MouseButton button) const;

    /**
     * @brief 창 왼쪽 위를 기준으로 커서 위치를 읽는다.
     * @param x 왼쪽에서부터의 거리 [논리 pixel]
     * @param y 위쪽에서부터의 거리 [논리 pixel]
     */
    void GetCursorPosition(double& x, double& y) const;

    /** @return 다른 GLFW 기능에 이 창을 전달할 때 사용하는 원본 창 주소. */
    GLFWwindow* GetNativeHandle() const;

    /** @return 마지막 읽기 뒤 쌓인 세로 휠 이동량을 돌려주고 누적값을 0으로 비운다. */
    double ConsumeScrollOffset();

private:

    void Init(const Properties& properties);

    void Shutdown();

    static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);

    static void ScrollCallback(GLFWwindow* window, double xOffset, double yOffset);

private:
    GLFWwindow* m_Handle = nullptr;

    // 입력 처리기가 전달한 세로 휠 이동량의 합. ConsumeScrollOffset이 읽을 때 비운다.
    double m_ScrollOffset = 0.0;
};

} // namespace aerosimlab::graphics
