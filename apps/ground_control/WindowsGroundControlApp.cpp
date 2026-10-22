#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include "GroundControlClient.h"
#include "communication/UdpSocket.h"
#include "simulation_core/ScenarioRunner.h"

#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace
{
constexpr wchar_t windowClassName[] = L"AeroSimLabGroundControlWindow";
constexpr UINT requestTimerId = 1;
constexpr UINT requestPollMilliseconds = 40;
constexpr auto telemetryRefreshInterval = std::chrono::milliseconds(250);
constexpr auto telemetryTimeout = std::chrono::milliseconds(350);
constexpr WORD standardWindowResourceId = 32512;

enum ControlId : int
{
    hostInputId = 100,
    portInputId,
    waypointXInputId,
    waypointYInputId,
    waypointZInputId,
    takeoffButtonId,
    stopButtonId,
    landButtonId,
    waypointButtonId,
    cameraOnButtonId,
    cameraOffButtonId,
    cameraFaultButtonId,
    cameraRecoverButtonId,
    statusTextId,
    telemetryTextId,
    recordingStartButtonId,
    recordingStopButtonId,
    recordingReplayButtonId,
    addDroneButtonId,
    droneListId
};

struct AppState
{
    HWND hostInput = nullptr;
    HWND portInput = nullptr;
    HWND waypointInputs[3]{};
    HWND statusText = nullptr;
    HWND telemetryText = nullptr;
    HWND droneList = nullptr;
    bool requestPending = false;
    bool statusRequest = false;
    std::uint32_t nextCommandId = 1;
    std::uint32_t nextSequence = 1;
    std::optional<aerosimlab::communication::UdpSocket> socket;
    std::optional<aerosimlab::communication::CommandPacket> queuedCommand;
    std::stop_source requestStopSource;
    std::future<aerosimlab::ground_control::CommandResult> request;
    std::chrono::steady_clock::time_point nextStatusPoll{};
};

HWND AddControl(HWND parent, const wchar_t* className, const wchar_t* text,
    DWORD style, int x, int y, int width, int height, int id)
{
    HWND control = CreateWindowExW(0, className, text,
        WS_CHILD | WS_VISIBLE | style,
        x, y, width, height, parent,
        reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
        GetModuleHandleW(nullptr), nullptr);
    SendMessageW(control, WM_SETFONT,
        reinterpret_cast<WPARAM>(GetStockObject(DEFAULT_GUI_FONT)), TRUE);
    return control;
}

void AddLabel(HWND parent, const wchar_t* text, int x, int y, int width)
{
    AddControl(parent, L"STATIC", text, 0, x, y, width, 22, 0);
}

std::wstring ReadText(HWND control)
{
    const int length = GetWindowTextLengthW(control);
    std::wstring text(static_cast<std::size_t>(length) + 1, L'\0');
    const int copied = GetWindowTextW(control, text.data(), length + 1);
    text.resize(static_cast<std::size_t>(copied));
    return text;
}

std::string ToUtf8(std::wstring_view text)
{
    if (text.empty())
        return {};
    const int size = WideCharToMultiByte(CP_UTF8, 0, text.data(),
        static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
    std::string result(static_cast<std::size_t>(size), '\0');
    WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()),
        result.data(), size, nullptr, nullptr);
    return result;
}

std::wstring FromUtf8(std::string_view text)
{
    if (text.empty())
        return {};
    const int size = MultiByteToWideChar(CP_UTF8, 0, text.data(),
        static_cast<int>(text.size()), nullptr, 0);
    std::wstring result(static_cast<std::size_t>(size), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()),
        result.data(), size);
    return result;
}

std::wstring WidenAscii(std::string_view text)
{
    return {text.begin(), text.end()};
}

std::uint16_t ReadPort(HWND control)
{
    const std::string text = ToUtf8(ReadText(control));
    unsigned int value = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size()
        || value == 0 || value > 65535)
        throw std::invalid_argument("포트는 1부터 65535 사이 숫자로 입력하세요.");
    return static_cast<std::uint16_t>(value);
}

double ReadNumber(HWND control)
{
    const std::string text = ToUtf8(ReadText(control));
    double value = 0.0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size() || !std::isfinite(value))
        throw std::invalid_argument("경유점 좌표에는 올바른 숫자를 입력하세요.");
    return value;
}

void SetStatus(AppState& state, const std::wstring& text)
{
    SetWindowTextW(state.statusText, text.c_str());
}

void SetControlsEnabled(AppState& state, bool enabled)
{
    const int buttonIds[]{takeoffButtonId, stopButtonId, landButtonId, waypointButtonId,
        cameraOnButtonId, cameraOffButtonId, cameraFaultButtonId, cameraRecoverButtonId,
        recordingStartButtonId, recordingStopButtonId, recordingReplayButtonId,
        addDroneButtonId};
    for (const int id : buttonIds)
        EnableWindow(GetDlgItem(GetParent(state.statusText), id), enabled);
}

void StartRequest(HWND window, AppState& state,
    aerosimlab::communication::CommandPacket packet, bool isStatusRequest = false)
{
    if (state.requestPending)
        return;

    try
    {
        const std::string host = ToUtf8(ReadText(state.hostInput));
        const std::uint16_t port = ReadPort(state.portInput);
        if (!state.socket)
            state.socket.emplace(0);
        packet.commandId = state.nextCommandId++;
        packet.sequence = state.nextSequence++;
        auto* socket = &*state.socket;
        state.requestPending = true;
        state.statusRequest = isStatusRequest;
        if (!isStatusRequest)
        {
            SetControlsEnabled(state, false);
            SetStatus(state, L"시뮬레이터 응답을 기다리는 중입니다...");
        }
        // 요청이 끝날 때까지 버튼을 잠가 여러 작업이 같은 소켓을 동시에 쓰지 않게 한다.
        state.requestStopSource = std::stop_source{};
        const std::stop_token stopToken = state.requestStopSource.get_token();
        state.request = std::async(std::launch::async,
            [host, port, packet = std::move(packet), socket, isStatusRequest, stopToken]
            {
                return aerosimlab::ground_control::SendCommand(
                    host, port, packet, *socket,
                    isStatusRequest ? telemetryTimeout : std::chrono::seconds(4),
                    stopToken);
            });
        SetTimer(window, requestTimerId, requestPollMilliseconds, nullptr);
    }
    catch (const std::exception& error)
    {
        SetStatus(state, FromUtf8(error.what()));
        if (isStatusRequest)
            state.nextStatusPoll = std::chrono::steady_clock::now() + telemetryRefreshInterval;
        state.requestPending = false;
        SetControlsEnabled(state, true);
    }
}

std::wstring CommandStatusMessage(const aerosimlab::communication::CommandReply& reply)
{
    constexpr std::array<std::pair<std::string_view, std::wstring_view>, 9> messages{{
        {"recording_started", L"입력 기록을 시작했습니다. 게임패드와 비행 명령을 기록합니다."},
        {"recording_stopped", L"기록을 마쳤습니다. 시뮬레이터를 닫기 전까지 재생할 수 있습니다."},
        {"recording_replay_started", L"입력 기록을 재생합니다."},
        {"recording_requires_grounded", L"기록은 드론이 지상에 있을 때 시작할 수 있습니다."},
        {"recording_not_active", L"진행 중인 입력 기록이 없습니다."},
        {"recording_empty", L"재생할 입력 기록이 없습니다."},
        {"recording_unavailable", L"기록 또는 재생이 이미 진행 중입니다."},
        {"drone_added", L"새 드론을 지면에 추가했습니다."},
        {"fleet_limit_reached", L"드론은 최대 8대까지 추가할 수 있습니다."}
    }};

    for (const auto& [status, message] : messages)
    {
        if (reply.status == status)
            return std::wstring(message);
    }

    if (reply.accepted)
        return L"명령이 적용됐습니다.";
    return L"명령이 거부됐습니다: " + WidenAscii(reply.status);
}

void ShowResult(AppState& state)
{
    try
    {
        const auto result = state.request.get();
        const auto& reply = result.reply;
        SetStatus(state, state.statusRequest
            ? L"시뮬레이터 연결됨" : CommandStatusMessage(reply));

        const auto& primaryDrone = reply.telemetry.drone;
        std::wostringstream telemetry;
        telemetry << L"적용 Tick: " << reply.appliedTick
                  << L"   응답 시간: " << result.responseTimeMilliseconds << L"ms\r\n"
                  << L"비행 상태: " << WidenAscii(aerosimlab::simulation::ToString(primaryDrone.mode))
                  << L"   위치: (" << primaryDrone.positionMeters.x << L", "
                  << primaryDrone.positionMeters.y << L", " << primaryDrone.positionMeters.z << L")m\r\n"
                  << L"카메라: " << WidenAscii(aerosimlab::communication::ToString(
                      reply.telemetry.camera.status))
                  << L"   생성 프레임: " << reply.telemetry.camera.frameCount;
        SetWindowTextW(state.telemetryText, telemetry.str().c_str());
        SendMessageW(state.droneList, LB_RESETCONTENT, 0, 0);
        for (const auto& droneTelemetry : reply.telemetry.drones)
        {
            std::wostringstream row;
            row << FromUtf8(droneTelemetry.name) << L"  |  "
                << WidenAscii(aerosimlab::simulation::ToString(droneTelemetry.mode))
                << L"  |  (" << droneTelemetry.positionMeters.x << L", "
                << droneTelemetry.positionMeters.y << L", " << droneTelemetry.positionMeters.z << L") m";
            const std::wstring text = row.str();
            SendMessageW(state.droneList, LB_ADDSTRING, 0,
                reinterpret_cast<LPARAM>(text.c_str()));
        }
    }
    catch (const std::exception& error)
    {
        SetStatus(state, state.statusRequest
            ? L"연결 끊김: 시뮬레이터 응답이 없습니다."
            : L"통신 실패: " + FromUtf8(error.what()));
    }
    state.requestPending = false;
    if (!state.statusRequest)
        SetControlsEnabled(state, true);
    state.statusRequest = false;
    state.nextStatusPoll = std::chrono::steady_clock::now() + telemetryRefreshInterval;
}

void CreateControls(HWND window, AppState& state)
{
    AddLabel(window, L"시뮬레이터 IPv4 주소", 20, 20, 120);
    state.hostInput = AddControl(window, L"EDIT", L"127.0.0.1",
        WS_BORDER | ES_AUTOHSCROLL, 145, 18, 220, 26, hostInputId);
    SendMessageW(state.hostInput, EM_SETLIMITTEXT, 64, 0);
    AddLabel(window, L"UDP 포트", 380, 20, 70);
    state.portInput = AddControl(window, L"EDIT", L"27015",
        WS_BORDER | ES_AUTOHSCROLL | ES_NUMBER, 450, 18, 80, 26, portInputId);

    AddLabel(window, L"비행 명령", 20, 62, 100);
    AddControl(window, L"BUTTON", L"이륙", BS_PUSHBUTTON,
        20, 88, 115, 34, takeoffButtonId);
    AddControl(window, L"BUTTON", L"정지·호버링", BS_PUSHBUTTON,
        145, 88, 145, 34, stopButtonId);
    AddControl(window, L"BUTTON", L"착륙", BS_PUSHBUTTON,
        300, 88, 115, 34, landButtonId);

    AddLabel(window, L"경유점 좌표 (미터)", 20, 137, 180);
    const wchar_t* coordinateNames[]{L"X", L"Y", L"높이"};
    for (int index = 0; index < 3; ++index)
    {
        const int x = 20 + index * 120;
        AddLabel(window, coordinateNames[index], x, 164, 35);
        state.waypointInputs[index] = AddControl(window, L"EDIT", L"0",
            WS_BORDER | ES_AUTOHSCROLL, x + 38, 160, 75, 26, waypointXInputId + index);
        SendMessageW(state.waypointInputs[index], EM_SETLIMITTEXT, 32, 0);
    }
    AddControl(window, L"BUTTON", L"경유점 이동", BS_PUSHBUTTON,
        390, 158, 140, 32, waypointButtonId);

    AddLabel(window, L"카메라", 20, 207, 100);
    const wchar_t* cameraLabels[]{L"켜기", L"끄기", L"고장 주입", L"복구"};
    const int cameraIds[]{cameraOnButtonId, cameraOffButtonId,
        cameraFaultButtonId, cameraRecoverButtonId};
    for (int index = 0; index < 4; ++index)
        AddControl(window, L"BUTTON", cameraLabels[index], BS_PUSHBUTTON,
            20 + index * 130, 233, 120, 32, cameraIds[index]);

    AddLabel(window, L"입력 기록", 20, 282, 140);
    AddControl(window, L"BUTTON", L"기록 시작", BS_PUSHBUTTON,
        20, 307, 155, 32, recordingStartButtonId);
    AddControl(window, L"BUTTON", L"기록 종료", BS_PUSHBUTTON,
        185, 307, 155, 32, recordingStopButtonId);
    AddControl(window, L"BUTTON", L"마지막 기록 재생", BS_PUSHBUTTON,
        350, 307, 180, 32, recordingReplayButtonId);

    AddLabel(window, L"편대 상태", 565, 62, 150);
    AddControl(window, L"BUTTON", L"드론 추가", BS_PUSHBUTTON,
        565, 88, 290, 34, addDroneButtonId);
    state.droneList = AddControl(window, L"LISTBOX", L"",
        WS_BORDER | WS_VSCROLL | LBS_NOINTEGRALHEIGHT,
        565, 132, 290, 360, droneListId);

    AddLabel(window, L"명령 결과", 20, 352, 100);
    state.statusText = AddControl(window, L"STATIC", L"시뮬레이터에 연결해 명령을 보냅니다.",
        SS_LEFT, 20, 378, 510, 34, statusTextId);
    state.telemetryText = AddControl(window, L"STATIC", L"드론과 카메라 상태가 여기에 표시됩니다.",
        SS_LEFT, 20, 415, 510, 72, telemetryTextId);
}

LRESULT CALLBACK WindowProcedure(HWND window, UINT message, WPARAM wParam, LPARAM lParam)
{
    auto* state = reinterpret_cast<AppState*>(GetWindowLongPtrW(window, GWLP_USERDATA));
    if (message == WM_NCCREATE)
    {
        const auto* createInfo = reinterpret_cast<const CREATESTRUCTW*>(lParam);
        state = static_cast<AppState*>(createInfo->lpCreateParams);
        SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(state));
    }

    if (state == nullptr)
        return DefWindowProcW(window, message, wParam, lParam);

    switch (message)
    {
    case WM_CREATE:
        CreateControls(window, *state);
        state->nextStatusPoll = std::chrono::steady_clock::now();
        SetTimer(window, requestTimerId, requestPollMilliseconds, nullptr);
        return 0;
    case WM_COMMAND:
    {
        aerosimlab::communication::CommandPacket packet;
        switch (LOWORD(wParam))
        {
        case takeoffButtonId:
            packet.droneCommand = aerosimlab::simulation::DroneCommand{
                aerosimlab::simulation::DroneCommand::Kind::TakeOff};
            break;
        case stopButtonId:
            packet.droneCommand = aerosimlab::simulation::DroneCommand{
                aerosimlab::simulation::DroneCommand::Kind::Stop};
            break;
        case landButtonId:
            packet.droneCommand = aerosimlab::simulation::DroneCommand{
                aerosimlab::simulation::DroneCommand::Kind::Land};
            break;
        case waypointButtonId:
            try
            {
                packet.droneCommand = aerosimlab::simulation::DroneCommand{
                    aerosimlab::simulation::DroneCommand::Kind::SetWaypoint,
                    {ReadNumber(state->waypointInputs[0]), ReadNumber(state->waypointInputs[1]),
                     ReadNumber(state->waypointInputs[2])}};
            }
            catch (const std::exception& error)
            {
                SetStatus(*state, FromUtf8(error.what()));
                return 0;
            }
            break;
        case cameraOnButtonId: packet.cameraCommand = aerosimlab::communication::CameraCommand::Enable; break;
        case cameraOffButtonId: packet.cameraCommand = aerosimlab::communication::CameraCommand::Disable; break;
        case cameraFaultButtonId: packet.cameraCommand = aerosimlab::communication::CameraCommand::InjectFault; break;
        case cameraRecoverButtonId: packet.cameraCommand = aerosimlab::communication::CameraCommand::Recover; break;
        case recordingStartButtonId:
            packet.sessionCommand = aerosimlab::communication::SessionCommand::StartRecording;
            break;
        case recordingStopButtonId:
            packet.sessionCommand = aerosimlab::communication::SessionCommand::StopRecording;
            break;
        case recordingReplayButtonId:
            packet.sessionCommand = aerosimlab::communication::SessionCommand::ReplayRecording;
            break;
        case addDroneButtonId:
            packet.fleetCommand = aerosimlab::communication::FleetCommand::AddDrone;
            break;
        default: return 0;
        }
        if (state->requestPending && state->statusRequest)
        {
            state->queuedCommand = std::move(packet);
            return 0;
        }
        StartRequest(window, *state, std::move(packet));
        return 0;
    }
    case WM_TIMER:
        if (wParam == requestTimerId && state->requestPending
            && state->request.wait_for(std::chrono::milliseconds::zero()) == std::future_status::ready)
        {
            ShowResult(*state);
            if (state->queuedCommand)
            {
                auto packet = std::move(*state->queuedCommand);
                state->queuedCommand.reset();
                StartRequest(window, *state, std::move(packet));
            }
        }
        else if (wParam == requestTimerId && !state->requestPending
            && std::chrono::steady_clock::now() >= state->nextStatusPoll)
        {
            aerosimlab::communication::CommandPacket packet;
            packet.fleetCommand = aerosimlab::communication::FleetCommand::GetStatus;
            StartRequest(window, *state, std::move(packet), true);
        }
        return 0;
    case WM_CLOSE:
        KillTimer(window, requestTimerId);
        state->queuedCommand.reset();
        if (state->requestPending)
        {
            // 작업이 빌려 쓴 UDP 소켓은 취소된 요청이 끝난 뒤 정리한다.
            state->requestStopSource.request_stop();
        }
        DestroyWindow(window);
        return 0;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProcW(window, message, wParam, lParam);
    }
}
}

int WINAPI wWinMain(
    _In_ HINSTANCE instance,
    _In_opt_ HINSTANCE,
    _In_ PWSTR,
    _In_ int showCommand)
{
    const HBRUSH background = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    const WNDCLASSW windowClass{
        .style = 0,
        .lpfnWndProc = WindowProcedure,
        .cbClsExtra = 0,
        .cbWndExtra = 0,
        .hInstance = instance,
        .hIcon = LoadIconW(nullptr, MAKEINTRESOURCEW(standardWindowResourceId)),
        .hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(standardWindowResourceId)),
        .hbrBackground = background,
        .lpszMenuName = nullptr,
        .lpszClassName = windowClassName
    };
    if (!RegisterClassW(&windowClass))
        return 1;

    AppState state;
    HWND window = CreateWindowExW(0, windowClassName, L"AeroSimLab 지상통제",
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
        CW_USEDEFAULT, CW_USEDEFAULT, 900, 550, nullptr, nullptr, instance, &state);
    if (!window)
        return 1;

    ShowWindow(window, showCommand);
    UpdateWindow(window);
    MSG message{};
    while (GetMessageW(&message, nullptr, 0, 0) > 0)
    {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
    return static_cast<int>(message.wParam);
}
