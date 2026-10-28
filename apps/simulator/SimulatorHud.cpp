#include "SimulatorHud.h"
#include "graphics/GlApi.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/ScenarioRunner.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <chrono>

namespace aerosimlab::simulator
{
namespace
{
void LoadWindowsKoreanFont()
{
    const char* windowsDirectory = std::getenv("WINDIR");
    if (windowsDirectory == nullptr)
        return;

    const std::filesystem::path fontPath =
        std::filesystem::path(windowsDirectory) / "Fonts" / "malgun.ttf";
    if (!std::filesystem::exists(fontPath))
        return;

    const std::string fontPathText = fontPath.string();
    ImGui::GetIO().Fonts->AddFontFromFileTTF(
        fontPathText.c_str(), 18.0F, nullptr, ImGui::GetIO().Fonts->GetGlyphRangesKorean());
}

std::string DroneName(simulation::DroneId id)
{
    return "드론 " + std::to_string(id);
}

const char* CaptureStatusLabel(simulation::CaptureAssessment assessment)
{
    switch (assessment)
    {
    case simulation::CaptureAssessment::Ready: return "목표가 카메라 화면에 보임";
    case simulation::CaptureAssessment::CameraUnavailable: return "카메라 꺼짐 또는 장애";
    case simulation::CaptureAssessment::NoNewFrame: return "새 영상 프레임 없음";
    case simulation::CaptureAssessment::TooNear: return "목표에 너무 가까움";
    case simulation::CaptureAssessment::TooFar: return "목표에서 너무 멂";
    case simulation::CaptureAssessment::SurfaceFacingAway: return "카메라가 목표를 향하지 않음";
    case simulation::CaptureAssessment::OutsideFrame: return "카메라 화면 밖";
    case simulation::CaptureAssessment::ResolutionTooLow: return "화면에서 목표가 너무 작음";
    case simulation::CaptureAssessment::Occluded: return "목표가 구조물에 가려짐";
    }
    return "판정할 수 없음";
}

const char* MeshEventLabel(communication::MeshEventKind kind)
{
    switch (kind)
    {
    case communication::MeshEventKind::LinkDisabled: return "링크 장애";
    case communication::MeshEventKind::LinkRestored: return "링크 복구";
    case communication::MeshEventKind::DirectedLinkDisabled: return "방향 링크 차단";
    case communication::MeshEventKind::DirectedLinkRestored: return "방향 링크 복구";
    case communication::MeshEventKind::NodeRadioDisabled: return "드론 라디오 차단";
    case communication::MeshEventKind::NodeRadioRestored: return "드론 라디오 복구";
    case communication::MeshEventKind::NeighborDiscovered: return "이웃 발견";
    case communication::MeshEventKind::RouteSearchStarted: return "경로 탐색";
    case communication::MeshEventKind::RouteFound: return "경로 복구";
    case communication::MeshEventKind::MessageDelivered: return "메시지 전달";
    case communication::MeshEventKind::PacketDropped: return "패킷 폐기";
    case communication::MeshEventKind::RouteError: return "경로 오류";
    case communication::MeshEventKind::ReliableRetry: return "중요 메시지 재전송";
    case communication::MeshEventKind::ReliableAcknowledged: return "목적지 확인 응답";
    case communication::MeshEventKind::ReliableFailed: return "메시지 확인 실패";
    case communication::MeshEventKind::QueueFull: return "대기 한도 초과";
    }
    return "통신 사건";
}

const char* FaultKindLabel(FaultScenarioTimeline::Event::Kind kind)
{
    using Kind = FaultScenarioTimeline::Event::Kind;
    switch (kind)
    {
    case Kind::PacketLoss: return "패킷 손실";
    case Kind::LinkLatencyJitter: return "지연 흔들림";
    case Kind::CameraFault: return "카메라";
    case Kind::RotorMotorFault: return "로터";
    case Kind::LinkState: return "양방향 링크";
    case Kind::DirectedLinkState: return "한 방향 링크";
    case Kind::RadioState: return "라디오";
    }
    return "장애";
}

bool IsFaultRecovery(const FaultScenarioTimeline::Event& event)
{
    using Kind = FaultScenarioTimeline::Event::Kind;
    switch (event.kind)
    {
    case Kind::PacketLoss: return event.value == 0.0;
    case Kind::LinkLatencyJitter: return event.valueTicks == 0U;
    case Kind::CameraFault:
    case Kind::RotorMotorFault:
    case Kind::LinkState:
    case Kind::DirectedLinkState: return !event.enabled;
    case Kind::RadioState: return event.enabled;
    }
    return false;
}

std::string FaultTargetLabel(const FaultScenarioTimeline::Event& event)
{
    using Kind = FaultScenarioTimeline::Event::Kind;
    switch (event.kind)
    {
    case Kind::LinkState:
    case Kind::DirectedLinkState:
        return DroneName(event.source) + " → " + DroneName(event.destination);
    case Kind::PacketLoss:
    case Kind::LinkLatencyJitter:
        return "전체 통신";
    case Kind::RotorMotorFault:
        return DroneName(event.source) + " 로터 "
            + std::to_string(event.rotorIndex + 1U);
    case Kind::CameraFault:
    case Kind::RadioState:
        return DroneName(event.source);
    }
    return {};
}

const char* NetworkModeLabel(communication::VirtualMeshMode mode)
{
    using Mode = communication::VirtualMeshMode;
    switch (mode)
    {
    case Mode::Direct: return "직접 통신";
    case Mode::LeaderCentric: return "리더 경유";
    case Mode::MultiHop: return "다중 홉 고정 경로";
    case Mode::AdaptiveMesh: return "장애 적응 다중 홉";
    }
    return "알 수 없는 방식";
}

std::string FormatRoute(const std::vector<communication::NetworkNodeId>& route)
{
    std::string result;
    for (const communication::NetworkNodeId node : route)
    {
        if (!result.empty())
            result += " → ";
        result += DroneName(node);
    }
    return result.empty() ? "경로 정보 없음" : result;
}

void CreateDefaultDockspace(ImGuiViewport* viewport)
{
    const ImGuiID dockspaceId = ImGui::GetID("AeroSimLabDockspaceV3");
    if (ImGui::DockBuilderGetNode(dockspaceId) == nullptr)
    {
        ImGui::DockBuilderAddNode(dockspaceId,
            ImGuiDockNodeFlags_DockSpace | ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::DockBuilderSetNodeSize(dockspaceId, viewport->WorkSize);
        ImGuiID sceneNode = 0;
        ImGuiID sideNode = 0;
        ImGui::DockBuilderSplitNode(
            dockspaceId, ImGuiDir_Right, 0.30F, &sideNode, &sceneNode);

        ImGuiID cameraNode = 0;
        ImGuiID statusNode = 0;
        ImGui::DockBuilderSplitNode(
            sideNode, ImGuiDir_Down, 0.52F, &cameraNode, &statusNode);

        ImGui::DockBuilderDockWindow("장면##SceneViewV3", sceneNode);
        ImGui::DockBuilderDockWindow("카메라##CameraViewV3", cameraNode);
        ImGui::DockBuilderDockWindow("AeroSimLab | 비행 상태##FlightStatusV3", statusNode);
        ImGui::DockBuilderFinish(dockspaceId);
    }
    ImGui::DockSpaceOverViewport(dockspaceId, viewport);
}

void DrawScenePanel(const std::function<std::uint32_t(int, int)>& renderScene)
{
    if (ImGui::Begin("장면##SceneViewV3"))
    {
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const ImVec2 imageSize{
            std::max(1.0F, available.x), std::max(1.0F, available.y)};
        const std::uint32_t texture = renderScene(
            static_cast<int>(imageSize.x), static_cast<int>(imageSize.y));
        ImGui::Image(static_cast<ImTextureID>(texture), imageSize,
            ImVec2{0.0F, 1.0F}, ImVec2{1.0F, 0.0F});
    }
    ImGui::End();
}

void DrawCameraPanel(
    const simulation::SimulationCore& simulation,
    const std::function<std::uint32_t(simulation::DroneId)>& renderCamera)
{
    if (ImGui::Begin("카메라##CameraViewV3"))
    {
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const auto droneIds = simulation.DroneIds();
        if (droneIds.empty())
        {
            ImGui::TextUnformatted("드론이 없습니다.");
        }
        else
        {
            constexpr int cameraColumns = 2;
            const int cameraRows = static_cast<int>(
                (droneIds.size() + cameraColumns - 1U) / cameraColumns);
            const float rowSpacing = ImGui::GetStyle().ItemSpacing.y;
            const float cellHeight = std::max(1.0F,
                (available.y - rowSpacing * static_cast<float>(cameraRows - 1))
                    / static_cast<float>(cameraRows));

            if (ImGui::BeginTable("드론별 카메라", cameraColumns,
                    ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_BordersOuterH
                        | ImGuiTableFlags_SizingStretchSame,
                    available))
            {
                for (const simulation::DroneId droneId : droneIds)
                {
                    ImGui::TableNextColumn();
                    const auto& camera = simulation.Camera(droneId);
                    const std::string label = DroneName(droneId);
                    ImGui::TextUnformatted(label.c_str());

                    const int imageWidth = camera.model.imageWidthPixels;
                    const int imageHeight = camera.model.imageHeightPixels;
                    if (camera.status != simulation::CameraStatus::Active
                        || imageWidth <= 0 || imageHeight <= 0)
                    {
                        ImGui::TextDisabled("%s",
                            simulation::ToString(camera.status).data());
                        continue;
                    }

                    const ImVec2 cell = ImGui::GetContentRegionAvail();
                    const float imageBoxHeight = std::max(1.0F,
                        cellHeight - ImGui::GetTextLineHeightWithSpacing()
                            - ImGui::GetStyle().ItemSpacing.y);
                    const float scale = std::min(
                        cell.x / static_cast<float>(imageWidth),
                        imageBoxHeight / static_cast<float>(imageHeight));
                    const ImVec2 imageSize{
                        static_cast<float>(imageWidth) * scale,
                        static_cast<float>(imageHeight) * scale};
                    const float imageX = ImGui::GetCursorPosX()
                        + std::max(0.0F, (cell.x - imageSize.x) * 0.5F);
                    ImGui::SetCursorPosX(imageX);
                    const std::uint32_t texture = renderCamera(droneId);
                    ImGui::Image(static_cast<ImTextureID>(texture), imageSize,
                        ImVec2{0.0F, 1.0F}, ImVec2{1.0F, 0.0F});
                    ImGui::TextDisabled("%d × %d", imageWidth, imageHeight);
                }
                ImGui::EndTable();
            }
        }
    }
    ImGui::End();
}

void DrawRecordingPanel(const SessionRecordingControls& controls)
{
    if (!ImGui::CollapsingHeader("기록과 다시 보기"))
        return;

    if (controls.commandRecording)
    {
        ImGui::Text("명령 기록 중 · %llu Tick",
            static_cast<unsigned long long>(controls.recordedTicks));
        if (ImGui::Button("명령 기록 마치기"))
            controls.stopCommandRecording();
    }
    else if (controls.commandReplay)
    {
        ImGui::Text("명령 기록 재생 중 · %llu Tick",
            static_cast<unsigned long long>(controls.recordedTicks));
    }
    else if (controls.canStartCommandRecording
        && ImGui::Button("조종 입력 기록 시작"))
    {
        controls.startCommandRecording();
    }
    else
    {
        ImGui::TextDisabled("모든 드론이 지상에 있고 다른 기록·임무가 없을 때 시작할 수 있습니다.");
    }

    if (controls.hasRecordedCommands)
    {
        ImGui::Text("기록된 입력: %llu Tick",
            static_cast<unsigned long long>(controls.recordedTicks));
        if (!controls.commandRecording && !controls.commandReplay
            && controls.canReplayCommands
            && ImGui::Button("처음부터 다시 재생"))
        {
            controls.replayCommands();
        }
    }
    else
    {
        ImGui::TextDisabled("기록된 조종 입력이 없습니다.");
    }
    ImGui::TextDisabled("조종 기록은 프로그램을 닫으면 사라집니다.");
    if (!controls.commandStatus.empty())
        ImGui::TextWrapped("%s", controls.commandStatus.c_str());

    ImGui::Separator();
    ImGui::Text("통신 장애 기록");
    if (controls.faultScenarioRecording)
    {
        ImGui::Text("장애 기록 중 · %llu개",
            static_cast<unsigned long long>(controls.faultScenarioEventCount));
        if (controls.canStopFaultScenarioRecording
            && ImGui::Button("장애 기록 저장"))
            controls.stopFaultScenarioRecording();
    }
    else
    {
        if (controls.canSaveFaultScenario
            && ImGui::Button("장애 시나리오 저장"))
            controls.stopFaultScenarioRecording();
        if (controls.canStartFaultScenarioRecording
            && ImGui::Button("장애 기록 시작"))
            controls.startFaultScenarioRecording();
        else if (!controls.canStartFaultScenarioRecording
            && !controls.faultScenarioReplaying)
            ImGui::TextDisabled("편대 임무가 실행 중일 때 기록할 수 있습니다.");

        if (controls.canReplayFaultScenario
            && ImGui::Button("저장한 장애 다시 적용"))
            controls.replayFaultScenario();
    }
    if (controls.faultScenarioReplaying)
    {
        ImGui::Text("장애 재생 중 · 예정 이벤트 %zu개",
            controls.faultScenarioEventCount);
        if (ImGui::BeginTable("FaultTimeline", 3,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH
                    | ImGuiTableFlags_SizingStretchSame))
        {
            ImGui::TableSetupColumn("남은 시간");
            ImGui::TableSetupColumn("동작");
            ImGui::TableSetupColumn("대상");
            ImGui::TableHeadersRow();
            for (const auto& event : controls.pendingFaultEvents)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%.1f초", static_cast<double>(event.tickOffset) / 60.0);
                ImGui::TableNextColumn();
                ImGui::Text("%s · %s", FaultKindLabel(event.kind),
                    IsFaultRecovery(event) ? "복구" : "발생");
                ImGui::TableNextColumn();
                const std::string target = FaultTargetLabel(event);
                ImGui::TextUnformatted(target.c_str());
            }
            ImGui::EndTable();
        }
    }
    ImGui::TextDisabled("패킷 손실·지연·링크·라디오·카메라·로터 고장을 저장합니다.");
    ImGui::TextDisabled("재생하려면 같은 드론 구성으로 편대 임무를 먼저 시작하세요.");
    if (!controls.faultScenarioStatus.empty())
        ImGui::TextWrapped("%s", controls.faultScenarioStatus.c_str());

    ImGui::Separator();
    if (controls.videoRecording)
    {
        ImGui::Text("EO 영상 녹화 중");
        if (controls.canStopVideoRecording
            && ImGui::Button("영상 녹화 마치기"))
        {
            controls.stopVideoRecording();
        }
        else if (!controls.canStopVideoRecording)
        {
            ImGui::TextDisabled("자동 정찰 녹화는 임무가 마무리할 때 저장합니다.");
        }
    }
    else if (controls.canStartVideoRecording
        && ImGui::Button("EO 영상 녹화 시작"))
    {
        controls.startVideoRecording();
    }
    else
    {
        ImGui::TextDisabled("카메라가 켜져 있고 다른 기록·임무가 없을 때 녹화할 수 있습니다.");
    }
    if (!controls.videoStatus.empty())
        ImGui::TextWrapped("%s", controls.videoStatus.c_str());
}

void DrawFlightStatusPanel(
    const simulation::SimulationCore& simulation,
    bool gamepadConnected,
    bool groundControlConnected,
    double missionDistanceMeters,
    const std::string& inspectionTargetName,
    const std::string& inspectionMissionStatus,
    bool missionCanStart,
    const std::function<void()>& startInspectionMission,
    const std::string& fleetReconnaissanceStatus,
    bool fleetReconnaissanceCanStart,
    const std::function<void()>& startFleetReconnaissance,
    bool fleetReturnCanStart,
    const std::function<void()>& startFleetReturn,
    const std::string& completeFleetMissionStatus,
    bool completeFleetMissionCanStart,
    const std::function<void(simulation::FormationShape)>& startCompleteFleetMission,
    const std::string& formationMissionStatus,
    bool formationMissionCanStart,
    bool formationMissionActive,
    simulation::FormationShape& selectedFormationShape,
    const std::function<void(simulation::FormationShape)>& startFormationTransit,
    communication::VirtualMeshMode& selectedNetworkMode,
    bool networkModeCanChange,
    const std::function<void(communication::VirtualMeshMode)>& setNetworkMode,
    float& packetLossPercent,
    const std::function<void(float)>& setPacketLossPercent,
    std::uint32_t& latencyJitterTicks,
    const std::function<void(std::uint32_t)>& setLatencyJitterTicks,
    const communication::VirtualMeshNetwork* formationNetwork,
    int& selectedFaultSource,
    int& selectedFaultDestination,
    int& selectedFaultRotor,
    int& selectedScheduleFaultKind,
    int& scheduleDelaySeconds,
    int& scheduleDurationSeconds,
    const std::function<void(simulation::DroneId, simulation::DroneId, bool)>&
        setFormationLinkDisabled,
    const std::function<void(simulation::DroneId, simulation::DroneId, bool)>&
        setFormationDirectedLinkDisabled,
    const std::function<void(simulation::DroneId, bool)>&
        setFormationNodeRadioEnabled,
    const std::function<void(simulation::DroneId, bool)>& setCameraFault,
    const std::function<void(simulation::DroneId, std::size_t, bool)>&
        setRotorMotorFault,
    simulation::CaptureAssessment captureAssessment,
    const SessionRecordingControls& recordingControls)
{
    if (ImGui::Begin("AeroSimLab | 비행 상태##FlightStatusV3"))
    {
        const auto& primary = simulation.State();
        ImGui::Text("주 드론: %s", DroneName(simulation.PrimaryDroneId()).c_str());
        ImGui::Text("비행 상태: %s", simulation::ToString(primary.mode).data());
        ImGui::Text("위치 [m]: X %.2f   Y %.2f   높이 %.2f",
            primary.positionMeters.x, primary.positionMeters.y, primary.positionMeters.z);
        ImGui::Text("모의 시간: %.2f초   진행 Tick: %llu",
            simulation.SimulationTimeSeconds(),
            static_cast<unsigned long long>(simulation.TickCount()));
        ImGui::Separator();
        ImGui::Text("정찰 대상: %s", inspectionTargetName.c_str());
        ImGui::Text("화면 확인: %s", CaptureStatusLabel(captureAssessment));
        ImGui::TextDisabled("목표가 카메라 화면에 잡히는지 보여 줍니다. 파손 여부는 사람이 영상으로 확인합니다.");
        ImGui::Text("정찰 임무: %s", inspectionMissionStatus.c_str());
        ImGui::Text("촬영 위치까지 %.1f m", missionDistanceMeters);
        if (missionCanStart && ImGui::Button("정찰 임무 시작"))
            startInspectionMission();
        ImGui::Separator();
        ImGui::Text("분산 정찰: %s", fleetReconnaissanceStatus.c_str());
        if (fleetReconnaissanceCanStart
            && ImGui::Button("여러 드론 정찰 시작"))
            startFleetReconnaissance();
        else if (!fleetReconnaissanceCanStart)
            ImGui::TextDisabled("모든 드론이 지상에 있고 다른 임무가 없어야 합니다.");
        if (fleetReturnCanStart && ImGui::Button("출발 지점으로 복귀"))
            startFleetReturn();
        ImGui::Separator();
        ImGui::Text("전체 임무: %s", completeFleetMissionStatus.c_str());
        if (completeFleetMissionCanStart
            && ImGui::Button("전체 임무 시작 (M01 → M02 → M03)"))
            startCompleteFleetMission(selectedFormationShape);
        else if (!completeFleetMissionCanStart)
            ImGui::TextDisabled("모든 드론이 지상에 있고 다른 임무가 없어야 합니다.");
        ImGui::Separator();
        ImGui::Text("편대 이동: %s", formationMissionStatus.c_str());
        const char* formationLabels[]{"V자", "일렬", "횡대"};
        int selectedShape = static_cast<int>(selectedFormationShape);
        if (ImGui::Combo("편대 대형", &selectedShape,
                formationLabels, IM_ARRAYSIZE(formationLabels)))
            selectedFormationShape = static_cast<simulation::FormationShape>(selectedShape);
        if (formationMissionCanStart && ImGui::Button("편대 이동 시작"))
            startFormationTransit(selectedFormationShape);
        else if (!formationMissionCanStart)
            ImGui::TextDisabled("모든 드론이 지상에 있을 때 시작할 수 있습니다.");

        const char* networkModeLabels[]{
            "직접 통신", "리더 경유", "다중 홉 고정 경로", "장애 적응 다중 홉"};
        int networkModeIndex = static_cast<int>(selectedNetworkMode);
        if (networkModeCanChange
            && ImGui::Combo("통신 방식", &networkModeIndex,
                networkModeLabels, IM_ARRAYSIZE(networkModeLabels)))
        {
            selectedNetworkMode = static_cast<communication::VirtualMeshMode>(
                networkModeIndex);
            setNetworkMode(selectedNetworkMode);
        }
        else if (!networkModeCanChange)
        {
            ImGui::Text("다음 임무 통신 방식: %s",
                NetworkModeLabel(selectedNetworkMode));
            ImGui::TextDisabled("통신 방식을 바꾸려면 모든 드론을 지상에 두세요.");
        }
        if (formationNetwork != nullptr)
        {
            ImGui::Separator();
            ImGui::Text("현재 통신 방식: %s", NetworkModeLabel(selectedNetworkMode));
            if (formationMissionActive)
            {
                ImGui::Text("통신 장애 시험");
                if (ImGui::SliderFloat("패킷 손실률", &packetLossPercent,
                        0.0F, 100.0F, "%.0f%%"))
                    setPacketLossPercent(packetLossPercent);
                constexpr std::uint32_t minimumJitterTicks = 0U;
                constexpr std::uint32_t maximumJitterTicks = 12U;
                if (ImGui::SliderScalar("지연 흔들림 상한", ImGuiDataType_U32,
                        &latencyJitterTicks, &minimumJitterTicks,
                        &maximumJitterTicks, "%u Tick"))
                    setLatencyJitterTicks(latencyJitterTicks);
                ImGui::TextDisabled("기본 지연에 0~상한 Tick을 더합니다. 1 Tick은 약 16.7ms입니다.");
                ImGui::TextDisabled("확인 응답이 없으면 정찰 경로를 60 Tick마다 최대 10회 다시 보냅니다.");
                const auto droneIds = simulation.DroneIds();
                if (droneIds.size() >= 2U)
                {
                    const int lastDroneIndex = static_cast<int>(droneIds.size() - 1U);
                    selectedFaultSource = std::clamp(selectedFaultSource, 0, lastDroneIndex);
                    selectedFaultDestination = std::clamp(
                        selectedFaultDestination, 0, lastDroneIndex);
                    std::array<std::string, communication::maximumFleetDroneCount> names;
                    std::array<const char*, communication::maximumFleetDroneCount> namePointers{};
                    for (std::size_t index = 0; index < droneIds.size(); ++index)
                    {
                        names[index] = DroneName(droneIds[index]);
                        namePointers[index] = names[index].c_str();
                    }

                    ImGui::SetNextItemWidth(-1.0F);
                    ImGui::Combo("송신 드론", &selectedFaultSource,
                        namePointers.data(), static_cast<int>(droneIds.size()));
                    ImGui::SetNextItemWidth(-1.0F);
                    ImGui::Combo("상대 드론", &selectedFaultDestination,
                        namePointers.data(), static_cast<int>(droneIds.size()));
                    const simulation::DroneId source = droneIds[
                        static_cast<std::size_t>(selectedFaultSource)];
                    const simulation::DroneId destination = droneIds[
                        static_cast<std::size_t>(selectedFaultDestination)];

                    ImGui::Separator();
                    ImGui::Text("장애 예약");
                    using FaultKind = FaultScenarioTimeline::Event::Kind;
                    constexpr std::array<FaultKind, 7> scheduleKinds{
                        FaultKind::PacketLoss,
                        FaultKind::LinkLatencyJitter,
                        FaultKind::CameraFault,
                        FaultKind::RotorMotorFault,
                        FaultKind::LinkState,
                        FaultKind::DirectedLinkState,
                        FaultKind::RadioState};
                    constexpr const char* scheduleLabels[]{
                        "패킷 손실", "지연 흔들림", "카메라 고장", "로터 고장",
                        "양방향 링크 단절", "한 방향 링크 단절", "라디오 차단"};
                    selectedScheduleFaultKind = std::clamp(selectedScheduleFaultKind,
                        0, static_cast<int>(scheduleKinds.size() - 1U));
                    ImGui::SetNextItemWidth(-1.0F);
                    ImGui::Combo("장애 종류", &selectedScheduleFaultKind,
                        scheduleLabels, IM_ARRAYSIZE(scheduleLabels));
                    ImGui::InputInt("시작까지 (초)", &scheduleDelaySeconds);
                    ImGui::InputInt("지속 시간 (초)", &scheduleDurationSeconds);
                    scheduleDelaySeconds = std::clamp(scheduleDelaySeconds, 0, 600);
                    scheduleDurationSeconds = std::clamp(
                        scheduleDurationSeconds, 1, 3600);
                    ImGui::TextDisabled(
                        "현재 선택한 드론·장애 값을 지정 시각에 적용하고 시간이 지나면 기본 상태로 돌립니다.");

                    FaultScenarioTimeline::Event scheduledFault;
                    scheduledFault.kind = scheduleKinds[
                        static_cast<std::size_t>(selectedScheduleFaultKind)];
                    scheduledFault.source = source;
                    scheduledFault.destination = destination;
                    scheduledFault.enabled = true;
                    switch (scheduledFault.kind)
                    {
                    case FaultKind::PacketLoss:
                        scheduledFault.value = static_cast<double>(packetLossPercent)
                            / 100.0;
                        break;
                    case FaultKind::LinkLatencyJitter:
                        scheduledFault.valueTicks = latencyJitterTicks;
                        break;
                    case FaultKind::RotorMotorFault:
                        scheduledFault.rotorIndex = static_cast<std::uint32_t>(
                            selectedFaultRotor);
                        break;
                    case FaultKind::RadioState:
                        scheduledFault.enabled = false;
                        break;
                    case FaultKind::CameraFault:
                    case FaultKind::LinkState:
                    case FaultKind::DirectedLinkState:
                        break;
                    }
                    if (recordingControls.canScheduleFault
                        && recordingControls.scheduleFault
                        && ImGui::Button("장애 예약"))
                    {
                        constexpr std::uint32_t ticksPerSecond = 60U;
                        recordingControls.scheduleFault(scheduledFault,
                            static_cast<std::uint32_t>(scheduleDelaySeconds)
                                * ticksPerSecond,
                            static_cast<std::uint32_t>(scheduleDurationSeconds)
                                * ticksPerSecond);
                    }
                    else if (!recordingControls.canScheduleFault)
                    {
                        ImGui::TextDisabled("편대 임무 중, 장애 기록을 멈춘 상태에서 예약하세요.");
                    }
                    if (recordingControls.scheduledFaultCount > 0U)
                    {
                        ImGui::Text("현재 예약: %zu개",
                            recordingControls.scheduledFaultCount);
                    }

                    ImGui::Separator();
                    const bool radioEnabled = formationNetwork->IsNodeRadioEnabled(source);
                    ImGui::Text("선택 드론 라디오: %s",
                        radioEnabled ? "송수신 가능" : "차단됨");
                    if (radioEnabled)
                    {
                        if (ImGui::Button("선택 드론 송수신 차단"))
                            setFormationNodeRadioEnabled(source, false);
                    }
                    else if (ImGui::Button("선택 드론 라디오 복구"))
                    {
                        setFormationNodeRadioEnabled(source, true);
                    }

                    const auto cameraStatus = simulation.Camera(source).status;
                    const bool cameraFaulted = cameraStatus
                        == simulation::CameraStatus::Fault;
                    ImGui::Text("선택 드론 카메라: %s",
                        cameraFaulted ? "고장"
                            : cameraStatus == simulation::CameraStatus::Active
                                ? "켜짐" : "꺼짐");
                    if (cameraFaulted)
                    {
                        if (ImGui::Button("선택 드론 카메라 복구"))
                            setCameraFault(source, false);
                    }
                    else if (ImGui::Button("선택 드론 카메라 고장 주입"))
                    {
                        setCameraFault(source, true);
                    }

#ifndef __EMSCRIPTEN__
                    constexpr int rotorCount = static_cast<int>(
                        simulation::QuadrotorParameters::rotorCount);
                    const char* rotorLabels[]{"로터 1", "로터 2", "로터 3", "로터 4"};
                    selectedFaultRotor = std::clamp(selectedFaultRotor, 0, rotorCount - 1);
                    ImGui::SetNextItemWidth(-1.0F);
                    ImGui::Combo("고장 로터", &selectedFaultRotor,
                        rotorLabels, rotorCount);
                    const auto rotorIndex = static_cast<std::size_t>(selectedFaultRotor);
                    if (simulation.IsRotorMotorFaulted(source, rotorIndex))
                    {
                        if (ImGui::Button("선택 로터 복구"))
                            setRotorMotorFault(source, rotorIndex, false);
                    }
                    else if (ImGui::Button("선택 로터 고장 주입"))
                    {
                        setRotorMotorFault(source, rotorIndex, true);
                    }
                    ImGui::TextDisabled("전원이 끊기면 로터가 감속하며 기체 자세가 무너질 수 있습니다.");
#endif

                    if (source == destination)
                    {
                        ImGui::TextDisabled("서로 다른 드론을 골라 주세요.");
                    }
                    else
                    {
                        const bool disabled = formationNetwork->IsLinkDisabled(source, destination);
                        ImGui::Text("강제 장애: %s", disabled ? "설정됨" : "해제됨");
                        const auto nextHop = formationNetwork->NextHop(source, destination);
                        if (nextHop)
                            ImGui::Text("현재 다음 드론: %s", DroneName(*nextHop).c_str());
                        else
                            ImGui::TextDisabled("현재 경로: 찾는 중 또는 연결 안 됨");
                        if (disabled)
                        {
                            if (ImGui::Button("링크 복구"))
                                setFormationLinkDisabled(source, destination, false);
                        }
                        else if (ImGui::Button("링크 끊기"))
                        {
                            setFormationLinkDisabled(source, destination, true);
                        }

                        ImGui::Separator();
                        const bool forwardDisabled =
                            formationNetwork->IsDirectedLinkDisabled(source, destination);
                        const bool reverseDisabled =
                            formationNetwork->IsDirectedLinkDisabled(destination, source);
                        ImGui::Text("방향 장애: %s → %s %s · %s → %s %s",
                            DroneName(source).c_str(), DroneName(destination).c_str(),
                            forwardDisabled ? "차단" : "연결",
                            DroneName(destination).c_str(), DroneName(source).c_str(),
                            reverseDisabled ? "차단" : "연결");
                        const std::string forwardLabel = DroneName(source)
                            + (forwardDisabled ? " → 복구##ForwardLink"
                                               : " → 차단##ForwardLink");
                        if (ImGui::Button(forwardLabel.c_str()))
                            setFormationDirectedLinkDisabled(
                                source, destination, !forwardDisabled);
                        ImGui::SameLine();
                        const std::string reverseLabel = DroneName(destination)
                            + (reverseDisabled ? " → 복구##ReverseLink"
                                               : " → 차단##ReverseLink");
                        if (ImGui::Button(reverseLabel.c_str()))
                            setFormationDirectedLinkDisabled(
                                destination, source, !reverseDisabled);
                    }
                }
            }
            const auto& metrics = formationNetwork->Metrics();
            ImGui::Text("전달 %llu · 중계 %llu · 폐기 %llu",
                static_cast<unsigned long long>(metrics.deliveredMessages),
                static_cast<unsigned long long>(metrics.forwardedMessages),
                static_cast<unsigned long long>(metrics.droppedMessages));
            if (ImGui::CollapsingHeader("현재 라우팅 경로"))
            {
                const auto droneIds = simulation.DroneIds();
                constexpr ImGuiTableFlags routeTableFlags =
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                    | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp;
                if (ImGui::BeginTable("FleetRoutes", 4, routeTableFlags,
                        ImVec2(0.0F, 150.0F)))
                {
                    ImGui::TableSetupColumn("보내는 드론");
                    ImGui::TableSetupColumn("목적지");
                    ImGui::TableSetupColumn("다음 홉");
                    ImGui::TableSetupColumn("연결 방식");
                    ImGui::TableHeadersRow();
                    for (const simulation::DroneId source : droneIds)
                    {
                        for (const simulation::DroneId destination : droneIds)
                        {
                            if (source == destination)
                                continue;

                            const auto nextHop = formationNetwork->NextHop(
                                source, destination);
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);
                            ImGui::TextUnformatted(DroneName(source).c_str());
                            ImGui::TableSetColumnIndex(1);
                            ImGui::TextUnformatted(DroneName(destination).c_str());
                            ImGui::TableSetColumnIndex(2);
                            ImGui::TextUnformatted(nextHop
                                ? DroneName(*nextHop).c_str() : "없음");
                            ImGui::TableSetColumnIndex(3);
                            const char* routeKind = !nextHop ? "경로 없음"
                                : *nextHop == destination ? "직접 연결" : "중계";
                            ImGui::TextUnformatted(routeKind);
                        }
                    }
                    ImGui::EndTable();
                }
            }
            if (ImGui::CollapsingHeader("최근 도착 패킷"))
            {
                const auto messages = formationNetwork->DeliveredMessages();
                const std::size_t firstVisibleMessage = messages.size() > 8U
                    ? messages.size() - 8U : 0U;
                if (firstVisibleMessage == messages.size())
                {
                    ImGui::TextDisabled("아직 도착한 패킷이 없습니다.");
                }
                for (std::size_t index = firstVisibleMessage;
                     index < messages.size(); ++index)
                {
                    const communication::DeliveredMeshMessage& delivered = messages[index];
                    const communication::VirtualMeshMessage& message = delivered.message;
                    const std::string label = "Tick "
                        + std::to_string(delivered.deliveredAtTick) + " · "
                        + message.kind + " · #" + std::to_string(message.sequence);
                    if (ImGui::TreeNode(label.c_str()))
                    {
                        ImGui::Text("보낸 드론: %s",
                            DroneName(message.source).c_str());
                        ImGui::Text("받은 드론: %s",
                            DroneName(message.destination).c_str());
                        const std::string route = FormatRoute(message.route);
                        ImGui::Text("전달 경로: %s", route.c_str());
                        ImGui::TextWrapped("내용: %s", message.payload.c_str());
                        ImGui::TreePop();
                    }
                }
            }
            const auto events = formationNetwork->Events();
            const std::size_t visibleEvents = std::min<std::size_t>(events.size(), 4U);
            for (std::size_t offset = 0; offset < visibleEvents; ++offset)
            {
                const auto& event = events[events.size() - 1U - offset];
                ImGui::Text("%.2f초  %s  %s → %s",
                    static_cast<double>(event.tick) / 60.0,
                    MeshEventLabel(event.kind), DroneName(event.source).c_str(),
                    DroneName(event.destination).c_str());
                ImGui::TextDisabled("  %s", event.detail.c_str());
            }
        }
        ImGui::Separator();
        DrawRecordingPanel(recordingControls);
        ImGui::TextDisabled("조작: A 이륙 · 왼쪽 스틱 이동 · RT/LT 높이 · B 착륙");
        ImGui::Separator();
        ImGui::Text("게임패드: %s   지상통제: %s",
            gamepadConnected ? "연결됨" : "연결 안 됨",
            groundControlConnected ? "최근 입력 수신" : "최근 입력 없음");
        ImGui::Separator();
        ImGui::Text("편대 상태 (%zu / %zu)",
            simulation.DroneCount(), communication::maximumFleetDroneCount);
        for (const simulation::DroneId id : simulation.DroneIds())
        {
            const auto& drone = simulation.State(id);
            ImGui::Text("%s  %-16s  (%.1f, %.1f, %.1f) m",
                DroneName(id).c_str(), simulation::ToString(drone.mode).data(),
                drone.positionMeters.x, drone.positionMeters.y, drone.positionMeters.z);
        }
    }
    ImGui::End();
}
}

SimulatorHud::SimulatorHud(GLFWwindow* window)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    LoadWindowsKoreanFont();

    if (const char* localAppData = std::getenv("LOCALAPPDATA"))
    {
        std::error_code error;
        const auto settingsDirectory = std::filesystem::path(localAppData) / "AeroSimLab";
        std::filesystem::create_directories(settingsDirectory, error);
        if (!error)
        {
            layoutFile_ = (settingsDirectory / "imgui.ini").string();
            ImGui::GetIO().IniFilename = layoutFile_.c_str();
        }
    }
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    if (!ImGui_ImplGlfw_InitForOpenGL(window, true))
    {
        ImGui::DestroyContext();
        throw std::runtime_error("시뮬레이터 HUD 입력을 준비하지 못했습니다.");
    }
    if (!ImGui_ImplOpenGL3_Init("#version 330"))
    {
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        throw std::runtime_error("시뮬레이터 HUD 화면을 준비하지 못했습니다.");
    }
    initialized_ = true;
}

SimulatorHud::~SimulatorHud()
{
    if (!initialized_)
        return;

#ifndef __EMSCRIPTEN__
    for (const PendingGpuQuery& query : backendGpuQueries_)
    {
        if (query.query != 0)
            glDeleteQueries(1, &query.query);
    }
#endif
    if (!layoutFile_.empty())
        ImGui::SaveIniSettingsToDisk(layoutFile_.c_str());
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void SimulatorHud::CollectCompletedGpuTimings(HudRenderTimings* timings)
{
#ifndef __EMSCRIPTEN__
    if (timings == nullptr || !timings->collectGpuTimings || !GLAD_GL_VERSION_3_3)
        return;

    for (PendingGpuQuery& query : backendGpuQueries_)
    {
        if (!query.pending)
            continue;

        GLint available = GL_FALSE;
        glGetQueryObjectiv(query.query, GL_QUERY_RESULT_AVAILABLE, &available);
        if (available != GL_TRUE)
            continue;

        GLuint64 durationNanoseconds = 0;
        glGetQueryObjectui64v(query.query, GL_QUERY_RESULT, &durationNanoseconds);
        const std::size_t resultIndex = timings->completedGpuFrameCount++;
        if (resultIndex < timings->completedGpuFrames.size())
        {
            timings->completedGpuFrames[resultIndex] = HudGpuTiming{
                query.frameIndex,
                static_cast<double>(durationNanoseconds) / 1000.0};
        }
        query.pending = false;
    }
#else
    static_cast<void>(timings);
#endif
}

void SimulatorHud::Render(const simulation::SimulationCore& simulation,
    bool gamepadConnected, bool groundControlConnected,
    double missionDistanceMeters,
    const std::string& inspectionTargetName,
    const std::string& inspectionMissionStatus, bool missionCanStart,
    const std::function<void()>& startInspectionMission,
    const std::string& fleetReconnaissanceStatus,
    bool fleetReconnaissanceCanStart,
    const std::function<void()>& startFleetReconnaissance,
    bool fleetReturnCanStart,
    const std::function<void()>& startFleetReturn,
    const std::string& completeFleetMissionStatus,
    bool completeFleetMissionCanStart,
    const std::function<void(simulation::FormationShape)>& startCompleteFleetMission,
    const std::string& formationMissionStatus, bool formationMissionCanStart,
    bool formationMissionActive,
    const std::function<void(simulation::FormationShape)>& startFormationTransit,
    communication::VirtualMeshMode& selectedNetworkMode,
    const std::function<void(communication::VirtualMeshMode)>& setNetworkMode,
    float& packetLossPercent,
    const std::function<void(float)>& setPacketLossPercent,
    std::uint32_t& latencyJitterTicks,
    const std::function<void(std::uint32_t)>& setLatencyJitterTicks,
    const communication::VirtualMeshNetwork* formationNetwork,
    const std::function<void(simulation::DroneId, simulation::DroneId, bool)>&
        setFormationLinkDisabled,
    const std::function<void(simulation::DroneId, simulation::DroneId, bool)>&
        setFormationDirectedLinkDisabled,
    const std::function<void(simulation::DroneId, bool)>&
        setFormationNodeRadioEnabled,
    const std::function<void(simulation::DroneId, bool)>& setCameraFault,
    const std::function<void(simulation::DroneId, std::size_t, bool)>&
        setRotorMotorFault,
    simulation::CaptureAssessment captureAssessment,
    const SessionRecordingControls& recordingControls,
    const std::function<std::uint32_t(int, int)>& renderScene,
    const std::function<std::uint32_t(simulation::DroneId)>& renderCamera,
    HudRenderTimings* timings)
{
    CollectCompletedGpuTimings(timings);

    // 패널을 만드는 동안 장면·카메라 그리기 콜백도 실행되므로 순수 UI 시간은 아니다.
    const auto interfaceBuildStart = timings != nullptr
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    CreateDefaultDockspace(viewport);
    DrawScenePanel(renderScene);
    DrawCameraPanel(simulation, renderCamera);
    DrawFlightStatusPanel(simulation, gamepadConnected, groundControlConnected,
        missionDistanceMeters, inspectionTargetName,
        inspectionMissionStatus,
        missionCanStart, startInspectionMission, fleetReconnaissanceStatus,
        fleetReconnaissanceCanStart, startFleetReconnaissance, fleetReturnCanStart,
        startFleetReturn, completeFleetMissionStatus, completeFleetMissionCanStart,
        startCompleteFleetMission, formationMissionStatus,
        formationMissionCanStart, formationMissionActive, selectedFormationShape_,
        startFormationTransit, selectedNetworkMode, formationMissionCanStart,
        setNetworkMode, packetLossPercent, setPacketLossPercent,
        latencyJitterTicks, setLatencyJitterTicks,
        formationNetwork, selectedFaultSource_,
        selectedFaultDestination_, selectedFaultRotor_,
        selectedScheduleFaultKind_, scheduleDelaySeconds_, scheduleDurationSeconds_,
        setFormationLinkDisabled,
        setFormationDirectedLinkDisabled,
        setFormationNodeRadioEnabled, setCameraFault, setRotorMotorFault,
        captureAssessment, recordingControls);

    if (timings != nullptr)
    {
        timings->interfaceBuildMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - interfaceBuildStart).count();
    }
    const auto frameFinalizeStart = timings != nullptr
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    ImGui::Render();
    if (timings != nullptr)
    {
        timings->frameFinalizeMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - frameFinalizeStart).count();
    }
    const auto backendDrawStart = timings != nullptr
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    ImDrawData* drawData = ImGui::GetDrawData();
    if (timings != nullptr)
    {
        timings->drawListCount = static_cast<std::size_t>(drawData->CmdListsCount);
        timings->vertexCount = static_cast<std::size_t>(drawData->TotalVtxCount);
        timings->indexCount = static_cast<std::size_t>(drawData->TotalIdxCount);
        for (const ImDrawList* list : drawData->CmdLists)
            timings->drawCommandCount += static_cast<std::size_t>(list->CmdBuffer.Size);
    }
#ifndef __EMSCRIPTEN__
    PendingGpuQuery* gpuQuery = nullptr;
    if (timings != nullptr && timings->collectGpuTimings && GLAD_GL_VERSION_3_3)
    {
        for (PendingGpuQuery& query : backendGpuQueries_)
        {
            if (query.pending)
                continue;
            if (query.query == 0)
                glGenQueries(1, &query.query);
            gpuQuery = &query;
            glBeginQuery(GL_TIME_ELAPSED, gpuQuery->query);
            break;
        }
    }
#endif
    ImGui_ImplOpenGL3_RenderDrawData(drawData);
#ifndef __EMSCRIPTEN__
    if (gpuQuery != nullptr)
    {
        glEndQuery(GL_TIME_ELAPSED);
        gpuQuery->frameIndex = timings->frameIndex;
        gpuQuery->pending = true;
    }
#endif
    if (timings != nullptr)
    {
        timings->backendDrawMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - backendDrawStart).count();
    }
}
}
