#include "FaultScenarioTimeline.h"

#include "simulation_core/QuadrotorModel.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace aerosimlab::simulator
{
namespace
{
using Json = nlohmann::json;

std::string_view EventName(FaultScenarioTimeline::Event::Kind kind)
{
    using Kind = FaultScenarioTimeline::Event::Kind;
    switch (kind)
    {
    case Kind::PacketLoss: return "packet_loss";
    case Kind::LinkLatencyJitter: return "link_latency_jitter";
    case Kind::CameraFault: return "camera_fault";
    case Kind::RotorMotorFault: return "rotor_motor_fault";
    case Kind::LinkState: return "link_state";
    case Kind::DirectedLinkState: return "directed_link_state";
    case Kind::RadioState: return "radio_state";
    }
    return {};
}

FaultScenarioTimeline::Event::Kind ReadEventKind(const Json& value)
{
    const std::string name = value.get<std::string>();
    if (name == "packet_loss") return FaultScenarioTimeline::Event::Kind::PacketLoss;
    if (name == "link_latency_jitter") return FaultScenarioTimeline::Event::Kind::LinkLatencyJitter;
    if (name == "camera_fault") return FaultScenarioTimeline::Event::Kind::CameraFault;
    if (name == "rotor_motor_fault") return FaultScenarioTimeline::Event::Kind::RotorMotorFault;
    if (name == "link_state") return FaultScenarioTimeline::Event::Kind::LinkState;
    if (name == "directed_link_state") return FaultScenarioTimeline::Event::Kind::DirectedLinkState;
    if (name == "radio_state") return FaultScenarioTimeline::Event::Kind::RadioState;
    throw std::invalid_argument("지원하지 않는 장애 종류입니다.");
}

std::uint64_t ReadDroneId(const Json& value)
{
    if (!value.is_number_unsigned() && !value.is_number_integer())
        throw std::invalid_argument("드론 ID는 0 이상의 정수여야 합니다.");
    if (value.is_number_integer() && value.get<std::int64_t>() < 0)
        throw std::invalid_argument("드론 ID는 음수일 수 없습니다.");
    return value.get<std::uint64_t>();
}

std::uint32_t ReadTickCount(const Json& value)
{
    if (!value.is_number_unsigned() && !value.is_number_integer())
        throw std::invalid_argument("지연 Tick 수는 0 이상의 정수여야 합니다.");

    const std::uint64_t ticks = value.get<std::uint64_t>();
    if (ticks > std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("지연 Tick 수가 허용 범위를 넘었습니다.");
    return static_cast<std::uint32_t>(ticks);
}

std::uint32_t ReadRotorIndex(const Json& value)
{
    if (value.is_number_unsigned())
    {
        const std::uint64_t index = value.get<std::uint64_t>();
        if (index < simulation::QuadrotorParameters::rotorCount)
            return static_cast<std::uint32_t>(index);
    }
    else if (value.is_number_integer())
    {
        const std::int64_t index = value.get<std::int64_t>();
        if (index >= 0
            && static_cast<std::uint64_t>(index)
                < simulation::QuadrotorParameters::rotorCount)
            return static_cast<std::uint32_t>(index);
    }
    throw std::invalid_argument("로터 번호는 0부터 3 사이의 정수여야 합니다.");
}
}

void FaultScenarioTimeline::StartRecording(std::uint64_t currentTick,
    const std::vector<simulation::DroneId>& droneIds, std::string missionKind)
{
    events_.clear();
    droneIds_ = droneIds;
    missionKind_ = std::move(missionKind);
    recordingStartTick_ = currentTick;
    replayEventIndex_ = 0;
    recording_ = true;
    replaying_ = false;
}

void FaultScenarioTimeline::StopRecording() noexcept
{
    recording_ = false;
}

bool FaultScenarioTimeline::SaveRecording(const std::filesystem::path& path,
    std::string& error) const
{
    if (recording_ || events_.empty())
    {
        error = "기록을 끝내고 장애 조작을 한 번 이상 저장해야 합니다.";
        return false;
    }

    Json document{
        {"version", 1},
        {"mission", missionKind_},
        {"drones", droneIds_},
        {"events", Json::array()}};
    for (const Event& event : events_)
    {
        Json encoded{
            {"tick", event.tickOffset},
            {"type", EventName(event.kind)}};
        switch (event.kind)
        {
        case Event::Kind::PacketLoss:
            encoded["probability"] = event.value;
            break;
        case Event::Kind::LinkLatencyJitter:
            encoded["max_extra_ticks"] = event.valueTicks;
            break;
        case Event::Kind::CameraFault:
            encoded["drone"] = event.source;
            encoded["faulted"] = event.enabled;
            break;
        case Event::Kind::RotorMotorFault:
            encoded["drone"] = event.source;
            encoded["rotor_index"] = event.rotorIndex;
            encoded["faulted"] = event.enabled;
            break;
        case Event::Kind::LinkState:
        case Event::Kind::DirectedLinkState:
            encoded["source"] = event.source;
            encoded["destination"] = event.destination;
            encoded["disabled"] = event.enabled;
            break;
        case Event::Kind::RadioState:
            encoded["drone"] = event.source;
            encoded["enabled"] = event.enabled;
            break;
        }
        document["events"].push_back(std::move(encoded));
    }

    try
    {
        std::filesystem::create_directories(path.parent_path());
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        if (!output)
        {
            error = "장애 시나리오 파일을 열지 못했습니다.";
            return false;
        }
        output << document.dump(2) << '\n';
        if (!output)
        {
            error = "장애 시나리오 파일을 저장하지 못했습니다.";
            return false;
        }
    }
    catch (const std::exception& exception)
    {
        error = exception.what();
        return false;
    }
    error.clear();
    return true;
}

bool FaultScenarioTimeline::LoadRecording(const std::filesystem::path& path,
    std::string& error)
{
    try
    {
        std::ifstream input(path, std::ios::binary);
        if (!input)
        {
            error = "저장된 장애 시나리오를 찾지 못했습니다.";
            return false;
        }

        const Json document = Json::parse(input);
        if (!document.is_object() || document.at("version") != 1
            || !document.at("mission").is_string()
            || document.at("mission").get<std::string>().empty()
            || !document.at("drones").is_array()
            || !document.at("events").is_array())
        {
            throw std::invalid_argument("장애 시나리오 형식이 올바르지 않습니다.");
        }

        std::vector<simulation::DroneId> droneIds;
        std::unordered_set<simulation::DroneId> uniqueDroneIds;
        for (const Json& id : document.at("drones"))
        {
            const simulation::DroneId droneId = ReadDroneId(id);
            if (!uniqueDroneIds.insert(droneId).second)
                throw std::invalid_argument("드론 ID가 중복됐습니다.");
            droneIds.push_back(droneId);
        }
        if (droneIds.empty())
            throw std::invalid_argument("드론 목록이 비어 있거나 ID가 중복됐습니다.");

        std::vector<Event> events;
        std::uint64_t previousTick = 0;
        bool firstEvent = true;
        for (const Json& encoded : document.at("events"))
        {
            Event event;
            event.tickOffset = encoded.at("tick").get<std::uint64_t>();
            event.kind = ReadEventKind(encoded.at("type"));
            if (!firstEvent && event.tickOffset < previousTick)
                throw std::invalid_argument("장애 시각은 오름차순이어야 합니다.");
            firstEvent = false;
            previousTick = event.tickOffset;

            switch (event.kind)
            {
            case Event::Kind::PacketLoss:
                event.value = encoded.at("probability").get<double>();
                if (!std::isfinite(event.value) || event.value < 0.0 || event.value > 1.0)
                    throw std::invalid_argument("패킷 손실률은 0부터 1 사이여야 합니다.");
                break;
            case Event::Kind::LinkLatencyJitter:
                event.valueTicks = ReadTickCount(encoded.at("max_extra_ticks"));
                break;
            case Event::Kind::CameraFault:
                event.source = ReadDroneId(encoded.at("drone"));
                event.enabled = encoded.at("faulted").get<bool>();
                if (!uniqueDroneIds.contains(event.source))
                    throw std::invalid_argument("카메라 장애 대상이 드론 목록에 없습니다.");
                break;
            case Event::Kind::RotorMotorFault:
                event.source = ReadDroneId(encoded.at("drone"));
                event.rotorIndex = ReadRotorIndex(encoded.at("rotor_index"));
                event.enabled = encoded.at("faulted").get<bool>();
                if (!uniqueDroneIds.contains(event.source))
                    throw std::invalid_argument("로터 장애 대상이나 번호가 올바르지 않습니다.");
                break;
            case Event::Kind::LinkState:
            case Event::Kind::DirectedLinkState:
                event.source = ReadDroneId(encoded.at("source"));
                event.destination = ReadDroneId(encoded.at("destination"));
                event.enabled = encoded.at("disabled").get<bool>();
                if (event.source == event.destination
                    || !uniqueDroneIds.contains(event.source)
                    || !uniqueDroneIds.contains(event.destination))
                    throw std::invalid_argument("링크 장애에는 서로 다른 두 드론이 필요합니다.");
                break;
            case Event::Kind::RadioState:
                event.source = ReadDroneId(encoded.at("drone"));
                event.enabled = encoded.at("enabled").get<bool>();
                if (!uniqueDroneIds.contains(event.source))
                    throw std::invalid_argument("라디오 장애 대상이 드론 목록에 없습니다.");
                break;
            }
            events.push_back(event);
        }
        if (events.empty())
            throw std::invalid_argument("저장된 장애 이벤트가 없습니다.");

        droneIds_ = std::move(droneIds);
        missionKind_ = document.at("mission").get<std::string>();
        events_ = std::move(events);
        recording_ = false;
        replaying_ = false;
        replayEventIndex_ = 0;
        error.clear();
        return true;
    }
    catch (const std::exception& exception)
    {
        error = exception.what();
        return false;
    }
}

bool FaultScenarioTimeline::StartReplay(std::uint64_t currentTick,
    const std::vector<simulation::DroneId>& droneIds,
    std::string_view missionKind, std::string& error)
{
    if (recording_ || events_.empty() || droneIds != droneIds_
        || missionKind != missionKind_)
    {
        error = "현재 임무·드론 구성과 기록한 내용이 다르거나 재생할 기록이 없습니다.";
        return false;
    }
    replayStartTick_ = currentTick;
    replayEventIndex_ = 0;
    replaying_ = true;
    error.clear();
    return true;
}

void FaultScenarioTimeline::StopReplay() noexcept
{
    replaying_ = false;
}

void FaultScenarioTimeline::RecordPacketLoss(std::uint64_t currentTick,
    double probability)
{
    Event event;
    event.kind = Event::Kind::PacketLoss;
    event.value = probability;
    Record(event, currentTick);
}

void FaultScenarioTimeline::RecordLinkLatencyJitter(
    std::uint64_t currentTick, std::uint32_t maximumExtraTicks)
{
    Event event;
    event.kind = Event::Kind::LinkLatencyJitter;
    event.valueTicks = maximumExtraTicks;
    Record(event, currentTick);
}

void FaultScenarioTimeline::RecordCameraFault(std::uint64_t currentTick,
    simulation::DroneId droneId, bool faulted)
{
    Event event;
    event.kind = Event::Kind::CameraFault;
    event.source = droneId;
    event.enabled = faulted;
    Record(event, currentTick);
}

void FaultScenarioTimeline::RecordRotorMotorFault(std::uint64_t currentTick,
    simulation::DroneId droneId, std::uint32_t rotorIndex, bool faulted)
{
    Event event;
    event.kind = Event::Kind::RotorMotorFault;
    event.source = droneId;
    event.rotorIndex = rotorIndex;
    event.enabled = faulted;
    Record(event, currentTick);
}

void FaultScenarioTimeline::RecordLinkState(std::uint64_t currentTick,
    simulation::DroneId source, simulation::DroneId destination, bool disabled)
{
    Event event;
    event.kind = Event::Kind::LinkState;
    event.source = source;
    event.destination = destination;
    event.enabled = disabled;
    Record(event, currentTick);
}

void FaultScenarioTimeline::RecordDirectedLinkState(std::uint64_t currentTick,
    simulation::DroneId source, simulation::DroneId destination, bool disabled)
{
    Event event;
    event.kind = Event::Kind::DirectedLinkState;
    event.source = source;
    event.destination = destination;
    event.enabled = disabled;
    Record(event, currentTick);
}

void FaultScenarioTimeline::RecordRadioState(std::uint64_t currentTick,
    simulation::DroneId droneId, bool enabled)
{
    Event event;
    event.kind = Event::Kind::RadioState;
    event.source = droneId;
    event.enabled = enabled;
    Record(event, currentTick);
}

std::vector<FaultScenarioTimeline::Event> FaultScenarioTimeline::TakeDueEvents(
    std::uint64_t currentTick)
{
    std::vector<Event> due;
    if (!replaying_ || currentTick < replayStartTick_)
        return due;

    const std::uint64_t elapsedTicks = currentTick - replayStartTick_;
    while (replayEventIndex_ < events_.size()
        && events_[replayEventIndex_].tickOffset <= elapsedTicks)
    {
        due.push_back(events_[replayEventIndex_]);
        ++replayEventIndex_;
    }
    return due;
}

bool FaultScenarioTimeline::IsRecording() const noexcept { return recording_; }
bool FaultScenarioTimeline::IsReplaying() const noexcept { return replaying_; }
bool FaultScenarioTimeline::ReplayFinished() const noexcept
{
    return replaying_ && replayEventIndex_ >= events_.size();
}
std::size_t FaultScenarioTimeline::EventCount() const noexcept { return events_.size(); }

void FaultScenarioTimeline::Record(Event event, std::uint64_t currentTick)
{
    if (!recording_ || currentTick < recordingStartTick_)
        return;
    event.tickOffset = currentTick - recordingStartTick_;
    events_.push_back(event);
}
}
