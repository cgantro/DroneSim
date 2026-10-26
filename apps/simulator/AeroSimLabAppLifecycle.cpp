#include "AeroSimLabApp.h"

#include "graphics/Window.h"
#include "simulation_core/ScopedDuration.h"

#include <glm/geometric.hpp>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iostream>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace aerosimlab::simulator
{
namespace
{
// 화면 스모크는 프레임마다 한 Tick만 실행해 짧게 끝낸다.
constexpr std::size_t smokeTestFrameLimit = 8;
// 네트워크 스모크는 느린 화면에 모의 진행이 묶이지 않도록 따라잡기 Tick을 실행한다.
constexpr std::size_t networkSmokeSimulationBatchesPerFrame = 5;
constexpr double benchmarkDurationSeconds = 5.0;
}

int AeroSimLabApp::Run()
{
#ifndef __EMSCRIPTEN__
    if (networkEnabled_)
    {
        udpSocket_ = std::make_unique<aerosimlab::communication::UdpSocket>(udpPort_);
    }
#endif
#ifndef __EMSCRIPTEN__
    // 그래픽 자산을 불러오는 동안 도착한 UDP 요청도 소켓 수신 큐에 남긴다.
    InitializeGraphics();
#endif
    startTime_ = std::chrono::steady_clock::now();
#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg([](void* application)
    {
        auto& app = *static_cast<AeroSimLabApp*>(application);
        try
        {
            if (!app.webGraphicsInitialized_)
            {
                app.webGraphicsInitialized_ = app.InitializeGraphicsStep();
                if (app.webGraphicsInitialized_)
                {
                    app.startTime_ = std::chrono::steady_clock::now();
                    app.webPreviousFrame_ = app.startTime_;
                    app.lastWebTelemetryUpdate_ = app.startTime_;
                }
                return;
            }

            if (app.window_->ShouldClose())
            {
                emscripten_cancel_main_loop();
                return;
            }
            app.RunFrame(app.webPreviousFrame_);
        }
        catch (const std::exception& error)
        {
            const std::string message = error.what();
            EM_ASM({
                window.showSimulatorStartupError(
                    "시뮬레이터를 시작하지 못했습니다. " + UTF8ToString($0));
            }, message.c_str());
            emscripten_cancel_main_loop();
        }
        catch (...)
        {
            EM_ASM({
                window.showSimulatorStartupError("시뮬레이터를 시작하지 못했습니다.");
            });
            emscripten_cancel_main_loop();
        }
    }, this, 0, false);
    return 0;
#else
    benchmarkCpuStart_ = std::clock();
    auto previousFrame = startTime_;

    if (benchmarkMode_)
    {
        while (!window_->ShouldClose()
            && std::chrono::steady_clock::now() - startTime_
                < std::chrono::duration<double>(benchmarkWarmupDurationSeconds))
        {
            RunFrame(previousFrame);
        }

        // 초기 셰이더·버퍼 생성 지연은 측정 구간에서 빼고, 워밍업 뒤 기준값부터 기록한다.
        benchmarkMeasurements_ = {};
        benchmarkStartRenderedFrames_ = renderedFrames_;
        benchmarkStartSimulationTicks_ = simulation_.TickCount();
        benchmarkCpuStart_ = std::clock();
        startTime_ = std::chrono::steady_clock::now();
        previousFrame = startTime_;
    }

    while (!window_->ShouldClose())
    {
        RunFrame(previousFrame);

        if (smokeTest_ && SmokeTestIsComplete())
            break;
        if (smokeTest_ && networkEnabled_
            && std::chrono::steady_clock::now() - startTime_ > std::chrono::seconds(30))
            return 4;
        if (benchmarkMode_ &&
            std::chrono::steady_clock::now() - startTime_ >= std::chrono::duration<double>(benchmarkDurationSeconds))
            break;
    }
#ifndef __EMSCRIPTEN__
    if (smokeTest_)
        DrainEoReadbacks();
#endif
    if (smokeTest_)
    {
        const int smokeTestResult = ValidateSmokeTestOutput();
        if (smokeTestResult != 0)
            return smokeTestResult;
    }
    if (benchmarkMode_)
    {
        const double elapsedSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime_).count();
        WriteBenchmarkReport(elapsedSeconds);
    }
    return 0;
#endif
}

void AeroSimLabApp::RunFrame(std::chrono::steady_clock::time_point& previousFrame)
{
    const auto applicationFrameStart = benchmarkMode_
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    frameMeasurements_ = {};
    {
        aerosimlab::simulation::ScopedDuration measureEvents(
            benchmarkMode_ ? &frameMeasurements_.eventPollingMicroseconds : nullptr);
        window_->PollEvents();
    }
    {
        aerosimlab::simulation::ScopedDuration measureGroundControl(
            benchmarkMode_ ? &frameMeasurements_.groundControlPollingMicroseconds : nullptr);
        PollGroundControl();
    }
    {
        aerosimlab::simulation::ScopedDuration measureGamepad(
            benchmarkMode_ ? &frameMeasurements_.gamepadPollingMicroseconds : nullptr);
        PollGamepad();
    }
    const auto now = std::chrono::steady_clock::now();
    const double elapsedSeconds = std::chrono::duration<double>(now - previousFrame).count();
    // 화면 스모크는 프레임마다 한 Tick, 네트워크 스모크는 프레임마다 최대 따라잡기만큼 진행한다.
    double frameSeconds = elapsedSeconds;
    if (smokeTest_ && !benchmarkMode_)
    {
        frameSeconds = networkEnabled_
            ? aerosimlab::simulation::FixedTickAccumulator::maximumFrameSeconds
            : aerosimlab::simulation::FixedTickAccumulator::fixedStepSeconds;
    }
    previousFrame = now;

    const std::size_t simulationBatches = smokeTest_ && networkEnabled_
        && !benchmarkMode_
        ? networkSmokeSimulationBatchesPerFrame
        : 1U;
    {
        aerosimlab::simulation::ScopedDuration measureSimulation(
            benchmarkMode_ ? &frameMeasurements_.simulationUpdateMicroseconds : nullptr);
        for (std::size_t batch = 0; batch < simulationBatches; ++batch)
            AdvanceSimulation(frameSeconds);
    }
    eoReadbackHappenedThisFrame_ = false;
    {
        aerosimlab::simulation::ScopedDuration measureRenderFrame(
            benchmarkMode_ ? &frameMeasurements_.renderFrameMicroseconds : nullptr);
        {
            aerosimlab::simulation::ScopedDuration measureRender(
                benchmarkMode_ ? &frameMeasurements_.renderMicroseconds : nullptr);
            RenderFrame();
        }
        {
            aerosimlab::simulation::ScopedDuration measureBufferSwap(
                benchmarkMode_ ? &frameMeasurements_.bufferSwapMicroseconds : nullptr);
            window_->SwapBuffers();
        }
    }
    ++renderedFrames_;
#ifdef __EMSCRIPTEN__
    const auto telemetryTime = std::chrono::steady_clock::now();
    if (telemetryTime - lastWebTelemetryUpdate_ >= std::chrono::milliseconds(250))
    {
        const auto& state = simulation_.State();
        const std::string mode{aerosimlab::simulation::ToString(state.mode)};
        const double speedMetersPerSecond = glm::length(state.velocityMetersPerSecond);
        const bool cameraActive = simulation_.Camera().status
            == aerosimlab::simulation::CameraStatus::Active;
        const auto& camera = simulation_.Camera();
        const std::string captureAssessment{
            aerosimlab::simulation::ToString(AssessInspectionView())};
        const std::uint64_t simulationTick = simulation_.TickCount();
        EM_ASM({
            if (window.updateDroneSimGroundControl) {
                window.updateDroneSimGroundControl(
                    UTF8ToString($0), $1, $2, $3, $4, $5, UTF8ToString($6),
                    UTF8ToString($7), $8);
            }
        }, mode.c_str(), state.positionMeters.x, state.positionMeters.y,
            state.positionMeters.z, speedMetersPerSecond, cameraActive,
            captureAssessment.c_str(), InspectionMissionStatus().c_str(), simulationTick);
        EM_ASM({
            if (window.updateDroneSimInspectionMetadata) {
                window.updateDroneSimInspectionMetadata(
                    $0, $1, $2, $3, $4, $5, $6, UTF8ToString($7), UTF8ToString($8));
            }
        },
            camera.positionMeters.x, camera.positionMeters.y, camera.positionMeters.z,
            camera.orientation.x, camera.orientation.y, camera.orientation.z,
            camera.orientation.w, inspectionMission_.facilityId.c_str(),
            inspectionMission_.zoneId.c_str());
        lastWebTelemetryUpdate_ = telemetryTime;
    }
    if (!webFirstFrameRendered_)
    {
        webFirstFrameRendered_ = true;
        EM_ASM({ window.completeSimulatorStartup(); });
    }
#endif
    if (benchmarkMode_)
    {
        const double applicationMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - applicationFrameStart).count();
        RecordBenchmarkFrame(applicationMicroseconds);
    }
}

bool AeroSimLabApp::SmokeTestIsComplete() const
{
    if (!networkEnabled_)
        return renderedFrames_ >= smokeTestFrameLimit;

    return smokeReplayStarted_
        && !commandRecording_.IsRecording()
        && !commandRecording_.IsReplaying()
        && simulation_.State().mode == aerosimlab::simulation::FlightMode::Grounded;
}

int AeroSimLabApp::ValidateSmokeTestOutput() const
{
    const bool cameraFrameHasPixels = eoFrame_.IsValid()
        && std::any_of(eoFrame_.pixels.begin(), eoFrame_.pixels.end(), [](std::uint8_t value)
        {
            return value != 0;
        });
    if (!cameraFrameHasPixels)
        return 5;

    const auto& camera = simulation_.Camera();
    if (camera.status == aerosimlab::simulation::CameraStatus::Active
        && eoFrame_.simulationTick != camera.lastFrameTick)
    {
        std::cerr << "EO 영상 Tick이 카메라의 마지막 Tick과 다릅니다. 영상="
                  << eoFrame_.simulationTick << ", 카메라="
                  << camera.lastFrameTick << '\n';
        return 6;
    }

    return 0;
}


}
