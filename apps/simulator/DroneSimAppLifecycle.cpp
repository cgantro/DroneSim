#include "DroneSimApp.h"

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

namespace dronesim::simulator
{
namespace
{
// 화면 스모크는 프레임마다 한 Tick만 실행해 짧게 끝낸다.
constexpr std::size_t smokeTestFrameLimit = 8;
constexpr double benchmarkDurationSeconds = 5.0;
}

int DroneSimApp::Run()
{
#ifndef __EMSCRIPTEN__
    InitializeGraphics();
#endif
    startTime_ = std::chrono::steady_clock::now();
#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg([](void* application)
    {
        auto& app = *static_cast<DroneSimApp*>(application);
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
        if (benchmarkMode_ &&
            std::chrono::steady_clock::now() - startTime_ >= std::chrono::duration<double>(benchmarkDurationSeconds))
            break;
    }
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

void DroneSimApp::RunFrame(std::chrono::steady_clock::time_point& previousFrame)
{
#ifdef __EMSCRIPTEN__
    static const bool collectWebFrameTimings = EM_ASM_INT({
        return new URLSearchParams(window.location.search).get("frame-probe") === "1";
    }) != 0;
    const auto frameStart = collectWebFrameTimings
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
#endif
    const auto applicationFrameStart = benchmarkMode_
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    frameMeasurements_ = {};
    {
        dronesim::simulation::ScopedDuration measureEvents(
            benchmarkMode_ ? &frameMeasurements_.eventPollingMicroseconds : nullptr);
        window_->PollEvents();
    }
    {
        dronesim::simulation::ScopedDuration measureGamepad(
            benchmarkMode_ ? &frameMeasurements_.gamepadPollingMicroseconds : nullptr);
        PollGamepad();
    }
#ifdef __EMSCRIPTEN__
    const auto inputComplete = collectWebFrameTimings
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
#endif
    const auto now = std::chrono::steady_clock::now();
    const double elapsedSeconds = std::chrono::duration<double>(now - previousFrame).count();
    // 화면 스모크는 프레임마다 한 Tick, 네트워크 스모크는 프레임마다 최대 따라잡기만큼 진행한다.
    double frameSeconds = elapsedSeconds;
    if (smokeTest_ && !benchmarkMode_)
    {
        frameSeconds = dronesim::simulation::FixedTickAccumulator::fixedStepSeconds;
    }
    previousFrame = now;

    {
        dronesim::simulation::ScopedDuration measureSimulation(
            benchmarkMode_ ? &frameMeasurements_.simulationUpdateMicroseconds : nullptr);
        AdvanceSimulation(frameSeconds);
    }
#ifdef __EMSCRIPTEN__
    const auto simulationComplete = collectWebFrameTimings
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    auto renderComplete = std::chrono::steady_clock::time_point{};
#endif
    {
        dronesim::simulation::ScopedDuration measureRenderFrame(
            benchmarkMode_ ? &frameMeasurements_.renderFrameMicroseconds : nullptr);
        {
            dronesim::simulation::ScopedDuration measureRender(
                benchmarkMode_ ? &frameMeasurements_.renderMicroseconds : nullptr);
            RenderFrame();
        }
#ifdef __EMSCRIPTEN__
        renderComplete = collectWebFrameTimings
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
#endif
        {
            dronesim::simulation::ScopedDuration measureBufferSwap(
                benchmarkMode_ ? &frameMeasurements_.bufferSwapMicroseconds : nullptr);
            window_->SwapBuffers();
        }
    }
#ifdef __EMSCRIPTEN__
    if (collectWebFrameTimings)
    {
        const auto frameComplete = std::chrono::steady_clock::now();
        const auto elapsedMilliseconds = [](auto start, auto end)
        {
            return std::chrono::duration<double, std::milli>(end - start).count();
        };
        EM_ASM({
            const samples = window.__droneSimFrameTimings;
            if (Array.isArray(samples)) {
                samples.push({
                    input: $0,
                    simulation: $1,
                    render: $2,
                    bufferSwap: $3,
                    total: $4
                });
            }
        },
            elapsedMilliseconds(frameStart, inputComplete),
            elapsedMilliseconds(inputComplete, simulationComplete),
            elapsedMilliseconds(simulationComplete, renderComplete),
            elapsedMilliseconds(renderComplete, frameComplete),
            elapsedMilliseconds(frameStart, frameComplete));
    }
#endif
    ++renderedFrames_;
#ifdef __EMSCRIPTEN__
    const auto telemetryTime = std::chrono::steady_clock::now();
    if (telemetryTime - lastWebTelemetryUpdate_ >= std::chrono::milliseconds(250))
    {
        const auto& state = simulation_.State();
        const std::string mode{dronesim::simulation::ToString(state.mode)};
        const double speedMetersPerSecond = glm::length(state.velocityMetersPerSecond);
        const double heightAboveGroundMeters = state.positionMeters.z
            - terrainGrid_.HeightAtLocalMeters(
                {state.positionMeters.x, state.positionMeters.y});
        const bool cameraActive = simulation_.Camera().status
            == dronesim::simulation::CameraStatus::Active;
        const std::string missionStatus = CompleteFleetMissionStatus();
        const std::uint64_t simulationTick = simulation_.TickCount();
        EM_ASM({
            if (window.updateDroneSimStatus) {
                window.updateDroneSimStatus(
                    UTF8ToString($0), $1, $2, $3, $4, $5,
                    UTF8ToString($6), $7, $8, $9, $10);
            }
        }, mode.c_str(), state.positionMeters.x, state.positionMeters.y,
            heightAboveGroundMeters, speedMetersPerSecond, cameraActive,
            missionStatus.c_str(), simulationTick, CompleteFleetMissionIsActive(),
            CompleteFleetMissionCanStart(),
            state.mode == dronesim::simulation::FlightMode::Grounded);
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

bool DroneSimApp::SmokeTestIsComplete() const
{
    return renderedFrames_ >= smokeTestFrameLimit;
}

int DroneSimApp::ValidateSmokeTestOutput() const
{
    const auto& camera = simulation_.Camera();
    if (camera.status != dronesim::simulation::CameraStatus::Active
        || camera.lastFrameTick == 0)
        return 5;

    return 0;
}


}
