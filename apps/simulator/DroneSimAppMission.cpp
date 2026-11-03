#include "DroneSimApp.h"

#include "graphics/Window.h"

#ifdef __EMSCRIPTEN__
#include "WebCameraRig.h"
#endif

#include <string>

namespace dronesim::simulator
{
bool DroneSimApp::StartCompleteFleetMission()
{
    if (!missionController_->StartCompleteFleetMission())
        return false;

    pendingControllerAction_.reset();
    pendingPilotCommand_.reset();
#ifdef __EMSCRIPTEN__
    webPilotAxes_.reset();
#endif
    TakeMissionControl("DroneSim | 편대 이동 임무 진행 중");
    return true;
}

bool DroneSimApp::StartFleetReconnaissance()
{
    if (!missionController_->StartFleetReconnaissance())
        return false;

    TakeMissionControl("DroneSim | 편대 정찰 임무 진행 중");
    return true;
}

bool DroneSimApp::StartFleetReturn()
{
    if (!missionController_->StartFleetReturn())
        return false;

    TakeMissionControl("DroneSim | 출발 지점으로 복귀 중");
    return true;
}

void DroneSimApp::TakeMissionControl(std::string_view windowTitle)
{
    externalControl_ = true;
    missionControlOwnsInput_ = true;
    if (window_)
        window_->SetTitle(std::string{windowTitle});
}
}
