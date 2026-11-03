#include "DroneSimApp.h"
#include "graphics/Window.h"

namespace dronesim::simulator
{
void DroneSimApp::PollGamepad()
{
    const auto update = gamepadController_.Poll(missionControlOwnsInput_,
        simulation_.State().mode, simulation_.State().yawRadians);
    switch (update.notice)
    {
    case dronesim::manual_control::GamepadNotice::Connected:
        window_->SetTitle("DroneSim | 게임패드 연결됨");
        break;
    case dronesim::manual_control::GamepadNotice::UnsupportedDevice:
        window_->SetTitle("DroneSim | 게임패드 형식 미지원: " + update.deviceName);
        break;
    case dronesim::manual_control::GamepadNotice::UnsupportedDeviceRemoved:
        window_->SetTitle("DroneSim | 드론 관찰 · EO 영상");
        break;
    case dronesim::manual_control::GamepadNotice::None:
        break;
    }
    if (update.connectionChanged && !update.connected)
        pendingControllerAction_.reset();
    if (update.connectionChanged && !update.connected && !missionControlOwnsInput_)
        pendingPilotCommand_.reset();
    if (update.externalInputReceived)
        externalControl_ = true;
    if (update.flightAction)
        pendingControllerAction_ = *update.flightAction;
    if (update.pilotCommand)
        pendingPilotCommand_ = *update.pilotCommand;
}
}
