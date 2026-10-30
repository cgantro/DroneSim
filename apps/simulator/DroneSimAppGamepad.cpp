#include "DroneSimApp.h"
#include "graphics/Window.h"

namespace dronesim::simulator
{
void DroneSimApp::PollGamepad()
{
    const auto update = gamepadController_.Poll(missionControlOwnsInput_,
        simulation_.State().mode, simulation_.State().yawRadians);
    if (update.windowTitle)
        window_->SetTitle(*update.windowTitle);
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
