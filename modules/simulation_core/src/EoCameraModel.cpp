#include "simulation_core/EoCameraModel.h"

namespace aerosimlab::simulation
{
void EoCameraModel::AdvanceOneTick(std::uint64_t tick) noexcept
{
    if (state_.status != CameraStatus::Active)
        return;

    ++state_.frameCount;
    state_.lastFrameTick = tick;
}

void EoCameraModel::SetEnabled(bool enabled) noexcept
{
    if (state_.status == CameraStatus::Fault)
        return;

    state_.status = enabled ? CameraStatus::Active : CameraStatus::Disabled;
}

void EoCameraModel::InjectFault() noexcept
{
    state_.status = CameraStatus::Fault;
}

void EoCameraModel::Recover() noexcept
{
    state_.status = CameraStatus::Active;
}

void EoCameraModel::Reset() noexcept
{
    state_ = {};
}

const CameraState& EoCameraModel::State() const noexcept
{
    return state_;
}

std::string_view ToString(CameraStatus status) noexcept
{
    switch (status)
    {
    case CameraStatus::Active: return "active";
    case CameraStatus::Disabled: return "disabled";
    case CameraStatus::Fault: return "fault";
    }
    return "unknown";
}
}
