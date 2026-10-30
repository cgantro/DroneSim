#include "BuildRevision.h"

#include "DroneSimBuildRevision.h"

namespace dronesim::simulator
{
/** @brief 빌드 때 생성된 헤더에 기록한 소스 리비전을 공개한다. */
std::string_view BuildRevision() noexcept
{
    return dronesim::build_info::sourceRevision;
}
}
