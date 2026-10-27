#include "BuildRevision.h"

#include "AeroSimLabBuildRevision.h"

namespace aerosimlab::simulator
{
std::string_view BuildRevision() noexcept
{
    return aerosimlab::build_info::sourceRevision;
}
}
