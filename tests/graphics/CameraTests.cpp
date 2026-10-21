#include "graphics/Camera.h"

#include <gtest/gtest.h>

namespace aerosimlab::graphics
{
TEST(GraphicsCamera, UsesSuppliedUpDirectionForRoll)
{
    Camera camera{{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, 1.0F};
    const glm::vec4 worldUp{0.0F, 1.0F, 0.0F, 0.0F};

    const glm::vec4 neutralViewUp = camera.GetViewMatrix() * worldUp;
    EXPECT_NEAR(neutralViewUp.y, 1.0F, 1.0e-6F);

    camera.SetUp({0.0F, 0.0F, 1.0F});
    const glm::vec4 rolledViewUp = camera.GetViewMatrix() * worldUp;
    EXPECT_NEAR(rolledViewUp.x, -1.0F, 1.0e-6F);
    EXPECT_NEAR(rolledViewUp.y, 0.0F, 1.0e-6F);
}
}
