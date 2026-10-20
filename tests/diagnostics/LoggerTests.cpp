#include "diagnostics/Logger.h"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>

TEST(Logger, CountsRecordsSubmittedAfterShutdown)
{
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto path = std::filesystem::temp_directory_path() /
        ("grasplink-logger-" + std::to_string(stamp) + ".jsonl");
    grasplink::diagnostics::Logger logger({path.string(), 1});

    logger.Shutdown();
    logger.Write(grasplink::diagnostics::LogLevel::Error, "test", "after shutdown");

    EXPECT_EQ(logger.GetDroppedRecordCount(), 1U);
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}
