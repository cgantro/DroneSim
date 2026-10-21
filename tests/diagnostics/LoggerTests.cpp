#include "diagnostics/Logger.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>

TEST(Logger, FlushesRecordsAndRejectsWritesAfterShutdown)
{
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto path = std::filesystem::temp_directory_path() /
        ("grasplink-logger-" + std::to_string(stamp) + ".jsonl");
    {
        grasplink::diagnostics::Logger logger({path.string(), 4});
        logger.Write(grasplink::diagnostics::LogLevel::Error, "arm\"control", "line 1\nline 2");
        logger.RecordMetric("solver_residual", std::numeric_limits<double>::quiet_NaN(), "m");
        logger.Shutdown();
        logger.Write(grasplink::diagnostics::LogLevel::Error, "test", "after shutdown");

        EXPECT_EQ(logger.GetDroppedRecordCount(), 1U);
    }

    std::ifstream output(path);
    ASSERT_TRUE(output.is_open());
    const std::string contents{std::istreambuf_iterator<char>(output), std::istreambuf_iterator<char>()};
    EXPECT_EQ(std::count(contents.begin(), contents.end(), '\n'), 2);
    EXPECT_NE(contents.find("\"category\":\"arm\\\"control\""), std::string::npos);
    EXPECT_NE(contents.find("\"message\":\"line 1\\nline 2\""), std::string::npos);
    EXPECT_NE(contents.find("\"value\":null"), std::string::npos);

    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}
