#include "diagnostics/Logger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <locale>
#include <stdexcept>
#include <utility>

namespace grasplink::diagnostics
{
namespace
{
const char* LevelName(LogLevel level)
{
    switch (level)
    {
    case LogLevel::Debug: return "debug";
    case LogLevel::Info: return "info";
    case LogLevel::Warning: return "warning";
    case LogLevel::Error: return "error";
    }
    return "unknown";
}
}

Logger::Logger(LoggerOptions options)
    : m_Options(std::move(options))
{
    if (m_Options.queueCapacity == 0)
        m_Options.queueCapacity = 1;

#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
    m_Worker = std::thread(&Logger::Consume, this);
#else
    OpenOutputFile();
#endif
}

Logger::~Logger()
{
    Shutdown();
}

void Logger::Write(LogLevel level, const std::string& category, const std::string& message) noexcept
{
    try
    {
        Enqueue({RecordKind::Log, level, 0, 0, category, message});
    }
    catch (...)
    {
        m_DroppedRecords.fetch_add(1, std::memory_order_relaxed);
    }
}

void Logger::RecordMetric(const std::string& name, double value, const std::string& unit) noexcept
{
    try
    {
        Enqueue({RecordKind::Metric, LogLevel::Info, 0, 0, name, {}, unit, value});
    }
    catch (...)
    {
        m_DroppedRecords.fetch_add(1, std::memory_order_relaxed);
    }
}

void Logger::Flush() noexcept
{
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
    try
    {
        std::unique_lock<std::mutex> lock(m_Mutex);
        m_Drained.wait(lock, [this] { return m_Queue.empty() && !m_Writing; });
    }
    catch (...)
    {
    }
#else
    try
    {
        if (m_Output.is_open())
            m_Output.flush();
    }
    catch (...)
    {
    }
#endif
}

void Logger::Shutdown() noexcept
{
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
    std::lock_guard<std::mutex> lifecycleLock(m_LifecycleMutex);
    {
        std::lock_guard<std::mutex> lock(m_Mutex);
        m_Accepting = false;
    }
    m_QueueChanged.notify_one();
    if (m_Worker.joinable())
        m_Worker.join();
#else
    Flush();
    if (m_Output.is_open())
        m_Output.close();
#endif
}

std::uint64_t Logger::GetDroppedRecordCount() const noexcept
{
    return m_DroppedRecords.load(std::memory_order_relaxed);
}

std::uint64_t Logger::GetWriteFailureCount() const noexcept
{
    return m_WriteFailures.load(std::memory_order_relaxed);
}

void Logger::Enqueue(Record record) noexcept
{
    try
    {
        const auto now = std::chrono::system_clock::now();
        record.timestampMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
        record.threadId = static_cast<std::uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
#else
        record.threadId = 0;
#endif

#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
        {
            std::lock_guard<std::mutex> lock(m_Mutex);
            if (!m_Accepting)
            {
                m_DroppedRecords.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            if (m_Queue.size() >= m_Options.queueCapacity)
            {
                const auto expendable = std::find_if(m_Queue.begin(), m_Queue.end(), [](const Record& queued)
                {
                    return queued.kind == RecordKind::Metric || queued.level == LogLevel::Debug || queued.level == LogLevel::Info;
                });
                if (record.kind != RecordKind::Log || record.level != LogLevel::Error || expendable == m_Queue.end())
                {
                    m_DroppedRecords.fetch_add(1, std::memory_order_relaxed);
                    return;
                }
                m_Queue.erase(expendable);
                m_DroppedRecords.fetch_add(1, std::memory_order_relaxed);
            }
            m_Queue.push_back(std::move(record));
        }
        m_QueueChanged.notify_one();
#else
        WriteRecord(record);
        if (m_Output.is_open())
        {
            m_Output.flush();
            if (!m_Output)
            {
                m_WriteFailures.fetch_add(1, std::memory_order_relaxed);
                m_Output.close();
            }
        }
#endif
    }
    catch (...)
    {
        m_DroppedRecords.fetch_add(1, std::memory_order_relaxed);
    }
}

void Logger::OpenOutputFile()
{
    try
    {
        const std::filesystem::path outputPath(m_Options.filePath);
        if (outputPath.has_parent_path())
            std::filesystem::create_directories(outputPath.parent_path());
        m_Output.open(outputPath, std::ios::out | std::ios::app);
    }
    catch (...)
    {
    }

    if (!m_Output.is_open())
    {
        m_WriteFailures.fetch_add(1, std::memory_order_relaxed);
        std::cerr << "Logger could not open output file: " << m_Options.filePath << '\n';
    }
    m_Output.imbue(std::locale::classic());
}

#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
void Logger::Consume()
{
    OpenOutputFile();

    for (;;)
    {
        Record record;
        {
            std::unique_lock<std::mutex> lock(m_Mutex);
            m_QueueChanged.wait(lock, [this] { return !m_Queue.empty() || !m_Accepting; });
            if (m_Queue.empty() && !m_Accepting)
                break;
            record = std::move(m_Queue.front());
            m_Queue.pop_front();
            m_Writing = true;
        }

        try
        {
            WriteRecord(record);
        }
        catch (const std::exception& error)
        {
            m_WriteFailures.fetch_add(1, std::memory_order_relaxed);
            m_Output.close();
            std::cerr << "Logger failed to write a record: " << error.what() << '\n';
        }
        catch (...)
        {
            m_WriteFailures.fetch_add(1, std::memory_order_relaxed);
            m_Output.close();
            std::cerr << "Logger failed to write a record.\n";
        }

        bool queueEmpty = false;
        {
            std::lock_guard<std::mutex> lock(m_Mutex);
            queueEmpty = m_Queue.empty();
        }
        if (queueEmpty && m_Output.is_open())
        {
            m_Output.flush();
            if (!m_Output)
            {
                m_WriteFailures.fetch_add(1, std::memory_order_relaxed);
                m_Output.close();
                std::cerr << "Logger failed to flush its output file.\n";
            }
        }
        {
            std::lock_guard<std::mutex> lock(m_Mutex);
            m_Writing = false;
            if (m_Queue.empty())
                m_Drained.notify_all();
        }
    }

    m_Drained.notify_all();
}
#endif

void Logger::WriteRecord(const Record& record)
{
    if (!m_Output.is_open())
        return;

    m_Output << "{\"timestamp_ms\":" << record.timestampMilliseconds
        << ",\"thread_id\":" << record.threadId;

    switch (record.kind)
    {
    case RecordKind::Log:
        m_Output << ",\"type\":\"log\",\"level\":\"" << LevelName(record.level)
            << "\",\"category\":\"";
        WriteEscapedJson(m_Output, record.name);
        m_Output << "\",\"message\":\"";
        WriteEscapedJson(m_Output, record.message);
        m_Output.put('\"');
        break;
    case RecordKind::Metric:
        m_Output << ",\"type\":\"metric\",\"name\":\"";
        WriteEscapedJson(m_Output, record.name);
        m_Output << "\",\"value\":";
        if (std::isfinite(record.value))
            m_Output << std::setprecision(17) << record.value;
        else
            m_Output << "null";
        m_Output << ",\"unit\":\"";
        WriteEscapedJson(m_Output, record.unit);
        m_Output.put('\"');
        break;
    }
    m_Output << "}\n";
    if (!m_Output)
        throw std::runtime_error("output stream rejected a record");
}

void Logger::WriteEscapedJson(std::ostream& output, const std::string& value)
{
    for (const unsigned char character : value)
    {
        switch (character)
        {
        case '"': output.write("\\\"", 2); break;
        case '\\': output.write("\\\\", 2); break;
        case '\b': output.write("\\b", 2); break;
        case '\f': output.write("\\f", 2); break;
        case '\n': output.write("\\n", 2); break;
        case '\r': output.write("\\r", 2); break;
        case '\t': output.write("\\t", 2); break;
        default:
            if (character < 0x20)
            {
                constexpr char hexDigits[] = "0123456789abcdef";
                const char escaped[] = {'\\', 'u', '0', '0', hexDigits[character >> 4], hexDigits[character & 0x0f]};
                output.write(escaped, sizeof(escaped));
            }
            else
                output.put(static_cast<char>(character));
        }
    }
}
} // namespace grasplink::diagnostics
