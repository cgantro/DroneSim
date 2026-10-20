#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#endif

namespace grasplink::diagnostics
{
/**
 * @brief 로그 메시지의 중요도를 나타낸다.
 * @details 큐가 가득 찼을 때 Error 로그는 Metric, Debug, Info 로그를 밀어내고 보존될 수 있다.
 */
enum class LogLevel
{
    Debug,
    Info,
    Warning,
    Error
};

/**
 * @brief Logger의 출력 파일과 대기열 크기를 설정한다.
 * @details 상대 경로는 프로세스의 현재 작업 디렉터리를 기준으로 한다. 대기열 용량은 생성 시 1 이상으로 보정한다.
 */
struct LoggerOptions
{
    std::string filePath = "logs/grasplink.jsonl";
    std::size_t queueCapacity = 8192;
};

/**
 * @brief 로그와 semantic metric을 별도 작업 스레드에서 JSON Lines 파일로 기록한다.
 * @details 호출 스레드는 레코드를 제한된 대기열에 넣고 바로 돌아온다. 대기열이 차면 낮은 우선순위 레코드를 버리며,
 * Shutdown은 이미 받은 레코드를 모두 기록한 뒤 작업 스레드가 끝날 때까지 기다린다.
 */
class Logger final
{
public:
    explicit Logger(LoggerOptions options = {});
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /**
     * @brief 분류와 메시지를 로그 대기열에 추가한다.
     * @param level 메시지의 중요도다.
     * @param category 로그를 발생시킨 시스템이나 기능의 이름이다.
     * @param message 파일에 기록할 메시지다.
     */
    void Write(LogLevel level, const std::string& category, const std::string& message) noexcept;

    /**
     * @brief 이름과 단위를 가진 수치 metric을 대기열에 추가한다.
     * @param name 측정값의 이름이다.
     * @param value 기록할 수치다. NaN과 Infinity는 JSON에서 null로 저장한다.
     * @param unit 값에 적용되는 단위 문자열이다.
     */
    void RecordMetric(const std::string& name, double value, const std::string& unit) noexcept;

    /** @brief 대기 중인 레코드가 모두 파일에 반영될 때까지 기다린다. */
    void Flush() noexcept;

    /** @brief 새 레코드 수신을 멈추고 남은 레코드를 기록한 뒤 작업 스레드를 종료한다. */
    void Shutdown() noexcept;

    /** @brief 대기열이 가득 차거나 종료 후 도착해 버린 레코드의 누적 수를 반환한다. */
    [[nodiscard]] std::uint64_t GetDroppedRecordCount() const noexcept;

    /** @brief 파일 열기, 쓰기 또는 flush에 실패한 누적 횟수를 반환한다. */
    [[nodiscard]] std::uint64_t GetWriteFailureCount() const noexcept;

private:
    enum class RecordKind
    {
        Log,
        Metric
    };

    struct Record
    {
        RecordKind kind;
        LogLevel level;
        std::int64_t timestampMilliseconds;
        std::uint64_t threadId;
        std::string name;
        std::string message;
        std::string unit;
        double value;
    };

    void OpenOutputFile();
    void Enqueue(Record record) noexcept;
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
    void Consume();
#endif
    void WriteRecord(const Record& record);
    static void WriteEscapedJson(std::ostream& output, const std::string& value);

    LoggerOptions m_Options;
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
    std::mutex m_LifecycleMutex;
    mutable std::mutex m_Mutex;
    std::condition_variable m_QueueChanged;
    std::condition_variable m_Drained;
    std::deque<Record> m_Queue;
#endif
    std::ofstream m_Output;
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
    std::thread m_Worker;
#endif
    std::atomic<std::uint64_t> m_DroppedRecords{0};
    std::atomic<std::uint64_t> m_WriteFailures{0};
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
    bool m_Writing = false;
#endif
    bool m_Accepting = true;
};
} // namespace grasplink::diagnostics
