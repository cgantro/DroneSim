#include "media/RecordedCaptureTransfer.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <fstream>
#include <limits>
#include <mutex>
#include <span>
#include <thread>
#include <utility>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <WinSock2.h>
#include <WS2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace aerosimlab::media
{
namespace
{
constexpr std::array<char, 8> transferMagic{'D', 'S', 'C', 'A', 'P', '0', '0', '1'};
constexpr std::uint32_t transferredFileCount = 3U;
constexpr std::size_t transferChunkBytes = 64U * 1024U;
constexpr std::uint32_t maximumHeaderBytes = 4096U;
constexpr auto socketPollInterval = std::chrono::milliseconds(100);

#ifdef _WIN32
using NativeSocket = SOCKET;
constexpr NativeSocket invalidSocket = INVALID_SOCKET;
void CloseSocket(NativeSocket socket)
{
    if (socket != invalidSocket)
    {
        shutdown(socket, SD_BOTH);
        closesocket(socket);
    }
}
int SocketError() { return WSAGetLastError(); }
bool IsSocketTimeout(int error) { return error == WSAETIMEDOUT || error == WSAEWOULDBLOCK; }
int SelectSocket(NativeSocket socket, fd_set* readSet, fd_set* writeSet,
    timeval* timeout)
{
    return select(0, readSet, writeSet, nullptr, timeout);
}
bool StartSocketRuntime(std::string& error)
{
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) == 0)
        return true;
    error = "Winsock을 초기화하지 못했습니다.";
    return false;
}
void StopSocketRuntime() { WSACleanup(); }
#else
using NativeSocket = int;
constexpr NativeSocket invalidSocket = -1;
void CloseSocket(NativeSocket socket)
{
    if (socket != invalidSocket)
    {
        shutdown(socket, SHUT_RDWR);
        close(socket);
    }
}
int SocketError() { return errno; }
bool IsSocketTimeout(int error) { return error == ETIMEDOUT || error == EAGAIN || error == EWOULDBLOCK; }
int SelectSocket(NativeSocket socket, fd_set* readSet, fd_set* writeSet,
    timeval* timeout)
{
    return select(socket + 1, readSet, writeSet, nullptr, timeout);
}
bool StartSocketRuntime(std::string&) { return true; }
void StopSocketRuntime() {}
#endif

std::string SocketErrorMessage(const char* action, int error);

bool ConnectWithTimeout(NativeSocket socket, const sockaddr_in& address,
    std::string& error)
{
#ifdef _WIN32
    u_long nonBlocking = 1;
    if (ioctlsocket(socket, FIONBIO, &nonBlocking) != 0)
    {
        error = SocketErrorMessage("녹화 자료 연결 대기 설정", SocketError());
        return false;
    }
    const int result = connect(socket,
        reinterpret_cast<const sockaddr*>(&address), sizeof(address));
    if (result == SOCKET_ERROR && WSAGetLastError() != WSAEWOULDBLOCK)
    {
        error = SocketErrorMessage("지상통제소 녹화 자료 연결", SocketError());
        nonBlocking = 0;
        ioctlsocket(socket, FIONBIO, &nonBlocking);
        return false;
    }
#else
    const int originalFlags = fcntl(socket, F_GETFL, 0);
    if (originalFlags < 0 || fcntl(socket, F_SETFL, originalFlags | O_NONBLOCK) < 0)
    {
        error = SocketErrorMessage("녹화 자료 연결 대기 설정", SocketError());
        return false;
    }
    const int result = connect(socket,
        reinterpret_cast<const sockaddr*>(&address), sizeof(address));
    if (result < 0 && errno != EINPROGRESS)
    {
        error = SocketErrorMessage("지상통제소 녹화 자료 연결", SocketError());
        fcntl(socket, F_SETFL, originalFlags);
        return false;
    }
#endif

    fd_set writeSet;
    FD_ZERO(&writeSet);
    FD_SET(socket, &writeSet);
    timeval timeout{3, 0};
    const int ready = SelectSocket(socket, nullptr, &writeSet, &timeout);
    int connectionError = 0;
#ifdef _WIN32
    int errorSize = sizeof(connectionError);
#else
    socklen_t errorSize = sizeof(connectionError);
#endif
    if (ready <= 0 || getsockopt(socket, SOL_SOCKET, SO_ERROR,
            reinterpret_cast<char*>(&connectionError), &errorSize) != 0
        || connectionError != 0)
    {
        if (ready == 0)
            error = "지상통제소 녹화 자료 연결이 3초 안에 완료되지 않았습니다.";
        else
            error = SocketErrorMessage("지상통제소 녹화 자료 연결",
                connectionError != 0 ? connectionError : SocketError());
#ifdef _WIN32
        u_long blocking = 0;
        ioctlsocket(socket, FIONBIO, &blocking);
#else
        fcntl(socket, F_SETFL, originalFlags);
#endif
        return false;
    }

#ifdef _WIN32
    u_long blocking = 0;
    ioctlsocket(socket, FIONBIO, &blocking);
    const DWORD sendTimeout = 2000U;
#else
    fcntl(socket, F_SETFL, originalFlags);
    timeval sendTimeout{2, 0};
#endif
    setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO,
        reinterpret_cast<const char*>(&sendTimeout), sizeof(sendTimeout));
    return true;
}

std::string SocketErrorMessage(const char* action, int error)
{
    return std::string(action) + "에 실패했습니다. 소켓 오류 " + std::to_string(error);
}

bool SendAll(NativeSocket socket, std::span<const char> bytes, std::string& error)
{
    std::size_t sent = 0;
    while (sent < bytes.size())
    {
#ifdef _WIN32
        const int result = send(socket, bytes.data() + sent,
            static_cast<int>(std::min(bytes.size() - sent,
                static_cast<std::size_t>(std::numeric_limits<int>::max()))), 0);
#else
        const ssize_t result = send(socket, bytes.data() + sent,
            bytes.size() - sent, MSG_NOSIGNAL);
#endif
        if (result <= 0)
        {
            error = SocketErrorMessage("녹화 자료 전송", SocketError());
            return false;
        }
        sent += static_cast<std::size_t>(result);
    }
    return true;
}

bool ReceiveAll(NativeSocket socket, std::span<char> bytes,
    std::stop_token stopToken, std::string& error)
{
    std::size_t received = 0;
    while (received < bytes.size() && !stopToken.stop_requested())
    {
#ifdef _WIN32
        const int result = recv(socket, bytes.data() + received,
            static_cast<int>(std::min(bytes.size() - received,
                static_cast<std::size_t>(std::numeric_limits<int>::max()))), 0);
#else
        const ssize_t result = recv(socket, bytes.data() + received,
            bytes.size() - received, 0);
#endif
        if (result == 0)
        {
            error = "녹화 자료를 다 받기 전에 연결이 끊겼습니다.";
            return false;
        }
        if (result < 0)
        {
            const int socketError = SocketError();
            if (stopToken.stop_requested())
                return false;
            if (IsSocketTimeout(socketError))
                continue;
            error = SocketErrorMessage("녹화 자료 수신", socketError);
            return false;
        }
        received += static_cast<std::size_t>(result);
    }
    return received == bytes.size();
}

void AppendBigEndian(std::vector<char>& bytes, std::uint64_t value, int byteCount)
{
    for (int shift = (byteCount - 1) * 8; shift >= 0; shift -= 8)
        bytes.push_back(static_cast<char>((value >> shift) & 0xFFU));
}

bool IsExpectedCaptureFile(const std::string& name)
{
    return !name.empty() && name != "." && name != ".."
        && name.find_first_of("/\\:") == std::string::npos
        && (name.ends_with(".mp4") || name.ends_with(".jsonl")
            || name.ends_with(".manifest.json"));
}

struct FileEntry
{
    std::filesystem::path path;
    std::string name;
    std::uint64_t size = 0;
    std::uint32_t checksum = 0;
};

bool DescribeFiles(const RecordedCaptureFiles& files,
    std::array<FileEntry, transferredFileCount>& entries, std::string& error)
{
    const std::array paths{files.video, files.metadata, files.manifest};
    const std::array<std::string, transferredFileCount> expectedExtensions{
        ".mp4", ".jsonl", ".manifest.json"};
    for (std::size_t index = 0; index < paths.size(); ++index)
    {
        std::error_code statusError;
        if (!std::filesystem::is_regular_file(paths[index], statusError)
            || statusError)
        {
            error = "전송할 녹화 파일을 찾지 못했습니다: "
                + paths[index].filename().string();
            return false;
        }
        FileEntry& entry = entries[index];
        entry.path = paths[index];
        entry.name = paths[index].filename().string();
        if (!entry.name.ends_with(expectedExtensions[index]))
        {
            error = "녹화 영상·메타데이터·목록의 파일 이름을 확인해 주세요.";
            return false;
        }
        if (index > 0 && entry.name == entries[0].name)
        {
            error = "전송할 파일 이름이 서로 겹칩니다.";
            return false;
        }
        entry.size = std::filesystem::file_size(entry.path, statusError);
        if (statusError || entry.size == 0U)
        {
            error = "비어 있거나 크기를 읽지 못한 녹화 파일입니다.";
            return false;
        }
        if (!CalculateFileCrc32(entry.path, entry.checksum, error))
            return false;
    }
    return true;
}

bool SendFileBytes(NativeSocket socket, const FileEntry& entry, std::string& error)
{
    std::ifstream input(entry.path, std::ios::binary);
    if (!input)
    {
        error = "녹화 파일을 열지 못했습니다: " + entry.name;
        return false;
    }
    std::array<char, transferChunkBytes> buffer{};
    std::uint64_t remaining = entry.size;
    while (remaining > 0U)
    {
        const auto requested = static_cast<std::streamsize>(
            std::min<std::uint64_t>(remaining, buffer.size()));
        input.read(buffer.data(), requested);
        if (input.gcount() != requested)
        {
            error = "전송 중 녹화 파일을 끝까지 읽지 못했습니다: " + entry.name;
            return false;
        }
        if (!SendAll(socket, std::span<const char>{buffer.data(),
                static_cast<std::size_t>(requested)}, error))
            return false;
        remaining -= static_cast<std::uint64_t>(requested);
    }
    return true;
}

bool ReceiveFileBytes(NativeSocket socket, const FileEntry& entry,
    const std::filesystem::path& destination, std::stop_token stopToken,
    std::string& error)
{
    const std::filesystem::path temporaryPath = destination.string() + ".partial";
    std::ofstream output(temporaryPath, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        error = "받은 파일을 저장할 수 없습니다: " + entry.name;
        return false;
    }

    std::array<char, transferChunkBytes> buffer{};
    std::uint64_t remaining = entry.size;
    std::uint32_t checksum = 0xFFFFFFFFU;
    while (remaining > 0U)
    {
        const std::size_t chunkSize = static_cast<std::size_t>(
            std::min<std::uint64_t>(remaining, buffer.size()));
        if (!ReceiveAll(socket, std::span<char>{buffer.data(), chunkSize},
                stopToken, error))
        {
            output.close();
            std::error_code ignored;
            std::filesystem::remove(temporaryPath, ignored);
            return false;
        }
        output.write(buffer.data(), static_cast<std::streamsize>(chunkSize));
        if (!output)
        {
            error = "받은 파일을 디스크에 쓰지 못했습니다: " + entry.name;
            output.close();
            std::error_code ignored;
            std::filesystem::remove(temporaryPath, ignored);
            return false;
        }
        for (std::size_t index = 0; index < chunkSize; ++index)
        {
            checksum ^= static_cast<unsigned char>(buffer[index]);
            for (int bit = 0; bit < 8; ++bit)
            {
                const std::uint32_t lowBitMask = 0U - (checksum & 1U);
                checksum = (checksum >> 1U) ^ (0xEDB88320U & lowBitMask);
            }
        }
        remaining -= chunkSize;
    }
    output.flush();
    const bool saved = static_cast<bool>(output);
    output.close();
    if (!saved)
    {
        error = "받은 파일을 마무리하지 못했습니다: " + entry.name;
        std::error_code ignored;
        std::filesystem::remove(temporaryPath, ignored);
        return false;
    }
    checksum ^= 0xFFFFFFFFU;
    if (checksum != entry.checksum)
    {
        error = "받은 파일의 CRC-32가 맞지 않습니다: " + entry.name;
        std::error_code ignored;
        std::filesystem::remove(temporaryPath, ignored);
        return false;
    }

    std::error_code fileError;
    if (std::filesystem::exists(destination, fileError))
    {
        std::uint32_t existingChecksum = 0;
        std::string checksumError;
        if (fileError || !CalculateFileCrc32(destination, existingChecksum, checksumError)
            || existingChecksum != entry.checksum)
        {
            error = "같은 이름의 다른 녹화 파일이 이미 있습니다: " + entry.name;
            std::filesystem::remove(temporaryPath, fileError);
            return false;
        }
        std::filesystem::remove(temporaryPath, fileError);
        return true;
    }
    std::filesystem::rename(temporaryPath, destination, fileError);
    if (fileError)
    {
        error = "받은 파일을 저장 폴더에 확정하지 못했습니다: " + entry.name;
        std::filesystem::remove(temporaryPath, fileError);
        return false;
    }
    return true;
}
}

bool CalculateFileCrc32(const std::filesystem::path& path,
    std::uint32_t& checksum, std::string& error)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        error = "파일을 열어 CRC-32를 계산하지 못했습니다.";
        return false;
    }
    std::array<unsigned char, transferChunkBytes> buffer{};
    checksum = 0xFFFFFFFFU;
    while (input)
    {
        input.read(reinterpret_cast<char*>(buffer.data()),
            static_cast<std::streamsize>(buffer.size()));
        const std::streamsize bytesRead = input.gcount();
        for (std::streamsize index = 0; index < bytesRead; ++index)
        {
            checksum ^= buffer[static_cast<std::size_t>(index)];
            for (int bit = 0; bit < 8; ++bit)
            {
                const std::uint32_t lowBitMask = 0U - (checksum & 1U);
                checksum = (checksum >> 1U) ^ (0xEDB88320U & lowBitMask);
            }
        }
    }
    if (!input.eof())
    {
        error = "파일을 끝까지 읽어 CRC-32를 계산하지 못했습니다.";
        return false;
    }
    checksum ^= 0xFFFFFFFFU;
    return true;
}

bool SendRecordedCapture(const std::string& destinationIpv4,
    std::uint16_t destinationPort, const RecordedCaptureFiles& files,
    std::string& error)
{
    if (destinationIpv4.empty() || destinationPort == 0)
    {
        error = "수신할 지상통제소 IPv4 주소와 포트를 확인해 주세요.";
        return false;
    }
    std::array<FileEntry, transferredFileCount> entries{};
    if (!DescribeFiles(files, entries, error))
        return false;

    bool socketRuntimeStarted = false;
    if (!StartSocketRuntime(error))
        return false;
    socketRuntimeStarted = true;
    const NativeSocket socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket == invalidSocket)
    {
        error = SocketErrorMessage("녹화 자료 전송 소켓 생성", SocketError());
        StopSocketRuntime();
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(destinationPort);
    if (inet_pton(AF_INET, destinationIpv4.c_str(), &address.sin_addr) != 1)
    {
        error = "녹화 자료 수신 주소는 숫자 형식의 IPv4여야 합니다.";
        CloseSocket(socket);
        StopSocketRuntime();
        return false;
    }

    bool success = false;
    if (!ConnectWithTimeout(socket, address, error))
    {
    }
    else
    {
        std::vector<char> header(transferMagic.begin(), transferMagic.end());
        AppendBigEndian(header, transferredFileCount, 4);
        for (const FileEntry& entry : entries)
        {
            AppendBigEndian(header, entry.name.size(), 2);
            AppendBigEndian(header, entry.size, 8);
            AppendBigEndian(header, entry.checksum, 4);
            header.insert(header.end(), entry.name.begin(), entry.name.end());
        }
        if (header.size() > maximumHeaderBytes)
        {
            error = "녹화 자료 안내 정보가 너무 큽니다.";
        }
        else
        {
            std::vector<char> framedHeader;
            AppendBigEndian(framedHeader, header.size(), 4);
            framedHeader.insert(framedHeader.end(), header.begin(), header.end());
            success = SendAll(socket, framedHeader, error);
            for (const FileEntry& entry : entries)
            {
                if (!success || !SendFileBytes(socket, entry, error))
                {
                    success = false;
                    break;
                }
            }
            if (success)
            {
                std::array<char, 1> acknowledgement{};
                success = ReceiveAll(socket, acknowledgement, {}, error)
                    && acknowledgement[0] == 'A';
                if (!success && error.empty())
                    error = "지상통제소가 파일 무결성을 확인하지 못했습니다.";
            }
        }
    }
    CloseSocket(socket);
    if (socketRuntimeStarted)
        StopSocketRuntime();
    return success;
}

struct RecordedCaptureReceiver::Impl
{
    std::atomic<NativeSocket> listener = invalidSocket;
    std::atomic<NativeSocket> activeConnection = invalidSocket;
    std::jthread worker;
    std::filesystem::path directory;
    std::uint16_t localPort = 0;
    std::atomic_bool running = false;
    mutable std::mutex resultMutex;
    std::optional<RecordedCaptureTransferResult> latestResult;

    void SaveResult(RecordedCaptureTransferResult result)
    {
        std::lock_guard lock(resultMutex);
        latestResult = std::move(result);
    }

    void HandleConnection(NativeSocket connection, std::stop_token stopToken)
    {
        RecordedCaptureTransferResult result;
        std::string error;
        std::array<char, 4> sizeBytes{};
        std::uint64_t headerSize = 0;
        if (!ReceiveAll(connection, sizeBytes, stopToken, error))
        {
            result.message = std::move(error);
            SaveResult(std::move(result));
            return;
        }
        for (const char byte : sizeBytes)
            headerSize = (headerSize << 8U) | static_cast<unsigned char>(byte);
        if (headerSize == 0U || headerSize > maximumHeaderBytes)
        {
            result.message = "녹화 자료 안내 정보의 크기가 올바르지 않습니다.";
            SaveResult(std::move(result));
            return;
        }
        std::vector<char> header(static_cast<std::size_t>(headerSize));
        if (!ReceiveAll(connection, header, stopToken, error))
        {
            result.message = std::move(error);
            SaveResult(std::move(result));
            return;
        }
        if (header.size() < transferMagic.size() + 4U
            || !std::equal(transferMagic.begin(), transferMagic.end(), header.begin()))
        {
            result.message = "지원하지 않는 녹화 자료 전송 형식입니다.";
            SaveResult(std::move(result));
            return;
        }

        std::size_t offset = transferMagic.size();
        std::uint64_t fileCount = 0;
        for (int index = 0; index < 4; ++index)
            fileCount = (fileCount << 8U)
                | static_cast<unsigned char>(header[offset++]);
        if (fileCount != transferredFileCount)
        {
            result.message = "영상·촬영 정보·저장 목록 세 파일이 필요합니다.";
            SaveResult(std::move(result));
            return;
        }

        std::array<FileEntry, transferredFileCount> entries{};
        std::array<bool, transferredFileCount> seenExtensions{};
        for (FileEntry& entry : entries)
        {
            if (offset > header.size() || header.size() - offset < 14U)
            {
                result.message = "녹화 자료 안내 정보가 중간에 끝났습니다.";
                SaveResult(std::move(result));
                return;
            }
            std::uint64_t nameSize = 0;
            std::uint64_t fileSize = 0;
            std::uint64_t checksum = 0;
            for (int index = 0; index < 2; ++index)
                nameSize = (nameSize << 8U)
                    | static_cast<unsigned char>(header[offset++]);
            for (int index = 0; index < 8; ++index)
                fileSize = (fileSize << 8U)
                    | static_cast<unsigned char>(header[offset++]);
            for (int index = 0; index < 4; ++index)
                checksum = (checksum << 8U)
                    | static_cast<unsigned char>(header[offset++]);
            if (offset > header.size() || nameSize == 0U
                || nameSize > header.size() - offset
                || fileSize == 0U)
            {
                result.message = "녹화 자료 이름 또는 크기가 올바르지 않습니다.";
                SaveResult(std::move(result));
                return;
            }
            entry.name.assign(header.data() + offset,
                static_cast<std::size_t>(nameSize));
            offset += static_cast<std::size_t>(nameSize);
            entry.size = fileSize;
            entry.checksum = static_cast<std::uint32_t>(checksum);
            if (!IsExpectedCaptureFile(entry.name))
            {
                result.message = "받을 수 없는 녹화 파일 이름입니다.";
                SaveResult(std::move(result));
                return;
            }
            const std::array<std::string, 3> extensions{
                ".mp4", ".jsonl", ".manifest.json"};
            const auto extension = std::find_if(extensions.begin(), extensions.end(),
                [&entry](const std::string& candidate)
                {
                    return entry.name.ends_with(candidate);
                });
            const std::size_t extensionIndex = static_cast<std::size_t>(
                std::distance(extensions.begin(), extension));
            if (extension == extensions.end() || seenExtensions[extensionIndex]
                || extensionIndex != static_cast<std::size_t>(&entry - entries.data()))
            {
                result.message = "영상·촬영 정보·저장 목록 파일 구성이 올바르지 않습니다.";
                SaveResult(std::move(result));
                return;
            }
            seenExtensions[extensionIndex] = true;
        }
        if (offset != header.size())
        {
            result.message = "녹화 자료 안내 정보 뒤에 예상하지 못한 데이터가 있습니다.";
            SaveResult(std::move(result));
            return;
        }

        std::array<std::filesystem::path, transferredFileCount> destinations{};
        for (std::size_t index = 0; index < entries.size(); ++index)
        {
            entries[index].path = directory / entries[index].name;
            destinations[index] = entries[index].path;
            if (!ReceiveFileBytes(connection, entries[index], destinations[index],
                    stopToken, error))
            {
                result.message = std::move(error);
                SaveResult(std::move(result));
                return;
            }
        }

        const auto manifest = std::find_if(entries.begin(), entries.end(),
            [](const FileEntry& entry) { return entry.name.ends_with(".manifest.json"); });
        result.verified = true;
        result.manifestPath = manifest->path;
        result.message = "영상·촬영 정보·저장 목록을 받고 CRC-32를 확인했습니다.";
        const std::array<char, 1> acknowledgement{'A'};
        if (!SendAll(connection, acknowledgement, error))
        {
            result.verified = false;
            result.message = std::move(error);
        }
        SaveResult(std::move(result));
    }

    void Receive(std::stop_token stopToken)
    {
        running = true;
        while (!stopToken.stop_requested())
        {
            const NativeSocket socket = listener.load();
            if (socket == invalidSocket)
                break;
            fd_set readSet;
            FD_ZERO(&readSet);
            FD_SET(socket, &readSet);
            timeval timeout{0, static_cast<long>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    socketPollInterval).count())};
            const int ready = SelectSocket(socket, &readSet, nullptr, &timeout);
            if (ready <= 0)
            {
                if (stopToken.stop_requested())
                    break;
                continue;
            }
            const NativeSocket connection = accept(socket, nullptr, nullptr);
            if (connection == invalidSocket)
                continue;
            activeConnection = connection;
#ifdef _WIN32
            const DWORD receiveTimeout = 200U;
#else
            timeval receiveTimeout{0, 200'000};
#endif
            setsockopt(connection, SOL_SOCKET, SO_RCVTIMEO,
                reinterpret_cast<const char*>(&receiveTimeout), sizeof(receiveTimeout));
            HandleConnection(connection, stopToken);
            if (activeConnection.exchange(invalidSocket) == connection)
                CloseSocket(connection);
        }
        running = false;
    }
};

RecordedCaptureReceiver::RecordedCaptureReceiver() : impl_(std::make_unique<Impl>())
{
}

RecordedCaptureReceiver::~RecordedCaptureReceiver()
{
    Stop();
}

bool RecordedCaptureReceiver::Start(std::uint16_t port,
    const std::filesystem::path& destinationDirectory, std::string& error)
{
    Stop();
    if (destinationDirectory.empty())
    {
        error = "녹화 자료를 저장할 폴더가 필요합니다.";
        return false;
    }
    if (!StartSocketRuntime(error))
        return false;
    std::error_code directoryError;
    std::filesystem::create_directories(destinationDirectory, directoryError);
    if (directoryError)
    {
        error = "지상통제소 녹화 자료 폴더를 만들지 못했습니다.";
        StopSocketRuntime();
        return false;
    }

    const NativeSocket listener = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener == invalidSocket)
    {
        error = SocketErrorMessage("녹화 자료 수신 소켓 생성", SocketError());
        StopSocketRuntime();
        return false;
    }
    const int reuseAddress = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<const char*>(&reuseAddress), sizeof(reuseAddress));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);
    if (bind(listener, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0
        || listen(listener, 1) != 0)
    {
        error = SocketErrorMessage("녹화 자료 수신 포트 열기", SocketError());
        CloseSocket(listener);
        StopSocketRuntime();
        return false;
    }
    sockaddr_in boundAddress{};
#ifdef _WIN32
    int addressSize = sizeof(boundAddress);
#else
    socklen_t addressSize = sizeof(boundAddress);
#endif
    if (getsockname(listener, reinterpret_cast<sockaddr*>(&boundAddress), &addressSize) != 0)
    {
        error = SocketErrorMessage("녹화 자료 포트 확인", SocketError());
        CloseSocket(listener);
        StopSocketRuntime();
        return false;
    }
    impl_->directory = destinationDirectory;
    impl_->localPort = ntohs(boundAddress.sin_port);
    impl_->listener = listener;
    impl_->worker = std::jthread([state = impl_.get()](std::stop_token stopToken)
    {
        state->Receive(stopToken);
    });
    return true;
}

void RecordedCaptureReceiver::Stop()
{
    if (!impl_)
        return;
    if (impl_->worker.joinable())
    {
        impl_->worker.request_stop();
        CloseSocket(impl_->listener.exchange(invalidSocket));
        CloseSocket(impl_->activeConnection.exchange(invalidSocket));
        impl_->worker.join();
        StopSocketRuntime();
    }
    impl_->localPort = 0;
    impl_->running = false;
}

bool RecordedCaptureReceiver::IsRunning() const noexcept
{
    return impl_ && impl_->running.load();
}

std::uint16_t RecordedCaptureReceiver::LocalPort() const noexcept
{
    return impl_ ? impl_->localPort : 0;
}

std::optional<RecordedCaptureTransferResult> RecordedCaptureReceiver::TakeLatestResult()
{
    if (!impl_)
        return std::nullopt;
    std::lock_guard lock(impl_->resultMutex);
    std::optional<RecordedCaptureTransferResult> result = std::move(impl_->latestResult);
    impl_->latestResult.reset();
    return result;
}
}
