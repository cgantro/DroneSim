param(
    [Parameter(Mandatory = $true)][string]$Simulator,
    [Parameter(Mandatory = $true)][string]$GroundControl,
    [Parameter(Mandatory = $true)][string]$GroundControlUiPath
)

$ErrorActionPreference = 'Stop'

Add-Type -TypeDefinition @'
using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;

public static class AeroSimLabUiTestWindow
{
    public const int CameraFaultButton = 111;
    public const int CameraRecoverButton = 112;
    public const int StatusText = 113;
    public const int TelemetryText = 114;
    public const int TakeoffButton = 105;
    public const int LandButton = 107;
    public const int RecordingStartButton = 115;
    public const int RecordingStopButton = 116;
    public const int RecordingReplayButton = 117;
    public const int PortInput = 101;

    private delegate bool EnumWindowsCallback(IntPtr window, IntPtr parameter);

    [DllImport("user32.dll")]
    private static extern bool EnumWindows(EnumWindowsCallback callback, IntPtr parameter);

    [DllImport("user32.dll")]
    public static extern IntPtr GetDlgItem(IntPtr window, int controlId);

    [DllImport("user32.dll", EntryPoint = "SendMessageW", CharSet = CharSet.Unicode)]
    private static extern IntPtr SendTextMessage(
        IntPtr window, uint message, IntPtr wParam, string text);

    [DllImport("user32.dll", EntryPoint = "SendMessageW")]
    public static extern IntPtr SendMessage(
        IntPtr window, uint message, IntPtr wParam, IntPtr lParam);

    [DllImport("user32.dll")]
    private static extern uint GetWindowThreadProcessId(IntPtr window, out uint processId);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    private static extern int GetClassName(IntPtr window, StringBuilder className, int capacity);

    [DllImport("user32.dll")]
    public static extern bool PostMessage(IntPtr window, uint message, IntPtr wParam, IntPtr lParam);

    public static bool SetText(IntPtr window, string text)
    {
        return SendTextMessage(window, 0x000C, IntPtr.Zero, text) != IntPtr.Zero;
    }

    public static IntPtr FindProcessWindow(uint expectedProcessId, string expectedClassName)
    {
        IntPtr found = IntPtr.Zero;
        EnumWindows(delegate(IntPtr window, IntPtr parameter)
        {
            uint processId;
            GetWindowThreadProcessId(window, out processId);
            if (processId != expectedProcessId)
                return true;

            var className = new StringBuilder(128);
            GetClassName(window, className, className.Capacity);
            if (className.ToString() != expectedClassName)
                return true;

            found = window;
            return false;
        }, IntPtr.Zero);
        return found;
    }

}

public static class AeroSimLabSessionTestClient
{
    private static readonly UdpClient client = new UdpClient(0);

    static AeroSimLabSessionTestClient()
    {
        client.Client.ReceiveTimeout = 400;
    }

    public static string Send(int port, uint commandId, uint sequence, string action)
    {
        var payload = String.Format(CultureInfo.InvariantCulture,
            "{{\"action\":\"{0}\",\"command_id\":{1},\"sequence\":{2},\"target\":\"session\"}}",
            action, commandId, sequence);
        var payloadLength = Encoding.UTF8.GetByteCount(payload);
        var message = String.Format(CultureInfo.InvariantCulture,
            "{{\"length\":{0},\"payload\":{1},\"type\":\"command\",\"version\":1}}",
            payloadLength, payload);
        var bytes = Encoding.UTF8.GetBytes(message);
        var destination = new IPEndPoint(IPAddress.Loopback, port);
        var deadline = DateTime.UtcNow.AddSeconds(4);

        while (DateTime.UtcNow < deadline)
        {
            client.Send(bytes, bytes.Length, destination);
            try
            {
                IPEndPoint source = new IPEndPoint(IPAddress.Any, 0);
                var response = client.Receive(ref source);
                if (source.Address.Equals(IPAddress.Loopback) && source.Port == port)
                    return Encoding.UTF8.GetString(response);
            }
            catch (SocketException error)
            {
                if (error.SocketErrorCode != SocketError.TimedOut
                    && error.SocketErrorCode != SocketError.WouldBlock)
                    throw;
            }
        }

        throw new TimeoutException("기록 명령에 시뮬레이터가 응답하지 않았습니다.");
    }
}
'@

function Invoke-GroundControlCli([string[]]$Arguments) {
    $previousErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $output = (& $GroundControl @Arguments 2>&1 | Out-String)
        return [PSCustomObject]@{
            ExitCode = $LASTEXITCODE
            Output = $output
        }
    }
    finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }
}

$sessionCommandId = 1
$sessionSequence = 1
function Invoke-SessionCommand([string]$Action, [string]$ExpectedStatus) {
    $response = [AeroSimLabSessionTestClient]::Send(
        $port, $script:sessionCommandId, $script:sessionSequence, $Action)
    $script:sessionCommandId++
    $script:sessionSequence++
    $reply = $response | ConvertFrom-Json
    if ($reply.payload.status -ne $ExpectedStatus) {
        throw "기록 명령 '$Action' 결과가 예상과 다릅니다: $($reply.payload.status)"
    }
}

$port = 31000 + (Get-Random -Minimum 0 -Maximum 18000)
$workingDirectory = Get-Location
$timeoutResult = Invoke-GroundControlCli @('--port', "$port", '--command', 'takeoff')
if ($timeoutResult.ExitCode -ne 3 -or $timeoutResult.Output -notmatch '응답 시간 초과') {
    throw "시뮬레이터가 없을 때 지상통제 프로그램이 시간 초과로 끝나지 않았습니다.`n$($timeoutResult.Output)"
}

$retryingClient = $null
$server = $null
$groundControlUiProcess = $null
$uiListener = $null
$uiWindow = [IntPtr]::Zero

try {
    $retryingClient = Start-Process -FilePath $GroundControl `
        -ArgumentList @('--port', "$port", '--command', 'takeoff') `
        -WorkingDirectory $workingDirectory `
        -WindowStyle Hidden `
        -PassThru
    Start-Sleep -Milliseconds 500

    $server = Start-Process -FilePath $Simulator `
        -ArgumentList @('--smoke-test', '--udp-port', "$port") `
        -WorkingDirectory $workingDirectory `
        -WindowStyle Hidden `
        -PassThru

    Start-Sleep -Milliseconds 200
    $server.Refresh()
    if ($server.HasExited) {
        throw "시뮬레이터가 UDP 시험 서버를 시작하지 못했습니다. 종료 코드: $($server.ExitCode)"
    }

    if (-not $retryingClient.WaitForExit(3000)) {
        throw '시뮬레이터가 켜진 뒤에도 재전송 명령에 응답하지 않았습니다.'
    }
    $retryingClient.Refresh()
    if ($retryingClient.ExitCode -ne 0) {
        throw "재전송 명령이 서버 시작 뒤 적용되지 않았습니다. 종료 코드: $($retryingClient.ExitCode)"
    }

    Start-Sleep -Milliseconds 1800
    $faultResult = Invoke-GroundControlCli @('--port', "$port", '--camera', 'fault')
    if ($faultResult.ExitCode -ne 0 -or $faultResult.Output -notmatch '카메라: fault') {
        throw "카메라 고장 명령이 적용되지 않았습니다.`n$($faultResult.Output)"
    }
    $recoveryResult = Invoke-GroundControlCli @('--port', "$port", '--camera', 'recover')
    if ($recoveryResult.ExitCode -ne 0 -or $recoveryResult.Output -notmatch '카메라: active') {
        throw "카메라 복구 명령이 적용되지 않았습니다.`n$($recoveryResult.Output)"
    }

    $portConflictError = $null
    try {
        $conflictingServer = Start-Process -FilePath $Simulator `
            -ArgumentList @('--smoke-test', '--udp-port', "$port") `
            -WorkingDirectory $workingDirectory `
            -WindowStyle Hidden `
            -PassThru -Wait
    }
    catch {
        $portConflictError = $_
    }
    if ($null -ne $portConflictError) {
        throw "이미 사용 중인 UDP 포트를 거부하지 않았습니다.`n$portConflictError"
    }
    if ($conflictingServer.ExitCode -eq 0) {
        throw '이미 사용 중인 UDP 포트를 거부하지 않았습니다.'
    }

    $landingResult = Invoke-GroundControlCli @('--port', "$port", '--command', 'land')
    if ($landingResult.ExitCode -ne 0) {
        throw "착륙 명령이 적용되지 않았습니다.`n$($landingResult.Output)"
    }
    Start-Sleep -Milliseconds 1800

    Invoke-SessionCommand 'record_start' 'recording_started'
    $takeoffResult = Invoke-GroundControlCli @('--port', "$port", '--command', 'takeoff')
    if ($takeoffResult.ExitCode -ne 0) {
        throw "기록 중 이륙 명령이 적용되지 않았습니다.`n$($takeoffResult.Output)"
    }
    Start-Sleep -Milliseconds 1800
    $recordedLanding = Invoke-GroundControlCli @('--port', "$port", '--command', 'land')
    if ($recordedLanding.ExitCode -ne 0) {
        throw "기록 중 착륙 명령이 적용되지 않았습니다.`n$($recordedLanding.Output)"
    }
    Start-Sleep -Milliseconds 1800
    Invoke-SessionCommand 'record_stop' 'recording_stopped'
    Invoke-SessionCommand 'record_replay' 'recording_replay_started'

    if (-not $server.WaitForExit(5000)) {
        throw '기록한 비행을 다시 재생한 뒤 시험 시뮬레이터가 종료되지 않았습니다.'
    }
    $server.Refresh()
    if ($server.ExitCode -ne 0) {
        throw "시뮬레이터 시험 서버가 오류로 종료했습니다: $($server.ExitCode)"
    }

    $groundControlUiProcess = Start-Process -FilePath $GroundControlUiPath `
        -WorkingDirectory $workingDirectory `
        -WindowStyle Normal `
        -PassThru
    $windowDeadline = [DateTime]::UtcNow.AddSeconds(5)
    while ($uiWindow -eq [IntPtr]::Zero -and [DateTime]::UtcNow -lt $windowDeadline) {
        $groundControlUiProcess.Refresh()
        $uiWindow = [AeroSimLabUiTestWindow]::FindProcessWindow(
            [uint32]$groundControlUiProcess.Id, 'AeroSimLabGroundControlWindow')
        Start-Sleep -Milliseconds 50
    }
    if ($uiWindow -eq [IntPtr]::Zero) {
        throw '지상통제 조작 창을 열지 못했습니다.'
    }
    $uiListener = [System.Net.Sockets.UdpClient]::new($port)
    $portInput = [AeroSimLabUiTestWindow]::GetDlgItem(
        $uiWindow, [AeroSimLabUiTestWindow]::PortInput)
    if (-not [AeroSimLabUiTestWindow]::SetText($portInput, "$port")) {
        throw '지상통제 창에 시험용 포트를 입력하지 못했습니다.'
    }

    $requiredControls = @(
        [AeroSimLabUiTestWindow]::TakeoffButton,
        [AeroSimLabUiTestWindow]::LandButton,
        [AeroSimLabUiTestWindow]::CameraFaultButton,
        [AeroSimLabUiTestWindow]::CameraRecoverButton,
        [AeroSimLabUiTestWindow]::RecordingStartButton,
        [AeroSimLabUiTestWindow]::RecordingStopButton,
        [AeroSimLabUiTestWindow]::RecordingReplayButton,
        [AeroSimLabUiTestWindow]::StatusText,
        [AeroSimLabUiTestWindow]::TelemetryText
    )
    foreach ($controlId in $requiredControls) {
        if ([AeroSimLabUiTestWindow]::GetDlgItem($uiWindow, $controlId) -eq [IntPtr]::Zero) {
            throw "지상통제 창의 조작 항목($controlId)을 찾지 못했습니다."
        }
    }
    $takeoffButton = [AeroSimLabUiTestWindow]::GetDlgItem(
        $uiWindow, [AeroSimLabUiTestWindow]::TakeoffButton)
    [void][AeroSimLabUiTestWindow]::SendMessage(
        $takeoffButton, 0x00F5, [IntPtr]::Zero, [IntPtr]::Zero)
    $receive = $uiListener.BeginReceive($null, $null)
    if (-not $receive.AsyncWaitHandle.WaitOne(5000)) {
        throw '지상통제 창에서 보낸 이륙 명령을 받지 못했습니다.'
    }
    $source = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
    $datagram = $uiListener.EndReceive($receive, [ref]$source)
    $uiCommand = [Text.Encoding]::UTF8.GetString($datagram) | ConvertFrom-Json
    if ($uiCommand.type -ne 'command' -or $uiCommand.payload.action -ne 'takeoff') {
        throw "지상통제 창에서 예상하지 않은 명령을 받았습니다: $uiCommand"
    }
    Write-Output '지상통제 창에서 보낸 이륙 명령을 확인했습니다.'
}
finally {
    if ($null -ne $retryingClient) {
        $retryingClient.Refresh()
    }
    if ($null -ne $retryingClient -and -not $retryingClient.HasExited) {
        Stop-Process -Id $retryingClient.Id -Force
    }
    if ($null -ne $server) {
        $server.Refresh()
    }
    if ($null -ne $server -and -not $server.HasExited) {
        Stop-Process -Id $server.Id -Force
    }
    if ($uiWindow -ne [IntPtr]::Zero) {
        [AeroSimLabUiTestWindow]::PostMessage(
            $uiWindow, 0x0010, [IntPtr]::Zero, [IntPtr]::Zero) | Out-Null
    }
    if ($null -ne $groundControlUiProcess) {
        $groundControlUiProcess.Refresh()
        if (-not $groundControlUiProcess.HasExited) {
            Stop-Process -Id $groundControlUiProcess.Id -Force
        }
    }
    if ($null -ne $uiListener) {
        $uiListener.Dispose()
    }
}
