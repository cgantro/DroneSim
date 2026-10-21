param(
    [Parameter(Mandatory = $true)][string]$Simulator,
    [Parameter(Mandatory = $true)][string]$GroundControl,
    [Parameter(Mandatory = $true)][string]$GroundControlUiPath
)

$ErrorActionPreference = 'Stop'

Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
using System.Text;

public static class AeroSimLabUiTestWindow
{
    public const int CameraFaultButton = 111;
    public const int CameraRecoverButton = 112;
    public const int StatusText = 113;
    public const int TelemetryText = 114;
    public const int TakeoffButton = 105;
    public const int StopButton = 106;
    public const int LandButton = 107;
    public const int WaypointButton = 108;
    public const int WaypointXInput = 102;
    public const int WaypointYInput = 103;
    public const int WaypointZInput = 104;
    public const int CameraOnButton = 109;
    public const int CameraOffButton = 110;
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
    public static extern bool IsWindowEnabled(IntPtr window);

    [DllImport("user32.dll", EntryPoint = "SendMessageW", CharSet = CharSet.Unicode)]
    private static extern IntPtr ReadControlTextMessage(
        IntPtr window, uint message, IntPtr capacity, StringBuilder text);

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

    public static string ReadControlText(IntPtr window)
    {
        var text = new StringBuilder(512);
        ReadControlTextMessage(window, 0x000D, (IntPtr)text.Capacity, text);
        return text.ToString();
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

function Invoke-GroundControlButton([int]$ControlId) {
    $button = [AeroSimLabUiTestWindow]::GetDlgItem($uiWindow, $ControlId)
    $statusText = [AeroSimLabUiTestWindow]::GetDlgItem(
        $uiWindow, [AeroSimLabUiTestWindow]::StatusText)
    $telemetryText = [AeroSimLabUiTestWindow]::GetDlgItem(
        $uiWindow, [AeroSimLabUiTestWindow]::TelemetryText)
    $previousStatus = [AeroSimLabUiTestWindow]::ReadControlText($statusText)
    $previousTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    [void][AeroSimLabUiTestWindow]::SendMessage(
        $button, 0x00F5, [IntPtr]::Zero, [IntPtr]::Zero)

    $deadline = [DateTime]::UtcNow.AddSeconds(5)
    while ([AeroSimLabUiTestWindow]::IsWindowEnabled($button)) {
        $statusChanged = [AeroSimLabUiTestWindow]::ReadControlText($statusText) -ne $previousStatus
        $telemetryChanged = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText) -ne $previousTelemetry
        if ($statusChanged -or $telemetryChanged) {
            return
        }
        if ([DateTime]::UtcNow -ge $deadline) {
            throw "지상통제 명령($ControlId)이 시작되지 않았습니다."
        }
        Start-Sleep -Milliseconds 40
    }

    while (-not [AeroSimLabUiTestWindow]::IsWindowEnabled($button)) {
        if ([DateTime]::UtcNow -ge $deadline) {
            throw "지상통제 명령($ControlId)에 시뮬레이터가 응답하지 않았습니다."
        }
        Start-Sleep -Milliseconds 40
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
$recoveredServer = $null
$groundControlUiProcess = $null
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
    $portInput = [AeroSimLabUiTestWindow]::GetDlgItem(
        $uiWindow, [AeroSimLabUiTestWindow]::PortInput)
    if (-not [AeroSimLabUiTestWindow]::SetText($portInput, "$port")) {
        throw '지상통제 창에 시험용 포트를 입력하지 못했습니다.'
    }

    $requiredControls = @(
        [AeroSimLabUiTestWindow]::TakeoffButton,
        [AeroSimLabUiTestWindow]::StopButton,
        [AeroSimLabUiTestWindow]::LandButton,
        [AeroSimLabUiTestWindow]::WaypointButton,
        [AeroSimLabUiTestWindow]::CameraOnButton,
        [AeroSimLabUiTestWindow]::CameraOffButton,
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

    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::CameraFaultButton)
    $commandStatusText = [AeroSimLabUiTestWindow]::GetDlgItem(
        $uiWindow, [AeroSimLabUiTestWindow]::StatusText)
    $telemetryText = [AeroSimLabUiTestWindow]::GetDlgItem(
        $uiWindow, [AeroSimLabUiTestWindow]::TelemetryText)
    $faultTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    if ($faultTelemetry -notmatch '카메라: fault') {
        $commandStatus = [AeroSimLabUiTestWindow]::ReadControlText($commandStatusText)
        throw "지상통제 창에서 보낸 카메라 고장 명령이 시뮬레이터 상태에 반영되지 않았습니다.`n$commandStatus`n$faultTelemetry"
    }
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::CameraRecoverButton)
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::CameraOffButton)
    $disabledTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    if ($disabledTelemetry -notmatch '카메라: disabled') {
        throw "지상통제 창의 카메라 끄기 명령이 시뮬레이터 상태에 반영되지 않았습니다.`n$disabledTelemetry"
    }
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::CameraOnButton)
    $enabledTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    if ($enabledTelemetry -notmatch '카메라: active') {
        throw "지상통제 창의 카메라 켜기 명령이 시뮬레이터 상태에 반영되지 않았습니다.`n$enabledTelemetry"
    }
    $waypointInputIds = @(
        [AeroSimLabUiTestWindow]::WaypointXInput,
        [AeroSimLabUiTestWindow]::WaypointYInput,
        [AeroSimLabUiTestWindow]::WaypointZInput
    )
    $waypointValues = @('2', '0', '1.5')
    foreach ($index in 0..2) {
        $waypointInput = [AeroSimLabUiTestWindow]::GetDlgItem($uiWindow, $waypointInputIds[$index])
        if (-not [AeroSimLabUiTestWindow]::SetText($waypointInput, $waypointValues[$index])) {
            throw '경유점 좌표를 입력하지 못했습니다.'
        }
    }
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::WaypointButton)
    $waypointTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    if ($waypointTelemetry -notmatch '비행 상태: following_waypoint') {
        throw "지상통제 창의 경유점 명령이 시뮬레이터 상태에 반영되지 않았습니다.`n$waypointTelemetry"
    }
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::StopButton)
    $stopTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    if ($stopTelemetry -notmatch '비행 상태: hovering') {
        throw "지상통제 창의 정지 명령이 시뮬레이터 상태에 반영되지 않았습니다.`n$stopTelemetry"
    }
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::LandButton)
    $landingTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    if ($landingTelemetry -notmatch '비행 상태: landing' -or $landingTelemetry -notmatch '카메라: active') {
        $commandStatus = [AeroSimLabUiTestWindow]::ReadControlText($commandStatusText)
        throw "지상통제 창의 착륙·카메라 복구 명령이 시뮬레이터 상태에 반영되지 않았습니다.`n$commandStatus`n$landingTelemetry"
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

    Start-Sleep -Milliseconds 1800

    $statusText = $commandStatusText
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::RecordingStartButton)
    if ([AeroSimLabUiTestWindow]::ReadControlText($statusText) -notmatch '입력 기록을 시작했습니다') {
        throw '지상통제 창에서 입력 기록을 시작하지 못했습니다.'
    }
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::TakeoffButton)
    $takeoffTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    if ($takeoffTelemetry -notmatch '비행 상태: taking_off') {
        throw "지상통제 창의 이륙 명령이 시뮬레이터 상태에 반영되지 않았습니다.`n$takeoffTelemetry"
    }
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::LandButton)
    $recordedLandingTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    if ($recordedLandingTelemetry -notmatch '비행 상태: landing') {
        throw "기록 중 지상통제 창의 착륙 명령이 시뮬레이터 상태에 반영되지 않았습니다.`n$recordedLandingTelemetry"
    }
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::RecordingStopButton)
    if ([AeroSimLabUiTestWindow]::ReadControlText($statusText) -notmatch '기록을 마쳤습니다') {
        throw '지상통제 창에서 입력 기록을 마치지 못했습니다.'
    }
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::RecordingReplayButton)
    if ([AeroSimLabUiTestWindow]::ReadControlText($statusText) -notmatch '입력 기록을 재생합니다') {
        throw '지상통제 창에서 입력 기록을 재생하지 못했습니다.'
    }

    if (-not $server.WaitForExit(5000)) {
        throw '기록한 비행을 다시 재생한 뒤 시험 시뮬레이터가 종료되지 않았습니다.'
    }
    $server.Refresh()
    if ($server.ExitCode -ne 0) {
        throw "시뮬레이터 시험 서버가 오류로 종료했습니다: $($server.ExitCode)"
    }

    $cameraOffButton = [AeroSimLabUiTestWindow]::GetDlgItem(
        $uiWindow, [AeroSimLabUiTestWindow]::CameraOffButton)
    [void][AeroSimLabUiTestWindow]::SendMessage(
        $cameraOffButton, 0x00F5, [IntPtr]::Zero, [IntPtr]::Zero)
    if ([AeroSimLabUiTestWindow]::IsWindowEnabled($cameraOffButton)) {
        throw '시뮬레이터가 꺼진 상태에서 지상통제 명령을 시작하지 못했습니다.'
    }

    Start-Sleep -Milliseconds 500
    $recoveredServer = Start-Process -FilePath $Simulator `
        -ArgumentList @('--smoke-test', '--udp-port', "$port") `
        -WorkingDirectory $workingDirectory `
        -WindowStyle Hidden `
        -PassThru
    $recoveryDeadline = [DateTime]::UtcNow.AddSeconds(5)
    while (-not [AeroSimLabUiTestWindow]::IsWindowEnabled($cameraOffButton)) {
        $recoveredServer.Refresh()
        if ($recoveredServer.HasExited) {
            throw "시뮬레이터를 다시 시작하지 못했습니다. 종료 코드: $($recoveredServer.ExitCode)"
        }
        if ([DateTime]::UtcNow -ge $recoveryDeadline) {
            throw '시뮬레이터를 다시 시작한 뒤에도 재시도 명령의 응답이 오지 않았습니다.'
        }
        Start-Sleep -Milliseconds 40
    }
    $recoveredTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    if ($recoveredTelemetry -notmatch '카메라: disabled') {
        throw "재시도한 카메라 끄기 명령의 응답을 확인하지 못했습니다.`n$recoveredTelemetry"
    }

    Write-Output '지상통제 명령과 시뮬레이터 종료 중 재시도·재기동 후 복구를 확인했습니다.'
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
    if ($null -ne $recoveredServer) {
        $recoveredServer.Refresh()
    }
    if ($null -ne $recoveredServer -and -not $recoveredServer.HasExited) {
        Stop-Process -Id $recoveredServer.Id -Force
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
}
