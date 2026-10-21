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
    [void][AeroSimLabUiTestWindow]::SendMessage(
        $button, 0x00F5, [IntPtr]::Zero, [IntPtr]::Zero)
    if ([AeroSimLabUiTestWindow]::IsWindowEnabled($button)) {
        throw "지상통제 명령($ControlId)이 시작되지 않았습니다."
    }

    $deadline = [DateTime]::UtcNow.AddSeconds(5)
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

    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::CameraFaultButton)
    $telemetryText = [AeroSimLabUiTestWindow]::GetDlgItem(
        $uiWindow, [AeroSimLabUiTestWindow]::TelemetryText)
    $faultTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    if ($faultTelemetry -notmatch '카메라: fault') {
        throw "지상통제 창에서 보낸 카메라 고장 명령이 시뮬레이터 상태에 반영되지 않았습니다.`n$faultTelemetry"
    }
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::CameraRecoverButton)
    Start-Sleep -Milliseconds 1800
    Invoke-GroundControlButton ([AeroSimLabUiTestWindow]::LandButton)
    $landingTelemetry = [AeroSimLabUiTestWindow]::ReadControlText($telemetryText)
    if ($landingTelemetry -notmatch '비행 상태: landing' -or $landingTelemetry -notmatch '카메라: active') {
        throw "지상통제 창의 착륙·카메라 복구 명령이 시뮬레이터 상태에 반영되지 않았습니다.`n$landingTelemetry"
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

    $statusText = [AeroSimLabUiTestWindow]::GetDlgItem(
        $uiWindow, [AeroSimLabUiTestWindow]::StatusText)
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

    Write-Output '지상통제 창의 비행·카메라·기록 명령과 시뮬레이터 상태를 확인했습니다.'
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
}
