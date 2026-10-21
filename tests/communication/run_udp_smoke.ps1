param(
    [Parameter(Mandatory = $true)][string]$Simulator,
    [Parameter(Mandatory = $true)][string]$GroundControl,
    [Parameter(Mandatory = $true)][string]$GroundControlUi
)

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
    public const int PortInput = 101;

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    public static extern IntPtr FindWindow(string className, string windowName);

    [DllImport("user32.dll")]
    public static extern IntPtr GetDlgItem(IntPtr window, int controlId);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    public static extern bool SetWindowText(IntPtr window, string text);

    [DllImport("user32.dll")]
    public static extern IntPtr SendMessage(IntPtr window, uint message, IntPtr wParam, IntPtr lParam);

    [DllImport("user32.dll")]
    private static extern uint GetWindowThreadProcessId(IntPtr window, out uint processId);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    private static extern int GetWindowTextLength(IntPtr window);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    private static extern int GetWindowText(IntPtr window, StringBuilder text, int capacity);

    [DllImport("user32.dll")]
    public static extern bool PostMessage(IntPtr window, uint message, IntPtr wParam, IntPtr lParam);

    public static uint ProcessId(IntPtr window)
    {
        GetWindowThreadProcessId(window, out uint processId);
        return processId;
    }

    public static string ReadText(IntPtr window)
    {
        var text = new StringBuilder(GetWindowTextLength(window) + 1);
        GetWindowText(window, text, text.Capacity);
        return text.ToString();
    }
}
'@

$port = 31000 + (Get-Random -Minimum 0 -Maximum 18000)
$workingDirectory = Get-Location
$timeoutReply = (& $GroundControl @('--port', "$port", '--command', 'takeoff') 2>&1 | Out-String)
if ($LASTEXITCODE -ne 3 -or $timeoutReply -notmatch '응답 시간 초과') {
    throw "시뮬레이터가 없을 때 지상통제 프로그램이 시간 초과로 끝나지 않았습니다.`n$timeoutReply"
}

$retryingClient = $null
$server = $null
$groundControlUi = $null
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

    $portConflictReply = (& $Simulator @('--smoke-test', '--udp-port', "$port") 2>&1 | Out-String)
    $portConflictExit = $LASTEXITCODE
    if ($portConflictExit -eq 0 -or $portConflictReply -notmatch 'Winsock 오류 10048') {
        throw "사용 중인 UDP 포트 오류를 정확히 알리지 못했습니다. 종료 코드: $portConflictExit`n$portConflictReply"
    }

    $groundControlUi = Start-Process -FilePath $GroundControlUi `
        -WorkingDirectory $workingDirectory `
        -WindowStyle Hidden `
        -PassThru
    $windowDeadline = [DateTime]::UtcNow.AddSeconds(5)
    while ($uiWindow -eq [IntPtr]::Zero -and [DateTime]::UtcNow -lt $windowDeadline) {
        $uiWindow = [AeroSimLabUiTestWindow]::FindWindow(
            'AeroSimLabGroundControlWindow', 'AeroSimLab 지상통제')
        Start-Sleep -Milliseconds 50
    }
    if ($uiWindow -eq [IntPtr]::Zero) {
        throw '지상통제 조작 창을 열지 못했습니다.'
    }
    if ([AeroSimLabUiTestWindow]::ProcessId($uiWindow) -ne $groundControlUi.Id) {
        throw '찾은 조작 창이 방금 실행한 프로그램의 창이 아닙니다.'
    }

    $portInput = [AeroSimLabUiTestWindow]::GetDlgItem(
        $uiWindow, [AeroSimLabUiTestWindow]::PortInput)
    [AeroSimLabUiTestWindow]::SetWindowText($portInput, "$port") | Out-Null

    function Invoke-CameraButton([int]$buttonId, [string]$expectedState) {
        $button = [AeroSimLabUiTestWindow]::GetDlgItem($uiWindow, $buttonId)
        [AeroSimLabUiTestWindow]::SendMessage(
            $uiWindow, 0x0111, [IntPtr]$buttonId, $button) | Out-Null

        $deadline = [DateTime]::UtcNow.AddSeconds(5)
        while ([DateTime]::UtcNow -lt $deadline) {
            $status = [AeroSimLabUiTestWindow]::ReadText(
                [AeroSimLabUiTestWindow]::GetDlgItem(
                    $uiWindow, [AeroSimLabUiTestWindow]::StatusText))
            $telemetry = [AeroSimLabUiTestWindow]::ReadText(
                [AeroSimLabUiTestWindow]::GetDlgItem(
                    $uiWindow, [AeroSimLabUiTestWindow]::TelemetryText))
            if ($status -match '명령이 적용됐습니다' -and $telemetry -match "카메라: $expectedState") {
                return $telemetry
            }
            Start-Sleep -Milliseconds 50
        }
        throw "지상통제 창에서 카메라 상태 '$expectedState' 응답을 받지 못했습니다."
    }

    $faultTelemetry = Invoke-CameraButton `
        ([AeroSimLabUiTestWindow]::CameraFaultButton) 'fault'
    $recoveredTelemetry = Invoke-CameraButton `
        ([AeroSimLabUiTestWindow]::CameraRecoverButton) 'active'

    if (-not $server.WaitForExit(3000)) {
        throw '명령을 받은 UDP 시험 서버가 정상 종료하지 않았습니다.'
    }
    $server.Refresh()
    if ($server.ExitCode -ne 0) {
        throw "시뮬레이터 시험 서버가 오류로 종료했습니다: $($server.ExitCode)"
    }
    Write-Output $faultTelemetry
    Write-Output $recoveredTelemetry
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
    if ($null -ne $groundControlUi) {
        $groundControlUi.Refresh()
        if (-not $groundControlUi.HasExited) {
            Stop-Process -Id $groundControlUi.Id -Force
        }
    }
}
