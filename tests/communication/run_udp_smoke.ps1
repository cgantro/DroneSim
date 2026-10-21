param(
    [Parameter(Mandatory = $true)][string]$Simulator,
    [Parameter(Mandatory = $true)][string]$GroundControl
)

$port = 31000 + (Get-Random -Minimum 0 -Maximum 18000)
$workingDirectory = Get-Location
$timeoutReply = (& $GroundControl @('--port', "$port", '--command', 'takeoff') 2>&1 | Out-String)
if ($LASTEXITCODE -ne 3 -or $timeoutReply -notmatch '응답 시간 초과') {
    throw "시뮬레이터가 없을 때 지상통제 프로그램이 시간 초과로 끝나지 않았습니다.`n$timeoutReply"
}

$retryingClient = $null
$server = $null

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

    $replies = @()
    $calls = @(
        @('--port', "$port", '--camera', 'fault'),
        @('--port', "$port", '--camera', 'recover')
    )
    foreach ($arguments in $calls) {
        $reply = (& $GroundControl @arguments 2>&1 | Out-String)
        $clientExit = $LASTEXITCODE
        if ($clientExit -ne 0 -or $reply -notmatch 'applied' -or $reply -notmatch '적용 Tick') {
            throw "지상통제 명령이 적용되지 않았습니다. 종료 코드: $clientExit`n$reply"
        }
        $replies += $reply
    }
    if ($replies[0] -notmatch '카메라: fault' -or $replies[1] -notmatch '카메라: active') {
        throw "카메라 장애와 복구 상태가 응답에 반영되지 않았습니다.`n$($replies -join "`n")"
    }

    if (-not $server.WaitForExit(3000)) {
        throw '명령을 받은 UDP 시험 서버가 정상 종료하지 않았습니다.'
    }
    $server.Refresh()
    if ($server.ExitCode -ne 0) {
        throw "시뮬레이터 시험 서버가 오류로 종료했습니다: $($server.ExitCode)"
    }
    Write-Output ($replies -join "`n")
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
}
