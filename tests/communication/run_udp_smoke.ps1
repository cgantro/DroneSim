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

$server = Start-Process -FilePath $Simulator `
    -ArgumentList @('--smoke-test', '--udp-port', "$port") `
    -WorkingDirectory $workingDirectory `
    -WindowStyle Hidden `
    -PassThru

try {
    Start-Sleep -Milliseconds 200
    $server.Refresh()
    if ($server.HasExited) {
        throw "시뮬레이터가 UDP 시험 서버를 시작하지 못했습니다. 종료 코드: $($server.ExitCode)"
    }

    $replies = @()
    $calls = @(
        @('--port', "$port", '--command', 'takeoff'),
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
    if ($replies[1] -notmatch '카메라: fault' -or $replies[2] -notmatch '카메라: active') {
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
    $server.Refresh()
    if (-not $server.HasExited) {
        Stop-Process -Id $server.Id -Force
    }
}
