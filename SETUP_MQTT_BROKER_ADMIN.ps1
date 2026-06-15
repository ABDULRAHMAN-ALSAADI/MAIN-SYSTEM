$ErrorActionPreference = "Stop"

$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator
)
if (-not $isAdmin) {
    throw "Run this script from PowerShell as Administrator."
}

$configPath = "C:\Program Files\mosquitto\mosquitto.conf"
$serviceName = "mosquitto"
$ruleName = "Robot Cell MQTT 1883"
$beginMarker = "# BEGIN ROBOT CELL CONTROL CENTER"
$endMarker = "# END ROBOT CELL CONTROL CENTER"

if (-not (Test-Path -LiteralPath $configPath)) {
    throw "Mosquitto configuration was not found at $configPath"
}

$hotspotIp = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction Stop |
    Where-Object { $_.IPAddress -eq "192.168.137.1" } |
    Select-Object -First 1
if (-not $hotspotIp) {
    throw "Windows Mobile Hotspot IP 192.168.137.1 is not active."
}

$backupPath = "$configPath.robot-cell-backup"
if (-not (Test-Path -LiteralPath $backupPath)) {
    Copy-Item -LiteralPath $configPath -Destination $backupPath
}

$content = Get-Content -LiteralPath $configPath -Raw
$escapedBegin = [regex]::Escape($beginMarker)
$escapedEnd = [regex]::Escape($endMarker)
$content = [regex]::Replace(
    $content,
    "(?ms)\r?\n?$escapedBegin.*?$escapedEnd\r?\n?",
    ""
).TrimEnd()

$managedBlock = @"

$beginMarker
listener 1883 0.0.0.0
allow_anonymous true
$endMarker
"@

Set-Content -LiteralPath $configPath -Value ($content + $managedBlock + [Environment]::NewLine) -Encoding ascii

Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue | Remove-NetFirewallRule
New-NetFirewallRule `
    -DisplayName $ruleName `
    -Direction Inbound `
    -Action Allow `
    -Protocol TCP `
    -LocalPort 1883 `
    -RemoteAddress "192.168.137.0/24" `
    -Profile Any | Out-Null

Restart-Service -Name $serviceName
Start-Sleep -Seconds 2

$listeners = Get-NetTCPConnection -State Listen -LocalPort 1883 |
    Select-Object LocalAddress, LocalPort, OwningProcess
$localOk = Test-NetConnection -ComputerName 127.0.0.1 -Port 1883 -InformationLevel Quiet
$hotspotOk = Test-NetConnection -ComputerName 192.168.137.1 -Port 1883 -InformationLevel Quiet

$listeners | Format-Table -AutoSize
Write-Host "Local MQTT 127.0.0.1:1883 reachable: $localOk"
Write-Host "Robot MQTT 192.168.137.1:1883 reachable: $hotspotOk"

if (-not ($localOk -and $hotspotOk)) {
    throw "Mosquitto restart completed, but one or more MQTT listener tests failed."
}

Write-Host "MQTT broker setup complete. ESP32 devices should reconnect automatically."
