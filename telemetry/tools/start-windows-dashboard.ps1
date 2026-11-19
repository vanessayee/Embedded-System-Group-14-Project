param(
    [string]$StateDir = (Join-Path $env:USERPROFILE 'Documents\Codex\csc14-local'),
    [switch]$Restart
)
$ErrorActionPreference = 'Stop'
$telemetryRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$settings = Join-Path $telemetryRoot 'nodered\settings.js'
$runtime = Join-Path $StateDir 'group14'
$nodeModules = Join-Path $StateDir 'deps\node_modules'
$nodeRed = Join-Path $nodeModules 'node-red\red.js'
$dbFile = Join-Path $StateDir 'db-local.xml'
if (!(Test-Path -LiteralPath $nodeRed) -or !(Test-Path -LiteralPath $dbFile)) {
    throw 'The existing local Node-RED dependencies and database credentials are required in StateDir.'
}
$services = @(Get-CimInstance Win32_Process -Filter "Name='node.exe'" |
    Where-Object { $_.CommandLine -and $_.CommandLine.Contains($settings) -and $_.CommandLine.Contains('node-red') })
if ($services.Count -and !$Restart) {
    Write-Output 'Dashboard service already running. Use -Restart after code changes.'
    return
}
foreach ($service in $services) { Stop-Process -Id $service.ProcessId }
$db = Import-Clixml -LiteralPath $dbFile
$env:NODE_PATH = $nodeModules
$env:CSC14_RUNTIME_DIR = $runtime
$env:CSC14_STATE_FILE = Join-Path $runtime 'nodered\ingest-state.json'
$env:CSC14_PG_HOST = '127.0.0.1'
$env:CSC14_PG_PORT = '55432'
$env:CSC14_PG_DATABASE = 'csc14_telemetry'
$env:CSC14_PG_WRITER_USER = 'csc14_writer'
$env:CSC14_PG_WRITER_PASSWORD = $db.writer
$env:CSC14_PG_READER_USER = 'csc14_reader'
$env:CSC14_PG_READER_PASSWORD = $db.reader
$env:CSC14_PG_ADMIN_USER = 'csc14_admin'
$env:CSC14_MQTT_HOST = '127.0.0.1'
$env:CSC14_MQTT_PORT = '1883'
$env:CSC14_ALLOW_MOTION_COMMANDS = '1'
$env:CSC14_NODERED_HOST = '127.0.0.1'
$env:CSC14_NODERED_PORT = '1880'
New-Item -ItemType Directory -Path (Join-Path $runtime 'nodered') -Force | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
foreach ($name in @('nodered.log','nodered.err')) {
    $logPath = Join-Path $runtime $name
    if (Test-Path -LiteralPath $logPath) {
        Copy-Item -LiteralPath $logPath -Destination (Join-Path $runtime "$stamp-$name")
    }
}
$nodeExe = (Get-Command node.exe).Source
$launch = @{
    FilePath = $nodeExe
    WindowStyle = 'Hidden'
    PassThru = $true
    WorkingDirectory = $telemetryRoot
    ArgumentList = @('"' + $nodeRed + '"','--settings','"' + $settings + '"','--userDir','"' + (Join-Path $runtime 'nodered') + '"')
    RedirectStandardOutput = (Join-Path $runtime 'nodered.log')
    RedirectStandardError = (Join-Path $runtime 'nodered.err')
}
$process = Start-Process @launch
Set-Content -LiteralPath (Join-Path $runtime 'nodered.pid') -Value $process.Id
Write-Output "Dashboard started (PID $($process.Id)): http://127.0.0.1:1880/csc14/dashboard"
