param(
 [string]$StateDir = (Join-Path $env:USERPROFILE 'Documents\Codex\csc14-local'),
 [switch]$Restart
)
$ErrorActionPreference='Stop'
$telemetryRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$configPath=Join-Path $telemetryRoot 'grafana\grafana.ini'
$grafanaHome=Join-Path $StateDir 'grafana-13.2.5'
$grafanaExe=Join-Path $grafanaHome 'bin\grafana.exe'
if(!(Test-Path -LiteralPath $grafanaExe)){throw 'Local Grafana installation not found.'}
$services=@(Get-CimInstance Win32_Process -Filter "Name='grafana.exe'" | Where-Object {$_.CommandLine -and $_.CommandLine.Contains($configPath)})
if($services.Count -and !$Restart){Write-Output 'Grafana is already running. Use -Restart after configuration changes.';return}
foreach($service in $services){Stop-Process -Id $service.ProcessId}
$env:CSC14_TELEMETRY_DIR=$telemetryRoot
$env:GF_PATHS_DATA=Join-Path $StateDir 'grafana-data'
$env:GF_PATHS_LOGS=Join-Path $StateDir 'grafana-logs'
$env:GF_PATHS_PLUGINS=Join-Path $StateDir 'grafana-data\plugins'
$env:GF_PATHS_PROVISIONING=Join-Path $telemetryRoot 'grafana\provisioning'
$env:GF_SERVER_HTTP_ADDR='127.0.0.1'
$env:GF_SERVER_HTTP_PORT='3000'
$env:GF_AUTH_ANONYMOUS_ENABLED='true'
$env:GF_DASHBOARDS_DEFAULT_HOME_DASHBOARD_PATH=Join-Path $telemetryRoot 'grafana\dashboards\csc14-robot.json'
$launch=@{
 FilePath=$grafanaExe;WindowStyle='Hidden';PassThru=$true;WorkingDirectory=$grafanaHome
 ArgumentList=@('server','--homepath',('"'+$grafanaHome+'"'),'--config',('"'+$configPath+'"'))
 RedirectStandardOutput=(Join-Path $StateDir 'grafana.log');RedirectStandardError=(Join-Path $StateDir 'grafana.err')
}
$process=Start-Process @launch
Write-Output "Grafana started (PID $($process.Id)): http://127.0.0.1:3000/d/csc14-robot?theme=light&kiosk"
