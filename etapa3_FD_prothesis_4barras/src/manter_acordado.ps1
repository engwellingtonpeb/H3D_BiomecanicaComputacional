# Impede a suspensao automatica do Windows enquanto o processo do treino
# (MATLAB) estiver rodando; termina sozinho quando ele termina.
# Nao altera configuracoes de energia (usa SetThreadExecutionState, como
# fazem players de video). Uso: powershell -File manter_acordado.ps1 -Id <PID>
param([int]$Id)
Add-Type -Name P -Namespace W -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint f);'
$ES_CONTINUOUS = [uint32]2147483648; $ES_SYSTEM_REQUIRED = [uint32]1
while (Get-Process -Id $Id -ErrorAction SilentlyContinue) {
    [void][W.P]::SetThreadExecutionState($ES_CONTINUOUS -bor $ES_SYSTEM_REQUIRED)
    Start-Sleep -Seconds 60
}
[void][W.P]::SetThreadExecutionState($ES_CONTINUOUS)
"processo $Id terminou; suspensao liberada"
