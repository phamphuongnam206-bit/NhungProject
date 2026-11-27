param([ValidateSet('robot_v1','robot_v2')][string]$Profile='robot_v1')
& (Join-Path $PSScriptRoot 'read_status.ps1') -Profile $Profile -Symbol g_robot_fault_snapshot
