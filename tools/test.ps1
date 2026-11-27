$root=Split-Path -Parent $PSScriptRoot
$zigCommand=Get-Command zig -ErrorAction SilentlyContinue
$zig=if($zigCommand){$zigCommand.Source}else{'C:\PIOWork\ziglang_014\ziglang\zig.exe'}
if (!(Test-Path -LiteralPath $zig)) { throw 'Zig host compiler not found; install Zig or pass it on PATH.' }
$out=Join-Path $root '.pio\host'
New-Item -ItemType Directory -Force -Path $out | Out-Null
Push-Location $root
try {
    python tools/check_map.py
    if($LASTEXITCODE -ne 0) { throw 'Map parity failed' }
    $cases=@(
      @('geometry','tests/test_geometry.c','src/static_map.c','src/odometry.c'),
      @('motion','tests/test_motion.c','src/motion_controller.c','src/static_map.c','src/odometry.c'),
      @('speed','tests/test_speed_pi.c','src/speed_pi.c'),
      @('safety','tests/test_safety.c','src/safety_manager.c','src/static_map.c','src/odometry.c'),
      @('fsm','tests/test_fsm.c','src/robot_fsm.c','src/diagnostic_manager.c','src/safety_manager.c','src/static_map.c','src/odometry.c','src/motion_controller.c')
    )
    foreach($case in $cases) {
      $exe=Join-Path $out "$($case[0]).exe"
      & $zig cc -std=c11 -Wall -Wextra -Iinclude $case[1..($case.Length-1)] -lm -o $exe
      if($LASTEXITCODE -ne 0) { throw "Compile failed: $($case[0])" }
      & $exe
      if($LASTEXITCODE -ne 0) { throw "Test failed: $($case[0])" }
    }
}
finally { Pop-Location }
