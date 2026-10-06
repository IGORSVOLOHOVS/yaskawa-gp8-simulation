param(
    [Parameter(Mandatory = $true, Position = 0)]
    [ValidateSet('build', 'install', 'uninstall', 'clean', 'run', 'publish', 'info', 'test')]
    [string]$Verb,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$RunArgs
)

$Name    = 'waam-manipulator'
$Code    = 'wmp'
$App     = 'YaskawaGP8Solver'
$Version = '1.0.0'
$Summary = 'High-performance C++26 Yaskawa GP8 robot kinematics solver and simulation engine.'
$Needs   = @()
$Tools   = @()

$Entry   = 'cpp_solver/build/benchmark_cpp'
$Console = $true
$Include = @('cpp_solver')
$Imports = @()
$Provides = @()

$CommitTypes  = @('feat', 'fix', 'docs', 'refactor', 'test', 'build', 'ci', 'chore')
$CommitScopes = @('core', 'build', 'ci', 'docs', 'deps')
$BranchTypes  = @('feat', 'fix', 'docs', 'refactor', 'test', 'build', 'ci', 'chore', 'hotfix', 'experiment')
$Reviewers    = @('igorsvolohovs')
$OverridePhrase = '1234567890'

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot

$envFile = Join-Path $PSScriptRoot '.claude\.env'
if (Test-Path $envFile) {
    foreach ($line in Get-Content -Encoding UTF8 $envFile) {
        if ($line -match '^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$' -and
            -not (Test-Path "env:$($Matches[1])")) {
            Set-Item "env:$($Matches[1])" $Matches[2].Trim('"', "'")
        }
    }
}

switch ($Verb) {
    'build' {
        cmake -B "$root/cpp_solver/build" -S "$root/cpp_solver" -DCMAKE_BUILD_TYPE=Release
        cmake --build "$root/cpp_solver/build" -j
        exit $LASTEXITCODE
    }
    'install' {
        mkdir -Force "$root/install/bin"
        cp "$root/cpp_solver/build/benchmark_cpp" "$root/install/bin/"
        cp "$root/cpp_solver/build/test_cpp" "$root/install/bin/"
        exit 0
    }
    'uninstall' {
        Remove-Item -Recurse -Force "$root/install/bin"
        exit 0
    }
    'clean' {
        Remove-Item -Recurse -Force "$root/cpp_solver/build"
        Remove-Item -Recurse -Force "$root/profiling"
        exit 0
    }
    'run' {
        python3 "$root/cpp_solver/web/server.py"
        exit 0
    }
    'publish' {
        Write-Host "Publishing release $Version..."
        exit 0
    }
    'info' {
        "Name    : $Name"
        "Code    : $Code"
        "Version : $Version"
        "Summary : $Summary"
        $installed = (Test-Path "$root/install/bin/benchmark_cpp") -or (Test-Path "$root/install/bin/test_cpp")
        if (-not $installed) {
            "Status  : Not installed"
            exit 1
        }
        "Status  : Installed"
        exit 0
    }
    'test' {
        git -C $root config core.hooksPath .claude/hooks/git
        if ($LASTEXITCODE -ne 0) { exit 1 }

        if (-not (Test-Path "$root/cpp_solver/build/test_cpp")) {
            cmake -B "$root/cpp_solver/build" -S "$root/cpp_solver" -DCMAKE_BUILD_TYPE=Release
            cmake --build "$root/cpp_solver/build" -j
        }
        & "$root/cpp_solver/build/test_cpp"
        exit $LASTEXITCODE
    }
}
