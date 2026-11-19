# This file only builds and runs your desktop program. Robot code belongs in src.
$ErrorActionPreference = 'Stop'

$lessonCompiler = 'C:\msys64\mingw64\bin\g++.exe'
$lessonSourceDir = Join-Path $PSScriptRoot 'src'
$lessonIncludeDir = Join-Path $PSScriptRoot 'include'
$lessonBuildDir = Join-Path $PSScriptRoot 'build'
$lessonExecutable = Join-Path $lessonBuildDir 'locomotion_tests.exe'

if (-not (Test-Path -LiteralPath $lessonCompiler -PathType Leaf)) {
    throw "Compiler not found: $lessonCompiler"
}

# Discover every .cpp below src, including module subfolders with spaces.
# Only one of these files should define main(). Headers are included by sources.
$lessonSources = @(Get-ChildItem -LiteralPath $lessonSourceDir -Recurse -File -Filter '*.cpp' |
    Sort-Object FullName | ForEach-Object { $_.FullName })
if ($lessonSources.Count -eq 0) {
    throw 'No .cpp files found inside src.'
}

New-Item -ItemType Directory -Path $lessonBuildDir -Force | Out-Null
$lessonArguments = @(
    '-std=c++11', '-Wall', '-Wextra', '-g',
    '-static-libgcc', '-static-libstdc++',
    '-I', $lessonSourceDir,
    '-o', $lessonExecutable
)
if (Test-Path -LiteralPath $lessonIncludeDir -PathType Container) {
    $lessonArguments += @('-I', $lessonIncludeDir)
}
$lessonArguments += $lessonSources

& $lessonCompiler @lessonArguments
if ($LASTEXITCODE -ne 0) {
    # Do not run an older executable when the new source fails to compile.
    exit $LASTEXITCODE
}

& $lessonExecutable
exit $LASTEXITCODE

# Press Ctrl + Shift + P.
# Search for Tasks: Run Task and select it.
# Select Build and run ServoCalibration test.