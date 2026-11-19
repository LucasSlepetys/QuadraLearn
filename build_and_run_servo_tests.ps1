# Build the isolated calibration, mapping, CAD reference and FK desktop tests.
$ErrorActionPreference = 'Stop'

$servoCompiler = 'C:\msys64\mingw64\bin\g++.exe'
$servoSourceDir = Join-Path $PSScriptRoot 'src'
$servoBuildDir = Join-Path $PSScriptRoot 'build'
$servoExecutable = Join-Path $servoBuildDir 'servo_calibration_tests.exe'

if (-not (Test-Path -LiteralPath $servoCompiler -PathType Leaf)) {
    throw "Compiler not found: $servoCompiler"
}

New-Item -ItemType Directory -Path $servoBuildDir -Force | Out-Null
$servoArguments = @(
    '-std=c++11', '-Wall', '-Wextra', '-g',
    '-static-libgcc', '-static-libstdc++',
    '-I', $servoSourceDir,
    (Join-Path $servoSourceDir 'ServoCalibration\servo_calibration.cpp'),
    (Join-Path $servoSourceDir 'JointMapping\joint_mapping.cpp'),
    (Join-Path $servoSourceDir 'Configuration\servo_configuration.cpp'),
    (Join-Path $servoSourceDir 'Configuration\robot_model.cpp'),
    (Join-Path $servoSourceDir 'Configuration\chosen_configurations.cpp'),
    (Join-Path $servoSourceDir 'ForwardKinematics\forward_kinematics.cpp'),
    (Join-Path $servoSourceDir 'CoordinateTransform\coordinate_transform.cpp'),
    (Join-Path $PSScriptRoot 'tests\servo_mapping_test.cpp'),
    '-o', $servoExecutable
)

& $servoCompiler @servoArguments
if ($LASTEXITCODE -ne 0) {
    # Do not run an older executable when compilation fails.
    exit $LASTEXITCODE
}

# Absolute path lets the test run from any terminal folder.
& $servoExecutable (Join-Path $PSScriptRoot 'tests\data\servo_calibration_reference.csv') `
                   (Join-Path $PSScriptRoot 'tests\data\cad_joint_references.csv') `
                   (Join-Path $PSScriptRoot 'tests\data\cad_home_points.csv')
exit $LASTEXITCODE

# control + shift + p
