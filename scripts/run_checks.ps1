$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

python tests/check_utf8.py
$env:PYTHONPATH = $RepoRoot
python tests/test_pid.py

Write-Host "Release checks passed."
