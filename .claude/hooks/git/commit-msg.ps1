param([string]$MsgFile)
$msg = [System.IO.File]::ReadAllText($MsgFile).Trim()
$pattern = '^(feat|fix|docs|refactor|test|build|ci|chore)(\([a-z0-9_-]+\))?: .+'
if ($msg -cnotmatch $pattern) {
    Write-Host "ERROR: Commit subject '$msg' does not follow conventional commit format: <type>(<scope>): <message>" -ForegroundColor Red
    exit 1
}
exit 0
