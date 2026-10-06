$branch = (git rev-parse --abbrev-ref HEAD).Trim()
if ($branch -cmatch '^(release|test)$' -or $branch -cmatch '^from-') {
    exit 0
}
$pattern = '^wmp-[0-9]+/(feat|fix|docs|refactor|test|build|ci|chore|hotfix|experiment)/[a-z0-9_-]+$'
if ($branch -cnotmatch $pattern) {
    Write-Host "ERROR: Branch '$branch' must follow format: wmp-<issue>/<type>/<description>" -ForegroundColor Red
    exit 1
}
exit 0
