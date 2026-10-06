$branch = (git rev-parse --abbrev-ref HEAD).Trim()

if ($branch -cmatch '^release$') {
    $commitMsg = (git log -1 --pretty=format:"%B").Trim()
    if ($commitMsg -notmatch '1234567890' -or $commitMsg -notmatch 'Why:') {
        Write-Host "ERROR: Direct push to release branch requires owner override phrase '1234567890' and 'Why: <reason>' in commit body." -ForegroundColor Red
        exit 1
    }
    exit 0
}

if ($branch -cmatch '^test$' -or $branch -cmatch '^from-') {
    exit 0
}

$pattern = '^wmp-[0-9]+/(feat|fix|docs|refactor|test|build|ci|chore|hotfix|experiment)/[a-z0-9_-]+$'
if ($branch -cnotmatch $pattern) {
    Write-Host "ERROR: Branch '$branch' must follow format: wmp-<issue>/<type>/<description>" -ForegroundColor Red
    exit 1
}

exit 0
