param(
    [Parameter(Mandatory = $true)]
    [string]$LaptopHost,

    [Parameter(Mandatory = $true)]
    [string]$LaptopUser,

    [string]$Branch = ""
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($Branch)) {
    $Branch = (git branch --show-current).Trim()
}
if ([string]::IsNullOrWhiteSpace($Branch)) {
    throw "Unable to determine the current Git branch."
}

Write-Host "Pushing branch '$Branch' directly to the Ubuntu laptop..."
git push lab "HEAD:refs/heads/$Branch"

$Target = "$LaptopUser@$LaptopHost"
$RemoteCommand = @"
set -e
cd ~/cr5_a4_ros1
git fetch origin
if git show-ref --verify --quiet refs/heads/$Branch; then
  git switch $Branch
else
  git switch -c $Branch --track origin/$Branch
fi
git reset --hard origin/$Branch
printf 'Laptop checkout updated to %s\n' \"`$(git rev-parse --short HEAD)\"
"@

ssh $Target $RemoteCommand
Write-Host "Deployment complete. Compile or launch through VS Code Remote-SSH or SSH."
