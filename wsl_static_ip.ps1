# only elevate to admin rights, if preset ip address is not already set. Otherwise just exit the script
$presetIpAddress = "172.17.208.1"
$scriptPath = $PSScriptRoot + "\wsl_static_ip.ps1"

$adapterInfo = netsh interface ip show address "vEthernet (WSL (Hyper-V firewall))" | findstr "IP Address"
$adapterInfoOld = netsh interface ip show address "vEthernet (WSL)" | findstr "IP Address"

$ipAddresses = @()

$adapterInfo | ForEach-Object {
    $ipAddress = $_ -replace '.*\b(\d+\.\d+\.\d+\.\d+)\b.*', '$1'
    $ipAddresses += $ipAddress
}
$adapterInfoOld | ForEach-Object {
    $ipAddress = $_ -replace '.*\b(\d+\.\d+\.\d+\.\d+)\b.*', '$1'
    $ipAddresses += $ipAddress
}

$ipMatched = $ipAddresses -contains $presetIpAddress

if ($ipMatched) {
    Write-Host "Preset IP address matched in the list of IP addresses. Exiting PowerShell."
    exit
}

#foreach ($ip in $ipAddresses) {
#    Write-Host "Found IP Address: $ip"
#}



# source for self-elevating admin rights https://serverfault.com/questions/11879/gaining-administrator-privileges-in-powershell/12306#12306
if (!
    #current role
    (New-Object Security.Principal.WindowsPrincipal(
        [Security.Principal.WindowsIdentity]::GetCurrent()
    #is admin?
    )).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator
    )
) {
    #elevate script and exit current non-elevated runtime
    Start-Process -FilePath 'powershell' -ArgumentList "-ExecutionPolicy Bypass -File $scriptPath" -Verb RunAs
    exit
}

# program, this will be ran as admin

echo "setting static vEthernet (WSL (Hyper-V firewall)) ip address..."


netsh interface ip add address "vEthernet (WSL)" 172.17.208.1 255.255.255.0
netsh interface ip add address "vEthernet (WSL (Hyper-V firewall))" 172.17.208.1 255.255.255.0

# to remove ip for testing purposes
# netsh interface ip delete address "vEthernet (WSL)" 172.17.208.1 255.255.255.0
# netsh interface ip delete address "vEthernet (WSL (Hyper-V firewall))" 172.17.208.1 255.255.255.0
