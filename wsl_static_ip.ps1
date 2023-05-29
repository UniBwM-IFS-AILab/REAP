echo "setting static vEthernet (WSL) ip address..."

netsh interface ip add address "vEthernet (WSL)" 172.17.208.1 255.255.255.0

# to remove ip for testing purposes
# netsh interface ip delete address "vEthernet (WSL)" 172.17.208.1 255.255.255.0

# start from within wsl
# /mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe <path to powershell script>/wsl_static_ip.ps1
