# source for self-elevating admin rights https://serverfault.com/questions/11879/gaining-administrator-privileges-in-powershell/12306#12306
#at top of script
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
    Start-Process `
        -FilePath 'powershell' `
        -ArgumentList (
            #flatten to single array
            '-File', $MyInvocation.MyCommand.Source, $args `
            | %{ $_ }
        ) `
        -Verb RunAs
    exit
}

# normal echo won't work
# echo "setting static vEthernet (WSL) ip address..."

# echo wont work as it is a background task, but you could redirect to file and output file content on first shell via bash script
# https://superuser.com/questions/1717816/how-to-run-command-when-starting-a-machine-in-wsl2
# put following line into [boot] /etc/wsl.conf before semicolon;
# echo "adding 172.17.208.1 as wsl ip" > /usr/init.log

netsh interface ip add address "vEthernet (WSL)" 172.17.208.1 255.255.255.0
netsh interface ip add address "vEthernet (WSL (Hyper-V firewall))" 172.17.208.1 255.255.255.0

# to remove ip for testing purposes
# netsh interface ip delete address "vEthernet (WSL)" 172.17.208.1 255.255.255.0
# netsh interface ip add address "vEthernet (WSL (Hyper-V firewall))" 172.17.208.1 255.255.255.0

# start from within wsl
# /mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe <path to powershell script>/wsl_static_ip.ps1
