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
