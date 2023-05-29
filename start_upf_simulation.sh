#!/bin/bash -i
# bash script for opening multiple terminal windows and starting the Simulation pipeline
# infos from https://learn.microsoft.com/en-us/windows/terminal/command-line-arguments?tabs=linux
# und commandos von https://github.com/microsoft/terminal/issues/7452
# auch hilfreich https://superuser.com/questions/1704004/how-to-open-multiple-tabs-and-run-ssh-from-the-command-line-in-windows-ternimal
# https://superuser.com/questions/1762286/windows-terminal-run-a-command-when-starting-wsl-and-keeping-the-shell-open
# https://stackoverflow.com/questions/72778785/keep-windows-terminal-tab-open-after-invoked-wsl-command


#### Explanation!!!
# https://stackoverflow.com/questions/66048183/what-is-the-difference-between-calling-a-command-via-wsl-command-and-opening
# https://stackoverflow.com/questions/44382433/why-bash-alias-doesnt-work-in-scripts

exec > /dev/null 2>&1

win_path="/mnt/c/Windows/System32/"
win_path2="C:/Users/%USERNAME%/AppData/Local/Microsoft/WindowsApps"

# $win_path/cmd.exe /c $win_path2/wt.exe -p "Companion Bash" -- wsl start_rtps \; new-tab -p "Companion Bash" -- wsl start_agent \; new-tab -p "Companion Bash" -- wsl start_offboard \; new-tab -p "Companion Bash" -- wsl start_upf4ros \; new-tab -p "Companion Bash" -- wsl start_upf_demo

# $win_path/cmd.exe /c $win_path2/wt.exe -p "Companion Bash" -- wsl bash -ic start_rtps \; new-tab -p "Companion Bash" -- wsl bash -ic start_agent \; new-tab -p "Companion Bash" -- wsl bash -ic start_offboard \; new-tab -p "Companion Bash" -- wsl bash -ic start_upf4ros \; new-tab -p "Companion Bash" -- wsl bash -ic start_upf_demo

# $win_path/cmd.exe /c $win_path2/wt.exe -p "Companion Bash" -- wsl bash --rcfile ~/.bashrc -ic start_rtps \; new-tab -p "Companion Bash" -- wsl bash --rcfile ~/.bashrc -ic start_agent \; new-tab -p "Companion Bash" -- wsl bash --rcfile ~/.bashrc -ic start_offboard \; new-tab -p "Companion Bash" -- wsl bash --rcfile ~/.bashrc -ic start_upf4ros \; new-tab -p "Companion Bash" -- wsl bash --rcfile ~/.bashrc -ic start_upf_demo

# $win_path/cmd.exe /c $win_path2/wt.exe -p "Companion Bash" wsl -e bash -li -c 'shopt -s expand_aliases\;start_rtps\; exec $BASH' \; new-tab -p "Companion Bash" wsl -e bash -li -c 'shopt -s expand_aliases\;start_agent\; exec $BASH' \; new-tab -p "Companion Bash" wsl -e bash -li -c 'shopt -s expand_aliases\;start_offboard\; exec $BASH' \; new-tab -p "Companion Bash" wsl -e bash -li -c 'shopt -s expand_aliases\;start_upf4ros\; exec $BASH' \; new-tab -p "Companion Bash" wsl -e bash -li -c 'shopt -s expand_aliases\;start_upf_demo\; exec $BASH'

pushd /mnt/c
$win_path/cmd.exe /c $win_path2/wt.exe -p "Companion" wsl -d Companion -e bash -li -c 'start_px4' \; new-tab -p "Companion" wsl -d Companion -e bash -li -c 'start_rtps_agent' \; new-tab -p "Companion" wsl -d Companion -e bash -li -c 'start_offboard_control' \; new-tab -p "Companion" wsl -d Companion -e bash -li -c 'start_upf4ros' \; new-tab -p "Companion" wsl -e bash -li -c 'start_plan_executor'
popd