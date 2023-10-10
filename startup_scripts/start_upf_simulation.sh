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

wsl_profile_name="Dev_Companion"
wsl_instance_name="Dev_Companion"


pushd /mnt/c
$win_path/cmd.exe /c $win_path2/wt.exe -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_px4_multiple 1' \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_rtps_agent' \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_offboard_control count:=1' \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_upf4ros' \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_plan_executor'
popd
