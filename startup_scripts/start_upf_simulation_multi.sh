#!/bin/bash -i

exec > /dev/null 2>&1


win_path="/mnt/c/Windows/System32/"
win_path2="C:/Users/%USERNAME%/AppData/Local/Microsoft/WindowsApps"

wsl_profile_name="Dev_Companion"
wsl_instance_name="Dev_Companion"

drone_count=3


pushd /mnt/c
$win_path/cmd.exe /c $win_path2/wt.exe -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_px4_multiple '$drone_count \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_rtps_agent' \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_offboard_control count:='$drone_count \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_upf4ros' \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_plan_multi count:='$drone_count
popd
