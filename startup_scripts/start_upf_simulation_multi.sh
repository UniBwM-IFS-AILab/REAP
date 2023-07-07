#!/bin/bash -i

exec > /dev/null 2>&1


win_path="/mnt/c/Windows/System32/"
win_path2="C:/Users/%USERNAME%/AppData/Local/Microsoft/WindowsApps"

wsl_profile_name="Dev_Companion"
wsl_instance_name="Dev_Companion"


pushd /mnt/c
$win_path/cmd.exe /c $win_path2/wt.exe -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_px4_multiple' \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_rtps_multiple' \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_offboard_control' \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_upf4ros' \; new-tab -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c 'start_plan_executor'
popd