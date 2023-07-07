# echo 'reading alias file'


# exports
# manually update px4_sim_host_addr when wsl started and ipconfig on windows with wsl ip address
export PX4_SIM_HOST_ADDR=172.17.208.1
export PATH="$PATH:/home/companion/.local/bin"
export PATH="$PATH:/home/companion/.local/lib/python3.8/site-packages"

export FASTRTPSGEN_DIR="/usr/local/bin/"

# ignore setuptools warning for ros2
export PYTHONWARNINGS="ignore:setup.py install is deprecated,ignore:easy_install command is deprecated"

# setting for avoiding gui errors:
# unset XDG_RUNTIME_DIR

#alias start_px4='cd ~/PX4-Autopilot; make px4_sitl_default none_iris'
alias start_px4='cd ~/PX4-Autopilot; make px4_sitl_rtps none_iris'
alias start_px4_multiple='~/PX4-Autopilot/Tools/sitl_multiple_run.sh 3'

alias start_rtps_holy='cd ~/PX4-Autopilot; make px4_sitl_rtps none_iris_mod'

alias start_rtps_agent='micrortps_agent -t UDP'
alias start_rtps_multiple='~/PX4-Autopilot/multi_drone_scripts/run_multiple_ros2_bridges.sh 3'
alias rebuild_agent='cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts; source build_ros2_workspace.bash; cd ~'

alias start_offboard_control='cd ~/px4_ros_com_ros2;sleep 2; ros2 run px4_ros_com offboard_control'
alias rebuild_offboard='rebuild_agent'

alias start_upf4ros='cd ~/PlanSys; source install/setup.bash; ros2 launch upf4ros2 upf4ros2.launch.py'
alias start_plan_executor='cd ~/PlanSys; source install/setup.bash; sleep 6; ros2 launch upf4ros2_demo traverse_areas.launch.py'

alias start_game_solve="cd ~/PlanSys; ros2 launch upf4ros2_demo roswrap.launch.py"
alias start_pyqgis_window="cd ~/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo; python3 -i qgis_window.py"

alias start_simulation='~/start_upf_simulation.sh'

# source ROS2 environment and also source ROS2 Bridge workspace
alias ros_setup='source /opt/ros/foxy/setup.bash; source ~/px4_ros_com_ros2/install/setup.bash'
ros_setup

# [navigation]

# cd shortcuts
alias cd_win='cd /mnt/c'

# utility shortcut
alias fullpath='echo '

# find all files with partial filename given in parameter
function find_file() {
    echo 'searching for files including: '$1;
    sudo find '/bin' '/home' '/etc' '/usr' -iname *$1*;
}

# for editing this file
alias edit_alias='sudo nano /etc/profile.d/aliases.sh'
alias reload_alias='. /etc/profile.d/aliases.sh'

# set alias for windows utility
alias explorer='/mnt/c/Windows/explorer.exe .'
alias cmd='/mnt/c/Windows/System32/cmd.exe '

# set alias for owning folder and content (in case of permission errors)
alias permission_folder='sudo chown -R companion '
# make a single file (e.g. shell script executable)
alias make_executable='sudo chmod +x '
