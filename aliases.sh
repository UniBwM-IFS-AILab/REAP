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
unset XDG_RUNTIME_DIR

alias start_px4='cd ~/PX4-Autopilot; make px4_sitl_rtps none_iris'
# append number of drones
alias start_px4_multiple='sleep 1; ~/PX4-Autopilot/Tools/simulation/sitl_multiple_run.sh '

alias start_rtps_agent='micrortps_agent -t UDP'
# append number of drones
alias start_rtps_multiple='~/PX4-Autopilot/multi_drone_scripts/run_multiple_ros2_bridges.sh '
alias rebuild_agent='cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts; source build_ros2_workspace.bash; cd ~'

#alias start_offboard_control='cd ~/px4_ros_com_ros2;sleep 2; ros2 run px4_ros_com offboard_control'
# default drone count is 3 but can be set via appending "count:=4"
alias start_offboard_control='cd ~/px4_ros_com_ros2;sleep 2; ros2 launch px4_ros_com offboard_control.launch.py'
alias rebuild_offboard='rebuild_agent'

alias start_upf4ros='cd ~/PlanSys; source install/setup.bash; ros2 launch upf4ros2 upf4ros2.launch.py'
alias start_plan_executor='cd ~/PlanSys; source install/setup.bash; sleep 6; ros2 launch upf4ros2_demo traverse_areas.launch.py count:=1'
# append count:=<number> when calling  alias start_plan_multi
alias start_plan_multi='cd ~/PlanSys; source install/setup.bash; sleep 6; ros2 launch upf4ros2_demo traverse_areas.launch.py '

# build command for Plansys
alias build_colcon='colcon build --symlink-install'

# commands for high level task manager and stochastic game
alias start_game_solve="cd ~/PlanSys; ros2 launch upf4ros2_demo roswrap.launch.py"
alias start_pyqgis_window="cd ~/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo; python3 -i qgis_window.py"

# commands for starting up the whole simulation pipeline
alias start_simulation='~/start_upf_simulation.sh'
alias start_simulation_multi='~/start_upf_simulation_multi.sh'

# source ROS2 environment and also source ROS2 Bridge workspace
alias ros_setup='source /opt/ros/galactic/setup.bash; source ~/px4_ros_com_ros2/install/setup.bash'
ros_setup

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
