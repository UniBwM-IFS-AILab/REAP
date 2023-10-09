# REAP-Framework

![image](https://user-images.githubusercontent.com/92592126/235185659-75937f07-1976-4b61-9ffa-43ad95e4c472.png)


This repository contains installation information for the REAP framework as described in the [ICAPS 2023 Demo paper](https://icaps23.icaps-conference.org/program/demos/#3216).<br />
A demo video provides an overview of the framework: https://www.youtube.com/watch?v=QtMjnMD5zzE.

To cite **REAP**, please use the following reference: <br />
Oliver Kraus, Lucas Mair, Jane Jean Kiam. "REAP: A Flexible Real-World Simulation Framework for AI-Planning of UAVs". In: ICAPS 2023 System Demonstrations.<br />
```
@conference{reap2023icaps,
    title = "REAP: A Flexible Real-World Simulation Framework for AI-Planning of UAVs",
    author = "Kraus, Oliver and Mair, Lucas and Kiam, {Jane Jean}",
    booktitle = {ICAPS 2023 System Demonstrations},
    year = "2023",
    month = jul
}
```



  * [System Requirements](#system-requirements)
  * [Installation with Tarball](#installation-with-tarball)
    + [Install Unreal Engine 4 with the AirSim Plugin](#install-unreal-engine-4-with-the-airsim-plugin)
    + [Create an UE4 project with imported LIDAR files](#create-an-ue4-project-with-imported-lidar-files)
    + [Import WSL2 Tarball](#import-wsl2-tarball)
    + [Allow Incoming Connections in Windows](#allow-incoming-connections-in-windows)
    + [Usage](#usage)
  * [Manual Installation (without Tarball)](#manual-installation-without-tarball)
    + [Setup of the AI-Planning Component](#setup-of-the-ai-planning-component)
    + [Setup of the Validation & Visualization Component](#setup-of-the-validation-and-visualization-component)
    + [Setup of the Ground Control Software](#setup-of-the-ground-control-software)
    + [Setup of the Environment Manipulation](#setup-of-the-environment-manipulation)
  * [Multidrone Simulation](#multidrone-simulation)
  * [Replanning](#replanning)
  * [Editing the Simulation Environment](#editing-the-simulation-environment)
  * [Roadmap](#roadmap)
  * [Contact Information](#contact-information)

You can quickly set up the framework with a provided tarball as descibed in [Installation with Tarball](#installation-with-tarball). For full installation details see the [Manual Installation (without Tarball)](#manual-installation-without-tarball). You can find information on how to customize the framework to your needs under [Editing the Simulation Environment](#editing-the-simulation-environment).

Below you can find an overview of the system architecture. The provided tarball contains all modules, except the "Simulator" module and the AirSim API from the "Environment Manipulation" module. QGroundControl and the Unreal Remote Control API can run either under Windows or the WSL instance if the IP settings are adjusted accordingly.

![Step11](https://github.com/UniBwM-IFS-AILab/REAP/assets/10440274/b409af9b-4e6e-4133-9120-26ca70d94665)

## System requirements
We tested REAP using the following development environment. Deviation may not necessary cripple the system, but we cannot guarantee that it will work.
  - Windows 11
  - WSL2 instance (Ubuntu 20.04) (Installation instructions here: https://learn.microsoft.com/en-us/windows/wsl/install)
  - ROS2 galactic installed on the same WSL2 Ubuntu instance (Installation instructions here: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

## Installation with Tarball



### Install Unreal Engine 4 with the AirSim Plugin
Follow the instructions under: https://microsoft.github.io/AirSim/build_windows/.

Furthermore, AirSim requires a settings file to operate (should be located under "%USERPROFILE%/Documents/AirSim"). Exchange the default settings.json file with the settings file provided in this repository. Alternative settings configurations can be found in the "AirSim_alternative_settings" directory. For those, just replace the content of the settings.json file with the chosen alternative. For detailed information about AirSim settings see: https://microsoft.github.io/AirSim/settings/.

### Create an UE4 project with imported LIDAR files
We recommend creating a custom unreal environment for importing LIDAR files. The general required steps are detailed here: https://microsoft.github.io/AirSim/unreal_custenv/. However instead of using a marketplace project, you should create a new & empty project from scratch and follow the steps as if it were a marketplace project.

Instructions on how to import LIDAR files into UE4 can be found here: https://docs.unrealengine.com/4.26/en-US/WorkingWithContent/LidarPointCloudPlugin/LidarPointCloudPluginQuickstartGuide/.
Sample data can be downloaded from here (choose LAS): https://www.ldbv.bayern.de/service/testdaten.html -> DOM40 (with color).

Make sure the `"OriginGeopoint"` field of settings.json corresponds to the real GPS coordinates of the `Player Start` Object within the imported LIDAR environment and rotate the point cloud in such a way that north aligns with the x-Axis within Unreal Engine. Google Maps can be helpful in making sure the GPS coordinates of the simulated Environment correspond to the real world.

### Import WSL2 Tarball
> **⚠ Info** Due to the size of the tarball, please write to us if you intend to download and install using the WSL2 Tarball. We will then send you a temporary link for direct download.

Download the tarball. This tarball contains a WSL2 instance with all required modules installed.
Install the downloaded tarball with:
```
wsl --import Dev_Companion <InstallLocation> <InstallTarFile.tar>
```
For more information see: https://learn.microsoft.com/en-us/windows/wsl/use-custom-distro.

**_NOTE:_**  The password for the imported WSL2 is: "dronesim".
> After importing the tarball, execute the following commands to prevent pushing to git under wrong user name:
```
git config --global --unset-all user.name
git config --global --unset-all user.email
```

The preconfigured WSL2 instance from the tarball makes use of aliases that can be edited via the custom linux alias `edit_alias` and after changing them updated via `reload_alias`.


### Allow Incoming Connections in Windows
Incoming TCP port 4560 and incoming UDP port 14540 are required for connecting PX4 running in WSL to the unreal simulation. The ports can be opened using the firewall configuration (run WF.msc -> left click inbound rules -> action: new rule).

As the IP address of `ipconfig` -> "Ethernet adapter vEthernet (WSL)" might change after restarting the Windows machine, it can be useful to set up a static address via a powershell script - provided in this repository - that gets executed on WSL startup. For this to work, the **Windows Terminal** from which the WSL instance is started needs to be run **as admin**.
In addition the following lines have to be added to /etc/wsl.conf within the WSL Ubuntu instance. Dont forget to adjust the path with your own location of the powershell script.
```
[boot]
command="/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe -executionpolicy bypass 'C:/<path-to-script>/wsl_static_ip.ps1'"
```
To test if the powershell script works, execute `wsl --shutdown`, wait a few seconds, then restart the WSL instance. `ipconfig` should now show a second ip address for "vEthernet (WSL)". Sometimes you also have to open a new terminal tab before running `ipconfig` as it might have cached the previous results.

If no static ip address is used, the AirSim settings.json `"LocalHostIp"` field has to be manually changed after each Windows reboot with the new WSL address.
Additional information can be found here: https://microsoft.github.io/AirSim/px4_sitl_wsl2/.


### Usage
  - Start the Unreal Engine with an imported LIDAR file. Make sure to use the LIDAR file given under [Create an UE4 project with imported LIDAR files](#create-an-ue4-project-with-imported-lidar-files) so everything works out-of-the-box.
 - Start the imported WSL2 instance. In the home folder there should be a shell script called "start_upf_simulation.sh". Make sure that the name of your imported WSL2 instance matches the configured name in the shellscript (in our case `Dev_Companion`; if not sure, print the name with `wsl --list` in the Windows Terminal). Execute this script and the simulated drone in UE4 should take off and execute the plan. If you wish to customize the LIDAR environment and the executed plan, see [Editing the Simulation Environment](#editing-the-simulation-environment).


## Manual Installation (without Tarball)
The following steps are meant for a manual setup of the REAP framework without the tarball. They also provide more detailed insights into individual modules within the framework.

Follow the instructions under [Installation with Tarball](#installation-with-tarball) up until the section [Import WSL2 Tarball](#import-wsl2-tarball) (i.e. install Unreal Engine 4 with the AirSim plugin). In the following, the **manual** setup of the remaining modules (see overview of the system architecture above) is described. First, setup a [WSL2 Ubuntu instance with ROS2](#system-requirements). Next, [allow incoming connections in Windows](#allow-incoming-connections-in-windows) and then follow the steps bellow.

### Setup of the AI-Planning Component
**PlanSys2** provides a framework for translating symbolic actions from generated AI plans into continuous ROS2 commands and is the central part of the AI-planning component. Follow the instructions under: https://plansys2.github.io/getting_started/index.html to install PlanSys2 on your **WSL2 instance** (follow installation instructions under "Getting Started" and run the example code to verify everything works correctly). If the main branch of PlanSys2 doesn't work you can use the 'galactic-devel' branch.
> **⚠ Info**
> You might need to install the following packages so PlanSys2 works as intended
>
> ``` sudo apt install ros-galactic-nav2-msgs ros-galactic-test-msgs ```
>
> After running `colcon build` as given in the installation instructions, you need to run `source install/setup.bash`





With PlanSys2 installed, we now install the **UPF4ROS2** plugin. This plugin allows usage of the Unified Planning Framework within PlanSys2. Information on the Unified Planning Framework can be found here: https://upf.readthedocs.io/en/latest/getting_started/introduction.html. Basically, it provides a Python wrapper for multiple planning engines.

Clone  UPF4ROS2 from: https://github.com/UniBwM-IFS-AILab/UPF4ROS2 into \<PlanSys2 Folder\>/src/ and follow the installation instructions given in the README.md.

Lastly, we describe the integration of **shapefiles** into the planning component. Shapefiles contain geo-referenced polygons which can be used to describe geographic features of an area. In our case, we used shapefiles that describe the land use of an area (e.g. "forest", "lake", "farmland", etc.). These shapefiles can be automatically processed to generate planning problems using the Planning Domain
Definition Language (PDDL). Thus, a flight mission can be automatically generated (e.g., “Explore all ’woods’ in an area”). For sample data, download "ATKIS® Basis-DLM - Download - Komplettdatensatz SHAPE" from https://geodaten.bayern.de/opengeodata/OpenDataDetail.html?pn=atkis_basis_dlm. The preprocessing steps for the shapefile depend, of course, on the file used and the intended use case. For reference, you can take a look at the scripts uploaded in the "**shapefile_preprocessing**" folder in this repository. The **geopandas** library allows preprocessing of geo-referenced data in Python. The general workflow is: 
 - read in shapefile and group the geo-referenced polygons into categories (e.g. "lakes" and "ponds" are grouped into the category "waters"). In our case, we group the polygons into the categories: "urban areas", "waters", "woods", "open areas" and "mountains"
 - the polygons are enveloped into rectangular areas, so it is possible to calculate features like centroid and adjacency matrix. However, this will introduce some error based on the shape and the size of the polygon
 - neighboring areas which belong to the same category are merged together. This was necessary because the original shapefile divides the areas into many small areas.
 - for each area, the centroid is calculated. All the areas are mapped to their corresponding centroid in a json file. Also, a PDDL problem file can be automatically generated from this data. The json file is given as an input to UPF4ROS, so the symbolic arguments (areas) can be mapped to continuous data (coordinates of centroid)

### Setup of the Validation and Visualization Component
The first module that will be installed is the flight control software PX4 and the ROS2 Bridge. General installation instructions can be found here: https://docs.px4.io/main/en/dev_setup/building_px4.html. At the time of this writing the main branch of PX4 corresponds to version 1.14. If the installation instructions differ based on the ROS2 version, use the ones for ROS2 Galactic. For testing successful installations use the command `make px4_sitl none_iris` from within the PX4-Autopilot directory.

Next you need to install the [PX4-ROS 2/DDS Bridge](https://docs.px4.io/main/en/middleware/uxrce_dds.html#install-standalone-from-source) in order to control the drones in the simulation via ROS2 Nodes.

**_NOTE:_**  The MicroXRCEAgent will only connect to PX4 if it is already connected to the Unreal simulation.

Finally we install the [Offboard Control](https://docs.px4.io/main/en/ros/ros2_offboard_control.html) example code so you can test your setup. After creating the workspace (e.g. `mkdir -p ~/offboard_control_ws/src/ && cd $_`) and cloning the repos, use the command:

```cd ~/offboard_control_ws/src/px4_ros_com/scripts; source build_ros2_workspace.bash```

for building the workspace.

You should be able to execute the following steps after running the Unreal Simulation in order to remotely control the drone. Execute each line in a separate terminal tab:
```
cd ~/PX4-Autopilot; make px4_sitl_default none_iris

cd Micro-XRCE-DDS-Agent/build; MicroXRCEAgent udp4 -p 8888

source /opt/ros/galactic/setup.bash; source ~/offboard_control_ws/install/setup.bash; ros2 run px4_ros_com offboard_control
```

**Troubleshooting:** If running the above steps does not succeed, try some of the following fixes:
 - The PX4 "error: etc/init.d-posix/rcS: 39: [: Illegal number:" so far didn't cause problems for the REAP framework. You can probably ignore it.
 - `apt install --user -U kconfiglib empy pyros-genmsg setuptools`
 - Using Java JDK version 11 (`sudo update-alternatives --config java`) might help, install version 11 if not available as choice.
 - We provide our aliases.sh file which might help. If you want to use it, copy its content into /etc/profile.d/aliases.sh. Normally shell scripts in that directory should be automatically sourced (from the /etc/profile executable).

* * *

#### Setting up the Action Server
The **Action Server** is based on the ["ROS2 Example Applications"](https://docs.px4.io/main/en/ros/ros2_comm.html#ros-2-example-applications) in the official PX4 documentation. If you want to read additional PX4 states, such as battery charge or gps home position, you first have to enable relevant message types by editing the file `dds_topic.yaml` in the PX4-Autopilot as described [here](https://docs.px4.io/main/en/middleware/uxrce_dds.html#dds-topics-yaml). Just append the following lines to the publication section of the `dds_topic.yaml` file within the directory `~/PX4-Autopilot/src/modules/uxrce_dds_client`:

```
  - topic: /fmu/out/battery_status
    type: px4_msgs::msg::BatteryStatus
  - topic: /fmu/out/home_position
    type: px4_msgs::msg::HomePosition
```

Afterwards you have to make a clean rebuild of PX4-Autopilot. You might also want to add the line `source ~/offboard_control_ws/install/setup.bash` to your aliases.sh or .bashrc file, to automatically load the offboard_control_ws workspace every time you start a new terminal tab within the WSL2 instance.

>**⚠ Info** If you already enabled the message types but they still aren't listed as topics after starting the MicroXRCEAgent, the following tips might help:
>Use these commands for cleanup:
>
>``` cd ~/PX4-Autopilot; make clean; rm -rf build ```


At this point the additional topic type `BatteryStatus publisher` should show up after starting the MicroXRCEAgent (which corresponds to the "ROS2 Bridge" module in the system diagram).
Next, copy the following files from within the `Offboard_Control` folder of this repo to `~/offboard_control_ws/src/px4_ros_com/src/examples/offboard/`:
- `battery_status_listener_lib.cpp`
- `vehicle_global_position_listener_lib.cpp`
- `GeodeticConverter.hpp`
- replace the original `offboard_control.cpp` with the one from this repo.

Also add the launch file `offboard_control.launch.py` to the directory `~/offboard_control_ws/src/px4_ros_com/launch/`.

* * *

#### Aerostack2 Installation
We use ROS2 msg types that are specifically tailored to drones from the [Aerostack2](https://github.com/aerostack2/aerostack2) framework. Be careful to install the main branch and not the 'galactic' one that is out of date. If you are using an EOL ubuntu distro (like we with 20.04), the rosdep update step should include the option `rosdep update --include-eol-distros`. For building we recommend to skip the gazebo-ignition packages because there are some compatibility issues with ROS2 galactic due to a major renaming. Use the command `colcon build --symlink-install --packages-skip as2_ign_gazebo_assets as2_platform_ign_gazebo` instead.

**Troubleshooting:**
- `sudo apt install libgflags-dev`
- `sudo apt install ignition-fortress`

* * *
To finally test the complete REAP setup, replace the file `CMakeLists.txt` in the directory `~/offboard_control_ws/src/px4_ros_com/` with the modified version of this repo as well.
Rebuild offboard_control (**Action Server**) via the command `cd ~/offboard_control_ws/src/px4_ros_com/scripts; source build_ros2_workspace.bash`.

Now, if the AI-planning subsystem has also been setup, you should be able to run the whole framework by executing the following lines (in separate tabs) after starting the Unreal Simulation. If you are using our provided `aliases.sh` you can also just execute the shellscript `start_upf_simulation.sh` from this repo instead.
```
cd ~/PX4-Autopilot; make px4_sitl_default none_iris

cd Micro-XRCE-DDS-Agent/build; MicroXRCEAgent udp4 -p 8888

source /opt/ros/galactic/setup.bash; source ~/offboard_control_ws/install/setup.bash; ros2 run px4_ros_com offboard_control

cd ~/PlanSys; source install/setup.bash; ros2 launch upf4ros2 upf4ros2.launch.py

cd ~/PlanSys; source install/setup.bash; ros2 launch upf4ros2_demo traverse_areas.launch.py
```

### Setup of the Environment Manipulation
You can externally control the simulation environment (i.e. spawn new objects, change weather conditions, etc.) using the **Remote Control API** for the Unreal Engine and the **AirSim API**.

To install the Remote Control API, follow the instructions under: https://docs.unrealengine.com/4.27/en-US/ProductionPipelines/ScriptingAndAutomation/WebControl/QuickStart/. This plugin allows you to call any public functions of objects in the Unreal Engine project via HTTP requests. For convenient testing of HTTP requests, you can use the Postman software (https://www.postman.com/). The object paths are changed while the editor is in "simulation mode" (i.e. you press the play button). Be sure to add the prefix "UEDPIE_0_" to the object path parameter. Otherwise, the functions will only work in "edit mode" (see: https://forums.unrealengine.com/t/does-web-remote-control-work-during-runtime/463743).

```
"objectPath" : "/Game/Maps/SunTemple.SunTemple:PersistentLevel.Bp_SpawnPoint_2"
``` 
needs to be changed to:
```
"objectPath" : "/Game/Maps/UEDPIE_0_SunTemple.SunTemple:PersistentLevel.Bp_SpawnPoint_2"
```

For setup information of the AirSim API see: https://microsoft.github.io/AirSim/apis/. This API allows you to impact the physics simulation (e.g. simulate wind or rain). An example python script ("remoteControlWeather.py") that has to run from within Windows is provided in this repo.

### Setup of the Ground Control Software

Follow the instructions (for Ubuntu Linux) under: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html to install QGroundControl under WSL2. You can start QGroundControl by executing the command `./QGroundControl.AppImage`. When the Unreal Simulation and PX4 are already running, it should automatically connect.

## Multidrone Simulation

For simulating multiple drones, you first have to modify the AirSim [settings.json](https://microsoft.github.io/AirSim/px4_multi_vehicle/) file. This allows you to create multiple drone actors in the Unreal Environment. The REAP repo contains an example for 3 drones in the file `REAP/AirSim_alternative_settings/settings_multi.json`. Just copy it and replace the content of your own settings.json, but you can modify it further with a custom amount.
The second step depends on your use case:
- If you just want to control multiple drones via QGroundControl, you can use the script `REAP/multi_drone_scripts/sitl_multiple_run.sh` to start multiple px4 instances. Run it from the directory `~/PX4-Autopilot/Tools/simulation/sitl_multiple_run.sh` in the WSL2 instance. When starting QGroundControl afterwards, there will be a selector available, where you can define a custom flight mission for each drone individually.
- If you want to use pddl-based planning with UPF4ROS, you can use the alternative startup script `start_upf_simulation_multi.sh` instead. You can modify the variable "drone_count" inside of it, but you have to make sure that the number matches your configured AirSim settings. You will need to use the content of our `aliases.sh` file for it to work, similarly to the `start_upf_simulation.sh`.

## Replanning
TODO

## Editing the Simulation Environment

In this section relevant files are described, that are required to customize the simulation environment to your needs.

 - **\<PlanSys2\>/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/plan_executor.py**: This file communicates with the **Planner** component in the system diagram. The PDDL problem/domain file can be given in the **init** function. However, it is also possible to directly define a planning problem via Python command using the Unified Planning Framework. A plan is calculated in the **get_plan_srv** function. Then, we loop through all the actions. An action is sent to the **Action Client** via ROS2 client-server architecture. When the Action Client responds that the action has been finished, the next action is sent until the plan is fully executed.
 - **\<PlanSys2\>/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/navigation_action_client.py**: This file corresponds to the **Action Client** component in the system diagram. The server which receives the actions from the **Planner** is created in the **init** function and is bound to the **execute_callback** function. In this function, the received actions can be preprocessed and are then sent to the Validation & Visualization side of the framework via ROS2 client-server architecture.
- **px4_ros_com_ros2/src/px4_ros_com/src/examples/offboard_control.cpp**: This file corresponds to the **Action Server** component in the system diagram. In this component we translate the symbolic planning actions into messages for the PX4 autopilot and implement the high-level flight logic. A reference of possible PX4 messages and the relevant fields can be found at https://docs.px4.io/main/en/msg_docs/.
- The lookup table (generated from the shapefile) which maps areas to their coordinates is stored in **\<PlanSys2\>/src/UPF4ROS2/upf4ros2_demo/params**
- PDDL problem and domain files are stored in **\<PlanSys2\>/src/UPF4ROS2/upf4ros2_demo/test/pddl**

## Roadmap
- Providing an interface for [ALNS](https://github.com/N-Wouda/ALNS).
- Upgrading to Unreal Engine 5 via the [Colosseum](https://github.com/CodexLabsLLC/Colosseum) fork of AirSim.
- A readme section about integrating photogrammetry models or environment streaming via Cesium plugin/Google Maps.
- Hardware integration for swarm testing, [Platform](www.diatone.us/products/roma-f7-6s-pnp-bnf-drone) and [Companion] (www.nvidia.com/de-de/autonomous-machines/embedded-systems/jetson-orin/)

## Contact Information

Write us for questions, help for installation, or even future collaboration at fmff.lrt@unibw.de
