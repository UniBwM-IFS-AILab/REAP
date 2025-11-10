# Dynamic REAP Simulation Environment

![image](media/header.png)

This repository contains installation information for the Dynamic REAP framework as described in the [ICAPS 2025 Demo paper](https://icaps25.icaps-conference.org/program/demos/).<br />
A demo video provides an overview of the framework: https://www.youtube.com/watch?v=mthOS7hksZQ.
The legacy version of REAP presented on [ICAPS 2023](https://icaps23.icaps-conference.org/program/demos/#3216) can be found under reap-legacy tag: https://github.com/UniBwM-IFS-AILab/REAP/releases/tag/reap-legacy

To cite **Dynamic REAP**, please use the following reference: <br/>
Kai Sommer, Björn Döschl, Jane Jean Kiam. "Dynamic REAP: Bringing Life into Simulations for UAV Planning and Acting Frameworks". In: ICAPS 2025 System Demonstrations.<br/>
```
@conference{dynamicreap2025icaps,
    title = "Dyanmic REAP: Bringing Life into Simulations for UAV Planning and Acting Frameworks",
    author = "Sommer, Kai and Döschl, Björn and Kiam, {Jane Jean}",
    booktitle = {ICAPS 2025 System Demonstrations},
    year = "2025",
    month = nov
}
```

## System requirements
We tested Dyanmic REAP using the following environment:
  - Windows 11
  - Unreal Engine 5.2.1
  - WSL2 instance with Ubuntu 22.04

## Setup of the simulation environment

![System Overview of Dynamic REAP](media/sys_overview.png)
The system overview shows the components required to run the simulation environment:
1. **Unreal Level**
2. **Unreal Blueprints**
* for autonomous NPC movement
* for events happening to NPCs: e.g. injury or loss of orientation
* Third-Person mode
* for georeferencing of objects

3. **Scripts**
* for sending mission data to the planning framework
* to export Unreal Engine object GPS positions and metadata to JSON

4. **AirSim** plugin for Unreal
  AirSim is not longer maintained. We are using the Coloseum fork: [Coloseum](https://github.com/CodexLabsLLC/Colosseum)

5. **Planning and Execution Framework**

### Using ready-made Unreal level

The easiest way for trying out is to use our ready-made Unreal level, which includes all necessary blueprints.
This Unreal level was designed to test Automated Planning for UAVs in search and rescue scenarios and includes a real-world map of the Tannheimer Tal.
Unfortunately, this Unreal project is about 60 GB in size, so we cannot upload it to GitHub. If you’re interested, we’d be happy to share the project with you directly via a file-sharing link (e.g. GigaMove).

### Building an own Unreal level

If you are interested in building your own Unreal Level, we recommend to start with a height map from here: [Unreal PNG Heightmap](https://manticorp.github.io/unrealheightmap/#latitude/47.5172006978394/longitude/10.6210327148438/zoom/11/outputzoom/13/width/505/height/505)
The blueprints necessary for NPC movement, georeferencing and further features of Dynamic REAP can be found in this repo.

## Integration of a planning and execution framework

In principle, you can connect any planning and execution framework to control the UAV in Dynamic REAP.
The only requirement is an interface to the PX4 SITL flight controller, e.g., via MAVLink.
As proof-of-concept we connected the AUSPEX framework, which already implements the required interfaces.
An installation for AUSPEX guide can be found here: [AUSPEX](https://github.com/UniBwM-IFS-AILab/AUSPEX)
Furthermore, AUSPEX is modular in design and can also be extended with your custom planning algorithms.

## Contact Information

Write us for questions, help for installation, or even future collaboration at fmff.lrt@unibw.de
