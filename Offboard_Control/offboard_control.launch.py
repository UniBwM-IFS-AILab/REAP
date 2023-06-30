#!/usr/bin/env python

"""
micro_ros_agent = ExecuteProcess(
    cmd=[[
        'micro-ros-agent udp4 --port 8888 -v '
    ]],
    shell=True
)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    #number of nodes to start
    count = 3
    offboard_control =  []

    if count < 2:
        name_prefix = ""
        offboard_control.append(
            Node(
                package='px4_ros_com',
                executable='offboard_control',
                output='screen',
                shell=True,
                arguments=[name_prefix]
            )
        )
    else:
        for x in range(0, count):
            name_prefix = 'vhcl'+ str(x) +'/'
            offboard_control.append(
                Node(
                    package='px4_ros_com',
                    executable='offboard_control',
                    output='screen',
                    shell=True,
                    arguments=[name_prefix]
                )
            )
    

    return LaunchDescription(
        #micro_ros_agent,
        offboard_control
    )
