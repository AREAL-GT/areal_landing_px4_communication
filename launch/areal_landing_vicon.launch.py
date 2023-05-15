
'''
Written by Adam Garlow

This launch file starts the computer-side ROS2 nodes for VICON data streaming.
'''

# Imports
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    launch_me = LaunchDescription() # Initialize launch description

    # Node for streaming VICON data in the IFL
    hostname = '192.168.10.2' # Works with Wifi in IFL
    buffer_size = 256
    topic_namespace = 'vicon'
    vicon_streaming_node = Node(
        package='vicon_receiver', executable='vicon_client', output='screen',
        parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 
            'namespace': topic_namespace}],
        # condition=LaunchConfigurationEquals(primary_launch_mode, 'indoor')
    )

    # Node for passing the VICON data to the PX4
    px4_pubsub_node = Node(
        package='areal_landing_px4_communication',
        executable='px4_mocap_pubsub',
    )

    launch_me.add_action(vicon_streaming_node)
    launch_me.add_action(px4_pubsub_node)

    return launch_me




















