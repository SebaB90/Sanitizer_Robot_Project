from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the launch description
    ld = LaunchDescription()

    # Define the node for the power publisher
    power_publisher_node = Node(
        package='task4_sanification',
        executable='power_publisher',
        output='screen'
    )

    # Define the node for the energy publisher
    energy_publisher_node = Node(
        package='task4_sanification',
        executable='energy_publisher',
        output='screen'
    )

    # Define the node for the energy navigation
    energy_navigation_node = Node(
        package='task4_sanification',
        executable='energy_navigation',
        output='screen'
    )

    # Add all nodes to the launch description
    ld.add_action(power_publisher_node)
    ld.add_action(energy_publisher_node)
    ld.add_action(energy_navigation_node)

    return ld
