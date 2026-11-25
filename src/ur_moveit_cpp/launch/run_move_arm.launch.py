import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('ur_description'), 'urdf', 'ur.urdf.xacro')
    srdf_path = os.path.join(get_package_share_directory('ur_moveit_config'), 'srdf', 'ur.srdf.xacro')

    # Añadimos el argumento obligatorio 'name': 'ur' a los mappings
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description(file_path=urdf_path, mappings={"name": "ur", "ur_type": "ur3"})
        .robot_description_semantic(file_path=srdf_path, mappings={"name": "ur", "ur_type": "ur3"})
        .to_moveit_configs()
    )

    move_arm_node = Node(
        package="ur_moveit_cpp",
        executable="move_arm",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}
        ],
    )

    # Fuerza el punto como separador decimal (locale es_ES usa coma y rompe
    # el parseo de URDF/RViz). Sustituye al 'export LC_NUMERIC="C"' manual.
    set_locale = SetEnvironmentVariable("LC_NUMERIC", "C")

    return LaunchDescription([set_locale, move_arm_node])
