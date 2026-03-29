import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory("cango_master")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # 1. 파일 경로 설정 (고정값으로 사용하거나 필요시 수정)
    map_config = os.path.join(pkg_dir, "maps", "map.yaml")
    nav2_params = os.path.join(pkg_dir, "config", "nav2_params.yaml")
    semantic_config = os.path.join(pkg_dir, "config", "semantic_transition.yaml")
    
    # 2. Nav2 공식 브링업 포함
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_config,
            "params_file": nav2_params,
            "autostart": "true",
            "semantic_config": semantic_config,
        }.items(),
    )

    # 3. 연구자님의 Cango Master 실행
    cango_master_node = Node(
        package="cango_master",
        executable="cango_master",
        name="cango_master",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        nav2_stack,
        cango_master_node,
    ])