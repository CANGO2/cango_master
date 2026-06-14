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
    master_params = os.path.join(pkg_dir, "config", "params.yaml")
    nav2_params = os.path.join(pkg_dir, "config", "nav2_params.yaml")
    
    # 2. Nav2 navigation만 실행하고, AMCL은 FASTLIO localizer와 TF가 겹치지 않도록 제외
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            nav2_params,
            {"yaml_filename": map_config},
        ],
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    # 3. 연구자님의 Cango Master 실행
    cango_master_node = Node(
        package="cango_master",
        executable="cango_master",
        name="cango_master",
        output="screen",
        emulate_tty=True,
        parameters=[master_params],
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager_localization,
        nav2_stack,
        cango_master_node,
    ])
