from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import sys

launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="seeker", description="namespace"),
    DeclareLaunchArgument(
        name="config",
        default_value="seeker_omni_depth",
        description="",
    ),
    DeclareLaunchArgument(
        name="cali_path",
        default_value="",
        description="path to cali. If not given, determined based on provided 'config' above",
    ),
    DeclareLaunchArgument(
        name="pub_disparity_img",
        default_value="true",
        description="是否发布视差图像",
    ),
    DeclareLaunchArgument(
        name="pub_disparity",
        default_value='true',
        description='是否发布视差消息'
    ),
    DeclareLaunchArgument(
        name="pub_imu",
        default_value="true",
        description="是否发布IMU数据",
    ),
    DeclareLaunchArgument(
        name="time_sync",
        default_value="true",
        description="是否启用时间同步",
    ),
    DeclareLaunchArgument(
        name="publish_bgra",
        default_value="true",
        description="是否发布彩色图像",
    ),
    DeclareLaunchArgument(
        name="publish_gray",
        default_value="false",
        description="是否发布灰度图像",
    ),
    DeclareLaunchArgument(
        name="undistort_color",
        default_value="true",
        description="是否校正彩色图像",
    ),
    DeclareLaunchArgument(
        name="undistort_gray",
        default_value="false",
        description="是否校正灰度图像",
    ),
]

def launch_setup(context):
    cali_path = LaunchConfiguration("cali_path").perform(context)
    if not cali_path:
        configs_dir = os.path.join(get_package_share_directory("seeker"), "config")
        available_configs = os.listdir(configs_dir)
        config = LaunchConfiguration("config").perform(context)
        if config in available_configs:
            cali_path = os.path.join(
                            get_package_share_directory("seeker"),
                            "config",config,"cali"
                        )
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    else:
        if not os.path.isfile(cali_path):
            return [
                LogInfo(
                    msg="ERROR: cali_path file: '{}' - does not exist. - not starting".format(
                        cali_path)
                    )
            ]

    node1 = Node(
        package="seeker",
        executable="seeker_node",
        namespace=LaunchConfiguration("namespace"),
        output='screen',
        parameters=[
            {"cali_path": cali_path},
            # {"use_image_transport": LaunchConfiguration("use_image_transport")},
            {"pub_disparity_img": LaunchConfiguration("pub_disparity_img")},
            {"pub_disparity": LaunchConfiguration("pub_disparity")},
            {"pub_imu": LaunchConfiguration("pub_imu")},
            {"time_sync": LaunchConfiguration("time_sync")},
            {"publish_bgra": LaunchConfiguration("publish_bgra")},
            {"publish_gray": LaunchConfiguration("publish_gray")},
            {"undistort_gray": LaunchConfiguration("undistort_gray")},
            {"undistort_color": LaunchConfiguration("undistort_color")}
        ],
        # remappings=[
        #     ('/seeker/poseimu', '/mavros/vision_pose/pose_cov')  # 格式：(原话题, 新话题)
        # ],
    )

    return [node1]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld

