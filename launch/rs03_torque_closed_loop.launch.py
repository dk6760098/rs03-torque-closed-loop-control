from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def int_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rs03_torque_closed_loop_control"),
        "config", "rs03_torque_closed_loop.yaml")
    arguments = [
        DeclareLaunchArgument("auto_enable", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("control_mode", default_value="velocity_pi",
                              choices=["velocity_pi", "position_pid", "mit_impedance"]),
        DeclareLaunchArgument("command_timeout_s", default_value="0.30"),
        DeclareLaunchArgument("max_torque_nm", default_value="0.50"),
        DeclareLaunchArgument("torque_slew_rate_nm_s", default_value="2.0"),
        DeclareLaunchArgument("max_velocity_command_rad_s", default_value="0.30"),
        DeclareLaunchArgument("position_max_offset_rad", default_value="0.15"),
        DeclareLaunchArgument("position_tracking_error_rad", default_value="0.25"),
        DeclareLaunchArgument("velocity_kp", default_value="0.40"),
        DeclareLaunchArgument("velocity_ki", default_value="0.30"),
        DeclareLaunchArgument("velocity_integral_limit_nm", default_value="0.20"),
        DeclareLaunchArgument("velocity_feedforward_nm", default_value="0.0"),
        DeclareLaunchArgument("position_kp", default_value="2.0"),
        DeclareLaunchArgument("position_ki", default_value="0.0"),
        DeclareLaunchArgument("position_kd", default_value="0.20"),
        DeclareLaunchArgument("position_integral_limit_nm", default_value="0.10"),
        DeclareLaunchArgument("position_feedforward_nm", default_value="0.0"),
        DeclareLaunchArgument("mit_kp", default_value="2.0"),
        DeclareLaunchArgument("mit_kd", default_value="0.20"),
        DeclareLaunchArgument("mit_feedforward_torque_nm", default_value="0.0"),
        DeclareLaunchArgument("mit_position_slew_rate_rad_s", default_value="0.05"),
        DeclareLaunchArgument("velocity_filter_alpha", default_value="0.20"),
        DeclareLaunchArgument("max_velocity_rad_s", default_value="0.80"),
        DeclareLaunchArgument("velocity_trip_samples", default_value="5"),
        DeclareLaunchArgument("max_temperature_c", default_value="60.0"),
        DeclareLaunchArgument("feedback_miss_limit", default_value="10"),
    ]
    parameters = {
        "auto_enable": ParameterValue(
            LaunchConfiguration("auto_enable"), value_type=bool),
        "control_mode": LaunchConfiguration("control_mode"),
        "command_timeout_s": float_param("command_timeout_s"),
        "max_torque_nm": float_param("max_torque_nm"),
        "torque_slew_rate_nm_s": float_param("torque_slew_rate_nm_s"),
        "max_velocity_command_rad_s": float_param("max_velocity_command_rad_s"),
        "position_max_offset_rad": float_param("position_max_offset_rad"),
        "position_tracking_error_rad": float_param("position_tracking_error_rad"),
        "velocity_kp": float_param("velocity_kp"),
        "velocity_ki": float_param("velocity_ki"),
        "velocity_integral_limit_nm": float_param("velocity_integral_limit_nm"),
        "velocity_feedforward_nm": float_param("velocity_feedforward_nm"),
        "position_kp": float_param("position_kp"),
        "position_ki": float_param("position_ki"),
        "position_kd": float_param("position_kd"),
        "position_integral_limit_nm": float_param("position_integral_limit_nm"),
        "position_feedforward_nm": float_param("position_feedforward_nm"),
        "mit_kp": float_param("mit_kp"),
        "mit_kd": float_param("mit_kd"),
        "mit_feedforward_torque_nm": float_param("mit_feedforward_torque_nm"),
        "mit_position_slew_rate_rad_s": float_param("mit_position_slew_rate_rad_s"),
        "velocity_filter_alpha": float_param("velocity_filter_alpha"),
        "max_velocity_rad_s": float_param("max_velocity_rad_s"),
        "velocity_trip_samples": int_param("velocity_trip_samples"),
        "max_temperature_c": float_param("max_temperature_c"),
        "feedback_miss_limit": int_param("feedback_miss_limit"),
    }
    arguments.append(Node(
        package="rs03_torque_closed_loop_control",
        executable="rs03_torque_closed_loop_node",
        name="rs03_torque_closed_loop",
        parameters=[config, parameters],
        output="screen",
    ))
    return LaunchDescription(arguments)
