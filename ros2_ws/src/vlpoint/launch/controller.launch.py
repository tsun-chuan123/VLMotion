from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    controller_host = LaunchConfiguration('controller_host')
    controller_port = LaunchConfiguration('controller_port')
    dispatch_method = LaunchConfiguration('dispatch_method')

    set_env = SetEnvironmentVariable('PYTHONUNBUFFERED', '1')

    controller_cmd = ExecuteProcess(
        cmd=[
            'python3', '-m', 'point.serve.controller',
            '--host', controller_host,
            '--port', controller_port,
            '--dispatch-method', dispatch_method,
        ],
        name='vlpoint_controller',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('controller_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('controller_port', default_value='11000'),
        DeclareLaunchArgument('dispatch_method', default_value='shortest_queue'),
        set_env,
        controller_cmd,
    ])
