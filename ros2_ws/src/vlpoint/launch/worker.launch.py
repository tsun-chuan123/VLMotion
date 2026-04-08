from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    worker_host = LaunchConfiguration('worker_host')
    worker_port = LaunchConfiguration('worker_port')
    controller_address = LaunchConfiguration('controller_address')
    worker_address = LaunchConfiguration('worker_address')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    limit_model_concurrency = LaunchConfiguration('limit_model_concurrency')
    stream_interval = LaunchConfiguration('stream_interval')
    mm_use_canny_edge = LaunchConfiguration('mm_use_canny_edge')
    mm_canny_low_threshold = LaunchConfiguration('mm_canny_low_threshold')
    mm_canny_high_threshold = LaunchConfiguration('mm_canny_high_threshold')
    mm_canny_blend_alpha = LaunchConfiguration('mm_canny_blend_alpha')

    set_env = SetEnvironmentVariable('PYTHONUNBUFFERED', '1')

    worker_cmd = ExecuteProcess(
        cmd=[
            'python3', '-m', 'point.serve.model_worker',
            '--host', worker_host,
            '--port', worker_port,
            '--worker-address', worker_address,
            '--controller-address', controller_address,
            '--model-path', model_path,
            '--device', device,
            '--limit-model-concurrency', limit_model_concurrency,
            '--stream-interval', stream_interval,
            '--mm-use-canny-edge', mm_use_canny_edge,
            '--mm-canny-low-threshold', mm_canny_low_threshold,
            '--mm-canny-high-threshold', mm_canny_high_threshold,
            '--mm-canny-blend-alpha', mm_canny_blend_alpha,
            '--load-4bit',
        ],
        name='vlpoint_worker',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('worker_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('worker_port', default_value='22000'),
        DeclareLaunchArgument('controller_address', default_value='http://10.0.0.1:11000'),
        DeclareLaunchArgument('worker_address', default_value='http://10.0.0.1:22000'),
        DeclareLaunchArgument('model_path', default_value='PME033541/vla13'),
        DeclareLaunchArgument('device', default_value='cuda'),
        DeclareLaunchArgument('limit_model_concurrency', default_value='5'),
        DeclareLaunchArgument('stream_interval', default_value='1'),
        DeclareLaunchArgument('mm_use_canny_edge', default_value='true'),
        DeclareLaunchArgument('mm_canny_low_threshold', default_value='100'),
        DeclareLaunchArgument('mm_canny_high_threshold', default_value='200'),
        DeclareLaunchArgument('mm_canny_blend_alpha', default_value='0.2'),
        set_env,
        worker_cmd,
    ])
