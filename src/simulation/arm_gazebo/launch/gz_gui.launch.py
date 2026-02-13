from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
import os
import subprocess


def _setup_sshfs_mount(context):
    """Mount Jetson's /home/nano via SSHFS so Gazebo GUI can resolve absolute paths."""
    jetson_ip = LaunchConfiguration('jetson_ip').perform(context)
    jetson_user = LaunchConfiguration('jetson_user').perform(context)
    remote_path = LaunchConfiguration('remote_path').perform(context)
    local_mount = LaunchConfiguration('local_mount').perform(context)

    jetson_password = LaunchConfiguration('jetson_password').perform(context)

    if not jetson_ip:
        print('[gz_gui] No jetson_ip provided, skipping SSHFS mount.')
        return []

    # Check if already mounted and accessible
    if subprocess.run(['mountpoint', '-q', local_mount]).returncode == 0:
        try:
            os.listdir(local_mount)
            print(f'[gz_gui] {local_mount} is already mounted and accessible, skipping.')
            return []
        except OSError:
            print(f'[gz_gui] {local_mount} is a stale mount, cleaning up...')
            subprocess.run(['fusermount', '-uz', local_mount], stderr=subprocess.DEVNULL)

    # Clean up any other stale SSHFS mount
    subprocess.run(['fusermount', '-uz', local_mount], stderr=subprocess.DEVNULL)

    # Create local mount point if it doesn't exist
    try:
        os.makedirs(local_mount, exist_ok=True)
    except PermissionError:
        print(f'[gz_gui] ERROR: Cannot create {local_mount} (permission denied).')
        print(f'[gz_gui] Run this once to fix:')
        print(f'[gz_gui]   sudo mkdir -p {local_mount} && sudo chown $USER:$USER {local_mount}')
        return []

    # Mount via SSHFS (pipe password via stdin using password_stdin option)
    sshfs_target = f'{jetson_user}@{jetson_ip}:{remote_path}'
    sshfs_opts = 'reconnect,ServerAliveInterval=15,StrictHostKeyChecking=no'

    if jetson_password:
        sshfs_opts += ',password_stdin'

    sshfs_cmd = [
        'sshfs', sshfs_target, local_mount,
        '-o', sshfs_opts,
    ]

    print(f'[gz_gui] Mounting: {jetson_user}@{jetson_ip}:{remote_path} -> {local_mount}')
    try:
        proc = subprocess.Popen(sshfs_cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        if jetson_password:
            _, stderr = proc.communicate(input=jetson_password.encode() + b'\n', timeout=30)
        else:
            _, stderr = proc.communicate(timeout=30)

        if proc.returncode == 0:
            print(f'[gz_gui] Mounted successfully.')
        else:
            print(f'[gz_gui] WARNING: SSHFS mount failed: {stderr.decode().strip()}')
    except FileNotFoundError:
        print(f'[gz_gui] ERROR: sshfs not found. Install with: sudo apt install sshfs')
    except subprocess.TimeoutExpired:
        proc.kill()
        print(f'[gz_gui] WARNING: SSHFS mount timed out.')

    return []


def generate_launch_description():
    install_dir = get_package_prefix('arm_description')
    install_gazebo_dir = get_package_prefix('arm_gazebo')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.join(install_dir, 'share'),
            os.path.join(install_gazebo_dir, 'share', 'arm_gazebo', 'worlds', 'models'),
        ])
    )

    jetson_ip_arg = DeclareLaunchArgument(
        'jetson_ip',
        default_value='',
        description='Jetson Nano IP address for SSHFS mount (leave empty to skip)'
    )
    jetson_user_arg = DeclareLaunchArgument(
        'jetson_user',
        default_value='nano',
        description='SSH username on the Jetson'
    )
    remote_path_arg = DeclareLaunchArgument(
        'remote_path',
        default_value='/home/nano',
        description='Remote path on Jetson to mount'
    )
    jetson_password_arg = DeclareLaunchArgument(
        'jetson_password',
        default_value='',
        description='SSH password for Jetson (uses sshpass; leave empty for key-based auth)'
    )
    local_mount_arg = DeclareLaunchArgument(
        'local_mount',
        default_value='/home/nano',
        description='Local mount point for Jetson filesystem'
    )

    sshfs_mount = OpaqueFunction(function=_setup_sshfs_mount)

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen'
    )

    return LaunchDescription([
        jetson_ip_arg,
        jetson_user_arg,
        jetson_password_arg,
        remote_path_arg,
        local_mount_arg,
        gz_resource_path,
        sshfs_mount,
        gz_gui,
    ])
