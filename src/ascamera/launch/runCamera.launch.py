#!/usr/bin/env python3
"""
ascamera_bringup.launch.py

Konversi dari run_ascamera_node.sh (ROS2).

CATATAN PENTING (tidak bisa dipindah ke launch file):
  - Cek keberadaan /etc/udev/rules.d/angstrong-camera.rules dan elevasi
    root via sudo. Ini harus tetap dilakukan sebelum `ros2 launch`
    dipanggil (mis. lewat wrapper shell script kecil), karena launch
    file jalan sebagai proses biasa, bukan re-exec sebagai root.
  - `source /opt/ros/<distro>/setup.bash` dan `source install/setup.bash`
    harus sudah dilakukan di shell sebelum menjalankan `ros2 launch`,
    ini bagian dari environment setup ROS2 dan tidak bisa dipindah
    ke dalam launch file itu sendiri.

Yang dipindah ke sini:
  - Deteksi gcc target dan set LD_LIBRARY_PATH secara dinamis.
  - Include ascamera.launch.py yang asli.
"""

import os
import subprocess

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def get_gcc_target():
    """Setara dengan: gcc -v 2>&1 | grep Target: | sed 's/Target: //g'"""
    try:
        output = subprocess.check_output(
            ['gcc', '-v'], stderr=subprocess.STDOUT
        ).decode()
        for line in output.splitlines():
            line = line.strip()
            if line.startswith('Target:'):
                return line.split('Target:', 1)[1].strip()
    except Exception as e:
        print(f'Warning: gagal mendeteksi gcc target: {e}')
    return ''


def generate_launch_description():
    # Direktori tempat launch file ini berada (setara $CUR_DIR di script asli)
    cur_dir = os.path.dirname(os.path.realpath(__file__))

    gcc_target = get_gcc_target()

    lib_path = os.path.join(cur_dir, 'ascamera', 'libs', 'lib', gcc_target)
    existing_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_path = f'{lib_path}:{existing_ld_path}' if existing_ld_path else lib_path

    # Cari ascamera.launch.py: coba dulu di workspace lokal, fallback ke package share
    ascamera_launch_file = os.path.join(
        cur_dir, 'install', 'ascamera', 'share', 'ascamera',
        'launch', 'ascamera.launch.py'
    )
    if not os.path.exists(ascamera_launch_file) and get_package_share_directory:
        try:
            ascamera_launch_file = os.path.join(
                get_package_share_directory('ascamera'),
                'launch', 'ascamera.launch.py'
            )
        except Exception:
            pass

    return LaunchDescription([
        LogInfo(msg=f'Target: {gcc_target}'),
        LogInfo(msg=f'lib path: {lib_path}'),
        SetEnvironmentVariable('LD_LIBRARY_PATH', new_ld_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ascamera_launch_file)
        ),
    ])