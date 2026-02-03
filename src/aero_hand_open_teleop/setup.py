# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from glob import glob
from setuptools import find_packages, setup

package_name = "aero_hand_open_teleop"
def get_sdk_files():
    data_files = []
    source_dir = '../aero_open_sdk'  # 源目录，相对于setup.py的位置
    target_dir = os.path.join('lib', package_name, 'aero_open_sdk')  # 目标目录

    # 遍历源目录下的所有文件和子目录
    for root, dirs, files in os.walk(source_dir):
        for file in files:
            file_path = os.path.join(root, file)
            # 计算相对于源目录的相对路径，以保持子目录结构
            relative_path = os.path.relpath(root, source_dir)
            install_dir = os.path.join(target_dir, relative_path)
            data_files.append((install_dir, [file_path]))
    
    return data_files

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/manus_teleop.launch.py"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ]+get_sdk_files(),
    install_requires=['setuptools', 'numpy', 'mediapipe', 'opencv-python'],
    zip_safe=True,
    maintainer="mohit",
    maintainer_email="mohityadav@tetheria.ai",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'manus_joint_states_retargeting = aero_hand_open_teleop.manus_joint_states_retargeting:main',
            'mediapipe_mocap = aero_hand_open_teleop.mediapipe_mocap:main',
            'mediapipe_retargeting = aero_hand_open_teleop.mediapipe_retargeting:main',
        ],
    },
)
