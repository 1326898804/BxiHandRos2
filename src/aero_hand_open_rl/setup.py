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
from setuptools import find_packages, setup

package_name = "aero_hand_open_rl"
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
    ]+get_sdk_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nan Wang",
    maintainer_email="nanwang@tetheria.ai",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rl_z_rotation_deploy = aero_hand_open_rl.rl_z_rotation_deploy:main",
        ],
    },
)
