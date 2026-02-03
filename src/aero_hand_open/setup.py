import os
from setuptools import find_packages, setup

package_name = "aero_hand_open"
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
    maintainer="Mohit Yadav",
    maintainer_email="mohityadav@tetheria.ai",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aero_hand_node = aero_hand_open.aero_hand_node:main",
        ],
    },
)
