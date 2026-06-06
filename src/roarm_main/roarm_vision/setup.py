from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roarm_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share',package_name,'config'),glob(os.path.join('config','*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dudu',
    maintainer_email='dudu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_detect_gazebo = roarm_vision.yolov8_detect_gazebo:main',
            'apriltag_detect = roarm_vision.apriltag_detect:main',
            'color_block_detect = roarm_vision.color_block_detect:main',
            'pick_place_cmd = roarm_vision.pick_place_cmd:main',
            'color_select = roarm_vision.color_select:main',
            'yolov8_detect_oak = roarm_vision.yolov8_detect_oak:main',
        ],
    },
)
