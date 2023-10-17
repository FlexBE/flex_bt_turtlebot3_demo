#!/usr/bin/env python

import os
from glob import glob
from setuptools import setup

package_name = 'flex_bt_turtlebot3_demo_flexbe_behaviors'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Josh Zutell',
    maintainer_email='joshua.zutell.18@cnu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_flex_planner_sm = flex_bt_turtlebot3_demo_flexbe_behaviors.turtlebot3_flex_planner_sm',
            'turtlebot3_multi_level_flex_planner_sm = flex_bt_turtlebot3_demo_flexbe_behaviors.turtlebot3_multi_level_flex_planner_sm',
            'turtlebot3_simple_recovery_sm = flex_bt_turtlebot3_demo_flexbe_behaviors.turtlebot3_simple_recovery_sm',
        ],
    },
)
