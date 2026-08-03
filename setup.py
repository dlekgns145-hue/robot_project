import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'robot_project'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Yahboom 로봇 - SLAM / YOLO / Follow Me / Navigation 프로젝트',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect = robot_project.perception.detect:main',
            'follow_person = robot_project.follow.follow_person:main',
            'nav = robot_project.navigation.nav:main',
            'integrated_main = robot_project.main:main',
        ],
    },
)
