from glob import glob
from setuptools import find_packages, setup

package_name = 'gps_nav2_goal'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Gendre',
    maintainer_email='samuel@example.com',
    description='Convert WGS84 geographic goals into Nav2 NavigateToPose goals.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_nav2_goal_node = gps_nav2_goal.gps_nav2_goal_node:main',
        ],
    },
)
