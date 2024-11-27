from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'turtlebots_aoc_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hmin2',
    maintainer_email='hmin2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_publisher = turtlebots_aoc_pkg.goal_publisher:main',
            'tb3_follower = turtlebots_aoc_pkg.tb3_follower:main',
            'multi_tb3_follower = turtlebots_aoc_pkg.multi_tb3_follower:main',
        ],
    },
)
