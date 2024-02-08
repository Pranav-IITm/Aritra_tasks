from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aritra_task1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),(os.path.join('share', package_name,'launch/'), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranav_balaji',
    maintainer_email='pranav_balaji@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'new_goal = aritra_task1.new_goal:main',
                                'spawn_turtle = aritra_task1.spawn_turtle:main',
                                'go_to_goal = aritra_task1.go_to_goal:main'
        ],
    },
)
