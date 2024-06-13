from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'test_path_creator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob(os.path.join('launch','*launch.py*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roslab',
    maintainer_email='roslab@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'testpath = test_path_creator.testpath:main',
            'GetGoal = test_path_creator.get_goal:main',
            'GetInitial = test_path_creator.get_initial:main',
            'GetData = test_path_creator.get_path_data:main',
        ],
    },
)
