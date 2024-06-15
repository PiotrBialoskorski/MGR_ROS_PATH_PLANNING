from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_generator'

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
    maintainer_email='bialoskorski.piotr23@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'RRT = path_generator.RRT_path:main'
        ],
    },
)
