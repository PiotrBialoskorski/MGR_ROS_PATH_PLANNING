from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'map_loader'

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
    maintainer='Piotr Bialoskorski',
    maintainer_email='bialoskorski.piotr23@gmail.com',
    description='Publishing used map',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
