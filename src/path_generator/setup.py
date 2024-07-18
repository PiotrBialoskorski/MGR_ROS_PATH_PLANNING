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
    maintainer='Piotr Bialoskorski',
    maintainer_email='bialoskorski.piotr23@gmail.com',
    description='Path generating algorithms',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'RRT = path_generator.RRT_path:main',
            'RRT_star = path_generator.RRT_star:main',
            'Voronoi_Dijsktra = path_generator.Voronoi_Dijkstra:main',
            'Voronoi_Astar = path_generator.Voronoi_A_star:main',
            'Vector_field = path_generator.Vector_field_algorithm:main',
            'Dyfusion = path_generator.Dyfusion_algorithm:main',
            'Vector_field_tangent = path_generator.Vector_field_tangent_algorithm:main',
            'Visible_Dijkstra = path_generator.Visibility_Dijkstra:main',
            'Visible_A_star = path_generator.Visibility_A_star:main',
            'pth_len = path_generator.path_len:main',
        ],
    },
)
