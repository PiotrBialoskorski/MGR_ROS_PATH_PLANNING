from setuptools import find_packages, setup

package_name = 'get_data'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piotr Bialoskorski',
    maintainer_email='bialoskorski.piotr23@gmail.com',
    description='Combining all data for path generator',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'GetGoal = get_data.get_goal:main',
            'GetInitial = get_data.get_initial:main',
            'GetData = get_data.get_path_data:main',
        ],
    },
)
