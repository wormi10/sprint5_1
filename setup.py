from setuptools import setup
import os
from glob import glob

package_name = 'sprint5_v2'  
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu@email.com',
    description='Sistema de navegación autónoma con ArUco markers',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_navigator = ' + package_name + '.aruco_navigator:main',
            'calibration_helper = ' + package_name + '.calibration_helper:main',
            'system_tester = ' + package_name + '.system_tester:main',
            'subcriptor = ' + package_name + '.subcriptor:main',
        ],
    },
)
