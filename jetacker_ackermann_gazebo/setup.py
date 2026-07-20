import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'jetacker_ackermann_gazebo'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*'),
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='aim',
    maintainer_email='aim@todo.todo',

    description='Gazebo simulation package for JetAcker Ackermann model',
    license='TODO: License declaration',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [],
    },
)
