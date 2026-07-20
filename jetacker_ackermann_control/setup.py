from setuptools import find_packages, setup

package_name = 'jetacker_ackermann_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aim',
    maintainer_email='aim@todo.todo',
    description='Ackermann controller for JetAcker',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'ackermann_controller = '
        'jetacker_ackermann_control.ackermann_controller:main',

        'navigate_to_pose_cli = '
        'jetacker_ackermann_control.navigate_to_pose_cli:main',
        'navigate_to_place = '
        'jetacker_ackermann_control.navigate_to_place:main',
    ],
},
)
