from setuptools import find_packages, setup

package_name = 'my_ur_control'

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
    maintainer='fran',
    maintainer_email='fran@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_trajectory = my_ur_control.simple_trajectory:main',
            'custom_controller = my_ur_control.custom_controller:main',
            'go_home_service = my_ur_control.go_home_service:main',
        ],
    },
)
