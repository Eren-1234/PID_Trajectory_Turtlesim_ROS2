from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['my_robot_controller/config.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eren',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot2 = my_robot_controller.robot2:main",
            "robot1 = my_robot_controller.robot1:main"
        ],
    },
)
