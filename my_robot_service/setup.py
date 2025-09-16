from setuptools import find_packages, setup

package_name = 'my_robot_service'

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
    maintainer='prince',
    maintainer_email='prince@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'robot_service_server = my_robot_service.robot_service_server:main',
        'robot_service_client = my_robot_service.robot_service_client:main',
    ],
},

)
