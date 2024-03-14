from setuptools import find_packages, setup

package_name = 'navigate_to_goal'

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
    maintainer='burger',
    maintainer_email='2423482852@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'get_object_range = navigate_to_goal.get_object_range:main',
        'get_robot_location = navigate_to_goal.get_robot_location:main',
        'go_to_goal = navigate_to_goal.go_to_goal:main',
        ],
    },
)
