from setuptools import find_packages, setup

package_name = 'teamwh_object_follower'

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
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_object2 = teamwh_object_follower.find_object2:main',
            'turn_turtlebot = teamwh_object_follower.turn_turtlebot:main',
            'get_obj_range = teamwh_object_follower.get_obj_range:main',
        ],
    },
)
