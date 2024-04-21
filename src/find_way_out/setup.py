from setuptools import find_packages, setup

package_name = 'find_way_out'

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
    maintainer='Worren',
    maintainer_email='zhu333@gatech.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'viewer = find_way_out.camera_situation_perception:main',
            'lidar = find_way_out.lidar_situation_perception:main',
            'move = find_way_out.move:main',   	
        ],
    },
)
