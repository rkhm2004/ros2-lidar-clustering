from setuptools import setup
import os
from glob import glob
package_name = 'lidar_clustering_py'
setup(
    name=package_name, version='0.0.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True, maintainer='user', maintainer_email='user@todo.todo',
    description='LiDAR Demo', license='TODO', tests_require=['pytest'],
    entry_points={'console_scripts': ['cluster_node = lidar_clustering_py.cluster_node:main']},
)