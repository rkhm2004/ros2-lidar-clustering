
from setuptools import setup

package_name = 'lidar_clustering_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harish',
    maintainer_email='harishmonish3108@gmail.com',
    description='LiDAR PointCloud clustering using DBSCAN',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'scan_cluster =                           lidar_clustering_py.scan_cluster_node:main',
        'pc_cluster = lidar_clustering_py.pc_cluster_node:main',
    ],
},

)
