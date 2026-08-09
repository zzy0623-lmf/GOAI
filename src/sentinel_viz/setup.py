from setuptools import find_packages, setup

package_name = 'sentinel_viz'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sentinel Team',
    maintainer_email='dev@sentinel.com',
    description='哨兵感知融合可视化',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'visualizer = sentinel_viz.visualizer:main',
        ],
    },
)
