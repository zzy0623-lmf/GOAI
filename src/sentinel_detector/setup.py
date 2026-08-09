from setuptools import find_packages, setup

package_name = 'sentinel_detector'

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
    description='哨兵快速检测模块 (YOLO)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fast_detector = sentinel_detector.fast_detector:main',
        ],
    },
)
