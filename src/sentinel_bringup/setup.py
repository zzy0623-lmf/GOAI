from setuptools import find_packages, setup

package_name = 'sentinel_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sentinel.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sentinel Team',
    maintainer_email='dev@sentinel.com',
    description='哨兵系统总启动 (GOAI 比赛版)',
    license='Apache-2.0',
)
