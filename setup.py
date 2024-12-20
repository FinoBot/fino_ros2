from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fino_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('fino_ros2/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='okzieyka.dev@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'finobot_driver = fino_ros2.finobot_driver:main',
            'joy_driver = fino_ros2.joy_driver:main'
        ],
    },
)
