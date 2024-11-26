import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'diff_drive_simple'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oskars',
    maintainer_email='oskarsvismanis@gmail.com',
    description='The most basic diff drive controller you could possibly have. ROS 2 edition.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive = diff_drive_simple.diff_drive:main'
        ],
    },
)
