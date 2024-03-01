from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'chapter2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu2004',
    maintainer_email='teturou@aw.wakwak.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = chapter2.simple_publisher:main',
            'simple_subscriber = chapter2.simple_subscriber:main',
            'simple_server = chapter2.simple_server:main',
            'simple_client = chapter2.simple_client:main',
        ],
    },
)
