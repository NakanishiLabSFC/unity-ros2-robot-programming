from setuptools import find_packages, setup

package_name = 'chapter5'

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
    maintainer='ubuntu2004',
    maintainer_email='teturou@aw.wakwak.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'nav2_script_control = chapter5.nav2_script_control:main',
            'nav2_unity_pipeline = chapter5.nav2_unity_pipeline:main', 
        ],
    },
)
