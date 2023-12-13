from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'astro_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrija',
    maintainer_email='andrija@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astro_serial_node = astro_serial.astro_serial_node:main'
        ],
    },
)
