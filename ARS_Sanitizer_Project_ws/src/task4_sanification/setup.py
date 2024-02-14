from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task4_sanification'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sebab',
    maintainer_email='sebastiano.bertame@studio.unibo.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'power_publisher = task4_sanification.power_publisher_node:main',
            'energy_publisher = task4_sanification.energy_publisher_node:main',
            'energy_navigation = task4_sanification.energy_navigation_node:main',
            'trigger_global_localization = task4_sanification.trigger_global_localization:main',
        ],
    },
)
