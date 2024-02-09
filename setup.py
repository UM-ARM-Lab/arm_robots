from setuptools import find_packages, setup
from glob import glob

package_name = 'arm_robots'

pkg_files_to_install = ['package.xml', ] + glob("rviz/*.rviz") + glob("launch/*.launch.*") 

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, pkg_files_to_install),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='peter',
    maintainer_email='pmitrano@umich.edu',
    description='simple python API for commanding some of our robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to_impedance_switch = arm_robots.scripts.utility.move_to_impedance_switch:main',
        ],
    },
)
