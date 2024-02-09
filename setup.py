from setuptools import find_packages, setup

package_name = 'arm_robots'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('rviz', ['rviz'])
        ('launch', ['launch'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='peter',
    maintainer_email='peter@thinkpad_t15',
    description='simple python API for commanding some of our robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
