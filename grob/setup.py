from setuptools import find_packages, setup

package_name = 'grob'

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
    maintainer='kaveet',
    maintainer_email='kaveet.grewal@uwaterloo.ca',
    description='ROS Package for G.R.O.B',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = grob.my_node:main',
            'decisions = grob.decision_maker:main',
            'localizer = grob.localizer:main',
            'controller = grob.controller:main',
            'scanner = grob.scanner:main',
            'planner = grob.planner:main',
        ],
    },
)
