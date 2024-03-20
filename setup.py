from setuptools import find_packages, setup

package_name = 'turtle_control'

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
    maintainer='Prof. Dr. Christian Pfitzner',
    maintainer_email='christian.pfitzner@thi.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_circle_node=turtle_control.turtle_circle_node:main',
            'turtle_move_goal_node=turtle_control.turtle_move_goal_node:main',
        ],
    },
)
  