from setuptools import find_packages, setup

package_name = 'mycobot_conveyor_belt'

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
    maintainer='marcel3245',
    maintainer_email='marcel3245@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'conveyor_belt = mycobot_conveyor_belt.conveyor_belt:main'
        ],
    },
)
