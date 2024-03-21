from setuptools import setup

package_name = 'auto_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.httpReq', package_name + '.master', package_name + '.bucketFinder', package_name + '.position_track', package_name + '.robot_control'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nus',
    maintainer_email='nus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2mover = auto_nav.ref.r2mover:main',
            'r2moverotate = auto_nav.ref.r2moverotate:main',
            'r2scanner = auto_nav.ref.r2scanner:main',
            'r2occupancy = auto_nav.ref.r2occupancy:main',
            'r2occupancy2 = auto_nav.ref.r2occupancy2:main',
            'r2auto_nav = auto_nav.ref.r2auto_nav:main',
            'master = auto_nav.master.masterNode:main',
            'position_track = auto_nav.position_track.positionTrackNode:main',
            'httpDoorNode = auto_nav.httpReq.httpDoorNode:main',
            'bucketFinderNode = auto_nav.bucketFinder.bucketFinderNode:main',
            'robotControlNode = auto_nav.robot_control.robotControlNode:main',
        ],
    },
)
