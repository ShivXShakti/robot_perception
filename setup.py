from setuptools import find_packages, setup

package_name = 'robot_perception'

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
    maintainer='scg',
    maintainer_email='kuldeeplakhansons@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_pose = robot_perception.dummy_pose:main',
            'pose_publisher = robot_perception.pose_publisher:main',
            'pose_sub = robot_perception.pose_sub:main',
            'obj_wrt_torso = robot_perception.obj_wrt_torso:main',
            'yolo_spatial_sub = robot_perception.yolo_spatial_sub:main',
            'pose_pub_hw = robot_perception.pose_pub_hw:main'
        ],
    },
)
