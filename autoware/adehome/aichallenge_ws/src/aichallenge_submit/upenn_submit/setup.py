from setuptools import setup
import os 
from glob import glob

package_name = 'upenn_submit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'ros2_numpy'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    package_data={
        package_name: ['resources/*'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nandan Tumu',
    maintainer_email='nandant@seas.upenn.edu',
    description='AI Challenge 2021 Competition Submission',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'upenn_node_test = upenn_submit:main',
            'publish_trajectory = upenn_submit.publish_trajectory:main',
            'publish_lane_selection = upenn_submit.publish_lane_selection:main',
            'subscribe_pointcloud = upenn_submit.subscribe_pointcloud:main',
        ],
    },
)
