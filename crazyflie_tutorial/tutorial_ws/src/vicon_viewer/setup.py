from setuptools import setup
from glob import glob

package_name = 'vicon_viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your-name',
    maintainer_email='your-email',
    description='Example 4: Vicon streaming and visualization',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drone_viewer = vicon_viewer.drone_viewer:main',
            'fake_mocap = vicon_viewer.fake_mocap:main',
        ],
    },
)