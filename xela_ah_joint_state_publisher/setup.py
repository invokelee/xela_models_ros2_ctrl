from setuptools import find_packages, setup
from glob import glob
from pathlib import Path

package_name = 'xela_ah_joint_state_publisher'

# launch_files = ['launch/xela_ah_jsp.launch.py']
# config_files = ['config/xela_ah_jsp_config.yaml']
launch_files = ['launch/*.launch.py']
config_files = ['config/*.yaml']

data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
    (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    (f'share/{package_name}/config', glob('config/*.yaml')),
]

# data_files = [
#     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
#     ('share/' + package_name, ['package.xml']),
#     ('share/' + package_name, launch_files),
#     ('share/' + package_name, config_files),
# ]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     ('share/' + package_name, '/launch')
    # ],
    data_files=data_files,
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Sanghoon Lee',
    maintainer_email='sanghoonlee@xelarobotics.com',
    description='Xela Allegrohand Joint State Publisher (URDF/XACRO + YAML keep_joints).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'xela_ah_joint_state_publisher_node = xela_ah_joint_state_publisher.joint_state_publisher_xela_ah:main',
        ],
    },
)
