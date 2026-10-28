from glob import glob
from setuptools import find_packages, setup

NAME = 'inventory_inspection_poc'
setup(
    name=NAME,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + NAME]),
        ('share/' + NAME, ['package.xml', 'README.md']),
        ('share/' + NAME + '/launch', glob('launch/*.launch.py')),
        ('share/' + NAME + '/config', glob('config/*.yaml')),
        ('share/' + NAME + '/maps', glob('maps/*')),
        ('share/' + NAME + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Capstone team',
    maintainer_email='student@example.com',
    description='Autonomous three-stop warehouse inspection with AMCL and Nav2',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'inspection_mission = inventory_inspection_poc.mission:main',
        'check_environment = inventory_inspection_poc.preflight:main',
    ]},
)
