import glob
from setuptools import find_packages, setup

package_name = 'fleet_adapter_template'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name,['config.yaml']),
        ('share/' + package_name + '/launch', [
            'demo.launch.xml',
        ]),
        ('share/' + package_name + '/maps', glob.glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yadunund',
    maintainer_email='yadunund@openrobotics.org',
    description='A template for an RMF fleet adapter',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=fleet_adapter_template.fleet_adapter:main',
            'dispatch_go_to_place=fleet_adapter_template.tasks.dispatch_go_to_place:main',
        ],
    },
)
