from setuptools import setup

package_name = 'depth_anything_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sonfiuss',
    maintainer_email='hthaison17@gmail.com',
    description='Depth Anything V2 metric depth publisher for RTAB-Map RGB-D.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_anything_node = depth_anything_node.depth_node:main',
        ],
    },
)
