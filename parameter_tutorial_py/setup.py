from glob import glob
import os

from setuptools import setup

package_name = 'parameter_tutorial_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        # (os.path.join('share', package_name), glob('param/*')),
        ('share/' + package_name, ['param/param.yaml']),
        # ('share/' + package_name, ['param']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaehyun Shim',
    maintainer_email='jhshim@robotis.com',
    description='ROS 2 packages for parameter_tutorial_py',
    license='Apache 2.0',
    author='Jaehyun Shim',
    author_email='jhshim@robotis.com',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parameter = parameter_tutorial_py.parameter:main'
        ],
    },
)
