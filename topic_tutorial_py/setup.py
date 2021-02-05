from setuptools import setup

package_name = 'topic_tutorial_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaehyun Shim',
    maintainer_email='jhshim@robotis.com',
    description='ROS 2 packages for topic_tutorial_py',
    license='Apache 2.0',
    author='Jaehyun Shim',
    author_email='jhshim@robotis.com',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_member_function = topic_tutorial_py.publisher_member_function:main',
            'subscriber_member_function = topic_tutorial_py.subscriber_member_function:main',
            'publisher_lambda = topic_tutorial_py.publisher_lambda:main',
            'subscriber_lambda = topic_tutorial_py.subscriber_lambda:main',
        ],
    },
)
