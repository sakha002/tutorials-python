from setuptools import setup

package_name = 'time_manager'

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
    maintainer='root',
    maintainer_email='hejazi.hossein@gmail.com',
    description=' example package for time publish',
    license='MIT License ',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'time_publisher = time_manager.time_manager_publisher:main',
            'listener = time_manager.time_manager_client:main',

        ],
    },
)




