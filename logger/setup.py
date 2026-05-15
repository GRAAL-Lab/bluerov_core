from setuptools import find_packages, setup

package_name = 'logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'simplekml',
    ],
    zip_safe=True,
    maintainer='Paolo',
    maintainer_email='paolo.lais1@gmail.com',
    description='Ros2 logger for mission status, position and objects',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'logger_node = logger.main:main'
        ],
    },
)
