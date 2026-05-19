from setuptools import setup

package_name = 'core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
        [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/explore_manu.launch.py']),
        (f'share/{package_name}/config', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kielas',
    maintainer_email='c1470759@outlook.com',
    description='roscar master control (ROS 2, Python)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
