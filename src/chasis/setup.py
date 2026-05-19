from setuptools import setup

package_name = 'chasis'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
        [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/c30d_ctrl.launch.py', 'launch/c30d_keyctrl.launch.py']),
        (f'share/{package_name}/config', ['config/c30d_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kielas',
    maintainer_email='c1470759@outlook.com',
    description='C30D chassis controller driver (ROS 2, Python + pyserial)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'c30d_ctrl_node = chasis.c30d_ctrl:main',
            'key_ctrl_node = chasis.key_ctrl:main',
        ],
    },
)
