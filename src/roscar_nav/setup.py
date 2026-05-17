from setuptools import setup

package_name = 'roscar_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', [
            'launch/localize.launch.py',
            'launch/navigate.launch.py',
        ]),
        (f'share/{package_name}/config', [
            'config/carto_localize.lua',
            'config/nav2_params.yaml',
        ]),
        (f'share/{package_name}/rviz', [
            'rviz/nav2_view.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='relog',
    maintainer_email='relog@example.com',
    description='Localization and Nav2 path planning for roscar.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_to_odom = roscar_nav.pose_to_odom:main',
        ],
    },
)
