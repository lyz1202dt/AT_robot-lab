from glob import glob

from setuptools import find_packages, setup

package_name = 'obstacle_game_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', sorted(glob('config/*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lyz',
    maintainer_email='cloud1202@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_game_ui_node = '
            'obstacle_game_ui.obstacle_game_ui_node:main',
            'obstacle_game_ui_run_node = '
            'obstacle_game_ui.obstacle_game_ui_run_node:main',
        ],
    },
)
