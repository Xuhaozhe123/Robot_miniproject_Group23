from setuptools import setup

package_name = 'maze_nav_fsm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Haozhe Xu',
    maintainer_email='xuhaozhe2022@bupt.edu.cn',
    description='Finite state machine for two-phase maze navigation with state-dependent costmaps.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maze_fsm = maze_nav_fsm.maze_fsm:main',
        ],
    },
)


