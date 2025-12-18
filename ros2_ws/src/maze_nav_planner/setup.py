from setuptools import setup

package_name = 'maze_nav_planner'

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
    description='Global A* planner for MicroROS-ESP32 maze navigation demo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_astar_planner = maze_nav_planner.global_astar_planner:main',
        ],
    },
)


