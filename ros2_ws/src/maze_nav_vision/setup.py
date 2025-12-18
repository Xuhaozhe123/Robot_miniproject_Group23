from setuptools import setup

package_name = 'maze_nav_vision'

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
    description='Blue tape soft obstacle detection node for maze navigation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blue_tape_detector = maze_nav_vision.blue_tape_detector:main',
        ],
    },
)


