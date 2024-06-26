from setuptools import setup
import os
import glob

package_name = 'final_challenge2024'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/final_challenge2024/launch', glob.glob(os.path.join('launch', '*launch.py'))),
        ('share/final_challenge2024/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        ('lib/'+package_name+"", glob.glob(os.path.join('final_challenge2024/', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='villac@mit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'city_stopping_controller = final_challenge2024.city_stopping_controller:main',
            'homography_transformer = final_challenge2024.homography_transformer:main',
            'stop_light_detector = final_challenge2024.stop_light_detector:main',
            'stop_detector = final_challenge2024.stop_detector:main',
            'track_simulator = final_challenge2024.track_racing.track_simulator:main',
            'yeet_node = final_challenge2024.yeet_node:main',
            'hello = final_challenge2024.hello:main',
            "city_driver = final_challenge2024.city_driver:main",
            "basement_pub = final_challenge2024.basement_point_publisher:main",
            "city_follow = final_challenge2024.trajectory_follower_city:main",
        ],
    },
)
