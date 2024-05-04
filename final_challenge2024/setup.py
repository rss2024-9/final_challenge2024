from setuptools import setup

package_name = 'final_challenge2024'

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
    maintainer='racecar',
    maintainer_email='villac@mit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "city_driver = final_challenge2024.city_driving.city_driver:main",
            "basement_pub = final_challenge2024.city_driving.basement_point_publisher:main"
        ],
    },
)
