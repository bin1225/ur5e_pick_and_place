from setuptools import setup

package_name = 'pick_and_place_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ],
    install_requires=['setuptools', 'moveit_py', 'pyyaml'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Pick and place node for UR5e + Robotiq',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_and_place = pick_and_place_pkg.pick_and_place:main',
            'move_test = pick_and_place_pkg.move_test:main',
            
        ],
    },
)
