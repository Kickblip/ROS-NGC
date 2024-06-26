from setuptools import setup

package_name = 'nav_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/locobot_startup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kickball',
    maintainer_email='wyatt.zilker@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_base = nav_testing.move_base:main'
        ],
    },
)
