from setuptools import setup
import os, glob

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        # ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # ⬇️ install launch + config so get_package_share_directory() can find them
        (os.path.join('share', package_name),
         glob.glob('launch_pub/*.py')),
        (os.path.join('share', package_name),
         glob.glob('launch_sub/*.py')),
        (os.path.join('share', package_name, 'config'),
        ['config/obd_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='jiajianchang1994@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'publisher = obstacle_avoidance.publisher:main',
        	'subscriber = obstacle_avoidance.subscriber:main',
        	'set_mode_node_pymavlink = obstacle_avoidance.set_mode_node_pymavlink:main',
        ],
    },
)
