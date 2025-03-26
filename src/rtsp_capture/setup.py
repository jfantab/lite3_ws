from setuptools import find_packages, setup
from glob import glob
package_name = 'rtsp_capture'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')) # a tuple
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jfantab',
    maintainer_email='jfantab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtsp_pub = rtsp_capture.publisher:main', # custom_name = pkg_name.file_name:main
            'rtsp_sub = rtsp_capture.subscriber:main'
        ],
    },
)
