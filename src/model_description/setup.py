from setuptools import setup
import os
from glob import glob

def get_files(pattern):
    return [f for f in glob(pattern) if os.path.isfile(f)]

package_name = 'model_description'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), get_files('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), get_files('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), get_files('meshes/*')),
        (os.path.join('share', package_name, 'config'), get_files('config/*')),
        (os.path.join('share', package_name, 'params'), get_files('params/*')),
        (os.path.join('share', package_name, 'turtlebot3_world'), get_files('turtlebot3_world/*')),
        (os.path.join('share', package_name, 'turtlebot3_world/meshes'), get_files('turtlebot3_world/meshes/*')),
    ],  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
