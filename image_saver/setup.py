from setuptools import setup

package_name = 'image_saver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/image_saver.launch.py']),
        ('share/' + package_name + '/config', ['config/image_saver_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Image saver node for Basler camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_saver_node = image_saver.image_saver_node:main',
        ],
    },
)
