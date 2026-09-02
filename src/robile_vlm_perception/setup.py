from setuptools import find_packages, setup

package_name = 'robile_vlm_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='mosayyebiahmad@gmail.com',
    description='Robile perception package with YOLO and VLM nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    scripts=[
        'scripts/yolo_node',
    ],
    entry_points={
        'console_scripts': [
                'yolo_node = robile_vlm_perception.yolo_node:main',
                'vlm_node = robile_vlm_perception.vlm_node:main',
                'depth_target_node = robile_vlm_perception.depth_target_node:main',
        ],
    },
)