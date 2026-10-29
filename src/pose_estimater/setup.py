from setuptools import find_packages, setup

package_name = 'pose_estimater'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='beng',
    maintainer_email='laubengyew@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detect = pose_estimater.object_detect:main',
            'object_pose_estimate = pose_estimater.object_pose_estimate:main',
            'object_pose_estimate2 = pose_estimater.object_pose_estimate2:main',
            'contact_box_pose_debug = pose_estimater.contact_box_pose_debug:main',
        ],
    },
)
