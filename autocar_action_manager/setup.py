from setuptools import find_packages, setup

package_name = 'autocar_action_manager'

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
    maintainer='albinovo',
    maintainer_email='albinrik@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_server = autocar_action_manager.autocar_drive_server:main',
            'drive_client = autocar_action_manager.autocar_drive_client:main',
        ],
    },
)
