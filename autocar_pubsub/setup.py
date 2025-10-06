from setuptools import find_packages, setup

package_name = 'autocar_pubsub'

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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = autocar_pubsub.autocar_publisher:main',
            'listener = autocar_pubsub.autocar_subscriber:main',
        ],
    },
)
