from setuptools import find_packages, setup

package_name = 'whag_bot'

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
    maintainer='user',
    maintainer_email='h.s.alharmali@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = whag_bot.publisher:main',
            'subscriber_node = whag_bot.subscriber:main',
            'omni_teleop = whag_bot.omni_teleop:main',
        ],
    },
)
