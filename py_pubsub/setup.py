from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sf515-David',
    maintainer_email='david960156@gmail.com',
    description='EMU3000 description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'talker2 = py_pubsub.publisher:main',
            'listener2 = py_pubsub.subscriber:main',
            'uwb_fan=py_pubsub.uwb_fan:main',

        ],
    },
)
