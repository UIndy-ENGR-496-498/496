from setuptools import setup

package_name = 'imu_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyserial'],  # Include pyserial if needed
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='IMU data publisher',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'publisher = imu_publisher.publisher:main',  # Match this with your Python file
        ],
    },
)


