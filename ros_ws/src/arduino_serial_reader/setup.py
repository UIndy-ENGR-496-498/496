from setuptools import setup
from setuptools import setup

package_name = 'arduino_serial_reader'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anas',
    maintainer_email='anas@example.com',
    description='A package to read serial data from Arduino.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_reader_node = arduino_serial_reader.serial_reader_node:main',
        ],
    },
    data_files=[('share/' + package_name, ['package.xml'])],  # <== Add this line
)

