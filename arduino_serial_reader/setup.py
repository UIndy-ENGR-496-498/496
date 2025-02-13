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
            # The left-hand string is how you’ll call it with ros2 run:
            #   ros2 run arduino_serial_reader serial_reader_node
            # The right-hand side must point to: <python_package>.<script_file>:<function_name>
            'serial_reader_node = arduino_serial_reader.serial_reader_node:main',
        ],
    },
)
