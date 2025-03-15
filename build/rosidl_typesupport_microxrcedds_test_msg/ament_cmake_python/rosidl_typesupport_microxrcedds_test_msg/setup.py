from setuptools import find_packages
from setuptools import setup

setup(
    name='rosidl_typesupport_microxrcedds_test_msg',
    version='3.0.1',
    packages=find_packages(
        include=('rosidl_typesupport_microxrcedds_test_msg', 'rosidl_typesupport_microxrcedds_test_msg.*')),
)
