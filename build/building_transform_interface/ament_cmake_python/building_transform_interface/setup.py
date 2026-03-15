from setuptools import find_packages
from setuptools import setup

setup(
    name='building_transform_interface',
    version='0.0.0',
    packages=find_packages(
        include=('building_transform_interface', 'building_transform_interface.*')),
)
