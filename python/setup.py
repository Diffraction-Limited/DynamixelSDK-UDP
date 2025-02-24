
from setuptools import setup, find_packages
import platform

setup(
    name='dynamixel_sdk_udp',
    version='3.8.1',
    packages=['dynamixel_sdk_udp'],
    package_dir={'': 'src'},
    license='Apache 2.0',
    description='Dynamixel SDK 3 + UDP. python package',
    long_description=open('README.txt').read(),
    url='https://github.com/Diffraction-Limited/DynamixelSDK-UDP',
    author='Adam Robichaud',
    author_email='arobichaud@diffractionlimited.com',
    install_requires=['pyserial']
)
