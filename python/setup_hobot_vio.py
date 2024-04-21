'''
COPYRIGHT NOTICE
Copyright 2024 D-Robotics, Inc.
All rights reserved.
'''
import os
import sys
import subprocess

import setuptools
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

# option can be x86 or aarch64
arch="aarch64"

classifiers = ['Operating System :: POSIX :: Linux',
               'License :: OSI Approved :: MIT License',
               'Intended Audience :: Developers',
               'Programming Language :: Python :: 3.10',
               'Topic :: Software Development',
               'Topic :: System :: Hardware']

setup(
    name="hobot_vio",
    version="2.3.0",
    author="d-robotics",
    author_email="technical_support@d-robotics.cc",
    description="python API for VIO",
    classifiers = classifiers,
    package_dir = {'': './'},
    packages = ['hobot_vio'],
    package_data = {'hobot_vio': ['libhbspdev.so', 'libsrcampy.so',]},
    include_package_data = True,
)
