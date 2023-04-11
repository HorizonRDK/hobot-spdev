'''
COPYRIGHT NOTICE
Copyright 2023 Horizon Robotics, Inc.
All rights reserved.
Date: 2023-04-11 15:50:01
LastEditTime: 2023-04-11 15:50:02
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
               'Programming Language :: Python :: 3.8',
               'Topic :: Software Development',
               'Topic :: System :: Hardware']

setup(
    name="hobot_vio",
    version="2.0.0",
    author="horizon",
    author_email="technical_support@horizon.ai",
    description="VIO of python API",
    classifiers = classifiers,
    package_dir = {'': './'},
    packages = ['hobot_vio'],
    package_data = {'hobot_vio': ['libhbspdev.so', 'libsrcampy.so',]},
    include_package_data = True,
)
