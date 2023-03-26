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
    name="hobot_sppy",
    version="2.0.0",
    author="Horizon Robotics",
    author_email="technical_support@horizon.ai",
    description="hobot multimedia python interface",
    classifiers = classifiers,
    package_dir = {'': './'},
    packages = ['hobot_sppy'],
    package_data = {'hobot_sppy': ['libhbspdev.so','libsppydev.so']},
    include_package_data = True,
)
