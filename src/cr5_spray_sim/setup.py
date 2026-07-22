#!/usr/bin/env python3
"""
Catkin Python 包配置 — cr5_spray_sim.

安装路径:
  src/cr5_spray_sim/ → devel/lib/python3/dist-packages/cr5_spray_sim/

提供:
  from cr5_spray_sim import aruco_compat
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["cr5_spray_sim"],
    package_dir={"": "src"},
)

setup(**setup_args)
