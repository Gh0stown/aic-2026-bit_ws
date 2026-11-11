#!/usr/bin/env python3
"""Standalone non-ROS entry point, with source-checkout import support."""
import os
import sys

_source = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if os.path.isdir(os.path.join(_source, "mlx90640_ros")):
    sys.path.insert(0, _source)

from mlx90640_ros.viewer import main

if __name__ == "__main__":
    sys.exit(main())
