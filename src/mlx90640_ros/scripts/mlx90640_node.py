#!/usr/bin/env python3
"""ROS entry point, also runnable from an unbuilt source checkout."""
import os
import sys

_source = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if os.path.isdir(os.path.join(_source, "mlx90640_ros")):
    sys.path.insert(0, _source)

from mlx90640_ros.node import main

if __name__ == "__main__":
    sys.exit(main())
