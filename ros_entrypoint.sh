#!/bin/bash
set -e

# Source the ROS2 setup script
source "/opt/ros/jazzy/setup.bash"

# Execute the command passed to the container
exec "$@"
