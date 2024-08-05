
# ROS2 Unit Testing

This repository contains a small example how to setup and write unit tests with and for ROS2 Jazzy, using the [Pytest framework](https://docs.pytest.org/en/stable/)

## Setup

The dockerfile provides a development setup that can be run on any operating system. First, build a container and then run it using:

```bash
docker build -t ros2_dev:jazzy . 
docker run -it --rm --name ros2_dev_container -v <PATH_TO_SRC_DIR>:/ros2_ws/src ros2_dev:jazzy
```

This will create an interactive docker container that will be deleted once it is stopped. The `src` directory is mounted into the container, allowing changes made on the host system or inside the docker container to be reflected on both sides.

Compile the sources using `colcon` and source the enviromnent inside the docker container. The `symlink` flag allows editing Python code files without having to recompile them:
```bash
colcon build --symlink-install
source install/setup.bash
```

## Run tests

Colcon comes with support to [run tests](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/CLI.html). If correctly setup, this should allow to run tests for both, nodes written in Python and nodes written in C++.

It also allows to specify arguments for `pytest`. For example, to run all tests and print the `pytest` output run:

```bash
colcon test --event-handlers console_cohesion+
```


Tests can be found in `src/demo/test`. By default, ROS2 will create tests to check the copyright licence and code formatting when creating a new package.