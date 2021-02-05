# ros2_tutorial_py
[![GitHub License](https://img.shields.io/github/license/JaehyunShim/ros2_tutorial_py)](https://github.com/JaehyunShim/ros2_tutorial_py/blob/master/LICENSE)
[![GitHub CI Status](https://github.com/JaehyunShim/ros2_tutorial_py/workflows/CI/badge.svg)](https://github.com/JaehyunShim/ros2_tutorial_py/actions?query=workflow%3ACI)
[![GitHub Lint Status](https://github.com/JaehyunShim/ros2_tutorial_py/workflows/Lint/badge.svg)](https://github.com/JaehyunShim/ros2_tutorial_py/actions?query=workflow%3ALint)
[![codecov](https://codecov.io/gh/JaehyunShim/ros2_tutorial_py/branch/master/graph/badge.svg)](https://codecov.io/gh/JaehyunShim/ros2_tutorial_py)
<!-- [![Documentation Status](https://readthedocs.org/projects/ros2-tutorial-cpp/badge/?version=latest)](https://ros2-tutorial-cpp.readthedocs.io/en/latest/?badge=latest)
[![Doxygen](https://img.shields.io/badge/doxygen-documentation-blue.svg)](https://jaehyunshim.github.io/docs.ros2_tutorial_py.org/) -->

## Contents
- ROS2 Python Topic (Pubisher/Subscriber)
- ROS2 Python Service (Client/Server)
- ROS2 Python Action (Action Client/Action Server)
- ROS2 Python Parameter

## Run
```sh
# Topic tutorial example
$ ros2 run topic_tutorial_py publisher_old_school
$ ros2 run topic_tutorial_py subscriber_old_school
$ ros2 run topic_tutorial_py publisher_member_function
$ ros2 run topic_tutorial_py subscriber_member_function
$ ros2 run topic_tutorial_py publisher_lambda
$ ros2 run topic_tutorial_py subscriber_lambda

# Service tutorial example
$ ros2 run service_tutorial_py client 1 2
$ ros2 run service_tutorial_py server

# Action tutorial example
$ ros2 run action_tutorial_py action_server
$ ros2 run action_tutorial_py action_client
$ ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
$ ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}" --feedback

# Parameter tutorial example
$ ros2 run parameter_example parameter
$ ros2 launch parameter_tutorial_py parameter.launch.py
$ ros2 launch parameter_tutorial_py parameter2.launch.py
$ ros2 param get /parameter my_parameter
$ ros2 param set /parameter my_parameter "world"
```

## Reference
- [ROS2 Tutorials](https://index.ros.org/doc/ros2/Tutorials/)
- [ROS2 Examples](https://github.com/ros2/examples)
- [ROS2 Demos](https://github.com/ros2/demos)

## Issue
- https://github.com/ros2/demos/issues/483
