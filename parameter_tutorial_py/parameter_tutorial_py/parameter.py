# Copyright 2016 Open Source Robotics Foundation, Inc.
# Copyright 2021 Jaehyun Shim
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
# from rclpy.exceptions import ParameterNotDeclaredException
# from rcl_interfaces.msg import ParameterType


class Paremeter(Node):  # inherit from Node

    def __init__(self):
        super().__init__('parameter')  # initialize 'Node' with the name 'parameter'
        timer_period = 2  # seconds
        self.declare_parameter('my_parameter', 'world')  # declare a parameter
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Get parameters
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info('Hello %s!' % my_param)

        # Set parameters
        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'Korea'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)


def main():
    rclpy.init()
    parameter = Paremeter()
    rclpy.spin(parameter)
    parameter.destroy_node()  # destory node explicitly
    rclpy.shutdown()


if __name__ == '__main__':
    main()
