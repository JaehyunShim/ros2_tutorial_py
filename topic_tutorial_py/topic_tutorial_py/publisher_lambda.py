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
from std_msgs.msg import String


class Publisher(Node):  # inherit from Node

    def __init__(self):
        super().__init__('publisher_lambda')  # initialize Node with the name "publisher_lambda"
        self.publisher = self.create_publisher(String, 'topic_lambda', 10)

        """Lambda expression cannot have multiple lines"""
        self.timer = self.create_timer(
            0.5,  # call timer_callback() every 0.5 seconds
            lambda: self.timer_callback())  # Lambda expression without arguments
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)
    publisher.destroy_node()  # destory node explicitly
    rclpy.shutdown()


if __name__ == '__main__':
    main()
