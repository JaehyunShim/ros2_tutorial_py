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


class Subscriber(Node):  # inherit from Node

    def __init__(self):
        super().__init__('subscriber_lambda')  # initialize Node with the name "subscriber_lambda"
        self.subscriber = self.create_subscription(
            String,
            'topic_lambda',
            lambda msg: self.get_logger().info('Received: "%s"' % msg.data),
            10)


def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()  # destory node explicitly
    rclpy.shutdown()


if __name__ == '__main__':
    main()
