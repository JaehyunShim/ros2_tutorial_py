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

from std_msgs.msg import String

g_node = None  # declare a global node here to use it in the callback function


def listener_callback(msg):
    global g_node
    g_node.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    global g_node
    rclpy.init(args=args)  # initialize rclpy
    g_node = rclpy.create_node('subscriber_old_school')  # intialize node
    subscriber = g_node.create_subscription(
      String, 'topic_old_school', listener_callback, 10)  # initialize subscriber
    subscriber  # to prevent unused variable warning

    while rclpy.ok():
        rclpy.spin_once(g_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
