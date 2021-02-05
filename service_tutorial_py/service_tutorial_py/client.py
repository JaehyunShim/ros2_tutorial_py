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

import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class Client(Node):  # inherit from Node

    def __init__(self):
        super().__init__('client')  # name the node "client"
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        self.timer = self.create_timer(0.5, lambda: self.send_request())

    def send_request(self):
        # Cancel timer and send a request only once
        self.timer.cancel()

        # Wait for server
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('server not available, waiting again...')

        # Define a service request
        self.req = AddTwoInts.Request()
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])

        # Call call_async() method
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        response = self.future.result()
        self.get_logger().info(
            'Result of add_two_ints: for %d + %d = %d' %
            (self.req.a, self.req.b, response.sum))


def main(args=None):
    rclpy.init(args=args)
    client = Client()
    rclpy.spin(client)
    client.destroy_node()  # destory node explicitly
    rclpy.shutdown()


if __name__ == '__main__':
    main()
