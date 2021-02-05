# Copyright Open Source Robotics Foundation, Inc.
# Copyright 2021, Jaehyun Shim, ROBOTIS CO., LTD.
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

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class Server(Node):  # inherit from Node

    def __init__(self):
        super().__init__('server')
        self.server = self.create_service(AddTwoInts, 'add_two_ints', self.server_callback)

    def server_callback(self, request, response):
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        response.sum = request.a + request.b
        return response


def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    server.destroy_node()  # destory node explicitly
    rclpy.shutdown()


if __name__ == '__main__ ':
    main()
