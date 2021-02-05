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

from action_tutorials_interfaces.action import Fibonacci
import rclpy
import rclpy.action
from rclpy.node import Node


class ActionClient(Node):  # inherit from Node

    def __init__(self):
        super().__init__('action_client')  # initialize the Node with the name "action_client"
        self.action_client_ = rclpy.action.ActionClient(self, Fibonacci, 'fibonacci')
        self.timer = self.create_timer(0.5, lambda: self.send_goal(10))

    def send_goal(self, order):
        # Cancel timer and send an action goal only once
        self.timer.cancel()

        # Wait for action server
        self.action_client_.wait_for_server()

        # Define a action goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Call send_goal_async() method
        self.send_goal_future = self.action_client_.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # result callback
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))


def main(args=None):
    rclpy.init(args=args)
    action_client = ActionClient()
    rclpy.spin(action_client)
    action_client.destroy_node()  # destory node explicitly
    rclpy.shutdown()


if __name__ == '__main__':
    main()
