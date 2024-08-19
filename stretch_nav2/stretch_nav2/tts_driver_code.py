#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
import yaml
import os
import threading
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tts_file import GPT4SpeechProcessor
import time

class Nav2GoalPublisher(Node):
    def __init__(self):
        super().__init__('nav2_goal_publisher')
        self.navigator = BasicNavigator()
        # Load the goal positions from the YAML file
        yaml_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'goals.yaml')
        with open(yaml_file, 'r') as file:
            self.goals = yaml.safe_load(file)['goal_names']
        # Give Initial Poses based on amcl_pose
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()
        self.current_goal_name = None
        self.goals_from_speech(self.goals)

    def pose_callback(self,msg):
        self.get_logger().info("Setting Initial Pose")
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        if msg is None:
            initial_pose.pose.position.x = 0.0
            initial_pose.pose.position.y = 0.0
            initial_pose.pose.orientation.z = 0.0
            initial_pose.pose.orientation.w = 1.0
        else:
            initial_pose.pose.position.x = msg.pose.pose.position.x
            initial_pose.pose.position.y = msg.pose.pose.position.y
            initial_pose.pose.orientation.z = msg.pose.pose.orientation.z
            initial_pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.navigator.setInitialPose(initial_pose)
            
    def goals_from_speech(self, goal_list):
        self.get_logger().info("Speech Recognition Started")
        goal_name_list = []
        GPT4SpeechProcessor().speak_text("Where do you want to go?")
        for goal_names in goal_list:
            print('Goal',goal_names)
            goal_name_list.append(goal_names)
        speech = GPT4SpeechProcessor(prefeeded_words=goal_name_list, logger = self.get_logger())
        words = speech.process_speech()
        if words == 0:
            self.goals_from_speech(goal_list)
        elif words == 1:
            rclpy.shutdown()

        else:
            self.publish_goal(words[0])
            # self.get_logger().info(goal_list[words[0]])
    # def service_callback(self, request):
    #     self.get_logger().info('Service called to send goal via Terminal')
    #     position_name = request.data.strip()
    #     self.publish_goal(position_name)
    #     self.get_logger().info(f"Going to {position_name}")
    
    def publish_goal(self, position_name):
        if position_name not in self.goals:
            self.get_logger().error(f"Position '{position_name}' not found in the YAML file.")
            return

        goal = self.goals[position_name]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal['x']
        goal_pose.pose.position.y = goal['y']

        # Convert theta to quaternion (yaw only, assuming roll and pitch are 0)
        # from math import sin, cos
        # qz = sin(goal['theta'] / 2.0)
        # qw = cos(goal['theta'] / 2.0)
        # goal_pose.orientation.z = qz
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Going to '{position_name}'")
        # self.get_logger().info(goal_pose)
        if self.current_goal_name:
            self.cancel_goal()
        self.navigator.goToPose(goal_pose)
        self.current_goal_name = position_name
        # self.while_navigating() # Can talk while navigating (tell about the place visiting)
        result = self.navigator.getResult()
        print(result)
        self.get_logger().info('Goal complete! Waiting for next Input...')
        speaking = GPT4SpeechProcessor()

        if result == TaskResult.UNKNOWN:
            self.get_logger().info('Goal complete! Waiting for next Input...')
            speaking.speak_text("We Will be reaching the desitination in sometime.....")
            time.sleep(2)
            speaking.speak_text("Do you want to go someplace else? I'm listening, Just say the place..")
            self.goals_from_speech(self.goals)
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Route was canceled, exiting.')
            speaking.speak_text("The path was canceled")
            rclpy.shutdown()
        elif result == TaskResult.FAILED:
            self.get_logger().info('Route failed!')
            speaking.speak_text("Path failed due to unkown reasons")

    def while_navigating(self):
        while not self.navigator.isTaskComplete():
            time.sleep(5)
            GPT4SpeechProcessor().speak_text("I am happy to assist you today")

    def cancel_goal(self):
        self.navigator.cancelTask()
        self.get_logger().info(f"Canceled current goal '{self.current_goal_name}'")
        self.current_goal_name = None


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalPublisher()
    print('Starting')
    rclpy.spin(node)
    print('shutdown')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
