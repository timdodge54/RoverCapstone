#!/usr/bin/python

"""This node listens to a Joy message as published by joystick_drivers joy_node and publishes a Twist
message to be interpreted by a driver node. If an AR tag has not been seen recently, zero the Twist."""

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray


class ArmComm:
    def __init__(self):
        rospy.init_node("arm_relay")
        rospy.loginfo("Starting arm_communication node")

        self.arm_comm = rospy.Publisher("arm_joints", Int16MultiArray, queue_size=1)
        rospy.loginfo("Publishing to arm_joints")
        self.lin_comm = rospy.Publisher("lin_joints", Int16MultiArray, queue_size=1)
        rospy.loginfo("Publishing to lin_joints")

        self.sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        rospy.loginfo("Subscribed to joy")
        self.arm_comm_command = None
        self.lin_comm_command = None

        rate = rospy.Rate(10)
        rospy.loginfo("Starting main loop")
        while not rospy.is_shutdown():
            if self.arm_comm_command is not None:
                rospy.loginfo("Publishing self.arm_comm_command:")
                rospy.loginfo(self.arm_comm_command)
                self.arm_comm.publish(self.arm_comm_command)
                self.arm_comm_command = None
            if self.lin_comm_command is not None:
                rospy.loginfo("Publishing self.lin_comm_command:")
                rospy.loginfo(self.lin_comm_command)
                self.lin_comm.publish(self.lin_comm_command)
                self.lin_comm_command = None
            rate.sleep()
        rospy.on_shutdown(self.shutting_down)

    def shutting_down(self):
        """Shut down the node"""
        rospy.loginfo("Shutting down arm_communication node")

    def joy_callback(self, data):
        """Receive joystick data, formulate Twist message.
        Use planner if a secondary button is pressed"""
        joy = data
        rospy.loginfo(data)
        bumper_l = joy.buttons[4]
        bumper_r = joy.buttons[5]
        trigger_l = joy.axes[2]
        trigger_r = joy.axes[5]
        ud_dpad = joy.axes[7]
        lr_dpad = joy.axes[6]
        elbow = 0
        if bumper_l != 0 and bumper_r == 0:
            elbow = -2
        elif bumper_r != 0 and bumper_l == 0:
            elbow = 2
        wrist = 0
        if trigger_l < 0:
            wrist = -2
        elif trigger_r < 0:
            wrist = 2
        base = 0
        if ud_dpad < 0:
            base = 2
        elif ud_dpad > 0:
            base = -2
        lin_base = 0
        if lr_dpad < 0:
            lin_base = 2
        elif lr_dpad > 0:
            lin_base = -2
        msg_to_publish_joints = Int16MultiArray()
        msg_to_publish_joints.data = [base, elbow, wrist]
        msg_to_publish_lin = Int16MultiArray()
        msg_to_publish_lin.data = [lin_base]
        self.arm_comm_command = msg_to_publish_joints
        self.lin_comm_command = msg_to_publish_lin



if __name__ == "__main__":
    try:
        ArmComm()
    except rospy.ROSInterruptException:
        pass
