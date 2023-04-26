import sensor_msgs.msg as sensor_msgs
import rospy


class Controller_wrapper:
    def __init__(self) -> None:
        rospy.init_node("controller_wrapper")
        self.sub_joy = rospy.Subscriber("/joy", sensor_msgs.Joy, self.joy_callback)
        self.rate = rospy.Rate(10)  # 10hz
        self.prev_scale: float
        self.prev_right_trig: float
        self.prev_left_trig: float
        self.prev_stick_x: float
        self.scale: float
        self.right_trig: float
        self.left_trig: float
        self.stick_x: float

    def joy_callback(self, msg: sensor_msgs.Joy):
        self.msg = msg
