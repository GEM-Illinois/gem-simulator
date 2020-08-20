import math
import os

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point, Pose, Quaternion
from controller import bicycleModel

class TrajectoryMarking:

    def __init__(self):
        self.count = 0 
        self.model = bicycleModel()
        rospy.Subscriber("/gazebo/set_model_state", ModelState, self.model_state_updated)
        rospy.sleep(0.5)

    def get_yaw_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def model_state_updated(self, model_state):
        if model_state.model_name == 'polaris':
            self.show_marker(model_state)

    def show_marker(self, model_state):
        x = model_state.pose.orientation.x
        y = model_state.pose.orientation.y
        z = model_state.pose.orientation.z
        w = model_state.pose.orientation.w
        
        curr_angle = self.get_yaw_from_quaternion(x, y, z, w)
        curr_velocity = math.sqrt(
            model_state.twist.linear.x ** 2 + model_state.twist.linear.y ** 2
        )

        next_position = model_state.pose.position
        next_position.x += math.cos(curr_angle) * curr_velocity * 0.2
        next_position.y += math.sin(curr_angle) * curr_velocity * 0.2
        next_position.z = 1

        marker_state = ModelState()
        marker_state.model_name = 'marker'
        marker_state.pose = Pose(
            position=next_position,
            orientation=Quaternion(0, 0, 0, 0)
        )
        self.model.modelStatePub.publish(marker_state)


if __name__ == '__main__':
    rospy.init_node("trajectory_marking_node", anonymous=True)
    trajectory_marking = TrajectoryMarking()
    rospy.sleep(0.5)
    rospy.spin()