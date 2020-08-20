# controller.py and bicycleModel adapted from ECE 498 SMA Spring 2020 MP2 Group 1

import math
import numpy as np

import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive


class bicycleModel():

    def __init__(self):

        """ Series of provided waypoints """
        pt1 = ModelState()
        pt2 = ModelState()
        pt3 = ModelState()
        pt4 = ModelState()
        pt5 = ModelState()
        pt6 = ModelState()
        pt7 = ModelState()

        pt1.pose.position.x = 10
        pt1.pose.position.y = 0
        pt1.twist.linear.x = .25
        pt1.twist.linear.y = .25

        pt2.pose.position.x = 10
        pt2.pose.position.y = 10
        pt2.twist.linear.x = .25
        pt2.twist.linear.y = .25

        pt3.pose.position.x = -10
        pt3.pose.position.y = -10
        pt3.twist.linear.x = .25
        pt3.twist.linear.y = .25

        pt4.pose.position.x = 10
        pt4.pose.position.y = -10
        pt4.twist.linear.x = .25
        pt4.twist.linear.y = .25

        pt5.pose.position.x = -10
        pt5.pose.position.y = -10
        pt5.twist.linear.x = .25
        pt5.twist.linear.y = .25

        pt6.pose.position.x = -10
        pt6.pose.position.y = 0
        pt6.twist.linear.x = .25
        pt6.twist.linear.y = .25

        pt7.pose.position.x = 0
        pt7.pose.position.y = 0
        pt7.twist.linear.x = .25
        pt7.twist.linear.y = .25


        self.waypointList = [pt1, pt2, pt3, pt4, pt5, pt6, pt7]


        # self.waypointList = []

        self.length = 1.88

        self.waypointSub = rospy.Subscriber("/gem/waypoint", ModelState, self.__waypointHandler, queue_size=1)
        self.waypointPub = rospy.Publisher("/gem/waypoint", ModelState, queue_size=1)


        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)



    def getModelState(self):
        """
            Description:
                Requests the current state of the polaris model when called

            Returns:
                modelState: contains the current model state of the polaris vehicle in gazebo
        """
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState


    def rearWheelModel(self, ackermannCmd):
        """
            Description:
                Contains the mathematical model that will represent the vehicle in gazebo

            Inputs:
                ackermannCmd (AckermannDrive): contains desired vehicle velocity and steering angle velocity
                                               that the model should follow

            Returns:
                A List containing the vehicle's x velocity, y velocity, and steering angle velocity
        """
        # Acquire the current state of the vehicle from Gazebo simulator
        currentModelState = self.getModelState()

        if not currentModelState.success:
            return

        # Convert the orientation of the vehicle from quaternion to euler angle
        current_quaternion_angle = currentModelState.pose.orientation
        current_angle = self.quaternion_to_euler(
                                current_quaternion_angle.x,
                                current_quaternion_angle.y,
                                current_quaternion_angle.z,
                                current_quaternion_angle.w)[2]

        # Compute the rate of change for the state variables by using the
        #     differential equations specified above
        desired_vehicle_velocity = ackermannCmd.speed
        steering_angle_velocity = ackermannCmd.steering_angle_velocity

        x_velocity = desired_vehicle_velocity * math.cos(current_angle)
        y_velocity = desired_vehicle_velocity * math.sin(current_angle)

        # Return the computed result
        return [x_velocity, y_velocity, steering_angle_velocity]

    def rearWheelFeedback(self, currentState, targetState):
        """
            Description:
                Feedback loop which drives the vehicles to the current waypoint

            Inputs:
                currentState (ModelState): The curret state of the vehicle in gazebo
                targetState  (ModelState): The desired target state of the vehicle in gazebo

            Returns:
                ackermannCmd (AckermannDrive): Will be used to compute the new x,y, and steering angle
                                               velocities of the model
        """
        # Acquire the current state of the vehicle from the input of the function
        current_quaternion_angle = currentState.pose.orientation
        current_vel = math.sqrt((currentState.twist.linear.x*currentState.twist.linear.x) + ((currentState.twist.linear.y*currentState.twist.linear.y)))
        #  Convert the orientation of the vehicle from quaternion to euler angle
        current_angle = self.quaternion_to_euler(
                                current_quaternion_angle.x,
                                current_quaternion_angle.y,
                                current_quaternion_angle.z,
                                current_quaternion_angle.w)[2]

        target_quaternion_angle = targetState.pose.orientation
        target_angle = self.quaternion_to_euler(target_quaternion_angle.x, target_quaternion_angle.y, target_quaternion_angle.z, target_quaternion_angle.w)[2]

        targetVel = math.sqrt(targetState.twist.linear.x ** 2 + targetState.twist.linear.y ** 2)
        targetAngVel = targetState.twist.angular.z

        # Compute the error vector
        t_x_r = targetState.pose.position.x
        t_y_r = targetState.pose.position.y

        c_x_r = currentState.pose.position.x
        c_y_r = currentState.pose.position.y

        rotation_matrix = np.array(
            [[math.cos(current_angle), math.sin(current_angle), 0],
             [-math.sin(current_angle), math.cos(current_angle), 0],
             [0, 0, 1]]
             )
        difference_vector = np.array(
            [[t_x_r - c_x_r],
            [t_y_r - c_y_r],
            [ target_angle - current_angle]]
            )
        error_vector = np.matmul(rotation_matrix, difference_vector)
        # Gain constants
        k1 = 5
        k2 = 5
        k3 = 0.5
        # Compute the controller input (speed and steering angle velocity)
        v_r = targetVel * math.cos(error_vector[2]) + k1 * error_vector[0]
        omega = targetAngVel + targetVel * (k2 * error_vector[1] + k3 * math.sin(error_vector[2]))
        # Pack the controller input to an AckermannDrive message and return the message
        return AckermannDrive(target_angle, omega, v_r, 0, 0)

    def setModelState(self, currState, targetState):
        """
            Description:
                Sets state of the vehicle in gazebo.

                This function is called by mp2.py at a frequency of apporximately 100Hz

            Inputs:
                currState   (ModelState): The curret state of the vehicle in gazebo
                targetState (ModelState): The desired target state of the vehicle in gazebo

            Returns:
                None
        """

        # Compute the controller input to the vehicle
        controller_input = self.rearWheelFeedback(currState, targetState)
        # Using the controller input from previous step to compute the rate of change for the state variables
        rate_of_change = self.rearWheelModel(controller_input)
        # Use the rate of change of the state variables to update state of the vehicle
        newState = ModelState()
        newState.model_name = 'polaris'
        newState.pose = currState.pose
        newState.twist.linear.x = rate_of_change[0]
        newState.twist.linear.y = rate_of_change[1]
        newState.twist.angular.z = rate_of_change[2]
        self.modelStatePub.publish(newState)

    def quaternion_to_euler(self, x, y, z, w):
        """
            Description:
                converts quaternion angles to euler angles. Note: Gazebo reports angles in quaternion format

            Inputs:
                x,y,z,w:
                    Quaternion orientation values

            Returns:
                List containing the conversion from quaternion to euler [roll, pitch, yaw]
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]


    def __waypointHandler(self, data):
        """
            Description:
                Callback handler for the /gem/waypoint topic. If a waypoint is published to
                this topic, this function will be called and append the published waypoint to
                the waypoint list.

            Inputs:
                data (ModelState): the desired state of the model

            Returns:
                None
        """
        self.waypointList.append(data)
