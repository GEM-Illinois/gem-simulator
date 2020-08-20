# demo.py (originally mp2.py) adapted from ECE 498 SMA Spring 2020 MP2 Group 1

import math
import time

import rospy
from gazebo_msgs.msg import  ModelState
from controller import bicycleModel


if __name__ == "__main__":
    rospy.init_node("model_dynamics")

    model = bicycleModel()

    endList = 0

    #wait till a waypoint is received
    while(not model.waypointList):
        pass
        rospy.sleep(7)

    targetState = ModelState()
    targetState = model.waypointList.pop(0)

    x_points = []
    y_points = []

    start = time.time()

    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        currState =  model.getModelState()
        if not currState.success:
            continue


        distToTargetX = abs(targetState.pose.position.x - currState.pose.position.x)
        distToTargetY = abs(targetState.pose.position.y - currState.pose.position.y)

        if(distToTargetX < 1 and distToTargetY < 1):
            if not model.waypointList:
                newState = ModelState()
                newState.model_name = 'polaris'
                newState.pose = currState.pose
                newState.twist.linear.x = 0
                newState.twist.linear.y = 0
                newState.twist.angular.z = 0
                model.modelStatePub.publish(newState)
                #only print time the first time waypontList is empty
                if(not endList):
                    endList = 1
                    end = time.time()
            else:
                if(endList):
                    start = time.time()
                    endList = 0
                targetState = model.waypointList.pop(0)
                markerState = ModelState()
                markerState.model_name = 'marker'
                markerState.pose = targetState.pose
                # model.modelStatePub.publish(markerState)
        else:
            model.setModelState(currState, targetState)
            x_points.append(currState.pose.position.x)
            y_points.append(currState.pose.position.y)
            print(distToTargetX,distToTargetY)
            markerState = ModelState()
            markerState.model_name = 'marker'
            markerState.pose = targetState.pose
            # model.modelStatePub.publish(markerState)

    rospy.spin()
