import math
import time


import rospy
from gazebo_msgs.msg import  ModelState
from controller import bicycleModel

from sensorFusion import sensorFusion


if __name__ == "__main__":
    rospy.init_node("model_dynamics")

    model = bicycleModel()
    fusion = sensorFusion()

    endList = 0;

    #wait till a waypoint is received
    while(not model.waypointList):
        pass
        rospy.sleep(7)

    targetState = ModelState()
    targetState = model.waypointList.pop(0)

    safe = True
    start = time.time()

    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state
        currState =  model.getModelState()
        if not currState.success:
            continue

        fusion.fuseSensors()
        distToTargetX = abs(targetState.pose.position.x - currState.pose.position.x)
        distToTargetY = abs(targetState.pose.position.y - currState.pose.position.y)

        pedDists = fusion.getPedDistances()
        if len(pedDists) > 0:
            for dist in pedDists:
                print("dist to ped", abs(abs(dist) - abs(currState.pose.position.y)))
                if(abs(abs(dist) - abs(currState.pose.position.y))<= 5):
                    safe = False
                    break
                else:
                    safe = True
        else:
            safe = True

        if(safe):
            #print("safe")
            if(distToTargetX < .55 and distToTargetY < .55):
                if not model.waypointList:
                    newState = ModelState()
                    newState.model_name = 'polaris_ranger_ev'
                    newState.pose = currState.pose
                    newState.twist.linear.x = 0
                    newState.twist.linear.y = 0
                    newState.twist.angular.z = 0
                    model.modelStatePub.publish(newState)
                    #only print time the first time waypontList is empty
                    if(not endList):
                        endList = 1
                        end = time.time()
                        print("Time taken:", end-start)
                else:
                    if(endList):
                        start = time.time()
                        endList = 0
                    #print("reached:", targetState.pose.position.x, targetState.pose.position.y)
                    targetState = model.waypointList.pop(0)
                    #print("headed to:",  targetState.pose.position.x, targetState.pose.position.y)
                    markerState = ModelState()
                    markerState.model_name = 'marker'
                    markerState.pose = targetState.pose
                    model.modelStatePub.publish(markerState)
            else:
                model.setModelState(currState, targetState)
                #print(distToTargetX,distToTargetY)
                markerState = ModelState()
                markerState.model_name = 'marker'
                markerState.pose = targetState.pose
                model.modelStatePub.publish(markerState)
        else:
            print("unsafe")
            newState = ModelState()
            newState.model_name = 'polaris_ranger_ev'
            newState.pose = currState.pose
            newState.twist.linear.x = 0
            newState.twist.linear.y = 0
            newState.twist.angular.z = 0
            model.modelStatePub.publish(newState)





    rospy.spin()
