#!/usr/bin/env python

import rospy
from myo_raw import *
from geometry_msgs.msg import Pose, Point, Twist, Quaternion, Vector3
from gazebo_msgs.msg import ModelState

# set topic and send data
def turtleTalkerPOSE():
    # pub_pose_turtle = rospy.Publisher('/turtlesim1/turtle1/cmd_vel', Twist, queue_size=10)
    pub_pose_p3at = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.init_node('myo_turtlesim_driver', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    m = MyoRaw()

    # connect and initialize Myo for POSE tranfer
    def initMyo(m):
        print("Initializing myo...")
        m.add_arm_handler(proc_arm)
        m.add_pose_handler(proc_pose)
        m.connect()

    # take ARM and store it
    def proc_arm(arm, xdir):
        m.Arm = arm.value
        m.XDirection = xdir.value

    # take POSE and process it
    def proc_pose(pose):
        p = Point()

        # FIST -> go ahead
        if pose.value == 1:
            linear = Vector3(2, 0, 0)
            angular = Vector3(0, 0, 0)
        # WAVE_IN + RIGHT ARM-> turn left
        elif pose.value == 2 and m.Arm == 1:
            linear = Vector3(0, 0, 0)
            angular = Vector3(0, 0, 1.8)
        # WAVE_IN + LEFT ARM-> turn right
        elif pose.value == 2 and m.Arm == 2:
            linear = Vector3(0, 0, 0)
            angular = Vector3(0, 0, -1.8)
        # WAVE_OUT + RIGHT ARM -> turn right
        elif pose.value == 3 and m.Arm == 1:
            linear = Vector3(0, 0, 0)
            angular = Vector3(0, 0, -1.8)
        # WAVE_OUT + LEFT ARM -> turn left
        elif pose.value == 3 and m.Arm == 2:
            linear = Vector3(0, 0, 0)
            angular = Vector3(0, 0, 1.8)
        # FINGERS_SPREAD -> go back
        elif pose.value == 4:
            linear = Vector3(-2, 0, 0)
            angular = Vector3(0, 0, 0)
        # PINCH -> go diagonally
        elif pose.value == 5:
            linear = Vector3(2, 0, 0)
            angular = Vector3(0, 0, 1.8)
        # DEFAULT -> do nothing
        else:
            linear = Vector3(0, 0, 0)
            angular = Vector3(0, 0, 0)

        # pub_pose_turtle.publish(Twist(linear,angular))
        pub_pose_p3at.publish("pioneer3at", Pose(p, Quaternion(0, 0, 0, 0)), Twist(linear, angular), "world")

    # disconnect Myo
    def byeMyo(m):
        print
        print("Disconnecting myo...")
        m.disconnect()
        print

    initMyo(m)
    # send processed POSE data on cmd_vel topic
    while not rospy.is_shutdown():
        try:
            m.run(1)
            # rospy.loginfo()
            # pub.publish()
        except KeyboardInterrupt:
            pass

    byeMyo(m)

if __name__ == '__main__':
    try:
        turtleTalkerPOSE()
    except rospy.ROSInterruptException:
        pass
