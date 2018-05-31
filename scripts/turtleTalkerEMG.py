#!/usr/bin/env python
# ElectroMyoGraphy (EMG)

import rospy
from myo_raw import *
from geometry_msgs.msg import Twist, Vector3

# set topic and send data
def turtleTalkerEMG():
    pub_emg = rospy.Publisher('/turtlesim1/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('myo_turtlesim_driver', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    m = MyoRaw()

    # connect and initialize Myo for EMG tranfer
    def initMyo(m):
        print("Initializing myo...")
        m.add_emg_handler(proc_emg)
        m.connect()

    # take EMG and process it
    # chance to analyze new gestures
    # emg list[8]
    # move Uint8
    def proc_emg(emg, move):
        print(emg)
        print(move)
        # pub_emg.publish(Twist(linear,angular))

    # disconnect Myo
    def byeMyo(m):
        print
        print("Disconnecting myo...")
        m.disconnect()
        print

    initMyo(m)
    # send processed EMG data on cmd_vel topic
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
        turtleTalkerEMG()
    except rospy.ROSInterruptException:
        pass
