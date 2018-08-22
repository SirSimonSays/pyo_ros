#!/usr/bin/env python
# Inertial Measurement Unit (IMU)

import rospy
from myo_raw import *
from math import sqrt, asin, atan2
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose, Point, Twist, Quaternion, Vector3
# from gazebo_msgs.msg import ModelState

# https://github.com/thalmiclabs/myo-bluetooth/blob/master/myohw.h#L292-L295
# typedef struct MYOHW_PACKED {
#     /// Orientation data, represented as a unit quaternion. Values are multiplied by MYOHW_ORIENTATION_SCALE.
#     struct MYOHW_PACKED {
#         int16_t w, x, y, z;
#     } orientation;
#
#     int16_t accelerometer[3]; ///< Accelerometer data. In units of g. Range of + -16.
#                               ///< Values are multiplied by MYOHW_ACCELEROMETER_SCALE.
#     int16_t gyroscope[3];     ///< Gyroscope data. In units of deg/s. Range of + -2000.
#                               ///< Values are multiplied by MYOHW_GYROSCOPE_SCALE.
# } myohw_imu_data_t;
#
# define MYOHW_ORIENTATION_SCALE   16384.0f ///< See myohw_imu_data_t::orientation
# define MYOHW_ACCELEROMETER_SCALE 2048.0f  ///< See myohw_imu_data_t::accelerometer
# define MYOHW_GYROSCOPE_SCALE     16.0f    ///< See myohw_imu_data_t::gyroscope

flag = False

def callback(data):
    global flag
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    flag = not flag

# set topic and send data
def turtleTalkerIMU():
    # pub_imu_turtle = rospy.Publisher('/turtlesim1/turtle1/cmd_vel', Twist, queue_size=10)
    # pub_imu_p3at = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    pub_imu_P3AT = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
    rospy.init_node('myo_turtlesim_driver', anonymous=True)
    sub_key = rospy.Subscriber('/key', Int16, callback)
    rate = rospy.Rate(10) # 10hz
    m = MyoRaw()

    # connect and initialize Myo for IMU tranfer
    def initMyo(m):
        print("Initializing myo...")
        m.add_imu_handler(proc_imu)
        m.connect()

    # take IMU data and process it
    def proc_imu(quatr, acc, gyro):
        quat = Quaternion(quatr[0]/16384.0, quatr[1]/16384.0, quatr[2]/16384.0, quatr[3]/16384.0)
        ## Normalize quaternion (x^2 + y^2 + z^2 + w^2 = 1) and accelerometer values
        # quatNorm = sqrt(quat.x*quat.x + quat.y*quat.y + quat.z*quat.z + quat.w*quat.w)
        # normQuat = Quaternion(quat.x/quatNorm, quat.y/quatNorm, quat.z/quatNorm, quat.w/quatNorm)
        # normAcc = Vector3(acc[0]/2048.0, acc[1]/2048.0, acc[2]/2048.0)
        # normGyro = Vector3(gyro[0]/16.0, gyro[1]/16.0, gyro[2]/16.0)

        # arm rotation, clockwise increase
        roll = atan2(2.0 * (quat.w * quat.x + quat.y * quat.z), 1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y))
        # move arm up(go straight) and down(go back)
        pitch = asin(max(-1.0, min(1.0, 2.0 * (quat.w * quat.y - quat.z * quat.x))))
        # move arm left and right (turn left and turn right)
        yaw = atan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))

        if(flag):
        # pub_imu_turtle.publish(Twist(Vector3(-pitch, 0.0, 0.0), Vector3(0.0, 0.0, yaw)))
        # pub_imu_p3at.publish("pioneer3at", Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 0)), Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)), "world")
            pub_imu_P3AT.publish(Twist(Vector3(-pitch, 0.0, 0.0), Vector3(0.0, 0.0, yaw)))
        else:
            pub_imu_P3AT.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

    # disconnect Myo
    def byeMyo(m):
        print
        print("Disconnecting myo...")
        m.disconnect()
        print

    initMyo(m)
    # send processed IMU data on cmd_vel topic
    while not rospy.is_shutdown():
        try:
            # 1 millis timeout between each packet
            m.run(1)
            # rate.sleep()
        except KeyboardInterrupt:
            pass

    byeMyo(m)

if __name__ == '__main__':
    try:
        turtleTalkerIMU()
    except rospy.ROSInterruptException:
        pass
