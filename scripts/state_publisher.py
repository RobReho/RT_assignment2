#! /usr/bin/env python

import rospy
import time
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations

regions_ = None
xposition_ = None
yposition_ = None
zposition_ = None
state_ = None
state_desc_ = ['Go to point', 'wall following', 'stopped','target reached']

def clbk_odom(msg):
    global xposition_, yposition_, zposition_

    xposition_ = msg.pose.pose.position.x
    yposition_ = msg.pose.pose.position.y
    zposition_ = msg.pose.pose.position.z


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }


def main():
    global regions_, position_ ,state_, state_desc_

    rospy.init_node('state_publisher')
    print("Robot state informations:")

    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)    #update laser scan data
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)       #update position coordinates

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("__________________________________________\n")
        state = rospy.get_param('state')
        state_ = state
        print("state:")
        print("     " + str(state_desc_[state]))
        if state == 0:
            print("     [" + str(rospy.get_param("des_pos_x")) + ";" + str(rospy.get_param("des_pos_y")) + "]\n")     
        
        print("position:")
        print("     " + str(xposition_))
        print("     " + str(yposition_))
        print("     " + str(zposition_))

        print("obstacle distance:")
        print("     " + str(regions_))

        print("target reached:")
        if state == 2:
            print("     Yes")
        else:
            print("     No")
            
        #print(rospy.get_param("target_reached"))

        print("__________________________________________\n")
        
        time.sleep(3)

    rate.sleep(1)


if __name__ == "__main__":
    main()