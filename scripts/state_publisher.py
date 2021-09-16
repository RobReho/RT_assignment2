#! /usr/bin/env python

import rospy
import time
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations

regions_ = None     # Laser directions
xposition_ = None   # Robot Base coordinates
yposition_ = None
zposition_ = None
state_ = None       # State number
state_desc_ = ['Go to point', 'wall following', 'target reached','stopped']

## Odometry callback
# Acquires the robot base coordinates as variables
def clbk_odom(msg):
    global xposition_, yposition_, zposition_

    xposition_ = msg.pose.pose.position.x
    yposition_ = msg.pose.pose.position.y
    zposition_ = msg.pose.pose.position.z


## Laserscan callback
# Groups laser data into 5 directions
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

    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)    # update laser scan data
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)       # update position coordinates

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("__________________________________________\n")
        
        # Print state: 'Go to point', 'wall following', 'target reached','stopped'
        state = rospy.get_param('state')
        state_ = state
        print("state:")
        print("     " + str(state_desc_[state]))
        # Print reaching target when present
        if state == 0:
            print("     [" + str(rospy.get_param("des_pos_x")) + ";" + str(rospy.get_param("des_pos_y")) + "]\n")     
        
        # Print robot base position
        print("position:")
        print("     " + str(xposition_))
        print("     " + str(yposition_))
        print("     " + str(zposition_))

        # Print the distance from the obstacle in every region
        print("obstacle distance:")
        print("     " + str(regions_))

        # Explicit when the target has been reached (boolean paramenter)
        print("target reached: " + str(rospy.get_param("target_reached")))  

        print("__________________________________________\n")
        
        time.sleep(3)   # Print informations every 3 seconds to prevent the console from being crowded

    rate.sleep(1)


if __name__ == "__main__":
    main()