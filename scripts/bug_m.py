#! /usr/bin/env python

## Modifications:
# - activation service switch
# - expiring timer

import rospy
import time
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist

import math

# publisher variable for robot's velocity
pub = None

# Client go to point service
srv_client_go_to_point_ = None

# Client wall follower service
srv_client_wall_follower_ = None

# Orientation
yaw_ = 0

# Orentationa angle allowed
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees

# Robot base position
position_ = Point()

# Target position
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0

# Laser directions
regions_ = None     

# int state variable
state_ = 0
state_desc_ = ['Go to point', 'wall following', 'target reached']

# initialize algorithm as not active
active_ = False     

# Timer
timeout = None      

# 0 - go to point
# 1 - wall following


# callbacks

## Callback function to activate or deactivate the service Bug0
def bug0Callback(req):
	global active_
	active_ = req.data
	res = SetBoolResponse()
	res.success = True
	res.message = 'The algorithm has been switched!'
	return res

## Odometry informations about the robot
def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


## Laser datas will be divided in 5 subcatecories (directions)
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

## Changes the state of the robot, updates the "state" and "target reached" 
# parameters and activate the appropriate services
def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:         # go to point
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
        rospy.set_param("target_reached", False)    # update parameter
        rospy.set_param("state", 0)     # go to point

    if state_ == 1:         # wall follower
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
        rospy.set_param("state", 1)     # wall following

    if state_ == 2:         # target reached
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
        rospy.set_param("target_reached", True)    
        rospy.set_param("state", 2)     # target reached 

# Sets robot in the right direction
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle



def main():
    time.sleep(2)
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_, active_
    global srv_client_go_to_point_, srv_client_wall_follower_, srv_client_user_interface_, pub, timeout

    rospy.init_node('bug0')

    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    #srv_client_user_interface_ = rospy.ServiceProxy('/user_interface', Empty)
    srv = rospy.Service('bug0_service', SetBool, bug0Callback)

    
    # initialize initial state as "target reached"
    change_state(2)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        if not active_:   # Switch variable that activates the Bug service
            rate.sleep()
            continue

        else:
            if state_ == 0:
                err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))

                #if timer is expired change state to 2: target reached and print target aborted
                if rospy.Time.now() >= timeout:
                    rospy.set_param("mode",4)
                    rospy.set_param("target_reached",True)
                    change_state(2)
                    print("The timer has expired. Target aborted.")

                if(err_pos < 0.3):
                    change_state(2) # target reached

                elif regions_['front'] < 0.5:
                    change_state(1) # wall follower

            elif state_ == 1:
                desired_yaw = math.atan2(
                    desired_position_.y - position_.y, desired_position_.x - position_.x)
                err_yaw = normalize_angle(desired_yaw - yaw_)
                err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                        2) + pow(desired_position_.x - position_.x, 2))

                #if timer is expired change state to 2: target reached and print target aborted
                if rospy.Time.now() >= timeout:
                    rospy.set_param("mode",4)
                    rospy.set_param("target_reached",True)
                    change_state(2)
                    print("The timer has expired. Target aborted.")

                if(err_pos < 0.3):
                    change_state(2)
                if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
                    change_state(0)

            elif state_ == 2:

                #give time to user to insert the command
                while rospy.get_param("/mode") !=  1 and rospy.get_param("/mode") != 2:
                    continue

                desired_position_.x = rospy.get_param('des_pos_x')
                desired_position_.y = rospy.get_param('des_pos_y')
                err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))

                # If the target is far from the robot position:
                if(err_pos > 0.35):
                    print "Starting timer"
                    now = rospy.Time.now()      # Starts the timer
                    timeout = now + rospy.Duration(40) # 40 seconds of timeout
                    change_state(0)     # change to go to point state

            rate.sleep()


if __name__ == "__main__":
    main()
