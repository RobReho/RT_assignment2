#! /usr/bin/env python

import rospy
import time
from std_srvs.srv import *
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Twist

srv_client_random_target_ = None
srv_client_manual_target_ = None
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
srv_bug0_client = None

## Bug algorithm boolean
bug_active = False

## Target position publisher 
msg_target_pub = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=1)

## Null velocity publisher (stop option)
cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

# Services
srv_client_random_target_ = rospy.ServiceProxy('/random_target', Empty)
srv_client_manual_target_ = rospy.ServiceProxy('/man_targ_service', Empty)
srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
srv_bug0_client = rospy.ServiceProxy('/bug0_service',SetBool)				#bug0 algorithm

def set_goal():
	global msg_target_pub
	
	#set the goal for move_base 
	mb_goal = MoveBaseActionGoal()
	
	mb_goal.goal.target_pose.header.frame_id = "map"
	mb_goal.goal.target_pose.pose.orientation.w = 1
	mb_goal.goal.target_pose.pose.position.x =  rospy.get_param("des_pos_x")
	mb_goal.goal.target_pose.pose.position.y =  rospy.get_param("des_pos_y")
			
	#publish the msg
	msg_target_pub.publish(mb_goal)



def result_callback(result):
    print(result.status.text)
    #rospy.set_param("target_reached", True)
    rospy.set_param("state", 3)     # state 3 = 'target reached'



def stop_robot():
    # set null velocity
    rospy.set_param("state", 2)     # state 2: 'stopped'
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    cmdvel_pub.publish(twist_msg)



def main():
    global  srv_client_random_target_, srv_client_user_interface_, bug_active
    global  srv_client_wall_follower_, target_, srv_client_manual_target_, srv_bug0_client

    rospy.init_node('controller')
   # rospy.wait_for_service("command_service")	#wait for the server in the roscore
    
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, result_callback)

    #srv_bug0_client(False)  # Deactivate bug algorithm as default

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        mode = rospy.get_param("mode")

        print("Please choose one of the 4 modes:")
        print("1. Move randomly to one of the positions: (-4,3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1) ")
        print("2. Manually inser one of the 6 positions among the list above;")
        print("3. Start following the external walls;")
        print("4. Stop in the last position;")
        print("5. Change algorithm;")

        mode = int(raw_input('mode :'))
        rospy.set_param("mode", mode)
        print("Thanks! Let's execute")

        if(mode == 1):
            resp = srv_client_wall_follower_(False)
            srv_client_random_target_()

            if bug_active == False:
                set_goal()

            print("Reaching position [" + str(rospy.get_param("des_pos_x")) + ";" + str(rospy.get_param("des_pos_y")) + "]\n")     
            rospy.set_param("state", 0)     # state 0 = go to point

        elif(mode == 2):
            resp = srv_client_wall_follower_(False)
            srv_client_manual_target_()
            
            if bug_active == False:
                set_goal()

            print("Reaching position [" + str(rospy.get_param("des_pos_x")) + ";" + str(rospy.get_param("des_pos_y")) + "]\n")     
            rospy.set_param("state", 0)     # state 0 = go to point
            
        elif(mode == 3):
            if rospy.get_param("state") != 2:        # while state differend than 'target reached'
                print("WARNING: wait until the target is reached!")
            else:
                print("Starting to follow the external walls:")
                resp = srv_client_wall_follower_(True)
                rospy.set_param("state", 1)     # state 1 = wall following

        elif(mode == 4):
            resp = srv_client_wall_follower_(False)
                
            #while rospy.get_param("state") != 2:        # while state differend than 'target reached'
            #    time.sleep(1)        #busy waiting

            print("Robot in stopped mode")
            stop_robot()

        elif(mode == 5):
            resp = srv_client_wall_follower_(False)

            while rospy.get_param("state") != 2:        # while state differend than 'target reached'
                time.sleep(1)        #busy waiting

            if bug_active == False:
                srv_bug0_client(True)
                bug_active = True
                print("Bug algorithm has been activated")
            
            else:
                srv_bug0_client(False)
                bug_active = False
                print("Move base algorithm has been acctivated")

        else:
            print("INVALID MODE INPUT")

    
    rate.sleep()


if __name__ == "__main__":
    main()
