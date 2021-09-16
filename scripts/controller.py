#! /usr/bin/env python

# This node implement the robot behavior according to the command inserted through the user interface.
# By default the active algorithm is move_base. The robot waits for a user command 
# start any behaviur.
# The robot can reach a random target, one chosen by the user, 
# can follow the walls, stop in the current position and switch the navigation
# algorithm from move_base to bug0 and viceversa.

import rospy
import time
from std_srvs.srv import *
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

srv_client_random_target_ = None
srv_client_user_interface_ = None
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
srv_bug0_client = None
mb_goal = None

## Bug algorithm boolean
bug_active = False

## Goal publishers
msg_target_pub = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=1)    # Publish next goal
cancel_trg_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size = 1)           # Cancel goal 

## Null velocity publisher (stop option)
cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


# Services
srv_client_random_target_ = rospy.ServiceProxy('/random_target', Empty)             # Service called to generate a random target
srv_client_user_interface_ = rospy.ServiceProxy('/user_interface', Empty)           # User interface: gets the desired mode and manual goal if needed
srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)    # Wall follower service
srv_bug0_client = rospy.ServiceProxy('/bug0_service',SetBool)			        	# Bug0 algorithm
srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)        # Go to point service

## Function called when move_base is the active algorithm.
#
# Gets the target position set in the parameters by the concerning services and 
# publishes it as goal position with a message of type move_base_msgs/MoveBaseActionGoal 
# on the topic move_base /goal.
def set_goal():
	global msg_target_pub, mb_goal
	
	#set the goal for move_base 
	mb_goal = MoveBaseActionGoal()
	
	mb_goal.goal.target_pose.header.frame_id = "map"
	mb_goal.goal.target_pose.pose.orientation.w = 1
	mb_goal.goal.target_pose.pose.position.x =  rospy.get_param("des_pos_x")
	mb_goal.goal.target_pose.pose.position.y =  rospy.get_param("des_pos_y")
			
	#publish the msg
	msg_target_pub.publish(mb_goal)


## Callback function of the move_base algorithm.
# It is called every time the algorithm publishes a message of to the topic /move_base/result.
# In case of success it switches the "target_reached" parameter to True
# and prints the status on the console
def result_callback(result):
    status = result.status.status

    if status == result.status.SUCCEEDED:
        rospy.set_param("state", 2)     # state 2 = 'target reached'
        rospy.set_param("target_reached", True)
        print(result.status.text)
        
## Function for stopping the robot
# The function is calling mode 4, which stops the robot.
# In case there is a target to be reached, the function sends a message of type
# actionlib_msgs/GoalID to the topic /move_base/cancel.
# It also sends a message to /cmd_vel with null velocity.
def stop_robot():
    global cancel_trg_pub, mb_goal
    # set null velocity
    rospy.set_param("state", 3)     # state 3: 'stopped'

    if mb_goal != None:
        cnl_goal = GoalID()

        cnl_goal.stamp.secs = 0
        cnl_goal.stamp.nsecs = 0
        cnl_goal.id = ""
        cancel_trg_pub.publish(cnl_goal)
    
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    cmdvel_pub.publish(twist_msg)
    
    

def main():
    global  srv_client_random_target_, srv_client_user_interface_, bug_active
    global  srv_client_wall_follower_, srv_bug0_client

    rospy.init_node('controller')
    rospy.wait_for_service("user_interface")      #wait for the user interface server in roscore

    # Subscribe to the topic /move_base/result to get the status with move_base algorithm
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, result_callback)        


    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        srv_client_user_interface_()        # Call user_interface service
        time.sleep(1)

        mode = rospy.get_param("mode")

        # 1. Move randomly to one of the positions: (-4,3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1) 
        if(mode == 1):     
            resp = srv_client_wall_follower_(False)    # Deactivate wall follower service (if active)

            rospy.set_param("target_reached", False)
            srv_client_random_target_()     # Get a random target from the service

            # wait for the target to be different than 0
            while rospy.get_param("des_pos_x") ==  0 and rospy.get_param("des_pos_y") == 0:
                continue

            if bug_active == False:     # If the functioning algorithm is move_base
                set_goal()

            print("Reaching position [" + str(rospy.get_param("des_pos_x")) + ";" + str(rospy.get_param("des_pos_y")) + "]\n")     
            rospy.set_param("state", 0)     # state 0 = go to point


        # 2. Manually inser one of the 6 positions among the list above;
        elif(mode == 2):
            resp = srv_client_wall_follower_(False)     # Deactivate wall follower service (if active)

            rospy.set_param("target_reached", False)

            # wait for the target to be different than 0
            while rospy.get_param("des_pos_x") ==  0 and rospy.get_param("des_pos_y") == 0:
                continue

            if bug_active == False:     # If the functioning algorithm is move_base
                set_goal()

            print("Reaching position [" + str(rospy.get_param("des_pos_x")) + ";" + str(rospy.get_param("des_pos_y")) + "]\n")     
            rospy.set_param("state", 0)     # state 0 = go to point


        # 3. Start following the external walls;
        elif(mode == 3):
            while rospy.get_param("target_reached") == False:        # while state is 'go to point'
                print("!!! WARNING: waiting until the target is reached!")
                time.sleep(3)        #busy waiting

            print("Starting to follow the external walls:")
            resp = srv_client_wall_follower_(True)      # Activate wall follower service
            rospy.set_param("state", 1)     # state 1 = wall following
            
                
        # 4. Stop in the last position;
        elif(mode == 4):
            resp = srv_client_wall_follower_(False)     # Deactivate wall follower service (if active)

            rospy.set_param("target_reached", True)

            print("Robot has stopped")
            stop_robot()    


        # 5. Change algorithm;
        elif(mode == 5):
            resp = srv_client_wall_follower_(False)     # Dectivate wall follower service (if active)

            while rospy.get_param("target_reached") == False:        # while state is 'go to point'
                print("!!! WARNING: waiting until the target is reached!")
                time.sleep(3)        #busy waiting

            if bug_active == False:     # If the functioning algorithm is move_base
                srv_bug0_client(True)   # switch to Bug0
                bug_active = True
                print("Bug algorithm has been activated")
            
            else:                       # If the functioning algorithm is Bug0
                srv_bug0_client(False)  # switch to move_base
                bug_active = False
                print("Move base algorithm has been activated")


        else:
            print("INVALID MODE INPUT")

    
    rate.sleep()


if __name__ == "__main__":
    main()
