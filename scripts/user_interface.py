#! /usr/bin/env python

## User interface
# different from the given one
# Gets mode command and user coordinates (in case of mode 2)

# import ros stuff
import rospy
import time
from std_srvs.srv import *
#from final_assignment.srv import Usrcommand


# service callback
def man_pos_clbk(request):

    # Selection of the use mode
    print("Please choose one of the 4 modes:")
    print("1. Move randomly to one of the positions: (-4,3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1) ")
    print("2. Manually inser one of the 6 positions among the list above;")
    print("3. Start following the external walls;")
    print("4. Stop in the last position;")
    print("5. Change algorithm;")

    while True:
        mode = int(raw_input('mode :'))     # Get command
        if mode >> 0 and mode << 6:         # Check is the input command is acceptable
            rospy.set_param("mode", mode)
            print("Thanks! Got it")
            break
        else:
            print("Invalid command input, please retry\n")
    

    # Ask for manual input in case of mode 2
    if rospy.get_param("mode") == 2:
        while True:
            print("Choose one point among [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)]\n")
            x = int(raw_input('target x coordinate:'))
            y = int(raw_input('target y coordinate:'))

            # check if it is an avaible target otherwise repeat 
            if x == -4:
                if y == -3 or y == 2 or y == 7:
                    break
                else:
                    print("Invalid target point, please try again\n")
            elif x == 5: 
                if y == -7 or y == -3 or y == 1:
                    break
                else:
                    print("Invalid target point, please try again\n")
            else:
                print("Invalid target point, please try again\n")
        rospy.set_param("/des_pos_x",x)
        rospy.set_param("/des_pos_y",y)

   
        # I use parameters instead of returning the values of x and y through the service
        # because the values need to be returned only in case of mode 1 and 2


    return []

## Main function to execute the service
def main():
    #initilize node
    rospy.init_node('user_interface')
    
    #call user target request service
    srv = rospy.Service("/user_interface",Empty,man_pos_clbk)

    rospy.spin()
        
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

