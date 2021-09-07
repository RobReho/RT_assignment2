#! /usr/bin/env python

# import ros stuff
import rospy
from std_srvs.srv import *


# service callback
def man_pos_clbk(request):
    while True:
        print("Choose one point among [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)]\n")
        x = int(raw_input('target x coordinate:'))
        y = int(raw_input('target y coordinate:'))

        #check if is an avaible target otherwise repeat 
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

    print("Chosen target [" + str(rospy.get_param("/des_pos_x")) + ";" + str(rospy.get_param("/des_pos_y")) + "]\n") 

    return []

## Main function to execute the service
def main():
    #initilize node
    rospy.init_node('man_targ_service')
    
    #call user target request service
    srv = rospy.Service("/man_targ_service",Empty,man_pos_clbk)

    rospy.spin()
        
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

