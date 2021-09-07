#! /usr/bin/env python

# import ros stuff
import rospy
import random
from std_srvs.srv import *

# service callback
def rand_targ(req):
    print("Choosing a random position among (-4,-3)(-4,2)(-4,7)(5,-7)(5,-3)(5,-1);")
    x = random.choice([-4, 5])
    if(x==-4):
        y = random.choice([-3, 2, 7])
    else:
        y = random.choice([-3, 1, -7])

    rospy.set_param("des_pos_x", x)
    rospy.set_param("des_pos_y", y)
    #print("Position set: x = " + str(x) + ", y = " + str(y))
    return []


def main():
    rospy.init_node('choose_target')

    x = rospy.get_param("des_pos_x")
    y = rospy.get_param("des_pos_y")
    
    srv = rospy.Service('random_target', Empty, rand_targ)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
