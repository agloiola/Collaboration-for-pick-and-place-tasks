#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from std_msgs.msg import Bool


robot_arrival = False

#Recebe o sinal de chegada do robô, para assim o braço realizar tentativas de pick
def robot_arrival_callback(msg):
        global robot_arrival
        robot_arrival = msg.data
        rospy.loginfo("Recebido sinal de chegada do robô móvel: " + str(robot_arrival))
                
if __name__ == '__main__':
    
    rospy.init_node('demo_detach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")
    
    rospy.Subscriber("/robot_arrival", Bool, robot_arrival_callback)

    rospy.loginfo("robot_arrival sinal do robô")
    while not robot_arrival:
        rospy.sleep(0.1)

    # Detach os links
    rospy.loginfo("Detaching turtlebot3_waffle_pi and object")
    req = AttachRequest()
    req.model_name_1 = "turtlebot3_waffle_pi"
    req.link_name_1 = "base_footprint"
    req.model_name_2 = "object"
    req.link_name_2 = "link_00"


    attach_srv.call(req)
    rospy.logwarn("Detach objeto")
    
'''rosservice call /link_attacher_node/detach "{model_name_1: 'turtlebot3_waffle_pi', 
link_name_1: 'base_footprint', 
model_name_2: 'object', 
link_name_2: 'link_00'}"
'''