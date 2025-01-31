#!/usr/bin/env python3


import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry



def get_robot_orientation():
    """Obtém a orientação inicial do robô no mapa"""
    try:
        msg = rospy.wait_for_message("/odom", Odometry, timeout=5)
        return msg.pose.pose.orientation
    except rospy.ROSException:
        rospy.logwarn("Não foi possível obter a orientação inicial!")
        return None

def movebase_client():
    
    rospy.init_node('movebase_client')
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    # Obtém a orientação inicial do robô
    initial_orientation = get_robot_orientation()
    if initial_orientation is None:
        rospy.logerr("Abortando: não foi possível obter a orientação inicial.")
        return

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.position.y = 0.3
    
    # Mantém a orientação inicial
    goal.target_pose.pose.orientation = initial_orientation


    client.send_goal(goal)  
    client.wait_for_result()
    
    pub = rospy.Publisher('/robot_arrival', Bool, queue_size=10)
    rospy.sleep(1)
    
    if client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("Robô moveu para a posição desejada")
        rospy.sleep(2)    
        pub.publish(True)  # Publica o sinal de chegada
        rospy.loginfo("Sinal de chegada publicado!")
    else:
        rospy.loginfo("Erro")
        pub.publish(False)  # Publica um sinal de erro, caso necessário
	
    rospy.sleep(2)     
    
	
if __name__ == "__main__":
    movebase_client()

