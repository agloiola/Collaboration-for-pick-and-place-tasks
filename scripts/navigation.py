#!/usr/bin/env python3


import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry

from tf.transformations import quaternion_from_euler  # Importação correta

'''
def get_robot_orientation():
    """Obtém a orientação inicial do robô no mapa"""
    try:
        msg = rospy.wait_for_message("/odom", Odometry, timeout=5)
        return msg.pose.pose.orientation
    except rospy.ROSException:
        rospy.logwarn("Não foi possível obter a orientação inicial!")
        return None
'''
def movebase_client():
    
    rospy.init_node('movebase_client')
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    # Obtém a orientação inicial do robô
    '''initial_orientation = get_robot_orientation()
    if initial_orientation is None:
        rospy.logerr("Abortando: não foi possível obter a orientação inicial.")
        return'''

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.position.y = 0.3
    
    
    
       # Converter de Euler para quaternion
    q = quaternion_from_euler(0, 0, 1.414)  # Exemplo de ângulo em radianos
    
    # Configurar a orientação do goal com o quaternion
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    
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

