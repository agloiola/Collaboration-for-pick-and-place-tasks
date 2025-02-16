#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

def get_robot_pose():
    """Obtém a posição e orientação do objeto publicado em /turtle_pose"""
    try:
        msg = rospy.wait_for_message("/turtle_pose", PoseStamped)
        rospy.loginfo(f"Posição recebida: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
        rospy.loginfo(f"Orientação recebida: {msg.pose.orientation.x}, {msg.pose.orientation.y}, {msg.pose.orientation.z}, {msg.pose.orientation.w}")
        return msg.pose  # Retorna posição + orientação
    except rospy.ROSException:
        rospy.logwarn("Não foi possível obter a nova posição do robô!")
        return None

def movebase_client():
    rospy.init_node('movebase_client')  

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Aguardando servidor move_base...")
    client.wait_for_server()
    
    # Obtém a posição e orientação do objeto
    object_pose = get_robot_pose()
    if object_pose is None:
        rospy.logerr("Abortando: não foi possível obter a posição do objeto.")
        return

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = 0.55
    goal.target_pose.pose.position.y = 0.35
    goal.target_pose.pose.orientation.w = 1

    rospy.loginfo("Enviando objetivo para o move_base...")
    client.send_goal(goal)  
    client.wait_for_result()
    rospy.sleep(1)
    
    
    goal.target_pose.pose = object_pose

    client.send_goal(goal)  
    client.wait_for_result()
    
    # Publica o sinal de movimentação do robô
    pub = rospy.Publisher('/robot_moveu', Bool, queue_size=10)
    rospy.sleep(1)
    
    if client.get_state() == GoalStatus.SUCCEEDED:
        rospy.logwarn("Robô moveu para a posição desejada")
        pub.publish(True)
        rospy.loginfo("Sinal de chegada publicado!")
    else:
        rospy.logwarn("Erro: O robô não conseguiu se mover para a posição desejada")
        pub.publish(False)
	
    rospy.sleep(2) 

if __name__ == "__main__":
    movebase_client()
