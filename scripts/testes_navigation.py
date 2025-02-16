#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler  # Importação correta


def movebase_client():
    
    rospy.init_node('movebase_client')
    

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position.x =-2.5
    goal.target_pose.pose.position.y = 0.5
    
    # Converter de Euler para quaternion
    q = quaternion_from_euler(0, 0, 2.042)  # Exemplo de ângulo em radianos
    
    # Configurar a orientação do goal com o quaternion
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    
    client.send_goal(goal)  
    client.wait_for_result()
    
    if client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("Robô moveu para a posição desejada")
    else:
        rospy.loginfo("Erro ao mover o robô")

if __name__ == "__main__":
    movebase_client()

