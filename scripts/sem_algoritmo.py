#!/usr/bin/env python3

import rospy, sys, copy, math, tf
from math import pi
import rospkg
import numpy as np

from gazebo_msgs.srv import GetModelState
import geometry_msgs.msg
import moveit_commander 
from geometry_msgs.msg import PoseStamped, Quaternion, Pose

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
from std_msgs.msg import Bool


GRIPPER_EFFORT = [10.0]
GRIPPER_OPEN = [0.037, -0.037]
GRIPPER_CLOSED = [0.015, -0.015]
GRIPPER_FRAME = "wx250s/right_finger_link"
GRIPPER_JOINT_NAMES = ['left_finger', 'right_finger']
REFERENCE_FRAME = 'world'

robot_arrived = False

class MoveItDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)  

        
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()
        
        arm_group = moveit_commander.MoveGroupCommander("interbotix_arm", ns="/wx250s")
        hand_group = moveit_commander.MoveGroupCommander("interbotix_gripper", ns="/wx250s")
        

        arm_group.set_max_velocity_scaling_factor(1.0)
        arm_group.set_max_acceleration_scaling_factor(1.0)
        
        # Allow some leeway in position (meters) and orientation (radians)
        arm_group.set_goal_position_tolerance(0.05)
        arm_group.set_goal_orientation_tolerance(0.1)
        
        # Permite o replanejamento
        arm_group.allow_replanning(True)
        
        # tempo por tentativa de planejamento
        arm_group.set_planning_time(10)
        
        # Limite de tentativas de pick
        max_pick_attempts = 8
            
        # Limite de tentativas de place
        max_place_attempts = 10
        
        # referencia do braço
        arm_group.set_pose_reference_frame(REFERENCE_FRAME)
    
        eef_link = arm_group.get_end_effector_link()
        touch_links = robot.get_link_names(group="interbotix_gripper")
        
        #----------------------------------------------------------------------#
        #Espera o sinal de chegada do robô
        rospy.Subscriber('/robot_arrival', Bool, self.robot_arrival_callback)
        
        rospy.loginfo("Esperando o robô se posicionar")
        while not robot_arrived:
            rospy.sleep(0.1)
	
        #----------------------------------------------------------------------#
        #Após o robô carregar o objeto , o braço inicia o procedimento de pegar o objeto
        
        
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
                
        scene_objects = self.create_scene(scene)
        rospy.loginfo('Cena Criada. Os objetos sao: ' + str(scene_objects))
        arm_group.set_support_surface_name('turtlebot3_waffle_pi')
        rospy.sleep(3)
        
        # abre a garra
        arm_group.set_named_target("Home")
        plan = arm_group.go(wait=True)
        if plan:
            rospy.loginfo("Braço na posição inicial")
        else:
            rospy.loginfo("ERRO")
            moveit_commander.roscpp_shutdown()
            sys.exit()    

        # abre a garra
        hand_group.set_named_target("Open")
        plan = hand_group.go(wait=True)
        if plan:
            rospy.loginfo("GARRA ABERTA")
        else:
            rospy.loginfo("ERRO AO ABRIR A GARRA")
            moveit_commander.roscpp_shutdown()
            sys.exit()    
          
        # Define a pose de destino do objeto
        place_pose = PoseStamped()
        place_pose.header.frame_id = "world"
        place_pose.pose.position.x =  0.4
        place_pose.pose.position.y = -0.24
        place_pose.pose.position.z = 0.13
        place_pose.pose.orientation.w = 1.0


        object_pose = scene.get_object_poses(['object'])['object']

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "world" 
        grasp_pose.pose = object_pose
        
        rospy.loginfo("Obteu a posicao do objeto...")     

        grasp_pose.pose.position.y += 0.01
        grasp_pose.pose.position.x += 0.02
        grasp_pose.pose.position.z += 0.06
        
        grasps = self.make_grasps(grasp_pose, ['object'])
       
       
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.0001)

        result = None
        n_attempts = 0
        
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Tentativa de pick: " +  str(n_attempts))
            result = arm_group.pick('object', grasps)
            rospy.sleep(1)
            
      
        if result != MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Tentativa de preensão falhou. Código de erro: {}".format(result))
        else:
            rospy.loginfo("PICK DEU CERTO...")
            rospy.loginfo("Preensão bem-sucedida após {} tentativas".format(n_attempts))
        
        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
            places = self.make_places(place_pose)
            
            cont = 1
            for place in places:
                self.gripper_pose_pub.publish(place)
               
            
            while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                #rospy.loginfo(place)
                result = arm_group.place('object', place)
                if result == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("PLACE DEU CERTO...")
                    break
                rospy.sleep(0.01)
                
            if result != MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")
            
       
        rospy.sleep(2)
        
        # Coloca o braço na posição inicial
        arm_group.set_named_target('Sleep')
        success = arm_group.go(wait=True)  
        if success:
            rospy.loginfo("O BRAÇO RETORNOU PARA A POSIÇÃO INICIAL")
        else:
            rospy.logerr("ERRO AO RETORNAR PARA A POSIÇÃO INICIAL")

       
        rospy.sleep(1)

        scene.clear()
        
        
    # Postura do gripper
    def make_gripper_posture(self, joint_positions):
    
        t = JointTrajectory()

        t.joint_names = GRIPPER_JOINT_NAMES

        tp = JointTrajectoryPoint()

        tp.positions = joint_positions

        tp.effort = GRIPPER_EFFORT

        tp.time_from_start = rospy.Duration(10)

        t.points.append(tp)

        return t

    # Gerar a translação do gripper na direção dada pelo vetor
    def make_gripper_translation(self, min_dist, desired, vector):
       
        g = GripperTranslation()

        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        g.direction.header.frame_id = GRIPPER_FRAME
        g.min_distance = min_dist
        g.desired_distance = desired

        return g

    # Gera uma lista de possiveis grasps para "pick"
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
     
        g = Grasp()  
        
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
     
                
        g.pre_grasp_approach = self.make_gripper_translation(0.02, 0.2, [1.0,0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.05, 0.1, [0.0, -1.0, 1.0])
        
        g.grasp_pose = initial_pose_stamped

        pitch_vals = [ 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
        
        yaw_vals = [ 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
      
        x_vals = list(np.arange(-0.03, 0.03, 0.005))
        y_vals = list(np.arange(-0.01, 0.01, 0.005))
        z_vals = list(np.arange(-0.01, 0.01, 0.005))

        
        grasps = []

        for yam in yaw_vals:
            for p in pitch_vals: 
                for x in x_vals:
                    for y in y_vals:
                        for z in z_vals:
                              # Copia a pose inicial
                            g.grasp_pose = deepcopy(initial_pose_stamped)
                                    
                            g.grasp_pose.pose.position.x += x
                            g.grasp_pose.pose.position.y += y
                            g.grasp_pose.pose.position.z += z
                            
                            q = quaternion_from_euler(0, p, yam)
                            
                            g.grasp_pose.pose.orientation.x = q[0]
                            g.grasp_pose.pose.orientation.y = q[1]
                            g.grasp_pose.pose.orientation.z = q[2]
                            g.grasp_pose.pose.orientation.w = q[3]                    
                        
                            g.id = str(len(grasps))
                                                
                            g.allowed_touch_objects = allowed_touch_objects
                            g.max_contact_force = 10.0
                            
                            g.grasp_quality = 1.0 - abs(p)

                            #adiciona o elemnto no final da lista
                            grasps.append(deepcopy(g))  
                                
        # Retorna a lista
        return grasps
    
    
    # Gera uma lista de poses para "place"
    def make_places(self, init_pose):
       
        place = PoseStamped()
        
        x_vals = [0, 0.005, 0.01, -0.005, -0.01]

        y_vals = [0, 0.005, 0.01, -0.005, -0.01]
        
        z_vals = [0, 0.005, 0.01, -0.005, -0.01]

        pitch_vals = [ 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
        
        yaw_vals = [ 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]

        places = []

        # Gera uma posição de "place" para cada angulo e posição
        for yam in yaw_vals:
            for p in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        for z in z_vals:
                        # Copia a pose inicial
                            place = deepcopy(init_pose)
                            
                            place.pose.position.x += x
                            place.pose.position.y += y
                            place.pose.position.z += z
                            
                    
                            q = quaternion_from_euler(0, p, yam)
                            
                            
                            place.pose.orientation.x = q[0]
                            place.pose.orientation.y = q[1]
                            place.pose.orientation.z = q[2]
                            place.pose.orientation.w = q[3]
                        
                            places.append(place)

        # Returna a lista
        return places
    
    #Recebe o sinal de chegada do robô, para assim o braço realizar tentativas de pick
    def robot_arrival_callback(self, msg):
        global robot_arrived
        robot_arrived = msg.data
        rospy.loginfo("Recebido sinal de chegada do robô móvel: " + str(robot_arrived))
                

     #Recebe a posição de uma objeto na cena
    def get_object_pose(self, object_name):
        try:
            rospy.wait_for_service('/gazebo/get_model_state')
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
            response = get_model_state(object_name, "world")
            
            if response.success:
                pose = response.pose
                return pose
            else:
                rospy.logwarn(f"O objeto {object_name} não foi encontrado no mundo.")
                return None
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Serviço falhou: {e}")
            return None
        
     # Cria uma cena no Rviz
    def create_scene(self, scene, reference_frame='world'):
        """Criar uma cena artificial no Rviz"""
        scene_objects = []

                
        # Criação de uma mesa (table1)
        table1_name = 'table1'
        table1_size = (0.15, 0.15, 0.1)  # x, y, z
        table1_pose = PoseStamped()
        table1_pose.header.frame_id = reference_frame
        table1_pose.pose.position.x = 0.4
        table1_pose.pose.position.y = -0.2
        table1_pose.pose.position.z = 0.05
        table1_pose.pose.orientation.w = 1.0
        scene.add_box(table1_name, table1_pose, table1_size)
        scene_objects.append(table1_name)
        
        
        # Adiciona o TurtleBot na cena
        turtlebot_pose = self.get_object_pose('turtlebot3_waffle_pi')
        
        turtlebot_name = 'turtlebot3_waffle_pi'
        turtlebot_pose_stamped = PoseStamped()
        turtlebot_pose_stamped.header.frame_id = reference_frame
        turtlebot_pose_stamped.pose = turtlebot_pose

        mesh_uri = "/home/aline/catkin_ws/src/turtlebot3/turtlebot3_description/meshes/bases/waffle_pi_base.stl" 

        #0.281 0.306 0.141
        scale = (0.0011, 0.0011, 0.0011) 

        # Adicione o TurtleBot como um mesh na cena
        scene.add_mesh(turtlebot_name, turtlebot_pose_stamped, mesh_uri, scale)
        scene_objects.append(turtlebot_name)

        rospy.loginfo(f"Modelo '{turtlebot_name}' adicionado à cena com sucesso.")
        
        # Cria um objeto a ser ser pego (object) com base na pose do gazebo
        object_pose = self.get_object_pose('object')
        rospy.loginfo(object_pose)
        
        object_name = 'object'
        object_pose_stamped = PoseStamped()
        object_pose_stamped.header.frame_id = reference_frame 
        object_pose_stamped.pose = object_pose  
        
        mesh_uri = "/home/aline/catkin_ws/src/cooperation_widowx_turtlebot3/cooperation_widowx_turtlebot3/models/object/meshes/teste_certo.stl"
        scale = (0.0012, 0.0012, 0.0012)
        
        scene.add_mesh(object_name, object_pose_stamped, mesh_uri, scale)
        scene_objects.append(object_name)

        rospy.loginfo(f"Modelo '{object_name}' adicionado à cena com sucesso.")
        rospy.loginfo(f"Posição do objeto: '{object_pose}'")

    
        try:    
            self.wait_for_scene_update(scene, scene_objects)

            return scene_objects
        except TimeoutError as error:
            rospy.logerr(error.args[0] + ' Objects not created.')
            scene.clear()
            return None
    


    def wait_for_scene_update(self, scene, expected_scene_objects=None,
                            expected_attached_objects=None, timeout=10):
        """Wait until moveit scene is updated"""
        if expected_scene_objects is None:
            expected_scene_objects = []
        if expected_attached_objects is None:
            expected_attached_objects = []

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            
            scene_objects_ok = \
                set(expected_scene_objects) == set(scene.get_known_object_names())
        
            attached_objects_ok = \
                set(expected_attached_objects) == set(scene.get_attached_objects())

            if scene_objects_ok and attached_objects_ok:
                return
            else:
                rospy.sleep(0.1)
                seconds = rospy.get_time()

    
        raise TimeoutError('Scene was not updated before timeout.')
    
    
    

if __name__ == "__main__":
    MoveItDemo()
