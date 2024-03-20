#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def pose2msg(x, y, z, roll, pitch, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quat = quaternion_from_euler(roll,pitch,yaw)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

if __name__ == "__main__":
    print("spawning seleted model")
    rospy.init_node('spawn_model')
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    model_name = "red_box"


    # Actualizar la ruta de la carpeta por la correcta:
    #with open("/home/diego/ws_moveit/src/irb120_robotiq85/irb120_robotiq85_gazebo/models/{}/model.sdf".format(model_name), "r") as f:
    #        model_xml = f.read()
    with open("/home/adri2/Workspace/3.3/src/irb120_robotiq85/irb120_robotiq85_gazebo/models/{}/model.sdf".format(model_name), "r") as f:
            model_xml = f.read()    
    

    # Posición y orientacion del objeto	
    #model_pose = pose2msg(0.5, 0, 1.2, 0, 0, 0)  #PARA MI EJEMPLO
    #model_pose = pose2msg(0.2, 0, 0.76,0, 0, 0)  #JACO 
    model_pose = pose2msg(0.5, 0, 1,0, 0, 0)
    spawn_model(model_name, model_xml, "", model_pose, "world")
    #spawn_model("box", model_xml, "", model_pose, "world")


    ##green_cylinder
    print("spawning green_cylinder")
    rospy.init_node('spawn_model')
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    model_name = "green_cylinder"


    # Actualizar la ruta de la carpeta por la correcta:
    #with open("/home/diego/ws_moveit/src/irb120_robotiq85/irb120_robotiq85_gazebo/models/{}/model.sdf".format(model_name), "r") as f:
    #        model_xml = f.read()
    with open("/home/adri2/Workspace/3.3/src/irb120_robotiq85/irb120_robotiq85_gazebo/models/{}/model.sdf".format(model_name), "r") as f:
            model_xml = f.read()    
    

    # Posición y orientacion del objeto	
    #model_pose = pose2msg(0.5, 0, 1.2, 0, 0, 0)  #PARA MI EJEMPLO
    #model_pose = pose2msg(0.2, 0, 0.76,0, 0, 0)  #JACO 
    model_pose = pose2msg(0.73, 0.2, 1,0, 0, 0)
    spawn_model(model_name, model_xml, "", model_pose, "world")
    #spawn_model("box", model_xml, "", model_pose, "world")


    ##yellow_ball
    print("spawning yellow_ball")
    rospy.init_node('spawn_model')
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    model_name = "yellow_ball"


    # Actualizar la ruta de la carpeta por la correcta:
    #with open("/home/diego/ws_moveit/src/irb120_robotiq85/irb120_robotiq85_gazebo/models/{}/model.sdf".format(model_name), "r") as f:
    #        model_xml = f.read()
    with open("/home/adri2/Workspace/3.3/src/irb120_robotiq85/irb120_robotiq85_gazebo/models/{}/model.sdf".format(model_name), "r") as f:
            model_xml = f.read()    
    

    # Posición y orientacion del objeto	
    #model_pose = pose2msg(0.5, 0, 1.2, 0, 0, 0)  #PARA MI EJEMPLO
    #model_pose = pose2msg(0.2, 0, 0.76,0, 0, 0)  #JACO 
    model_pose = pose2msg(0.7, 0, 1,0, 0, 0)
    spawn_model(model_name, model_xml, "", model_pose, "world")
    #spawn_model("box", model_xml, "", model_pose, "world")

    # FIN YELLOW

    ##red_cylinder
    print("spawning red_cylinder")
    rospy.init_node('spawn_model')
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    model_name = "red_cylinder"


    # Actualizar la ruta de la carpeta por la correcta:
    #with open("/home/diego/ws_moveit/src/irb120_robotiq85/irb120_robotiq85_gazebo/models/{}/model.sdf".format(model_name), "r") as f:
    #        model_xml = f.read()
    with open("/home/adri2/Workspace/3.3/src/irb120_robotiq85/irb120_robotiq85_gazebo/models/{}/model.sdf".format(model_name), "r") as f:
            model_xml = f.read()    
    

    # Posición y orientacion del objeto	
    #model_pose = pose2msg(0.5, 0, 1.2, 0, 0, 0)  #PARA MI EJEMPLO
    #model_pose = pose2msg(0.2, 0, 0.76,0, 0, 0)  #JACO 
    model_pose = pose2msg(0.73, 0.1, 1,0, 0, 0)
    spawn_model(model_name, model_xml, "", model_pose, "world")
    #spawn_model("box", model_xml, "", model_pose, "world")

    # FIN red_cylinder
    
""""
    from moveit_commander import RobotCommander,PlanningSceneInterface
    from moveit_commander import roscpp_initialize, roscpp_shutdown
    import moveit_commander
    from moveit_commander.conversions import pose_to_list
    import moveit_msgs.msg
    import geometry_msgs.msg
    from moveit_msgs.msg import Grasp, PlaceLocation
    from trajectory_msgs.msg import JointTrajectoryPoint
    from std_msgs.msg import String

    from math import pi
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    #from moveit_python import *

    from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
    from moveit_msgs.msg import PickupAction, PickupGoal
    from moveit_msgs.msg import PlaceAction, PlaceGoal
    from moveit_msgs.msg import PlaceLocation
    from moveit_msgs.msg import MoveItErrorCodes

    # Instanciar la clase “PlanningSceneInterface”
    scene = moveit_commander.PlanningSceneInterface()
    # Crear mensaje de pose para posición y orientación del objeto
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "sistema_de_referencia"
    box_pose.pose.orientation.w = 1.0
    # Dar nombre al objeto y añadirlo a la escena
    box_name = "red_box"
    scene.add_box(box_name, box_pose, size=(0.2, 0.3, 0.4))



"""