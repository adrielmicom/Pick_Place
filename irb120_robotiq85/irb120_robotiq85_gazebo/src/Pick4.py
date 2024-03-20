#! /usr/bin/env python3
#la linea de arriba es super importante
#Programar cinematica inversa
# Especifica que el intérprete de Python 3 debe usarse para ejecutar este script.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Inicialización, objetos robot y escena, grupos MoveIt
moveit_commander.roscpp_initialize(sys.argv)
# Inicializa MoveIt y ROS, utilizando los argumentos del sistema.
rospy.init_node('moving_irb120_robot', anonymous=True)
# Inicializa un nodo ROS llamado 'moving_irb120_robot'.
robot = moveit_commander.RobotCommander()
# Crea un objeto para administrar el robot.
scene = moveit_commander.PlanningSceneInterface()
# Crea un objeto para interactuar con la escena.
arm_group = moveit_commander.MoveGroupCommander("irb_120")
# Crea un grupo de movimiento para el brazo del robot.
hand_group = moveit_commander.MoveGroupCommander("robotiq_85")
# Crea un grupo de movimiento para la herramienta (gripper) del robot.

box_name = "box_name"

planning_frame = hand_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
#¿ultimo eslabon con capacidad movil ejemplo pinza?
eef_link = hand_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())




# Inicialización, objetos robot y escena, grupos MoveIt
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
# Crea un publicador para mostrar trayectorias planificadas en RViz.

# Inverse Kinematics (IK): mover TCP (brida) a una orientación dada (Euler) y posición x, y, z dada. Los ángulos se convierten a cuaternios (mensaje de pose)

# Función para mover el brazo del robot a una posición de destino utilizando coordenadas de Euler y posición XYZ
def move_pose_arm(roll, pitch, yaw, x, y, z):
    pose_goal = geometry_msgs.msg.Pose()
    quat = quaternion_from_euler(roll*(pi/180), pitch*(pi/180), yaw*(pi/180))
    # Convierte los ángulos de Euler a cuaternios.
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    arm_group.set_pose_target(pose_goal)
    # Establece el objetivo de pose para el brazo.

    plan = arm_group.go(wait=True)
    # Planifica y ejecuta el movimiento del brazo.
    
    arm_group.stop()  # Para garantizar que no haya movimiento residual
    arm_group.clear_pose_targets()
    # Limpia los objetivos de pose después de completar el movimiento.

# Forward Kinematics (FK) para la herramienta

# Función para mover el gripper o herramienta del robot a una posición específica
def move_joint_hand(gripper_finger1_joint):
    joint_goal = hand_group.get_current_joint_values()
    joint_goal[2] = gripper_finger1_joint  # Movimiento en el eje maestro del gripper.

    hand_group.go(joint_goal, wait=True)
    # Planifica y ejecuta el movimiento del gripper.
    
    hand_group.stop()  # Para garantizar que no haya movimiento residual



def wait_for_state_update( box_is_known=False, box_is_attached=False, timeout=4
    ):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    # box_name = box_name
    # scene = scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
    ## or dies before actually publishing the scene update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed.
    ## To avoid waiting for scene updates like this at all, initialize the
    ## planning scene interface with  ``synchronous = True``.

    # Asegurando que se Reciban las Actualizaciones de Colisión
    # Si el nodo de Python acaba de crearse (https://github.com/ros/ros_comm/issues/176),
    # o muere antes de publicar realmente el mensaje de actualización de la escena, el mensaje
    # podría perderse y la caja no aparecerá. Para asegurarnos de que las actualizaciones se realicen,
    # esperamos hasta que veamos los cambios reflejados en las listas get_attached_objects() y get_known_object_names().
    # Para este tutorial, llamamos a esta función después de agregar, quitar, adjuntar o desvincular un objeto en la escena de planificación.
    # Luego esperamos hasta que se hayan realizado las actualizaciones o hayan pasado "timeout" segundos.
    # Para evitar esperar actualizaciones de la escena de esta manera en absoluto, inicializa la interfaz de la escena de planificación con "synchronous = True".
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

def add_box(timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    # box_name = self.box_name
    # scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene between the fingers:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.position.x = 0.5
    box_pose.pose.position.y = 0.0 # above the panda_hand frame
    box_pose.pose.position.z = 0.15
    box_name = "box"
    #self.box_name="box"
    rospy.loginfo("QUIERO AÑADIR EL OBJETO")
    scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    # self.box_name = box_name
    # return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    rospy.loginfo("ESPERANDO")
    return wait_for_state_update(box_is_known=True, timeout=timeout)


def attach_box(timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    # box_name = self.box_name
    # robot = self.robot
    # scene = self.scene
    # eef_link = self.eef_link
    # group_names = self.group_names

#A continuación, adjuntaremos la caja a la muñeca del Panda. Manipular objetos requiere que
# el robot pueda tocarlos sin que la escena de planificación informe el contacto como una
# colisión. Al agregar nombres de enlaces al array "touch_links", le estamos diciendo a la
# escena de planificación que ignore las colisiones entre esos enlaces y la caja. Para el robot Panda,
# establecemos "grasping_group = 'hand'". Si estás utilizando un robot diferente,
# debes cambiar este valor al nombre de tu grupo de efector final.
    grasping_group = "robotiq_85"
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return wait_for_state_update(
        box_is_attached=True, box_is_known=False, timeout=timeout
    )



if __name__ == '__main__':
    rospy.loginfo("INICIO FUNCION ADDBOX")
    #hola=add_box()   21/02   fecha comentado
    #rospy.loginfo(hola)  21/02   fecha comentado
    #rospy.loginfo("fiiin add booox")    21/02   fecha comentado
    #move_pose_arm(0,  90, 0, x +0.1 , y +0.05 , z )
    #
    #
    #rotacion ultima pinza probar de 0 a 360

    rospy.loginfo("Moving arm to HOME point")
    move_pose_arm(0, 90, 0, 0.4, 0, 0.6)
    # Mueve el brazo a una posición inicial. 
    rospy.sleep(1)

    rospy.loginfo("Prueba")
    move_pose_arm(0, 0, 0, 0.5, 0, 0.6)
    # Mueve el brazo a una posición inicial.
    rospy.sleep(1)

    rospy.loginfo("P2")
    move_pose_arm(0, 0, 0, 0.5, 0, 0.5)
    # Mueve el brazo a una posición inicial.
    rospy.sleep(1)


    rospy.loginfo("P3")
    move_pose_arm(0, 90, 0, 0.5, 0, 0.5)
    # Mueve el brazo a una posición inicial.
    rospy.sleep(1)


    rospy.loginfo("P4 Baaja")
    move_pose_arm(0, 90, 0, 0.5, 0, 0.28)
    # Mueve el brazo a una posición inicial.
    rospy.sleep(1)



    rospy.loginfo("cerrar sin coger, ")
    move_joint_hand(0.30)

    rospy.loginfo("============ Press `Enter` to attach a Box to the Panda robot ...")
    hola=attach_box()
    rospy.loginfo(hola)
    rospy.sleep(1)

    rospy.loginfo(" COGIENDO OBJETO")
    move_joint_hand(0.45)
    rospy.sleep(1)

    rospy.loginfo("SUBIENDO")
    move_pose_arm(0, 90, 0, 0.5, 0, 0.5)
    # Mueve el brazo a una posición inicial.
    rospy.sleep(1)


    rospy.loginfo("INICIANDO MOVIEMIENTO")
    move_pose_arm(0, 0, 0, 0, 0.30, 0.5)
    # Mueve el brazo a una posición inicial.
    rospy.sleep(1)



    rospy.loginfo("LLEGUE, BAJAME")
    move_pose_arm(0, 90, 0, -0.42, 0.16, 0.5)
    # Mueve el brazo a una posición inicial.
    rospy.sleep(1)


    rospy.loginfo("bajando")
    move_pose_arm(0, 90, 0, -0.42, 0.16, 0.05)
    # Mueve el brazo a una posición inicial.
    rospy.sleep(1)
    

    rospy.loginfo("SOLTANDO")
    move_joint_hand(0)

    rospy.sleep(1)


    rospy.loginfo("subiend")
    move_pose_arm(0, 90, 0, -0.42, 0.16, 0.5)
    # Mueve el brazo a una posición inicial.
    rospy.sleep(1)
    

    rospy.loginfo("Moving arm to HOME point")
    move_pose_arm(0, 90, 0, 0.4, 0, 0.6)
    # Mueve el brazo a una posición inicial. 
    rospy.sleep(1)

    # rospy.loginfo("Pose 1")
    # move_pose_arm(180, 90, 0, 0.4, 0, 0.6)
    # # Mueve el brazo a una posición inicial.
    # rospy.sleep(1)

    # rospy.loginfo("Pose 2")
    # move_pose_arm(0, 90, 0, 0.6, 0, 0.6)
    # # Mueve el brazo a una posición inicial.
    # rospy.sleep(1)

    # rospy.loginfo("Opening gripper")
    # move_joint_hand(0)
    # Abre el gripper.

    # rospy.sleep(1)
    # Espera durante 1 segundo.


    # rospy.loginfo("cerrar 0.8, CIERRE COMPLETO")
    # move_joint_hand(0.8)



    rospy.loginfo("All movements finished. Shutting down")
    # Indica que todos los movimientos han terminado y se apaga el nodo.
    moveit_commander.roscpp_shutdown()
