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

if __name__ == '__main__':
    # Ejemplo de uso de IK (en un bucle)
    for i in range(2):
        # Inicia un bucle para realizar movimientos.

        rospy.loginfo("Moving arm to HOME point")
        move_pose_arm(0, 90, 0, 0.4, 0, 0.6)
        # Mueve el brazo a una posición inicial.
        
        rospy.loginfo("Opening gripper")
        move_joint_hand(0)
        # Abre el gripper.
        
        rospy.sleep(1)
        # Espera durante 1 segundo.

        rospy.loginfo("Moving arm to point_1")
        move_pose_arm(45, 45, 45, 0.5, -0.25, 0.3)
        # Mueve el brazo a una posición llamada 'point_1'.
        
        rospy.sleep(1)

        rospy.loginfo("Moving arm to point_2")
        move_pose_arm(0, -90, 90, 0.5, 0.25, 0.3)
        # Mueve el brazo a una posición llamada 'point_2'.
        
        rospy.sleep(1)

        rospy.loginfo("Closing gripper to 0.4")
        move_joint_hand(0.4)
        # Cierra el gripper a 0.4 (posición deseada).
        
        rospy.sleep(1)

        rospy.loginfo("Moving arm to point_3")
        move_pose_arm(0, 0, 90, 0.2, 0, 0.8)
        # Mueve el brazo a una posición llamada 'point_3'.
        
        rospy.loginfo("Opening gripper")
        move_joint_hand(0)
        # Abre el gripper.
        
        rospy.sleep(1)

        rospy.loginfo("Moving arm to point_4")
        move_pose_arm(0, 0, 0, 0, -0.5, 0.4)
        # Mueve el brazo a una posición llamada 'point_4'.
        
        rospy.loginfo("Closing gripper to 0.6")
        move_joint_hand(0.6)
        # Cierra el gripper a 0.6 (posición deseada).
        
        rospy.sleep(1)

    rospy.loginfo("All movements finished. Shutting down")
    # Indica que todos los movimientos han terminado y se apaga el nodo.
    moveit_commander.roscpp_shutdown()
