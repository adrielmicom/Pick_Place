#! /usr/bin/env python3
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

# Importa las bibliotecas y módulos necesarios.

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
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
# Crea un publicador para mostrar trayectorias planificadas en RViz.

# Forward Kinematics (FK): movimiento del grupo robot eje a eje (para robots de 6 ejes). Ángulos adaptados a grados

# Función para mover el brazo del robot a posiciones específicas
def move_joint_arm(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5):
    joint_goal = arm_group.get_current_joint_values()
    # Obtiene la posición actual de las articulaciones del brazo.
    joint_goal[0] = joint_0 * (pi/180)
    joint_goal[1] = joint_1 * (pi/180)
    joint_goal[2] = joint_2 * (pi/180)
    joint_goal[3] = joint_3 * (pi/180)
    joint_goal[4] = joint_4 * (pi/180)
    joint_goal[5] = joint_5 * (pi/180)
    # Convierte ángulos de grados a radianes.

    arm_group.go(joint_goal, wait=True)
    # Mueve el brazo a las nuevas posiciones.
    arm_group.stop()  # Para garantizar que no haya movimiento residual

# Función para mover el gripper o herramienta del robot a una posición específica
def move_joint_hand(gripper_finger1_joint):
    joint_goal = hand_group.get_current_joint_values()
    # Obtiene la posición actual del gripper.
    joint_goal[2] = gripper_finger1_joint  # Mueve el eje maestro del gripper.

    hand_group.go(joint_goal, wait=True)
    # Mueve el gripper a la nueva posición.
    hand_group.stop()  # Para garantizar que no haya movimiento residual

if __name__ == '__main__':
    # Print estado actual del robot (opcional)
    print("============ Printing robot state ============")
    print(robot.get_current_state())
    print("")
    # Muestra el estado actual del robot en la consola (opcional).

    # Ejemplo: FK en un loop. Movimiento del robot y de la herramienta eje a eje. Ángulos en grados (robot).
    for i in range(2):
        # Inicia un bucle para realizar movimientos del robot.

        rospy.loginfo("Moving arm to pick pose")
        move_joint_arm(0, 50, -20, 0, 60, 0)
        # Mueve el brazo a una posición de recoger.
        rospy.sleep(2)

        rospy.loginfo("Closing gripper")
        move_joint_hand(0.5)
        # Cierra el gripper.
        rospy.sleep(1)

        rospy.loginfo("Moving arm to place pose")
        move_joint_arm(150, 20, 10, 0, 60, 90)
        # Mueve el brazo a una posición de colocar.
        rospy.sleep(1)

        rospy.loginfo("Opening gripper")
        move_joint_hand(0)
        # Abre el gripper.
        rospy.sleep(1)

        rospy.loginfo("Moving arm to HOME")
        move_joint_arm(0, 0, 0, 0, 45, 0)
        # Mueve el brazo a la posición de inicio.

    rospy.loginfo("All movements finished. Shutting down")
    # Indica que todos los movimientos han terminado y se apaga el nodo.
    moveit_commander.roscpp_shutdown()
