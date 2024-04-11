#import api_pick_and_place as api
from api_pick_and_place import Pick_Place  # Importa la clase Pick_Place del archivo pick_place_class.py
from api_pick_and_place import Object
import rospy
import sys

def main():
    rospy.init_node('pick_place_example', anonymous=True)
    
    # Instancia de tu clase Pick_Place
    clase = Pick_Place()

    # Ahora puedes llamar a las funciones de tu instancia, por ejemplo:
    object_name = "cylinder"  # Asegúrate de que los objetos ya estén agregados a la escena
    pose = clase.get_object_pose(object_name)
    print(f"La posición actual de {object_name} es: {pose}")
    # Continúa con el resto de las operaciones que necesites realizar

    objetos=clase.object_list
    print(f"los objetos son: {objetos}")


    
    pose1=clase.pose2msg(0, 0, 0, 0.6, 0.19, 0.18)
    pose2=clase.pose2msg(0,45,0,0.6,0,0.35)
    print(f"La posición1 {pose1}")
    msg1=clase.msg2pose(pose1)
    print(f"El msg1 {msg1}")

    """
    print("move pose arm")
    clase.move_pose_arm(pose2)
    rospy.sleep(1)
    print("move joint_arm")
    clase.move_joint_arm(0, 0, 0, 0.5, 0.0, 0.6)
    rospy.sleep(1)
    print("move hand")
    clase.move_joint_hand(0.6)
    rospy.sleep(1)
    print("VUELTA A CASA")
    clase.back_to_home()
    rospy.sleep(1)
    """

    print("PLACE CON POSE")
    clase.place(pose)
    rospy.sleep(1)
    print("move pose arm con POSE")
    clase.move_pose_arm(pose)
    rospy.sleep(1)
    print("PLACE CON POSE2")
    clase.place(pose2)
    rospy.sleep(1)
    clase.back_to_home()

    """
    print("vamos posicion 1")
    poseprueba=clase.pose2msg(0, 0, 0, 0.5, 0.2,0.6)
    clase.move_pose_arm(poseprueba)
    rospy.sleep(1)
    print("vamos posicion 2")
    poseprueba=clase.pose2msg(0, 0, 0, 0.5, 0.1,0.36)
    clase.move_pose_arm(poseprueba)
    rospy.sleep(1)
    # pick cylinder
    object_name = "cylinder"
    pose = clase.get_object_pose(object_name)
    print(pose.position.y)
    pose.position.z += 0.16
    clase.pickup(object_name, poseprueba)
    """



    #pick_place.MyAlgorithm()

if __name__ == "__main__":
    main()

"""
if __name__ == "__main__":

    # Crea una instancia de la clase Pick_Place
    pick_place = Pick_Place()

    # Define la pose de la caja que deseas recoger
    model_pose = pick_place.pose2msg(0.5, 0, 1, 0, 0, 0)

    # Realiza la acción de recoger la caja en la posición especificada
    pick_place.pickup("red_box", model_pose)
"""
