#import api_pick_and_place as api

from api_pick_and_place import Pick_Place  # Importa la clase Pick_Place del archivo pick_place_class.py
import rospy
import sys
if __name__ == "__main__":

    # Crea una instancia de la clase Pick_Place
    pick_place = Pick_Place()

    # Define la pose de la caja que deseas recoger
    model_pose = pick_place.pose2msg(0.5, 0, 1, 0, 0, 0)

    # Realiza la acción de recoger la caja en la posición especificada
    pick_place.pickup("red_box", model_pose)

