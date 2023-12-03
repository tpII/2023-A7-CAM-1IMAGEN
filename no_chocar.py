##### Tomo decision del funcionamiento del auto segun distancia actual al objeto detectado ####
import cv2
import numpy as np
import time
import mqtt

class NoChocar:
    W_FRAME = 480
    D_MAX = 5    # Distancia maxima a la que que se acercará

    def __init__(self):
        self.dist_max = 5;      # Distancia maxima a la que que se acercará

    def no_chocar(self, dist_actual, coor_x):
        if dist_actual <= NoChocar.D_MAX:   # Si la distancia es la maxima permitida
            # Frenar
            #mqtt.send_message("F");
            print("FRENAR")
            time.sleep(3)   # Se detiene 3 segundos
            if coor_x > NoChocar.W_FRAME/2:     # Defino si el obj se encuentra hacia la izq o der
                # Doblar a la izquierda
                #mqtt.send_message("I");
                print("DOBLAR A LA IZQUIERDA")
            else:
                # Doblar a la derecha
                #mqtt.send_message("D");
                print("DOBLAR A LA DERECHA")
        else:
            # Avanzar
            #mqtt.send_message("A");
            print("AVANZAR")
