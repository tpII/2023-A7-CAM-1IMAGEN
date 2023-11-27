import cv2
import numpy as np
import math

class ArucoDetector:

    def __init__(self):
        self.diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parametros = cv2.aruco.DetectorParameters()
        self.aruco_center = None
        self.aruco_scale = None
        self.aruco_esquina_x = None
        self.aruco_esquina_y = None

    def detect_markers(self, frame, matrix, dist):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        esquinas, ids, candidatos_malos = cv2.aruco.detectMarkers(gray, self.diccionario, parameters=self.parametros)
        if esquinas:
            print(esquinas)
            #ACA ACCEDO A LA POSICION X DE LA PRIMER ESQUINA. ES UN ARREGLO DE 4 DIMENSIONES. DIM1 ARUCO ENTERO, DIM 2 ESQUINA, DIM 3 TUPLA XY, DIM 4 X o Y
            #Me guardo las esquinas superiores del aruco
            self.aruco_esquina_x = [esquinas[0][0][0][0], esquinas[0][0][1][0]]
            self.aruco_esquina_y = [esquinas[0][0][0][1], esquinas[0][0][1][1]]
            print("Coordenadas X de las esquinas sueperiores: ", self.aruco_esquina_x)
            print("Coordenadas Y de las esquinas sueperiores: ", self.aruco_esquina_y)
        else:
            print("No se detectaron esquinas.")
        try:
            if np.all(ids is not None):
                for i in range(len(ids)):
                    rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(esquinas[i], 0.02, matrix, dist)
                    (rvec - tvec).any()
                    cv2.aruco.drawDetectedMarkers(frame, esquinas)
                    c_x = np.mean(esquinas[i][:, :, 0])
                    c_y = np.mean(esquinas[i][:, :, 1])
                    self.aruco_center = (c_x, c_y)
                    self.calculate_scale(tvec)
                    print("----------------------------------")
                    print("Escala: ", self.aruco_scale)
                    cv2.putText(frame, "id" + str(ids[i]), (int(c_x), int(c_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 225, 250), 2)
        except Exception as e:
            print(f"Error en la detección de marcadores: {e}")

    def calculate_scale(self, tvec):
        # Define las dimensiones conocidas del ArUco (ancho y alto en metros)
        aruco_width_m = 4  # Modifica este valor según las dimensiones reales de tu ArUco
        arucox_p = max(self.aruco_esquina_x)-min(self.aruco_esquina_x)
        arucoy_p = max(self.aruco_esquina_y)-min(self.aruco_esquina_y)
        aruco_width_p = math.sqrt((arucox_p**2+arucoy_p**2))
        print("aruco_width", aruco_width_p)
        # Calcula la escala utilizando las dimensiones conocidas y las coordenadas 3D del ArUco
        self.aruco_scale = aruco_width_m / aruco_width_p
        #YA TENEMOS LA ESCALA

    def calculate_distance(self, obj_center_x, obj_center_y, frame):
        if self.aruco_scale is not None:
            # Calcula la distancia en centímetros usando la escala y la posición en píxeles en el eje X
            linea_x = max(obj_center_x, self.aruco_center[0]) - min(obj_center_x, self.aruco_center[0])
            print("Coordenadas del ARUCO: ", self.aruco_center)
            print("ObjetoX: ", obj_center_x)
            print("linea_X", linea_x)
            linea_y = max(obj_center_y, self.aruco_center[1]) - min(obj_center_y, self.aruco_center[1])
            print("ObjetoY: ", obj_center_y)
            print("linea_Y", linea_y)
            linea_largo = math.sqrt(linea_x**2 + linea_y**2)
            print("linea total: ", linea_largo)
            distance_cm = (self.aruco_scale * linea_largo)
            cv2.circle(frame, (int(obj_center_x), int(obj_center_y)), 10, 255, -1)
            cv2.circle(frame, (int(self.aruco_center[0]),int(self.aruco_center[1])), 10, 0, -1)
            return distance_cm
        else:
            return None

    def get_aruco_center(self):
        return self.aruco_center

    def get_aruco_scale(self):
        return self.aruco_scale
