import cv2
import numpy as np

class ArucoDetector:

    def __init__(self):
        self.diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parametros = cv2.aruco.DetectorParameters()
        self.aruco_center = None
        self.aruco_scale = None

    def detect_markers(self, frame, matrix, dist):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        esquinas, ids, candidatos_malos = cv2.aruco.detectMarkers(gray, self.diccionario, parameters=self.parametros)
        if esquinas:
            # La lista de esquinas no está vacía
            esquina1 = esquinas[0][0]
            print("Esquina 1:", esquina1)
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
                    self.aruco_scale = self.calculate_scale(tvec, 0)
                    print("----------------------------------")
                    print(self.aruco_scale)
                    cv2.putText(frame, "id" + str(ids[i]), (int(c_x), int(c_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 225, 250), 2)
        except Exception as e:
            print(f"Error en la detección de marcadores: {e}")

    def calculate_scale(self, tvec, aruco_id):
        # Define las dimensiones conocidas del ArUco (ancho y alto en metros)
        aruco_width = 0.02  # Modifica este valor según las dimensiones reales de tu ArUco
        aruco_height = 0.02 # Modifica este valor según las dimensiones reales de tu ArUco

        # Calcula la escala utilizando las dimensiones conocidas y las coordenadas 3D del ArUco
        scale_x = aruco_width / abs(tvec[0, 0, 2])
        scale_y = aruco_height / abs(tvec[0, 0, 1])

        # Usa la escala según la dirección en la que se encuentra el ArUco
        if aruco_id == 0:  # Modifica esto según la ID del ArUco que deseas seguir
            scale = scale_x
        else:
            scale = scale_y

        return (scale_x,scale_y)

    def calculate_distance(self, obj_center_x, obj_center_y, aruco_id):
        if self.aruco_scale is not None:
            # Calcula la distancia en centímetros usando la escala y la posición en píxeles en el eje X
            distance_cm = (self.aruco_scale[0] * obj_center_x)
            return distance_cm
        else:
            return None

    def get_aruco_center(self):
        return self.aruco_center

    def get_aruco_scale(self):
        return self.aruco_scale
