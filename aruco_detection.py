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

        try:
            if np.all(ids is not None):
                for i in range(len(ids)):
                    rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(esquinas[i], 0.02, matrix, dist)
                    (rvec - tvec).any()
                    cv2.aruco.drawDetectedMarkers(frame, esquinas)
                    c_x = np.mean(esquinas[i][:, :, 0])
                    c_y = np.mean(esquinas[i][:, :, 1])
                    self.aruco_center = (c_x, c_y)
                    self.aruco_scale = self.calculate_scale(tvec)
                    cv2.putText(frame, "id" + str(ids[i]), (int(c_x), int(c_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 225, 250), 2)
        except Exception as e:
            print(f"Error en la detección de marcadores: {e}")

    def calculate_scale(self, tvec):
        # Aquí puedes calcular la escala en función del tvec u otros parámetros
        # Por ejemplo, si conoces la distancia real entre dos puntos en el objeto y la distancia en píxeles,
        # puedes calcular la escala.
        # Asumamos que la distancia real es de 10 unidades y la distancia en píxeles es la mitad del ancho de la imagen.
        real_distance = 10.0  # Unidades en el mundo real
        pixel_distance = self.aruco_center[0]  # La mitad del ancho de la imagen
        scale = real_distance / pixel_distance
        return scale

    def calculate_distance(self, obj_center):
        if self.aruco_scale is not None:
            # Calcula la distancia en centímetros usando la escala y la posición en píxeles
            distance_cm = self.aruco_scale * obj_center[0]  # Cambia esta fórmula según tus necesidades
            return distance_cm
        else:
            return None

    def get_aruco_center(self):
        return self.aruco_center

    def get_aruco_scale(self):
        return self.aruco_scale
