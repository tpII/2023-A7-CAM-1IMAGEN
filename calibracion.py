import numpy as np
import cv2
import glob


class Calibracion():
    def __init__(self):
        self.tablero = (6, 4)
        self.tam_frame = (1280, 726)
        # Criterio
        self.criterio = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Preparamos los puntos del tablero
        self.puntos_obj = np.zeros((self.tablero[0] * self.tablero[1], 3), np.float32)
        self.puntos_obj[:, :2] = np.mgrid[0:self.tablero[0], 0:self.tablero[1]].T.reshape(-1, 2)

        # Preparamos las listas para almacenar los puntos de mundo real y de la imagen
        self.puntos_3d = []
        self.puntos_img = []

    def calibracion_cam(self):
        fotos = glob.glob(
            'C:\\Users\\Cris\\Desktop\\Facultad\\Taller de Proyecto II\\2023-A7-CAM-1IMAGEN\\capturas\\*.png')
        for foto in fotos:
            print(foto)
            img = cv2.imread(foto)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Buscamos las esquinas del tablero
            ret, esquinas = cv2.findChessboardCorners(gray, self.tablero, None)

            if ret:
                # Refinamos las esquinas
                esquinas2 = cv2.cornerSubPix(gray, esquinas, (11, 11), (-1, -1), self.criterio)

                # Guardamos los puntos en 3D y en 2D
                self.puntos_3d.append(self.puntos_obj)
                self.puntos_img.append(esquinas2)

                # Dibujamos las esquinas en la imagen
                cv2.drawChessboardCorners(img, self.tablero, esquinas2, ret)
                cv2.imshow("img", img)

        # Calibración de la cámara
        ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(self.puntos_3d, self.puntos_img, self.tam_frame,
                                                                    None, None)

        # Ejemplo de aplicación de la perspectiva para obtener M
        src_points = np.array([[0, 0], [self.tam_frame[0] - 1, 0], [self.tam_frame[0] - 1, self.tam_frame[1] - 1],
                               [0, self.tam_frame[1] - 1]], dtype=np.float32)
        dst_points = np.array([[0, 0], [self.tam_frame[0] - 1, 0], [self.tam_frame[0] - 1, self.tam_frame[1] - 1],
                               [0, self.tam_frame[1] - 1]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src_points, dst_points)

        print("Matriz de la cámara original: ", cameraMatrix)
        print("Coeficientes de distorsión original: ", dist)
        print("Matriz de transformación de perspectiva: ", M)

        return cameraMatrix, dist, M
