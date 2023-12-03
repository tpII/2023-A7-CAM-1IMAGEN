# main.py
import cv2
import numpy as np
from calibracion import *
from aruco_detection import ArucoDetector
from yolo_detection import YoloDetector
from no_chocar import *
import time

def apply_perspective_transform(frame, M):
    transformed_frame = cv2.warpPerspective(frame, M, (frame.shape[1], frame.shape[0]))
    return transformed_frame

def main():
    # Inicializamos el detector de ArUcoq
    aruco_detector = ArucoDetector()
    # Inicializamos el detector de YOLO
    yolo_detector = YoloDetector(model_path='yolov8n.pt')  # Especifica la ruta correcta a tu modelo YOLO

    # Calibración de la cámara
    calibracion = Calibracion()
    #matrix, dist, M = calibracion.calibracion_cam()
    matrix = np.array([[1178.8, 0, 622.96],
                      [0, 617.02, 304.99],
                        [0, 0, 1]])
    dist = np.array([[-1.0978, 1.0662, 0.047216, -0.049225, -0.45228]])
    M = np.array([[1, 0, 0],
                [0, 1, 0],
                [1.226e-19, 0, 1]])

    print("Matriz de la cámara: ", matrix)
    print("Coeficiente de Distorsión ", dist)

    # Inicializamos la cámara
    cap = cv2.VideoCapture("http://192.168.137.129:81/stream")
    #cap = cv2.VideoCapture("rtsp://192.168.137.246:8554/mjpeg/1")
    width = 1024
    height = 768
    font = cv2.FONT_HERSHEY_PLAIN
    starting_time = time.time()
    frame_id = 0

    # Agregar una lista para almacenar las escalas correspondientes a cada ArUco
    aruco_scales = None  # Modificar según la cantidad de ArUcos

    nc = NoChocar


    while True:
        ret, frame = cap.read()
        frame = apply_perspective_transform(frame, M)
        # Detección de ArUco
        ocho_x, ocho_y, zero_x, zero_y, ocho_ok, zero_ok = aruco_detector.detect_markers(frame, matrix, dist)
        aruco_scale = aruco_detector.get_aruco_scale()
        aruco_id = aruco_detector.get_aruco_id()
        if aruco_scale is not None:
            aruco_scales = aruco_scale

        # Detección de YOLO
        annotated_frame, boxes = yolo_detector.detect_objects(frame)

        for box in boxes:
            x, y, w, h = box.xywh[0]
            x1 = x
            y1 = y+(h/2)

            #if cv2.waitKey(1) & 0xFF == ord("p"):
            #    print("ANCHO: ",w, "LARGO: ",h)
            obj_center = [int(x1.item()), int(y1.item())]
            if zero_x is not None and ocho_x is not None:
                if not (x1+(w/2) < ocho_x or x1-(w/2) > zero_x):
                    # Dibujar línea entre el centro del ArUco seleccionado y los objetos detectados
                    aruco_center = aruco_detector.get_aruco_center()

                    if aruco_center is not None:
                        aruco_center = (int(aruco_center[0]), int(aruco_center[1]))

                    # Calcular y mostrar la distancia en el video - le paso las coordenadas x,y del centro del objeto
                        distance_result = aruco_detector.calculate_distance(int(obj_center[0]), int(obj_center[1]),
                                                                            annotated_frame)
                        if distance_result is not None and aruco_id == 1:
                            #distance_cm = (distance_result/(w*aruco_scale))
                            #distance_cm = distance_result/15
                            distance_cm = distance_result*11
                            cv2.putText(annotated_frame, f"{distance_cm:.2f} cm", obj_center, cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5, (255, 255, 255), 2)
                            cv2.line(annotated_frame, aruco_center, obj_center, (0, 255, 0), 2)

                            # Manejo del auto segun la distancia actual
                            print("-------------------------------:::::::  ",int(obj_center[0]))
                            nc.no_chocar(distance_cm, int(obj_center[0]))

        # Mostramos el frame resultante
        if zero_ok and ocho_ok:
            cv2.line(annotated_frame, (ocho_x, ocho_y), (ocho_x, 0), (0, 255, 0), 2)
            cv2.line(annotated_frame, (zero_x, zero_y), (zero_x, 0), (0, 255, 0), 2)
            cv2.circle(annotated_frame, (ocho_x, ocho_y), 10, 255, -1)
            cv2.circle(annotated_frame, (zero_x, zero_y), 10, 255, -1)
        cv2.imshow("Combined Detection", annotated_frame)


        # Oprimiendo la tecla "q" finalizamos el proceso
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
