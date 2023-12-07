# main.py
import cv2
import numpy as np
from calibracion import *
from aruco_detection import ArucoDetector
from yolo_detection import YoloDetector
import mqtt
import time

W_FRAME = 480

# Inicializar variables compartidas
distance_cm = None
obj_center = None
def apply_perspective_transform(frame, M):
    transformed_frame = cv2.warpPerspective(frame, M, (frame.shape[1], frame.shape[0]))
    return transformed_frame

# Función para enviar por MQTT
def send_mqtt(distance_cm, x_coordinate):
    # Aquí puedes colocar el código para enviar por MQTT
    mqtt.send_message(distance_cm, x_coordinate)

# Función para el cálculo de distancia
def calculate_distance(aruco_detector, obj_center, annotated_frame, start_time):
    aruco_center = aruco_detector.get_aruco_center()
    if aruco_center is not None:
        aruco_center = (int(aruco_center[0]), int(aruco_center[1]))

    # Calcular y mostrar la distancia en el video
    distance_result = aruco_detector.calculate_distance(int(obj_center[0]), int(obj_center[1]), annotated_frame)
    if distance_result is not None and aruco_detector.get_aruco_id() == 1:
        distance_cm = distance_result * 6.2
        elapsed_time = time.time() - start_time
        print(f"Tiempo transcurrido desde el inicio: {elapsed_time:.2f} segundos")

        cv2.putText(annotated_frame, f"{distance_cm:.2f} cm", obj_center, cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 2)
        cv2.line(annotated_frame, aruco_center, obj_center, (0, 255, 0), 2)
        if distance_cm < 25:
            send_mqtt(distance_cm, obj_center[0])
            print("LE MANDEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")
            distance_cm= 9999
            time.sleep(6)
            send_mqtt(distance_cm, obj_center[0])


def main():
    # Restablecer el evento al inicio del programa
    # Inicializamos el detector de ArUcoq
    aruco_detector = ArucoDetector()
    # Inicializamos el detector de YOLO
    yolo_detector = YoloDetector(model_path='yolov8n.pt')  # Especifica la ruta correcta a tu modelo YOLO
    # Calibración de la cámara
    #calibracion = Calibracion()
    #matrix, dist, M = calibracion.calibracion_cam()
    matrix = np.array([[8606.9, 0, 575.19],
                    [0, 1642.5, 448.25],
                    [0, 0, 1]])
    dist = np.array([[-6.4083, 49.944, 0.12854, 0.0032317, -161.25]])
    M = np.array([[1, 0, 0],
               [0, 1, 0],
              [1.226e-19, 0, 1]])

    print("Matriz de la cámara: ", matrix)
    print("Coeficiente de Distorsión ", dist)

    # Inicializamos la cámara
    cap = cv2.VideoCapture("http://192.168.137.189:81/stream")
    #cap = cv2.VideoCapture(1)
    #cap = cv2.VideoCapture("rtsp://192.168.137.246:8554/mjpeg/1")
    width = 640
    height = 480
    font = cv2.FONT_HERSHEY_PLAIN
    starting_time = time.time()
    frame_id = 0

    # Agregar una lista para almacenar las escalas correspondientes a cada ArUco
    aruco_scales = None  # Modificar según la cantidad de ArUcos

    while True:
        start_time = time.time()  # Guarda el tiempo de inicio
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
            y1 = y + (h / 2)

            obj_center = [int(x1.item()), int(y1.item())]
            if zero_x is not None and ocho_x is not None:
                if not (((x1 + (w / 2) < ocho_x or x1 - (w / 2) > zero_x)) or (y1 + h / 2) > ocho_y):
                    # Dibujar línea entre el centro del ArUco seleccionado y los objetos detectados
                    calculate_distance(aruco_detector, obj_center, annotated_frame, start_time)

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
