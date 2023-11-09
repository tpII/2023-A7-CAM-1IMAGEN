import cv2
import numpy as np
import time

class reconocimiento():
    def __init__(self):
        cap = cv2.VideoCapture(0)  # 0 para la cámara por defecto, puedes ajustarlo si tienes varias cámaras
        model = cv2.dnn.readNet("C:\\Users\\54295\\PycharmProjects\\tdpII-A7\\darknet\\yolov3-tiny.weights", "C:\\Users\\54295\\PycharmProjects\\tdpII-A7\darknet\\cfg\\yolov3-tiny.cfg")

        # Asignamos los nombres de/ las clases pre entrenadas
        classes = []
        with open("C:\\Users\\54295\\PycharmProjects\\tdpII-A7\\darknet\\data\\coco.names", "r") as f:
            classes = [line.strip() for line in f. readlines( )]
        #. Obteremos los nombres de las capas
        layer_names = model.getLayerNames()
        output_layers = [layer_names[i - 1] for i in model.getUnconnectedOutLayers()]

        colors = np.random.uniform(0,255, size=(len(classes), 3))

        # Cargamos la camara y aplícamos la detección de objetos
        camera = cv2.VideoCapture(0)
        font = cv2.FONT_HERSHEY_PLAIN
        starting_time = time.time()
        frame_id = 0
        while True:
            _, frame = camera.read()
            frame_id += 1
            height, width, channels = frame.shape
            while True:
                _, frame = camera.read()
                frame_id += 1
                height, width, channels = frame.shape
                # Detección de objetos
                blob = cv2.dnn.blobFromImage(frame, 0.00392, (416,416), (0, 0, 0), True, crop=False)
                model.setInput(blob)
                outs = model.forward(output_layers)
                # Información que aparecerá en pantalla class ids =
                class_ids = []
                confidences = []
                boxes = []
                for out in outs:
                    for detection in out:
                        scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]
                        if confidence > 0.2:
                            # Objeto detectado
                            center_x = int(detection[0]* width)
                            center_y = int(detection [1] * height)
                            w = int(detection[2] * width)
                            h = int(detection[3] * height)
                            # Coordenadas del rectangulo
                            x = int(center_x - w/2)
                            y = int(center_y - h/2)
                            boxes.append([x, y, w, h])
                            confidences.append(float (confidence) )
                            class_ids.append(class_id)
                indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)
                for i in range(len(boxes)):
                    if i in indexes:
                        x,y, w, h = boxes[i]
                        label = str(classes[class_ids[i]])
                        confidence = confidences[i]
                        color = colors[class_ids[i]]
                        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                        cv2. rectangle(frame, (x, y), (x + w, y + 30), color, -1)
                        cv2.putText(frame, label + " " + str(round(confidence, 2)), (x, y + 30), font, 3, (255,255,255), 3)
                elapsed_time = time.time() - starting_time
                fps = frame_id / elapsed_time
                cv2.putText(frame, "FPS: " + str(round(fps, 2)), (10, 50), font, 3, (0, 0, 0), 3)
                cv2. imshow("Reconocimiento de objetos", frame)
                # Oprimtendo la tecla "q" finalizamos el proceso
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            camera.release()
            cv2.destroyAllWindows()
