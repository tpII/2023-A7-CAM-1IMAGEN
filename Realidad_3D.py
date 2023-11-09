# ------------- Importo librerias     ##
import cv2
import numpy as np
import time
from calibracion import *
from reconocimiento_obj import *


#--------------- Detector aruco -----------
# Cargamos el diccionario de ArUco
diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)

# Inicializamos parámetros del detector de ArUco
parametros = cv2.aruco.DetectorParameters_create()

#------------------------- Lectura de la camara ----------------
cap=cv2.VideoCapture(0)
cap.set(3,1280)  # Definimos un ancho y alto para siempre
cap.set(4,720)
cont = 0  # cantidad de capturas

# Calibracion
calibracion = calibracion()
matrix, dist = calibracion.calibracion_cam()
print("Matriz de la camara: ", matrix)
print("Coeficiente de Distorsion ", dist)

# Inicializacion para reconocer objetos
#cap = cv2.VideoCapture(0)  # 0 para la cámara por defecto, puedes ajustarlo si tienes varias cámaras
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
font = cv2.FONT_HERSHEY_PLAIN
starting_time = time.time()
frame_id = 0
# Fin inicializacion reconocimiento objetos


while True:
    #Parte reconocimiento objetos

    _, frame = cap.read()      #leemos la camara
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

    ### Parte de manejo ArUco
    #ret, frame = cap.read()     # Leemos la camara
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)     # Pasamos a escala de grises
    # Detectamos los marcadores en la imagen
    # Camera matrix: Calibracion de la imagen
    esquinas, ids, candidatos_malos = cv2.aruco.detectMarkers(gray, diccionario, parameters=parametros)

    try:
        # Si hay marcadores encontrados por Detectamos
        if np.all(ids != None):
            # Iterar en marcadores
            for i in range(len(ids)):
                # Estime la pose de cada marcador y devuelva los valores rvec y tvec --- diferentes de los coeficientes de la camara
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(esquinas[i], 0.02, matrix, dist)

                print("Matriz de la camara: ", matrix)
                print("Coeficiente de Distorsion ", dist)
                print("RVEC: ", rvec)
                print("tvec ", tvec)
                #Eliminamos el error de la matriz de valores numpy
                (rvec - tvec).any()

                # Dibuja un cuadrado alrededor de los marcadores
                cv2.aruco.drawDetectedMarkers(frame, esquinas)

                (topLeft, topRight, bottomRight, bottomLeft) = esquinas[0][0][0],esquinas[0][0][1],esquinas[0][0][2],esquinas[0][0][3]
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                  # draw the bounding box of the ArUCo detection
                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                            # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                if topLeft[1]!=topRight[1] or topLeft[0]!=bottomLeft[0]:
                    rot1=np.degrees(np.arctan((topLeft[0]-bottomLeft[0])/(bottomLeft[1]-topLeft[1])))
                    rot2=np.degrees(np.arctan((topRight[1]-topLeft[1])/(topRight[0]-topLeft[0])))
                    rot=(np.round(rot1,3)+np.round(rot2,3))/2
                    print(rot1,rot2,rot)
                else:
                    rot=0

                # draw the ArUco marker ID on the image
                rotS=",rotation:"+str(np.round(rot,3))
                cv2.putText(frame, ("position: "+str(cX) +","+str(cY)),
                (100, topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0, 80), 2)
                cv2.putText(frame, rotS,(400, topLeft[1] -15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0, 80), 2)

                # Dibujamos los ejes
                cv2.drawFrameAxes(frame, matrix, dist, rvec, tvec, 0.01) # Con 0.01 decimos que tan largos son

                # Dibujando linea del aruco al objeto detectado
                cv2.line(frame, topLeft, (20,20), (255, 0, 0), 2)

                # Coordenada X del centro del marcador
                c_x = (esquinas[i][0][0][0] + esquinas[i][0][1][0] + esquinas[i][0][2][0] + esquinas[i][0][3][0]) / 4

                # Coordenada Y del centro del marcador
                c_y = (esquinas[i][0][0][1] + esquinas[i][0][1][1] + esquinas[i][0][2][1] + esquinas[i][0][3][1]) / 4

                # Mostramos el ID
                cv2.putText(frame, "id" + str(ids[i]), (int(c_x), int(c_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 225, 250), 2)

                # Extraemos los puntos de las esquinas en coordenadas separadas
                c1 = (esquinas[0][0][0][0], esquinas[0][0][0][1])
                c2 = (esquinas[0][0][1][0], esquinas[0][0][1][1])
                c3 = (esquinas[0][0][2][0], esquinas[0][0][2][1])
                c4 = (esquinas[0][0][3][0], esquinas[0][0][3][1])
                v1, v2 = c1[0], c1[1]
                v3, v4 = c2[0], c2[1]
                v5, v6 = c3[0], c3[1]
                v7, v8 = c4[0], c4[1]

    except:
        if ids is None or len(ids) == 0:
            print("*********** Marker Detection Failed ***************")
    
    #cv2.imshow('Realidad Virtual', frame)

    k = cv2.waitKey(1)
    # Oprimtendo la tecla "q" finalizamos el proceso
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    #cap.release()
    # cv2.destroyAllWindows()

    # ALmacenamos las fotos para la calibracion
    if k == 97:
        print("Imagen Guardada")
        cv2.imwrite("cali{}.png".format(cont), frame)
        cont = cont + 1

    if k == 27:
        break
        
cap.release()
cv2.destroyAllWindows()
