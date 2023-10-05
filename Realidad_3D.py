# ------------- Importo librerias
import cv2
import numpy as numpyfrom Calibracion import*


#--------------- Detector aruco -----------
# Inicializamos parametros del detector de arucos
parametros = cv2.aruco.DetectorParameters_create()

# Cargamos el diccionario de nuestro aruco
diccionario = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)

#------------------------- Lectura de la camara ----------------
cap=  cv2.VideoCapture(0)
cap.set(3,1280)  # Definimos un ancho y alto para siempre
cap.set(4,720)
cont = 0  # cantidad de capturas

# Calibracion
calibracion = calibracion()
matrix, dist = calibracion.calibracion_cam()
print("Matriz de la camara: ", matrix
print("Coeficiente de Distorsion ", dist)

while true:
    ret, frame = cap.read()     # Leemos la camara
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)     # Pasamos a escala de grises
    # Detectamos los marcadores en la imagen
    # Camera matrix: Calibracion de la imagen
    esquinas, ids, candidatos_malos = cv2.aruco.detectMarkers(gray, diccionario, parameters = parametros, cameraMatrix = matrix, distCoeff = dist)

    try:
        # Si hay marcadores encontrados por Detectamos
        if np.all(ids != None)
            # Iterar en marcadores
            for i in range(0, len(ids)):
                # Estime la pose de cada marcador y devuelva los valores rvec y tvec --- diferentes de los coeficientes de la camara
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(esquinas[i], 0.02, matrix, dist)

                #Eliminamos el error de la matriz de valores numpy
                (rvec - tvec).any()

                # Dibuja un cuadrado alrededor de los marcadores
                cv2.aruco.drawDetectedMarkers(frame, esquinas)

                # Dibujamos los ejes
                cv2.aruco.drawAxis(frame, matrix, dist, rvec, tvec, 0.01) # Con 0.01 decimos que tan largos son

                # Coordenada X del centro del marcador
                c_x = (esquinas[i][0][0][0] + esquinas[i][0][1][0] + esquinas[i][0][2][0] + esquinas[i][0][3][0]) / 4

                # Coordenada Y del centro del marcador
                c_y = (esquinas[i][0][0][1] + esquinas[i][0][1][1] + esquinas[i][0][2][1] + esquinas[i][0][3][1]) / 4

                # Mostramos el ID
                cv2-putText(frame, "id" + str(ids[i]), (int(c_x), int(c_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 225, 250), 2)

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
        id ids is None or len(ids) == 0:
            print("*********** Marker Detection Failed ***************")
    
    cv2.imshow('Realidad Virtual', frame)

    k = cv2.waitKey(1)

    # ALmacenamos las fotos para la calibracion
    if k == 97:
        print("Imagen Guardada")
        cv2.imwrite("cali{}.png".format(cont), frame)
        cont = cont + 1

    if k == 27:
        break
        
cap.release()
cv2.destroyAllWindows()
