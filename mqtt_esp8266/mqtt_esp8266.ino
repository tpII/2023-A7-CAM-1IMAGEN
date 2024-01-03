#include <ESP8266WiFi.h>
#include <ESPPubSubClientWrapper.h>
#include "Servo.h"

const float  DMAX = 35.0;
const int    W_FRAME = 480;
const char* ssid = "NITRO5";
const char* password = "142536re";
const char* mqtt_server = "192.168.137.1";
float distance_cm = 40;
float obj_center_x = 0.0;
const char* fin = "F";
int potencia = 125;


ESPPubSubClientWrapper client(mqtt_server);
long lastMsg = 0;
char msg[50];
int value = 0;
char ultimo;


int servo_pin = 0; 
int motor_pin_a = 5;
int motor_pin_b = 14;

Servo myservo;  
int angle = 0;

void adelante_max(){
  analogWrite(motor_pin_b, 1);
  Serial.println();
  Serial.print("adelante max \r");
}

void adelante(){
  analogWrite(motor_pin_b, 135);
  Serial.println();
  Serial.print("adelante");
}
void atras(){
  analogWrite(motor_pin_a, 100);
  Serial.println();
  Serial.print("atras");
}
void frenar(){
  analogWrite(motor_pin_a, 255);
  analogWrite(motor_pin_b, 255);
  Serial.println();
  Serial.print("freno \r");
}

void derecha(){
  myservo.write(0);
  Serial.println();
  Serial.print("derecha");

}
void izquierda(){
  myservo.write(180);
  Serial.println();
  Serial.print("izquierda \r");
}
 
void centro(){
  myservo.write(80);
  Serial.println();
  Serial.print("centro \r");
}

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  }


float prev_dist_actual = 0.0;  // Variable global para almacenar el valor anterior de dist_actual

void callback(char* topic, byte* payload, unsigned int length) {
  char* payload_str = reinterpret_cast<char*>(payload);

  // Parsear el mensaje en formato "valor1,valor2"
  char* value1_str = strtok(payload_str, ",");
  char* value2_str = strtok(NULL, ",");

  if (value1_str != NULL && value2_str != NULL) {
    // Convertir las cadenas a valores flotantes
    float dist_actual = atof(value1_str);
    obj_center_x = atof(value2_str);

    Serial.print("Received distance_cm: ");
    Serial.println(dist_actual);
    Serial.print("Received obj_center_x: ");
    Serial.println(obj_center_x);

    // Verificar si la distancia ha variado en al menos 5 cm desde la última recibida
    if (fabs(dist_actual - prev_dist_actual) >= 5.0) {
      // Actualizar la variable prev_dist_actual
      prev_dist_actual = dist_actual;

      // Aquí puedes agregar lógica adicional según tus necesidades
      // ...

      // Ejemplo: llamar a la función no_chocar con los valores almacenados
      if (DMAX > dist_actual) {
        no_chocar(dist_actual, obj_center_x);
      }
    }
  }
}

void no_chocar(float dist_actual, float coor_x) {
    frenar();
    Serial.print("--------------------");
    if (coor_x > W_FRAME/2) {
        izquierda();
        atras();
        delay(1300);
        adelante_max();
        delay(500);
        frenar();
        Serial.print("OKKK");
    }
    else {
        derecha();
        atras();
        delay(1300);
        adelante_max();
        delay(500);
        frenar();
        Serial.print("OKKK");
    }
  centro();
  adelante_max();
  delay(500);
  frenar();
  adelante();
  }

void connectSuccess(uint16_t count) {
  Serial.println("Connected to MQTT-Broker!\nThis is connection nb: ");
  Serial.println(count);
  lastMsg = millis();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup...");
  setup_wifi();
  Serial.println("WiFi setup complete.");
  client.setCallback(callback);
  client.onConnect(connectSuccess);
  client.on("lightOff", [](char* topic, byte* payload, unsigned int length) {digitalWrite(BUILTIN_LED, HIGH);});
  client.on("lightOn", [](char* topic, byte* payload, unsigned int length) {digitalWrite(BUILTIN_LED, LOW);});
  client.on("disconnect", [](char* topic, byte* payload, unsigned int length) {client.disconnect();});
  client.subscribe("inTopic");
  myservo.attach(servo_pin);
  pinMode(motor_pin_a, OUTPUT);
  pinMode(motor_pin_b, OUTPUT);
  Serial.println("Setup complete.");
  centro();
  frenar();
  adelante();
}

void loop() {
  client.loop();
}
