#include <ESP8266WiFi.h>
#include <ESPPubSubClientWrapper.h>
#include "Servo.h"

const char* ssid = "NITRO5";
const char* password = "142536re";
const char* mqtt_server = "broker.mqtt-dashboard.com";

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

void adelante(){
  analogWrite(motor_pin_b, 125);
}
void atras(){
  analogWrite(motor_pin_a, 125);
}
void frenar(){
  analogWrite(motor_pin_a, 255);
  analogWrite(motor_pin_b, 255);
}

void derecha(){
  myservo.write(0);

}
void izquierda(){
  myservo.write(180);
}
 
void centro(){
  myservo.write(80);
}

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  //WiFi.mode(WIFI_STA);
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

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.print((char)payload[0]);
    if ((char)payload[0] == 'd' and (ultimo!= 'd')) {
    derecha();
    Serial.print("entre ");
  }
  if ((char)payload[0] == 'i' and (ultimo!= 'i')) {
    izquierda();
    Serial.print("entre ");
  }
  if ((char)payload[0] == 'c' and (ultimo!= 'c')) {
    centro();
  }
  if ((char)payload[0] == 'a' and (ultimo!= 'a')) {
    adelante();
  }
  if ((char)payload[0] == 'f' and (ultimo!= 'f')) {
    frenar();
  }
  ultimo = (char)payload[0];
  Serial.println();
}
void connectSuccess(uint16_t count) {
  Serial.println("Connected to MQTT-Broker!\nThis is connection nb: ");
  Serial.println(count);
  lastMsg = millis();
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setCallback(callback);
  client.onConnect(connectSuccess);
  client.on("lightOff", [](char* topic, byte* payload, unsigned int length) {digitalWrite(BUILTIN_LED, HIGH);});
  client.on("lightOn", [](char* topic, byte* payload, unsigned int length) {digitalWrite(BUILTIN_LED, LOW);});
  client.on("disconnect", [](char* topic, byte* payload, unsigned int length) {client.disconnect();});
  client.subscribe("inTopic");
  myservo.attach(servo_pin);
  pinMode(motor_pin_a, OUTPUT);
  pinMode(motor_pin_b, OUTPUT);
  frenar();
  centro();
}

void loop() {
  client.loop();
}
