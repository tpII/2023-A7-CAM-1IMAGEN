import paho.mqtt.publish as publish

mqtt_server = "192.168.137.1"
mqtt_port = 1883
topic = "inTopic"

def send_message(message1, message2):
    publish.single(topic, f"{message1},{message2}", hostname=mqtt_server, port=mqtt_port)

send_message(20, 241)
