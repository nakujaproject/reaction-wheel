import paho.mqtt.client as mqtt
import json

state = 1


def on_connect(client, userdata, flags, rc):  # The callback for when the client connects to the broker
    print("Connected with result code {0}".format(str(rc)))  
    client.subscribe("NakujaTelemetry")  


def on_message(client, msg):  # The callback for when a PUBLISH message is received from the server.
    print("Message received-> " + msg.topic + " " + str(msg.payload))  # Print a received msg
    state = state + 1
    if state > 3 :
        state = 1

    print("state" + state)
    meas = {"state": state}
    ret= client.publish("NakujaState",json.dumps(meas))


client = mqtt.Client("testing")  
client.on_connect = on_connect  # Define callback function for successful connection
client.on_message = on_message  # Define callback function for receipt of a message
client.connect('broker.hivemq.com', 1883)
client.loop_forever()  # Start networking daemon