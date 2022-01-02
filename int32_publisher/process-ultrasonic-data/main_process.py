# Process the incoming MQTT Data from the ESP32's Ultrasonic Sensor
# Save and visuzalize the data
# Written by Ahmed Al Bayati
# IoT Simplified
# https://iotsimplified.com/

import paho.mqtt.client as mqtt
import threading
import time
import datetime as dt
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import csv
from itertools import count

# Enter your broker address
BROKER_ADDR = ""

# Plot Style
plt.style.use("fivethirtyeight")
# Used for index counting
index = count()

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("/height_reading/qos1")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    # Msg comes in as a byte format. Needs to be decoded to UTF-8 characters.
    height_decoded = msg.payload.decode("utf-8")
    
    # Data is saved to the sensor_data.csv file for later analysis and visualization
    with open("sensor_data.csv",'a+',newline='') as data_file:
        data_writer = csv.writer(data_file,delimiter=',',quoting=csv.QUOTE_MINIMAL)
        data_writer.writerow([next(index),height_decoded])

# MQTT setup functions
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_ADDR, 1883, 60)

# Animate function for the plot. For live plotting of the data in sensor_data.csv
def animate(i):

    time_arr = []
    height_arr = []

    # Read the data in the csv file and add them to the lists initilized above
    with open("sensor_data.csv",mode='r') as data_file:
        data_reader = csv.reader(data_file,delimiter=',')

        for row in data_reader:
            time_arr.append(int(row[0]))
            height_arr.append(int(row[1]))

    # Update plot and labels
    plt.cla()
    plt.plot(time_arr,height_arr)

    plt.title('Height Data Over Time')
    plt.ylabel("Height")


# Thread: mqtt blocking loop. It has its own thread.
def mqtt_loop():

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    client.loop_forever()

# Function to be called for visualizing the data. plt.show() is blocking.
def visualize_data_loop():
    ani = animation.FuncAnimation(plt.gcf(),animate,interval=1000)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    print("Start of program")
    
    # Create a thread for the mqtt functions
    mqtt_thread = threading.Thread(target=mqtt_loop)
    mqtt_thread.start()
    
    # Run the matplotlib on the main thread
    visualize_data_loop()



