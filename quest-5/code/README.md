# Code Readme

Quest 5: Cruise Control
Authors: Ritam Das, Brian Macomber, and Raghurama Bukkarayasamudram
Date: 2020-12-6

# index.html

For index.html, we went with a very basic layout that displays our frontend properly. It showcases our name, the quest, and two buttons to indicate START and STOP. I use socket.io in order to communicate from the client to the server. With each button, there are two onclick events that are triggered. On pressing "START", the START() function will be called which sets the flag to 1, and emits a "START" message via socket.emit. Likewise, "STOP" will emit a "STOP" message via socket.emit to the console log on the server.js file. This successfully proves web client to server communication to control the car.

# server.js

This file creates and hosts a server through UDP connection and node.js. This was taken from our code from last quest and altered in order to fit the requirements. Upon establishing a successful connection, the server will receive any messages from the client. In this case, it will receive a START or STOP message that will be outputted to the console.log. Server.js will also be responsible for using the console.log output after receiving the message from the client to communicate with the crawler device to start and stop moving.

# crawler.c

This file is the code that is on the crawler itself. Most of the functionality is implemented as separate RTOS Tasks to help organization. It first initializes WiFi so the crawler can communication over UDP with the node server. Then it calibrates the ESC by setting the PWM to te neutral value for 3 seconds, then the calibration sound should be heard. The speed control task will not begin until it has received at "START" signal over UDP from the node server. Once this happens, the car will start moving until it received a "STOP" message from the server, or the Lidar v4 at the front of the car detects an object within 40cm. While all of this happens, the alphanumeric display is always displaying the current speed relative to the motion of the wheels, the optical encoder is counting pulses and calculating wheel speed in m/s, and the Lidar is constantly checking the distance.
