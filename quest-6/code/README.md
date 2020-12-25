# Code Readme

#### toastESP.c

This file uses a thermistor to gauge temperature in the toaster and start/stop buttons that trigger a timer and display on the alphanumeric, and establishes UDP communication between the ESP32s and node server. Upon pressing start, the timer indicating toast time will begin. Stop will stop the timer and reset it. UDP communication is vital in delivering relevant info to and from ESPs and node server. 

#### secondaryESP.c

This code uses the garmin v4 LIDAR to check if there is "bread in the toaster". In actual terms, it checks to see if there is an object within a certain distance (20 cm). If it does detect something within the threshold, it will first print that "toast is detected", and then, signal the servo motor to turn to "close the toaster door". The LIDAR distance readings will also be printed out to the console for the user to see. I added a very long task delay to turn the servo the other way to signal that the toast is ready. Otherwise, it will keep the door as is, and just print to console that there is "no toast detected".

#### nodeserver.js

The node server creates the web client locally on port 3000. On here it creates the '/' endpoint where it renders index.html, the '/data' endpoint where it streams data from the test_data.csv file. In addition to this, it creates a socket on local port 1131 to talk to the ESP-32s through UDP. It receives the formatted temperature data. This temperature data is displayed on the web client as a function of time.

#### index.html

This file is the front end of our smart toaster. Its functionalities include starting and stopping/resetting the toaster timer, displaying real-time webcam feed of the toaster, and graphing temperature v. time (of the toast). The design includes two buttons- Start and Stop, and when pressed, the start button will send a signal to the node-server which will send a signal to the ESP to start the "toasting process" as indicated by the timer. A similar functionality exists for the stop button which will reset the timer to signal the esp.

Underneath the buttons, there is the live video feed of the raspberry pi which was taken from a previous skill.

Lastly, there is one graph that displays the toast temperature v. time (s). This reads a csv file that was generated from the thermistor readings from the node server. Our graph was done using canvas.js with the code adopted from the Fish Feeder quest.
