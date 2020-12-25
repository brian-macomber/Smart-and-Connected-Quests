# Quest 3 - Hurricane Box

Authors: Brian Macomber, Raghurama Bukkarayasamudram, & Ritam Das

## Date: 2020-10-23

## Summary

In this quest, we built a hurricane box that tracks data and communicates information to a web client visualizing this data in graph format. We wired a thermistor and an accelerometer to the ESP32. We also wired an output LED to the ESP32, where a PWM cycle enabled a user to set a desired LED intensity (ranging from 0-9) through the web client. The temperature, and acceleration data were sent to the node server through a UDP socket and then sent to the front-end and plotted. The front-end also involved a separate port (1132) displaying realtime Pi web cam footage. To control the LED from the front-end, we used HTTP post requests, which carried an led signal value that was sent back to the ESP32 through the UDP socket. In the front end we plotted real-time sensor data using Canvas.js and embedded a text field to control the LED intensity. The web client was also port forwarded to http://team3ec444.ddns.net:1131 so that the device could be monitored and controlled from any IP.

## Self-Assessment

### Objective Criteria

| Objective Criterion                                | Rating | Max Value |
| -------------------------------------------------- | :----: | :-------: |
| Control alert LED remotely                         |   1    |     1     |
| Graph real time data that can be accessed remotely |   1    |     1     |
| Display live video that can be accessed remotely   |   1    |     1     |

### Qualitative Criteria

| Qualitative Criterion                          | Rating | Max Value |
| ---------------------------------------------- | :----: | :-------: |
| Quality of solution                            |   5    |     5     |
| Quality of report.md including use of graphics |   3    |     3     |
| Quality of code reporting                      |   3    |     3     |
| Quality of video presentation                  |   3    |     3     |

## Solution Design

### Web Client

The web client (frontend portion) includes three graphs- one main graph that displays temperature, roll, and roll vs time, two others for pitch v. time and roll v. time. All of this is based on real-time data measured in seconds. This was created based on adapted code from the previous quest. The backend portion gets the real-time data from the sensors which was formatted into csv files which was used for the graphs. Likewise, there was a form that uses numeric input that communicates via POST to get the message back to the ESP in order to connect with the LED intensity.

### Node Server

The node server creates the web client locally on port 8080, and this is port forwarded to http://team3ec444.ddns.net:1131 to be viewed remotely from any IP. On here it creates the '/' endpoint where it renders index.html, the '/data' endpoint where it streams data from the test_data.csv file, and '/led' where it posts the brightness of the led recieved from the user pressing 'submit' on the UI. In addition to this, it creates a socket on local port 1131 to talk as the server through UDP to the ESP-32. It recieves the formatted accelerometer and temperature data, and sends back confirmation along wit the LED brightness so the backend can update the LED. This part utilizes ideas from skill 20 and skill 17 along with helper code about UDP communication from whizzer.

### Embedded System

The ESP32 reads from two different sensors (a thermistor and accelerometer) and outputs to an LED based on the led intensity inputted by the user on the web client. Code was adapted from skill 13 (thermistor), skill 23 (accelerometer), and skill 24 (PWM to control LED). The section of the file regarding the thermistor converted an adc voltage reading to temperature measured in celsius. The accelerometer involved multiple functions: one to read from an 8-bit register, one to write to an 8-bit register, one to read 16 bits, two separate functions to calculate static acceleration values in the X, Y, and Z directions and convert them to tilt data (roll & pitch). The LED used a PWM cycle which allows the user to set their desired LED intensity on a scale of 0-9 from the web client. The relevant outputs of each function were then printed in csv format from main.

#### Wiring

The accelerometer was wired to the ESP32 using SCL and SDA pins with internal 10k pull up resistors. The LED was wired to GPIO12 with internal pull up resistor. The thermistor was wired using voltage divider with 10k resistor.

![IMG_5695](https://user-images.githubusercontent.com/37518854/97058558-26610880-155c-11eb-9bc0-0d4d7151f3d4.jpeg)

## Investigative Question

What are steps you can take to make your device and system low power?

- One possibility is to turn off sensors when they are not needed

- Turn on wifi only when necessary

## Sketches and Photos

<center><img src="./images/ece444.png" width="25%" /></center>  
<center> </center>

## Supporting Artifacts

- [![](http://img.youtube.com/vi/7r5HrHVoQv8/0.jpg)](http://www.youtube.com/watch?v=7r5HrHVoQv8 "Quest 3 Team Demo").

## Modules, Tools, Source Used Including Attribution

Ritam wired the device and wrote the embedded C code.

Brian wrote the node.js server and set up port-forwarding.

Ram wrote the front-end HTML file that plotted sensor data and interacted with the user.

## References

---
