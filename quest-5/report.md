# Quest 5: Cruise Control

Authors: Ritam Das, Brian Macomber, and Raghurama Bukkarayasamudram 
Date: 2020-12-6

---

## Summary

This quest tasked us with adding autonomous capabilties to a crawler vehicle. We also implemented steering control and speed control. We used one lidar to attain distance and a pulse counter to track wheel speed. There is also an alphanumeric display to show realtime speed of the crawler in m/s. The crawler can also be stopped and started from a server through UDP communication, along with web client that uses socket.io to send data from the client to the server which will signal the crawler.

### Objective Criteria

| Objective Criterion                 | Rating | Max Value |
| ------------------------------------| :----: | :-------: |
| Remote Start/Stop                   |   1    |     1     |
| Stops before collision              |   1    |     1     |
| PID for speed control               |   0    |     1     |
| Measures wheel speed                |   1    |     1     |
| Displays speed on the alpha display |   1    |     1     |
| Sucessfully traverse A to B         |   1    |     1     |
| Controls Steering                   |   0    |     1     |

### Qualitative Criteria

| Qualitative Criterion                          | Rating | Max Value |
| ---------------------------------------------- | :----: | :-------: |
| Quality of solution                            |    3   |     5     |
| Quality of report.md including use of graphics |    3   |     3     |
| Quality of code reporting                      |    3   |     3     |
| Quality of video presentation                  |    3   |     3     |

## Solution Design

##### Crawler
The crawler uses the ESC module to control the speed of the car. The ESC is powered using the onboard rechargable 7.2V battery, and using the 5V output of the ESC, we powered the H-bridge along with the servo for turning the wheels (since the turning servo is a high current device). Both of these are controlled on the ESP32 using PWM to change the speed/direction of the car. Additionally, the Lidar v4 is powered using the 3.3v from the ESP and is connected to the I2C bus 0 (slave address of 0x62) on the ESP. This sensor is mounted at the front of the car to for object detection, and it will trigger an emergency stop if an object is too close. On one of the wheels of the car, a black and white encoder design is mounted along with an optical encoder to count the number of black stripes in a given rotation, in a given period of time. Using the period of time, cicumference of the wheel, and the number of black stripes in that period of time, we calculated the speed of the car in m/s. This speed is then displayed on the Alphanumeric Display on the car. The Alphanumeric display is also wired using I2C, but on Bus 1 (slave address 0x70). All of these components can be seen in Figure 2 and Figure 3, and the crawler diagram can be seen in Figure 1. Lastly, the ESP32 is connected to WiFi and communicates wireless over UDP with the node server to recieve start and stop instructions.


##### Node Server/Front End:
A node server was connected to an ESP32 over UDP socket communication on our local network. The web client allowed for start and stop functionality of the crawler. A boolean variable is set to either true or false permitting the car to either drive or stop.

## Investigative Question

### How would you change your solution if you were asked to provide ‘adaptive’ cruise control?

We would have the lidar mounted on the front to read distance and judge the speed and acceleration relative to the crawler through a series of calculations and mathematical manipulations. The crawler would then slow down to match or go just below that measured speed. We could also use multiple lidars and use a decision algorithm to determine good lidar readings, which would result in better speed and acceleration readings for adaptive capabilities.

## Sketches and Photos
Figure 1: Diagram

![Screen Shot 2020-12-07 at 1 54 36 PM](https://user-images.githubusercontent.com/37518854/101392436-c61bf100-3893-11eb-80da-895cb1f70fde.png)

Figure 2: Crawler Circuit

![crawler1](/quest-5/images/crawler1.jpg)

Figure 3: Lidar + Wheel Speed sensor

![crawler2](/quest-5/images/crawler2.jpg)


## Supporting Artifacts

#### Demo Video
[![](http://img.youtube.com/vi/tB2MOrUDSFE/0.jpg)](http://www.youtube.com/watch?v=tB2MOrUDSFE)

#### Quest Breakdown
[![](http://img.youtube.com/vi/U2N28mCyBY4/0.jpg)](http://www.youtube.com/watch?v=U2N28mCyBY4)

## Modules, Tools, Source Used Including Attribution

## References

- Socket.io: https://socket.io/docs/v3/index.html
- HTML Button: https://www.w3schools.com/tags/tryit.asp?filename=tryhtml_button_css
- Crawler speed & servo control: http://whizzer.bu.edu/briefs/design-patterns/dp-esc-buggy
- Speed Sensor: https://learn.sparkfun.com/tutorials/qrd1114-optical-detector-hookup-guide#example-circuit
- Alphanumeric Display: https://learn.adafruit.com/14-segment-alpha-numeric-led-featherwing
- Lidar v4: http://static.garmin.com/pumac/LIDAR-Lite%20LED%20v4%20Instructions_EN-US.pdf

---
