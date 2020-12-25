# Quest 1: Fish Feeder

Authors: Brian Macomber, Raghurama Bukkarayasamudram, Ritam Das

## Date: 2020-09-21

## Summary

Leveraging the features and power of the ESP32, our team was able to build a timer function to control the actuation of a servo to "feed fish" at regular intervals. We also produced a digital output on the alphanumeric display counting down the time until the next feeding. We drew inspiration and abstracted ideas from previously completed code examples in cluster 1, namely the i2c_display, timer_example, RTOS, and servo skills.

#### Investigative Question

What approach can you use to make setting the time interval dynamic (not hard coded)? Elaborate.

One approach to setting the time interval dynamically would be to allow the user to enter the length of the timer
every time the timer finishes. This way there is no hard coded value for the timer, and the user has freedom to start the new fish food timer whenever they want.
Futhermore, the system could also somehow monitor how much food is exactly dropped in at the last alarm, and calculate the timer until the next food drop based on how much food the fish just recieved.

## Self-Assessment

### Objective Criteria

| Objective Criterion                  | Rating | Max Value |
| ------------------------------------ | :----: | :-------: |
| 1. Must keep track of time           |   1    |     1     |
| 2. Must report time for every second |   1    |     1     |
| 3. Must use a servo                  |   1    |     1     |

### Qualitative Criteria

| Qualitative Criterion                          | Rating | Max Value |
| ---------------------------------------------- | :----: | :-------: |
| Quality of solution                            |   5    |     5     |
| Quality of report.md including use of graphics |   3    |     3     |
| Quality of code reporting                      |   3    |     3     |
| Quality of video presentation                  |   3    |     3     |

## Solution Design

## Sketches and Photos

![image](https://user-images.githubusercontent.com/35698105/93964006-8467b980-fd2c-11ea-9065-e4b0b1ed2ab4.png)

## Supporting Artifacts

Video Demonstration:

[![](http://img.youtube.com/vi/HcJgBgHboic/0.jpg)](http://www.youtube.com/watch?v=HcJgBgHboic "Team 3: Fish Feeder")

## Modules, Tools, Source Used Including Attribution

## References

- Timer: https://github.com/espressif/esp-idf/tree/17ac4ba/examples/peripherals/timer_group
- Servo Control: https://github.com/espressif/esp-idf/tree/master/examples/peripherals/mcpwm/mcpwm_servo_control
- I2C Display: https://github.com/BU-EC444/code-examples/tree/master/i2c-display

---
