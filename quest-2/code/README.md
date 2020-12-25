# Code Readme

### Contributors: Raghurama Bukkarayasamudram, Ritam Das, Brian Macomber


#### adc_example_main.c
adc_example_main.c was adapted from the esp-idf examples. It includes 3 functions respective to each sensor and a display_console function to output readings in CSV format. Separate adc channels were initialized for each sensor and a task was created in main for the display console function.

#### data_to_csv.js
data_to_csv.js was adapted from online sources and from part of skill 16 and 17. Essentially, this file reads the console log data from each of the sensors and stores it in a file called "test_data.csv". It also has an error function to account for situations where the data was not stored properly or if console data would not show up. It sends a "Saved" message after each set of data is stored, and it repeats.

#### index.html
index.html was adapted using examples on the CanvasJS website. It includes embedded javascript where there is a function that is called every time the webpage is opened, and within that a function that is called initially, and every 2 seconds after that. This reoccuring function queries the data from the local /data endpoint and renders the canvasjs chart that plots the sensor data.
