/*
Contributors: Raghurama Bukkarayasamudram, Ritam Das, Brian Macomber
Date: 10/8/2020
Quest 2 - Tactile Internet
*/

var express = require("express");
var app = express();
var path = require("path");
var fs = require("fs");
var csv = require("csv-parse");
const SerialPort = require("serialport");
const Readline = require("@serialport/parser-readline");

//running on mac
const port = new SerialPort("/dev/cu.SLAB_USBtoUART", { baudRate: 115200 });
// running on windows
// const port = new SerialPort("/COM5", { baudRate: 115200 });

const parse = port.pipe(new Readline({ delimiter: "\n" }));

// clear csv file every time the program is started
fs.truncate("test_data.csv", 0, function () {
  console.log("file cleared");
});

// Read the port data and write to the csv file
port.on("open", () => {
  console.log("serial port open");
});
parse.on("data", (data) => {
  fs.appendFile("test_data.csv", data, function (err) {
    if (err) throw err;
  });
});

// viewed at http://localhost:8080
app.get("/", function (req, res) {
  res.sendFile(path.join(__dirname + "/index.html"));
});

// request data at http://localhost:8080/data or just "/data"
app.get("/data", function (req, res) {
  var data = []; // Array to hold all csv data
  fs.createReadStream("test_data.csv")
    .pipe(csv())
    .on("data", (row) => {
      data.push(row); // Add row of data to array
    })
    .on("end", () => {
      res.send(data); // Send array of data back to requestor
    });
});

app.listen(8080);
