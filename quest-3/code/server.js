/*
Contributors: Raghurama Bukkarayasamudram, Ritam Das, Brian Macomber
Date: 10/23/2020
Quest 3 - Hurricane Box
*/

var dgram = require("dgram");

var express = require("express");
var app = express();
const bodyParser = require("body-parser");
var path = require("path");
var fs = require("fs");
var csv = require("csv-parse");

var backToESP = "0";

app.use(bodyParser.urlencoded({ extended: true }));

// clear csv file every time the program is started
fs.truncate("test_data.csv", 0, function () {
  console.log("file cleared");
});

// ************ Web client to get data from and graph ****************** //

// viewed at http://localhost:8080
app.get("/", function (req, res) {
  res.sendFile(path.join(__dirname + "/index.html"));
});

app.post("/led", function (req, res) {
  res.send("Led level: " + req.body.brightness);
  console.log(req.body.brightness);
  // send data back to esp
  backToESP = req.body.brightness;
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

// ******************************************************** //

// ************ Create socket to communicate with esp ****************** //

// Port and IP
var PORT = 1131;
var HOST = "192.168.7.196"; //ip of my laptop

// Create socket
var server = dgram.createSocket("udp4");

// Create server
server.on("listening", function () {
  var address = server.address();
  console.log(
    "UDP Server listening on " + address.address + ":" + address.port
  );
});

// On connection, print out received message
server.on("message", function (message, remote) {
  console.log(remote.address + ":" + remote.port + " - " + message);

  //here we can append data to csv file
  //add data to csv
  fs.appendFile("test_data.csv", message, function (err) {
    if (err) throw err;
  });

  // Send Ok acknowledgement - and send led status back to esp
  server.send(backToESP, remote.port, remote.address, function (error) {
    if (error) {
      console.log("MEH!");
    } else {
      console.log("Sent: Ok!");
    }
  });
});
// ******************************************************** //

// Bind server to port and IP
server.bind(PORT, HOST);

//run app
app.listen(8080);
