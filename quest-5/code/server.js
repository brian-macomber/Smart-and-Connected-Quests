/* 
Contributors: Raghurama Bukkarayasamudram, Ritam Das, Brian Macomber
Date: 12/06/2020
Quest 5 - Cruise Control
*/

/////////////////////////////////////////////////////////
//                      Modules                       //
/////////////////////////////////////////////////////////

var express = require("express");
var app = require("express")();
var http = require("http").Server(app);
var io = require("socket.io")(http);
var dgram = require("dgram");

/////////////////////////////////////////
// UDP comms with Leader ESP

// Port and IP
var PORT = 1131;
var HOST = "10.0.0.113"; //ip of my laptop

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
});

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

// Points to index.html to serve webpage
app.get("/", function (req, res) {
  res.sendFile(__dirname + "/index.html");
});

// When a new client connects

//get message from front end
io.on("connection", (socket) => {
  console.log(socket.id);
  console.log("a user connected");

  socket.on("message", (arg) => {
    console.log(arg);
    server.send(arg, PORT, HOST, function (error) {
      if (error) {
        console.log("ohno");
      } else {
        console.log("sent!");
      }
    })
  });

  socket.on("disconnect", function () {
    console.log("user disconnected");
  });
});

///////////////////////////////////////
//          MAIN                    //
///////////////////////////////////////
// Bind server to port and IP
server.bind(PORT, HOST);

// Listening on localhost:3000
http.listen(3000, function () {
  console.log("listening on *:3000");
});
