/* 
Contributors: Raghurama Bukkarayasamudram, Ritam Das, Brian Macomber
Date: 11/13/2020
Quest 4 - E-Voting v2
*/

// Here is code for the nodejs server:
// - create webclient
// - send and recieve messages from webclient (using socket.io)
// - Create Database
// - Add entries (votes) to database once recieved from ESP

//We're going to addapt code from an example that Matt gave:
/////////////////////////////////////////////////////////
//                      Modules                       //
/////////////////////////////////////////////////////////

var level = require("level");
var express = require("express");
var app = require("express")();
var http = require("http").Server(app);
var io = require("socket.io")(http);
var rimraf = require("rimraf");

/////////////////////////////////////////
// UDP comms with Leader ESP

var dgram = require("dgram");
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

// Create or open the underlying LevelDB store
var db = level("./mydb", { valueEncoding: "json" });

// On connection, print out received message
server.on("message", function (message, remote) {
  console.log(remote.address + ":" + remote.port + " - " + message);

  // Get current time
  var date = Date.now();

  // Fill in data structure
  var value = [
    {
      id: String.fromCharCode(message[0]),
      vote: String.fromCharCode(message[2]),
    },
  ];

  // Here we want to add the message (vote) to the database
  db.put([date], value, function (err) {
    if (err) return console.log("Ooops!", err); // some kind of I/O error
  });

  //send database with all new entires to the front end
  readDB();
});

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

// Points to index.html to serve webpage
app.get("/", function (req, res) {
  res.sendFile(__dirname + "/index.html");
});

// Function to stream from database
function readDB(arg) {
  db.createReadStream()
    .on("data", function (data) {
      console.log(data.key, "=", data.value);
      // Parsed the data into a structure but don't have to ...
      var dataIn = { [data.key]: data.value };
      // Stream data to client
      io.emit("message", dataIn);
    })
    .on("error", function (err) {
      console.log("Oh my!", err);
    })
    .on("close", function () {
      console.log("Stream closed");
    })
    .on("end", function () {
      console.log("Stream ended");
    });
}

// When a new client connects

io.on("connection", function (socket) {
  console.log("a user connected");

  // Call function to stream database data
  readDB();

  socket.on("flag", function (data) {
    console.log("reset!");
  });

  socket.on("disconnect", function () {
    console.log("user disconnected");
  });
});

//get message from front end
io.on("message", function (msg) {
  console.log("recieved msg: " + msg);
  //delete database here, then create new one
  rimraf("./mydb", function () {
    console.log("done");
  });

  //create new one
  // var db = level("./mydb", { valueEncoding: "json" });

  //call readDB to stream new database to front end
  readDB();
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
