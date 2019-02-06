//Webserver for First Robotics Team 2168


//Require Modules
var dgram = require("dgram"); 
var express = require('express');

var app = express.createServer() //create express web server
    , io = require('socket.io').listen(app) // have web server listen on port
    , fs = require('fs') //create file system


// every time a http request is received on port 8080, deliver the index.html page
//in the root of this folder
app.listen(8090);
app.use("/js", express.static(__dirname + '/js'));
app.use("/style", express.static(__dirname + '/style'));
app.use("/images", express.static(__dirname + '/images'));
app.get('/', function (req, res) {
  res.sendfile(__dirname + '/index.html');
}); 


//create socket for receiving data from the robot, and update the plot with it
var server = dgram.createSocket("udp4");

//the server throws a message event every time a new message is received.the plot 
//listens for the 'update' event, so when new data is received we trigger the update
//event.

server.on("message", function (msg, rinfo) {
        var jsonObj = JSON.parse(msg.toString('utf8'));
	

        var robmsg = jsonObj["S"];

        io.sockets.emit('update', jsonObj);

        console.log(jsonObj);
       console.log(robmsg);
        io.sockets.emit('write_me', jsonObj["D_P"]);


    });


//print what address the server is listening on, when the listening event is thrown
server.on("listening", function () {
	var address = server.address();
	console.log("server listening " +
		    address.address + ":" + address.port);
    });

//bind data socks to this port
server.bind(41234);

//When an IO messages are received from the web, we forward to the robot
io.sockets.on('connection', function (socket) {
    socket.on('send_message', function(data) {

   socket.emit('get_message',data);

var message = new Buffer(data.message);

server.send(message, 0, message.length, 4444, "0.0.0.0", function(err, bytes) {
  
});
    });

});