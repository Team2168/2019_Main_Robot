//Webserver for First Robotics Team 2168


//Require Modules
var net = require("net"); 		//tcp server
var express = require('express');	//web server
var carrier = require('carrier');	//IO Reader

//data to store client socket in once connection is made
var clientSocket;



var app = express.createServer() //create express web server
    , io = require('socket.io').listen(app) // have Socket.IO listen on web page
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


//create TCP Server
var server = net.createServer();



//the server throws a 'connection' event every time a new client connects to the server.
//on a new connection, we get a new client socket called 'sock'
server.on('connection', function(sock) {

	//store the socket in a global variable for use in Socket.IO events
	clientSocket = sock;
    
	//output client IP and Port
    	console.log('CONNECTED: ' + sock.remoteAddress +':'+ sock.remotePort);
    	
	//wait until we get a full line of data from the client. (This implies that data
	//received from the client is terminated with '\n'
	carrier.carry(sock, function(line) {
		
		//data from client should be in JSON format. We then parse the string
		//to create a JSON object
    		var jsonObj = JSON.parse(line.toString('utf8'));

		//We blast a socket.io message with the completed JSON object. This is
		//used to update the plot as well as the web test fields.
        	io.sockets.emit('update', jsonObj);

		//print JSON object to screen
        	console.log(jsonObj);

  	 });
});

	
//The TCP Server throws a listening event, when it is set to listen, this function will
//print to the console the address in which the TCP is listening on
server.on("listening", function () {
	var address = server.address();
	console.log("server listening " +
		    address.address + "port " + address.port);
    });

//The TCP throws a error when ever an error occurs.
server.on("error", function () {
	
	console.log("wtf, Something with the TCP Server went wrong"
		  );
    });


//bind TCP server to this port
var PORT = 41235;
server.listen(PORT);


//When an IO messages are received from the web, we forward to the robot
io.sockets.on('connection', function (socket) {
	socket.on('send_message', function(data) {
		//write data received from webpage back to the connected TCP client
		clientSocket.write(data.message);
	
		//echo data back to webpage
   		socket.emit('get_message',data);
		
		//log data to console.
		console.log(data.message);
    });

});

