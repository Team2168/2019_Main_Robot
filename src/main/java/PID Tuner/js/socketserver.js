var io = require('socket.io').listen(3000);

io.sockets.on('connection', function(socket) {
    socket.on('send_message', function(data) {
    data.message = data.message + ' yo<br/>';
    socket.emit('get_message',data);
    });
}); 