import socketIOClient from 'socket.io-client';

const socketHost = window.location.hostname; // read from browser URL
const socketPort = 8081;
const socket = socketIOClient(socketHost+':'+socketPort+'/ui', { transports: ['websocket'] });

export default (store) => {
	console.log("Initializing communicator");

	socket.on('telemetry', (data) => {
		store.dispatch({ type: 'TELEMETRY', payload: data });
	});

	return (next) => (action) => {
		if (action.type === 'TRANSMIT') {
			socket.emit(action.payload.msg, action.payload.data);
			console.log("Transmitting", action.payload);
		}
		next(action);
	}
}