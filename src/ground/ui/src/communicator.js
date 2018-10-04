import socketIOClient from 'socket.io-client';

socketHost = window.location.hostname; // read from browser URL
socketPort = 8081;
socket = socketIOClient(socketHost+':'+socketPort+'/ui', { transports: ['websocket'] });

export default (store) => {
    socket.on('telemetry', (data) => {
        store.dispatch('TELEMETRY', data);
    })

    // middleware
    return (store) => (next) => (action) => {
        console.log('dispatching', action)
        let result = next(action)
        console.log('next state', store.getState())
        return result
    }
}