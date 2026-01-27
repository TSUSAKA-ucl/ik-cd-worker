// ******** Worker State Singleton ******** //
export const st = Object.freeze({
  initializing: 1,
  waitingRobotType: 2,
  generatorMaking: 3,
  generatorReady: 4,
  slrmReady: 5,
});
export const sst = Object.freeze({
  dormant: 1,
  converged: 2,
  moving: 3,	// Cartesian mode
  rewinding: 4,
  jMoving: 5, // joint space mode
});
// ******** WebSocket Bridge Singleton ******** //
export const bridge = {
  url: null,
  socket: null,
  messageQueue: [],
  connect: function() {
    this.socket = new WebSocket(this.url);
    this.socket.onopen = () => {
      console.log('WebSocket connected');
      while (this.messageQueue.length > 0) {
	this.socket.send(this.messageQueue.shift());
      }
    };
    this.socket.onclose = (e) => {
      console.log('webSocket closed, will retry...', e.code,e.reason);
      this.scheduleReconnect();
    }
    this.socket.onerror = (err) => {
      console.error('WebSocket error', err);
      this.socket.close();	// the socket must be closed to reconnect
    }
  },
  reconnectTimer: null,
  scheduleReconnect: function() {
    if (this.reconnectTimer) return;
    this.reconnectTimer = setTimeout(()=>{
      this.reconnectTimer = null;
      if (this.url) {
	console.log('Reconnecting...');
	this.connect();
      }
    }, 3000); // 3秒後に再接続
  },
};
