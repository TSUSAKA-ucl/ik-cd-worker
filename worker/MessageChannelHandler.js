// messageチャネルに
// RPCタイプの送信コマンドと、流しっぱなし受信して
// レスポンスのコマンドスロットに応じてキューにいれるだけの非同期コマンドを
// 定義して、ハンドルするクラス
// 反対側のportは、RPCタイプのコマンドが来たら所定のレスポンスを返してくるため
// こちら側ではタイムアウト処理だけ行う
// 送受信するオブジェクトは、commandプロパティと必要に応じてuuidスロットを持つ
// RPCの場合はuuidスロットはクラス内で生成される
class MessageChannelHandler {
  constructor(port) {
    this.port = port;
    this.rpcCallbacks = new Map();
    this.onCallbacks = new Map();
    this.port.onmessage = (event) => {
      const { command, uuid, ...rest } = event.data;
      // if (this.rpcCallbacks[uuid]) {
      if (uuid && this.rpcCallbacks.has(uuid)) {
	const {resolve, timeoutId} = this.rpcCallbacks.get(uuid);
	clearTimeout(timeoutId);
	this.rpcCallbacks.delete(uuid);
	resolve(rest);
      } else {
	if (this.onCallbacks.has(command)) {
	  this.onCallbacks.get(command)(rest);
	}
      }
    };
  }

  async callRpc(data, timeout = 1000) {
    const uuid = crypto.randomUUID();
    return new Promise((resolve, reject) => {
      const timeoutId = setTimeout(() => {
	if (this.rpcCallbacks.has(uuid)) {
	  this.rpcCallbacks.delete(uuid);
	  reject(new Error('RPC timeout: ' + data.command));
	}
      }, timeout);
      this.rpcCallbacks.set(uuid, { resolve, timeoutId });
      this.port.postMessage({ command: data.command, uuid, ...data });
    });
  }

  post(data) {
    this.port.postMessage(data);
  }

  on(command, callback) {
    this.onCallbacks.set(command, callback);
  }
}
export default MessageChannelHandler;
