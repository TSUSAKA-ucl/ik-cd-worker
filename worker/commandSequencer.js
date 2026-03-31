// 各コマンドの完了状態を管理（Promiseとその制御関数を保持）
const commands = {
  init: createDeferred(),
  setup: createDeferred(),
  process: createDeferred(),
  shutdown: createDeferred(),
};

// 半順序（依存関係）の定義
const dependencies = {
  setup: ['init'],
  process: ['setup'],
  shutdown: ['init'], // 最低限 init が終われば終了処理へ
};

self.onmessage = async (e) => {
  const { command, data } = e.data;
  const target = commands[command];

  if (!target) return; // 未定義のコマンドは無視

  try {
    // 1. 依存するコマンドがすべて「成功」するのを待つ
    const deps = dependencies[command] || [];
    await Promise.all(deps.map(dep => commands[dep].promise));

    // 2. 実際の処理
    console.log(`実行中: ${command}`);
    
    if (command === 'init') {
      const res = await fetch(data.url);
      if (!res.ok) throw new Error('Fetch failed');
      // 成功を通知
      target.resolve();
    } 
    else if (command === 'shutdown') {
      // 終了処理をしてWorkerを止める
      target.resolve();
      self.close(); 
    }
    // ... 他のコマンド処理
    else {
      target.resolve();
    }

  } catch (err) {
    // 3. エラーが発生したらそのコマンドを Reject する
    // これにより、このコマンドに依存している後続も自動的に await で止まる（catchへ飛ぶ）
    target.reject(err);
    console.error(`コマンド [${command}] でエラーが発生したため停止しました:`, err);
  }
};

// Promiseを外部から解決・拒否するためのユーティリティ
function createDeferred() {
  let resolve, reject;
  const promise = new Promise((res, rej) => {
    resolve = res;
    reject = rej;
  });
  return { promise, resolve, reject };
}
