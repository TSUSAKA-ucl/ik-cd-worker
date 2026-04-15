// 0. このik-workerが状態量を作成してseq番号とqueueを管理する
// 0. 開始時にqueueの先頭(front)は、必ずOKの状態量が一つ存在する
// 0. seq番号単調増加でik-workerがqueueに積む(あるいは満員で積めない)時に1増加する
// 1. ik-workerは、cdチェックするサイクル毎にseq番号を付けた状態量を可能ならqueueする
// 1. ik-workerは、queueが満員だったらqueueせずにseqだけ増やしてcd-workerにpostMessageする
// 2. 相手cd-workerは、message channelの中のデータの最新のものをチェックしその結果を返す
// 2. cd-workerは、全てのmessageに対して、答えを返すとは限らない。処理可能な時の最新のものだけ処理して返す。
// 2. cd-workerは、可能なチェック結果(pair)をseq付きで(送ってきたik-workerに)返す
// 2. cd-workerが、チェックする状態量のseqは飛び飛びになることはあるが、単調増加で過去に遡らない
// 2. その結果、回答のseq番号は、必ずqueueのrearの番号以上である
// 3. ik-workerはdetect collisionのタイミングでcd-worker回答を調べる
// 4. 回答がOKで、そのデータがqueueにあれば、それを残しそのseqより古いデータをqueueから削除する
// 5. 回答がOKで、そのseqがrear-1のseqより大きければ、先頭(front)を残して全て削除する。
// 6. 回答がOKで、そのseqがrear-1のseq以下だがqueueの中に無いことはありえない
// 6. 回答がOKの場合は、上記どちらもrewindしない
// 7. 回答がNGだったら、先頭(OK)を残してその番号まで全てをqueueから削除する
// 8. 回答がNGで、そのseqがrear-1のseqより大きければ先頭(front OK)を残して全て削除する
// 9. 回答がNGであれば必ずqueueの先頭にrewindする。
//    その結果queueの先頭は事実上最新のOKデータとなる。
//    回答がOKであればrewindしない。queueの先頭はOKだが古いデータである可能性がある。
// 9. いずれにせよqueueが空になる(rear===frontになる)ことはない

// onmessage handlerから使用する部分
// msg.seq, msg.data
// msg.data.size() === 0 :> OK
// msg.data.size() > 0 :> NG(collision)

// const rqueue = new RewindQueue(32);
//  

// elementはqueueするもの
// newestResultはonmessageで受けた最新のデータ(elementとは別物)
// newestSeqは同最新受信データのシーケンス番号のコピー
// seqは、このクラスオブジェクト内で管理する
// 操作はFloat64のTypedArrayの一部分へのwrite/readで行う
// 
class RewindQueue {
  newestResult; // collisionPairsの場合はArray of [i,j]
  newestSeq;
  // enqueue(element)
  // getOnmessageHandler()
  // getRewindElement()
  // getLastSeq() // 最後にqueueされたelementのseq番号
  #queueBody;
  #queueMaxSize;
  #elementSize;
  #front = 0;
  #seq = 0;
  #rear;
  constructor (queueMaxSize, initialElement) {
    const elementSize = initialElement.length+1;
    this.#queueBody = new Float64Array(queueMaxSize * elementSize);
    // (+1はseq番号の分)
    this.#queueMaxSize = queueMaxSize;
    this.#elementSize = elementSize;
    this.#front = 0;
    this.#seq = 0;
    this.#queueBody[0] = this.#seq; // 最初の状態量はOKでseq=0
    this.#queueBody.set(initialElement, 1);
    this.#rear = elementSize; // rearは次に積む位置を指す
    this.newestResult = [];
    this.newestSeq = 0;
  }
  getOnmessageHandler() {
    // onmessageのスロットデコードで該当したときに呼ぶべき関数
    return (rest) => {
      const sequence = rest.sequence;
      const data = rest.data;
      this.#cleanUpQueue(sequence, !(data.length > 0));
      this.newestResult = data;
      this.newestSeq = sequence;
    };
  }
  #stepIndex (index) {
    index += this.#elementSize;
    if (index >= this.#queueMaxSize * this.#elementSize) {
      index = 0;
    }
    return index;
  }
  #stepBackIndex (index) {
    index -= this.#elementSize;
    if (index < 0) {
      index = (this.#queueMaxSize - 1) * this.#elementSize;
    }
    return index;
  }
  #currentSize () {
    if (this.#rear >= this.#front) {
      return (this.#rear - this.#front) / this.#elementSize;
    } else {
      return (this.#queueMaxSize * this.#elementSize - this.#front + this.#rear) / this.#elementSize;
    }
  }
  // element(だけseqを付けてrewind queueに積み
  enqueue(element) { // Float64Array (elementSize)
    this.#seq++;
    if (this.#rear !== this.#front) { // queueが満員ならばqueueしない
      this.#queueBody[this.#rear] = this.#seq;
      this.#queueBody.set(element, this.#rear+1);
      this.#rear = this.#stepIndex(this.#rear);
      // console.debug("rewindQueue: enqueue: seq: " + this.#seq + ", front seq: " + this.#queueBody[this.#front] + ", rear-1 seq: " + this.#queueBody[this.#stepBackIndex(this.#rear)]);
    } else {
      // console.warn("WARNING: rewindQueue: enqueue: queue is full.");
      // console.warn("WARNING: front seq: " + this.#queueBody[this.#front] +
      // 		   ", rear-1 seq: " + this.#queueBody[this.#stepBackIndex(this.#rear)] +
      // 		   ", new seq: " + this.#seq);
    }
  }
  // 4.5. 7.8.9. ik-workerがcd-workerからの回答を処理するための関数
  #searchSeq (seq) {
    let i = this.#front;
    while (i !== this.#rear) {
      if (this.#queueBody[i] === seq) {
	return i;
      }
      i = this.#stepIndex(i);
    }
    return -1; // 見つからなかった
  }
  #cleanUpQueue (seq, isOK) {
    if (isOK) {
      const idx = this.#searchSeq(seq);
      if (idx !== -1) { // 4. OKで、そのseqがqueueにあった!!
	// そのseqがqueueにあれば、そのseqより古いデータをqueueから削除する。
	// すなわちそのseqの位置をfrontにする。
	this.#front = idx;
	// console.log("rewindQueue: cleanUpQueue: OK and seq found in queue. size:" + this.#currentSize() +
	// 	    ", seq: " + seq + ", front seq: " + this.#queueBody[this.#front] +
	// 	    ", rear-1 seq: " + this.#queueBody[this.#stepBackIndex(this.#rear)]);
      } else { // 5. OKだが、そのseqがqueueにない!!
	// そのseqがqueueに無ければ、先頭(front)を残して全て削除する。
	// しかし、rear-1のseqより大きい場合は、これは事実上queueがオーバーフローしている状態で良くない。
	const rear1Seq = this.#queueBody[this.#stepBackIndex(this.#rear)];
	// if (seq > rear1Seq) {
	//   console.warn("WARNING: rewindQueue: cleanUpQueue: OK but seq not found in queue. size:" + this.#currentSize() +
	// 	       ", seq: " + seq + ", front seq: " + this.#queueBody[this.#front] +
	// 	       ", rear-1 seq: " + this.#queueBody[this.#stepBackIndex(this.#rear)]);
	// }
	const rear1 = this.#stepBackIndex(this.#rear);
	if (seq > this.#queueBody[rear1]) { // 5.
	  // 先頭(front)を残して全て削除する。
	  this.#rear = this.#stepIndex(this.#front);
	} else {
	  if (seq > rear1Seq) {
	    // 6. そのseqがrear-1のseq以下だがqueueの中に無いことはありえない
	    console.error("FATAL: rewindQueue: cleanUpQueue: OK but seq not found in queue");
	  }
	}
      }
    } else {
      // 7. 先頭を残してそれより前を全て消す。すなわち何もしない
      if (seq > this.#queueBody[this.#front]) {
	// 8.seqがrear-1のseqより大きければ先頭(front OK)を残して全て削除する
	// 4.により先頭だけがOKが確定していて、それ以降は全てチェックが飛ばされた状態なので、先頭を残して全て削除する。
	this.#rear = this.#stepIndex(this.#front);
	// console.debug("rewindQueue: cleanUpQueue: remain only front seq: " + this.#queueBody[this.#front] + ", rear-1 seq: " + this.#queueBody[this.#stepBackIndex(this.#rear)]);
      }
    }
  }
  getRewindElement () {
    return this.#queueBody.subarray(this.#front+1, this.#front+this.#elementSize);
  }
}

export { RewindQueue };
