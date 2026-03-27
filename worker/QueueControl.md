# ik-workerとcd-workerを分離したときのik-worker側のrewind用queueの動作

## 前提。まず1対1版。複数の巻き戻しタイミングを考慮する必要がない

0. ik-workerが状態量を作成してseq番号とqueueを管理する
0. 開始時にqueueの先頭(front)は、必ずOKの状態量が一つ存在する
0. seq番号単調増加でik-workerがqueueに積む(あるいは満員で積めない)時に1増加する
1. ik-workerは、cdチェックするサイクル毎にseq番号を付けた状態量を可能ならqueueする
1. ik-workerは、queueが満員だったらqueueせずにseqだけ増やしてcd-workerにpostMessageする
2. cd-workerは、message channelの中のデータの最新のものをチェックしその結果を返す
2. cd-workerが、チェックする状態量のseqは飛び飛びになることはあるが、単調増加で過去に遡らない
2. cd-workerは、チェック結果(pair)をseq付きで(送ってきたik-workerに)返す
2. その結果、回答のseq番号は、必ずqueueのrearの番号以上である
3. ik-workerはdetect collisionのタイミングでcd-worker回答を調べる
4. 回答がOKで、そのデータがqueueにあれば、それを残しそのseqより古いデータをqueueから削除する
5. 回答がOKで、そのseqがrear-1のseqより大きければ、先頭(front)を残して全て削除する。
6. 回答がOKで、そのseqがrear-1のseq以下だがqueueの中に無いことはありえない
6. 回答がOKの場合は、上記どちらもrewindしない
7. 回答がNGだったら、先頭(OK)を残してその番号まで全てをqueueから削除する
8. 回答がNGで、そのseqがrear-1のseqより大きければ先頭(front OK)を残して全て削除する
9. 回答がNGであれば必ずqueueの先頭にrewindする。queueの先頭は最新のOKデータが入っているはず

queueには最低1個の状態量が存在している
回答のseq番号は、queueの中にあるかまたはrear-1のseq番号より大きい
queueの先頭は必ずOKの状態量である
チェック結果がNGで確定したデータより前の不明(回答なし)データは消されて存在しない
queueの先頭はOKが確定した状態量の中で最新である

