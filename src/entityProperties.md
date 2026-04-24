# robot(this.el)には、どのようなプロパティーがあり、いつ付くか

## robot.id
DOMがマウントされたら

## robot.axes, robot.realAxes, robot.endLink
`robot-loader`がモデルをload完了してDOMを作り上げたら`emit('robot-registered'`

## robot.workerRef, robot.workerRef.current
`robot-dom-ready`イベントハンドラで作成

## robot.abId
ik-workerから`'generator_ready'`を受信したら(`shapes`があれば)値が付く
その後`'ik-worker-ready`をemit

## robot.isWorkerReady
これがtrueで`robot.abId`が未定義ならば`shapes.json`がうまくなかった

# robotはどのようなイベントをemitし、それはどのフラグが変わったときか

## `robot-dom-ready`
robot-loader(`urdfLoader2`)のawaitがresolveされたら、`this.el.model`をnon nullにしてemit


* 'attach': 子がattach(reparenting)完了したイベント
* `attached`: 親に対して子が付いたことをemitしている。複数子がattachするためrobot.attachedは配列
'ik-worker-arrival'
`ik-worker-ready`
'ik-worker-start'
* `robot-dom-ready`
* `robot-registered`

`loaded`
`model-error`
`model-loaded`
`thumbmenu-select`
`thumbstickdown`
`thumbstickmoved`
`triggerup`
`gripup`
