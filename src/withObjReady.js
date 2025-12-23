// small utility for A-Frame events
// to call a function when an object is ready

function withObjReady(el, eventName, flag, fn, arg=flag) {
  if (flag) {
    fn(arg);
  } else {
    const onLoaded = (e) => {
      // el.removeEventListener('model-loaded', onLoaded);
      fn(arg);
    };
    el.addEventListener(eventName, onLoaded, {once: true});
  }
}

function traverseModelReady(el, fn) {
  withObjReady(el,
	       'model-loaded',
	       el.getObject3D('mesh'),
	       (o) => { o.traverse(fn); });
}

export { withObjReady, traverseModelReady };
