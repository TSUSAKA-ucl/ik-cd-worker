'use client';
import { customLogger } from './customLogger.js';
globalThis.__customLogger = customLogger;
import AFRAME from 'aframe';

function kebabToCamelCase(str) {
  // remove 'set-' prefix if present
  const netStr = str.startsWith('set-') ? str.slice(4) : str;
  return netStr.replace(/-([a-z])/g, (match, p1) => p1.toUpperCase());
}
function kebabToSnakeCase(str) {
  return str.replace(/-([a-z])/g, (match, p1) => '_' + p1.toLowerCase());
}

function registerBooleanMessageComponent(componentName,
					 messageType = kebabToSnakeCase(componentName)) {
  AFRAME.registerComponent(componentName, {
    schema: {
      default: false,
    },
    init: function () {
      globalThis.__customLogger.log('called init function of IkWorkerParams:',componentName);
      this.setFunction = () => {
	globalThis.__customLogger.log('called set function of IkWorkerParams:',componentName);
	const propertyName = kebabToCamelCase(componentName);
	if (this.el.workerRef?.current) {
	  globalThis.__customLogger.log(`Posting message to worker: ${messageType} with value ${this.data}`);
	  this.el.workerRef.current.postMessage({
	    type: messageType,
	    [propertyName]: this.data,
	  });
	} else {
	  globalThis.__customLogger.warn(`Worker reference not found for ${messageType}`);
	}
      };
    },
    update: function () {
      globalThis.__customLogger.log('called update function of IkWorkerParams:',componentName);
      if (this.el.ikWorkerReady) {
	this.setFunction();
      }
      else {
	this.el.addEventListener('ik-worker-ready', this.setFunction,
				 { once: true });
      }
    }
  });
}

registerBooleanMessageComponent('set-exact-solution');
registerBooleanMessageComponent('set-ignore-joint-limits');
registerBooleanMessageComponent('set-ignore-collisions');
registerBooleanMessageComponent('set-joint-limit-keep-moving');

AFRAME.registerComponent('suppress-cd-worker', {
  sceneOnly: true,
  schema: {
    default: true,
  },
  update: function () {
    this.el.suppressCdWorker = this.data;
  }
});

// sceneEl.cdWorkerに付いているcd-worker関係のプロパティーを使って
// cd-workerがreadyになるのを(eventで)待って'**log_timing'をpostMessageする
AFRAME.registerComponent('cd-worker-log-timing', {
  schema: {
    timing: { type: 'boolean', default: true },
  },
  update: function () {
    const sceneEl = this.el.sceneEl;
    const postLogTimingMessage = () => {
      if (typeof sceneEl.cdWorker?.current?.postMessage === 'function') {
	globalThis.__customLogger
	  .log('Posting log_timing message to cd-worker with ',
	       `value ${this.data.timing}`);
	sceneEl.cdWorker.current.postMessage({type: '**log_timing',
						 timing: this.data.timing});
      } else {
	globalThis.__customLogger
	  .warn('cdWorker is not ready to receive messages');
	globalThis.__customLogger
	  .warn('cdWorker:', sceneEl.cdWorker);
      }
    };
    if (sceneEl.cdWorker?.ready) {
      postLogTimingMessage();
    } else {
      this.el.sceneEl.addEventListener('cd-worker-ready',
				       postLogTimingMessage,
				       { once: true });
    }
  }
});
