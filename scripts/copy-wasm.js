import fs from 'fs';
import path from 'path';
const srcDir = path.resolve('node_modules/@ucl-nuee/ik-cd-worker/public');
const dstDir = path.resolve('public');
const wasmDst = path.join(dstDir, 'wasm');
fs.mkdirSync(wasmDst, { recursive: true });
(wk => fs.copyFileSync(path.join(srcDir, wk),
		       path.join(dstDir, wk)))('ik_cd_worker.js');
['cd_module.js', 'slrm_module.js', 'cd_module.wasm', 'slrm_module.wasm'].
  map(fn => {
    ((s,d) => {fs.copyFileSync(s,d);
	       // console.log(`Copied ${s} -> ${d}`);
	      })(path.join(srcDir, 'wasm', fn),
      path.join(wasmDst, fn))
  });
