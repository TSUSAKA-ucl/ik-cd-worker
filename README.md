# Web Worker for Inverse Kinematics(with Singularity Low Sensitive Motion Resolve) and Collision Detection 

0. Download [emscripten](https://github.com/emscripten-core/emsdk.git) 
   and `source emsdk_env.sh`
1. Show [`README-submodules.md`](README-submodules.md) and
   init and update submodules.
2. building of `slrm_module`
   ```
   cd wasm/slrm_wasm/build/
   emcmake cmake -DCMAKE_BUILD_TYPE=Release ..
   make
   ```
3. building of `cd_module`
   ```
   cd ../../cd_wasm/build/
   emcmake cmake -DCMAKE_BUILD_TYPE=Release ..
   make
   ```
