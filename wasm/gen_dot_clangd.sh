#!/bin/sh
cat <<EOF
CompileFlags:
  Add:
    - "--sysroot=$EMSDK/upstream/emscripten/cache/sysroot"
    - "-isystem$EMSDK/upstream/emscripten/cache/sysroot/include/c++/v1"
    - "-isystem$EMSDK/upstream/emscripten/cache/sysroot/include"
  Remove:
    - "-s*"
    - "--bind"
    - "-msimd128"
EOF
# Since `clangd` assumes an x86_64 target, it does not accept
# the `-msimd128` option; therefore, you should specify it
# under `Remove:` in your `.clangd` file.
#
# do `./gen_dot_clangd.sh > .clangd`
