import { defineConfig } from "vite";
import { resolve } from "path";

// ik-workerとcd-workerに分離: それぞれを --mode ik, --mode cdで別々に作る

export default defineConfig(({ mode }) => {
  const isIK = mode === "ik";
  return {
    build: {
      target: "esnext",
      lib: {
        entry: isIK
          ? resolve(__dirname, "worker/ik-worker.js")
          : resolve(__dirname, "worker/cd-worker.js"),
        formats: ["es"],
        fileName: () => (isIK ? "ik-worker.js" : "cd-worker.js"),
      },
      outDir: "public",
      emptyOutDir: false, // 既存の public を消さない(wasmを消さないため)
      rollupOptions: {
	external: [
          "/wasm/slrm_module.js",
	  "/wasm/cd_module.js"
	]
      },
    },
    publicDir: false,
  };
});
