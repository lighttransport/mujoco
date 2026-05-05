# @lighttransport/mujoco-wasm

Physics-only MuJoCo WebAssembly bindings for JavaScript and TypeScript.

This package contains the minimal `mujoco_physics` Emscripten build:

- `mujoco_physics.js`: ES module loader
- `mujoco_physics.wasm`: WebAssembly binary
- `mujoco_physics.d.ts`: TypeScript declarations

## Install

```sh
npm install @lighttransport/mujoco-wasm
```

## Usage

```ts
import loadMujocoPhysics from "@lighttransport/mujoco-wasm";

const mj = await loadMujocoPhysics();
const model = mj.loadModelFromArrayBuffer(mjbArrayBuffer);
if (!model) {
  throw new Error("failed to load model");
}

const data = new mj.PhysicsData(model);
mj.mj_step(model, data);

data.delete();
model.delete();
```

The loader resolves `mujoco_physics.wasm` relative to the JavaScript module by
default. If your bundler serves WebAssembly assets from another location, pass a
custom `locateFile` option:

```ts
const mj = await loadMujocoPhysics({
  locateFile: (path) => `/assets/${path}`,
});
```

The physics-only build loads precompiled MuJoCo binary models (`.mjb`) via
`loadModelFromArrayBuffer`. It does not include the browser filesystem path used
by the full MuJoCo WASM package.
