# WASM build notes

## Direction

For a minimal physics-only WASM module, the correct runtime input is a precompiled MuJoCo binary model (`mjb`) injected from JS as an `ArrayBuffer`. This removes the browser filesystem dependency and avoids XML parsing at runtime.

## What changed

- Added a new Emscripten target: `mujoco_physics_wasm`
- Added a smaller native core library for it: `mujoco_physics_core`
- New entry module name: `loadMujocoPhysics`
- Output artifact name: `mujoco_physics.js` / `mujoco_physics.wasm`
- Filesystem is disabled for this target with `-s FILESYSTEM=0`
- The binding surface is intentionally small:
  - `loadModelFromArrayBuffer`
  - `PhysicsModel`
  - `PhysicsData`
  - `mj_step`, `mj_forward`, `mj_resetData`
  - direct typed-array views for `qpos`, `qvel`, `act`, `ctrl`, `sensordata`

## Build

```bash
emcmake cmake -B build
cmake --build build --target mujoco_physics_wasm
```

## JS usage

```ts
import loadMujocoPhysics from "./dist/mujoco_physics.js";

const mod = await loadMujocoPhysics();
const model = mod.loadModelFromArrayBuffer(mjbArrayBuffer);
const data = new mod.PhysicsData(model);
mod.mj_step(model, data);
```

## Size optimizations

The physics-only target applies aggressive size reduction:

- **Source exclusion**: only `src/engine/` and `src/thread/` are compiled into `mujoco_physics_core`. Visualization (`engine_vis_*`), printing (`engine_print.*`), finite-difference derivatives (`engine_derivative_fd.*`), and inverse dynamics (`engine_inverse.*`) are excluded. `engine_name.*` stays because `engine_io.c` references `mj_id2name`.
- **Compile-time guards**: `MUJOCO_DISABLE_FWDINV_COMPARE`, `MUJOCO_DISABLE_PLUGINS`, `MUJOCO_DISABLE_THREADING` stub out unused subsystems.
- **No C++ exceptions**: `-fno-exceptions` + `-s DISABLE_EXCEPTION_CATCHING=1`
- **No threading**: threading stubs replace the full thread pool implementation
- **No filesystem**: `-s FILESYSTEM=0`
- **No plugins**: plugin API is stubbed to return nullptr / error on register
- **LTO**: `-flto` across the physics core, dependencies (ccd, qhull), and the final link
- **Closure compiler**: `--closure=1` minifies the JS glue
- **emmalloc**: `-s MALLOC=emmalloc` uses a smaller allocator (~1 KB vs ~10 KB dlmalloc)
- **No longjmp**: `-s SUPPORT_LONGJMP=0` (engine uses `exit()`, not longjmp)
- **No assertions**: `-s ASSERTIONS=0`
- **No debug info**: source maps and `-g` are omitted
