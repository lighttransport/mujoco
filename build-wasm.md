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

## Prerequisites

- [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html) (tested with 4.0.x)
- CMake 3.16+
- A working C/C++ toolchain (for CMake host tools)

Ensure `emcmake` / `emcc` are on your `PATH`:

```bash
source /path/to/emsdk/emsdk_env.sh
```

## Build

### Physics-only module (minimal)

```bash
emcmake cmake -B build-wasm \
  -DMUJOCO_BUILD_TESTS_WASM=OFF \
  -DMUJOCO_WASM_THREADS=OFF
cmake --build build-wasm --target mujoco_physics_wasm -j$(nproc)
```

Output artifacts in `wasm/dist/`:

| File | Description |
|---|---|
| `mujoco_physics.wasm` | WASM binary (~483 KB, ~174 KB gzipped) |
| `mujoco_physics.js` | ES module loader (~21 KB, ~9 KB gzipped) |
| `mujoco_physics.d.ts` | TypeScript declarations |

### Full WASM module (all bindings, filesystem, exceptions)

```bash
emcmake cmake -B build-wasm \
  -DMUJOCO_WASM_THREADS=ON
cmake --build build-wasm --target mujoco_wasm -j$(nproc)
```

### CMake options reference

| Option | Default | Description |
|---|---|---|
| `MUJOCO_BUILD_PHYSICS_WASM` | `ON` | Build the minimal physics-only WASM module |
| `MUJOCO_BUILD_TESTS_WASM` | `ON` | Build tests for WASM bindings |
| `MUJOCO_WASM_THREADS` | `OFF` | Enable multithreading for the full WASM module |
| `MUJOCO_PHYSICS_ENABLE_THREADS` | `OFF` | Enable threading in the physics-only core |
| `MUJOCO_PHYSICS_ENABLE_EXCEPTIONS` | `OFF` | Enable C++ exceptions in the physics-only core |
| `MUJOCO_PHYSICS_ENABLE_PLUGINS` | `OFF` | Enable plugin support in the physics-only core |

### Preparing an mjb model file

The physics-only module loads precompiled binary models (`.mjb`), not XML.
Use the native MuJoCo CLI or Python bindings to convert:

```bash
# Using the native mujoco binary
mujoco compile model.xml model.mjb
```

```python
# Using Python
import mujoco
model = mujoco.MjModel.from_xml_path("model.xml")
mujoco.mj_saveModel(model, "model.mjb", None, 0)
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
