# MuJoCo Physics-only WASM API and USD Physics Bridge

This document describes how to use the physics-only WASM build for browser simulation,
and how to connect physics-annotated USD content to the exposed MuJoCo WASM API.

## Scope and Current Status

- The physics-only WASM target is documented in [`build-wasm.md`](../build-wasm.md).
  It produces `mujoco_physics.js` / `mujoco_physics.wasm` and is designed for a minimal
  runtime surface.
- The USD physics schema coverage is documented in [`usd-physics.md`](usd-physics.md).
- Current web status: TinyUSDZ path is geometry-focused today. `mjcPhysics` attributes
  are not yet read end-to-end in the existing web loader pipeline, and joints/actuators/
  scene options are not auto-mapped from USD to MuJoCo in that path.

Practical implication: for physics-annotated USD, use a bridge pipeline:
1. Extract relevant physics annotations from USD in JavaScript.
2. Build a MuJoCo model procedurally with `MjSpec` / `Mjs*` APIs.
3. Compile and simulate using `PhysicsModel` / `PhysicsData`.

## Physics-only WASM API Surface

Load module:

```ts
import loadMujocoPhysics from "./dist/mujoco_physics.js";
const mj = await loadMujocoPhysics();
```

Supported core entry points:

- `loadModelFromArrayBuffer(buffer)`
- `new mj.PhysicsData(model)`
- `mj.mj_forward(model, data)`
- `mj.mj_step(model, data)`
- `mj.mj_stepN(model, data, nstep)`
- `mj.mj_resetData(model, data)`

Procedural model construction helpers:

- `new mj.MjSpec()`
- `mj.MjsBody.add/setPos/setQuat/setMass`
- `mj.MjsGeom.add/setType/setSize/setPos/setQuat/setMass/setRGBA/setFriction/setConType/setConAffinity/setCondim`
- `mj.MjsJoint.add/addFree/setType/setAxis/setRange/setDamping`
- Geom constants: `GEOM_PLANE`, `GEOM_BOX`, `GEOM_SPHERE`, ...
- Joint constants: `JNT_FREE`, `JNT_BALL`, `JNT_SLIDE`, `JNT_HINGE`

Memory management:

- Embind handles are not garbage-collected automatically.
- Always call `.delete()` on objects you create or receive (`PhysicsData`,
  `PhysicsModel`, `MjSpec`, and temporary handles where relevant).

## USD Physics Mapping Strategy

Use the following mapping as a baseline when converting extracted USD physics annotations
into MuJoCo spec objects.

| USD annotation | MuJoCo physics-only WASM mapping |
|---|---|
| `mjc:option:timestep` | `spec.setTimestep(dt)` |
| gravity from scene metadata | `spec.setGravity(gx, gy, gz)` |
| `UsdPhysicsJoint` + `MjcJointAPI` type/axis/range/damping | `MjsJoint.add` + `setType` + `setAxis` + `setRange` + `setDamping` |
| `MjcCollisionAPI` `condim` | `MjsGeom.setCondim` |
| `MjcCollisionAPI` `contype`/`conaffinity` (if provided by bridge rules) | `MjsGeom.setConType` / `setConAffinity` |
| `UsdPhysicsMaterialAPI` + `MjcMaterialAPI` friction terms | `MjsGeom.setFriction(slide, roll, spin)` |

Not covered by this minimal bridge example:

- Actuators (`MjcActuator`)
- Tendons (`MjcTendon`)
- Equality constraints (`MjcEquality*`)

Those can be added later once the bridge defines schema extraction and mapping rules for
those prims.

## End-to-end Example (TypeScript)

The example assumes you already extracted USD physics annotations into a JSON-like object.
This keeps the runtime path explicit and avoids claiming direct TinyUSDZ `mjcPhysics`
auto-ingestion in current web builds.

```ts
import loadMujocoPhysics from "./dist/mujoco_physics.js";

type SceneGeom = {
  name: string;
  type: "box" | "sphere";
  size: [number, number, number];
  pos: [number, number, number];
  mass?: number;
  condim?: number;
  contype?: number;
  conaffinity?: number;
  friction?: [number, number, number];
};

type SceneJoint = {
  name: string;
  body: string;
  type: "hinge" | "slide" | "ball" | "free";
  axis?: [number, number, number];
  range?: [number, number];
  damping?: number;
};

type SceneBody = {
  name: string;
  pos: [number, number, number];
  geoms: SceneGeom[];
};

type PhysicsScene = {
  timestep: number;
  gravity: [number, number, number];
  bodies: SceneBody[];
  joints: SceneJoint[];
};

function toGeomType(mj: any, type: SceneGeom["type"]): number {
  switch (type) {
    case "box":
      return mj.GEOM_BOX;
    case "sphere":
      return mj.GEOM_SPHERE;
    default:
      throw new Error(`Unsupported geom type: ${type}`);
  }
}

function toJointType(mj: any, type: SceneJoint["type"]): number {
  switch (type) {
    case "hinge":
      return mj.JNT_HINGE;
    case "slide":
      return mj.JNT_SLIDE;
    case "ball":
      return mj.JNT_BALL;
    case "free":
      return mj.JNT_FREE;
    default:
      throw new Error(`Unsupported joint type: ${type}`);
  }
}

export async function buildAndSimulate(physicsScene: PhysicsScene) {
  const mj = await loadMujocoPhysics();

  const spec = new mj.MjSpec();
  spec.setModelName("usd_physics_bridge");
  spec.setTimestep(physicsScene.timestep);
  spec.setGravity(
    physicsScene.gravity[0],
    physicsScene.gravity[1],
    physicsScene.gravity[2],
  );

  const world = spec.worldBody();
  if (!world) {
    throw new Error("world body is null");
  }

  // Optional ground plane.
  const ground = mj.MjsGeom.add(world, "ground");
  if (!ground) {
    throw new Error("failed to add ground geom");
  }
  mj.MjsGeom.setType(ground, mj.GEOM_PLANE);
  mj.MjsGeom.setSize(ground, 5, 5, 0.1);

  const bodyMap = new Map<string, any>();

  for (const b of physicsScene.bodies) {
    const body = mj.MjsBody.add(world, b.name);
    if (!body) {
      throw new Error(`failed to add body: ${b.name}`);
    }
    bodyMap.set(b.name, body);
    mj.MjsBody.setPos(body, b.pos[0], b.pos[1], b.pos[2]);

    for (const g of b.geoms) {
      const geom = mj.MjsGeom.add(body, g.name);
      if (!geom) {
        throw new Error(`failed to add geom: ${g.name}`);
      }
      mj.MjsGeom.setType(geom, toGeomType(mj, g.type));
      mj.MjsGeom.setSize(geom, g.size[0], g.size[1], g.size[2]);
      mj.MjsGeom.setPos(geom, g.pos[0], g.pos[1], g.pos[2]);

      if (g.mass !== undefined) {
        mj.MjsGeom.setMass(geom, g.mass);
      }
      if (g.condim !== undefined) {
        mj.MjsGeom.setCondim(geom, g.condim);
      }
      if (g.contype !== undefined) {
        mj.MjsGeom.setConType(geom, g.contype);
      }
      if (g.conaffinity !== undefined) {
        mj.MjsGeom.setConAffinity(geom, g.conaffinity);
      }
      if (g.friction) {
        mj.MjsGeom.setFriction(geom, g.friction[0], g.friction[1], g.friction[2]);
      }
    }
  }

  for (const j of physicsScene.joints) {
    const body = bodyMap.get(j.body);
    if (!body) {
      throw new Error(`joint references unknown body: ${j.body}`);
    }

    if (j.type === "free") {
      const freeJ = mj.MjsJoint.addFree(body);
      if (!freeJ) {
        throw new Error(`failed to add free joint: ${j.name}`);
      }
      continue;
    }

    const joint = mj.MjsJoint.add(body, j.name);
    if (!joint) {
      throw new Error(`failed to add joint: ${j.name}`);
    }

    mj.MjsJoint.setType(joint, toJointType(mj, j.type));
    if (j.axis) {
      mj.MjsJoint.setAxis(joint, j.axis[0], j.axis[1], j.axis[2]);
    }
    if (j.range) {
      mj.MjsJoint.setRange(joint, j.range[0], j.range[1]);
    }
    if (j.damping !== undefined) {
      mj.MjsJoint.setDamping(joint, j.damping);
    }
  }

  const model = spec.compile();
  if (!model) {
    spec.delete();
    throw new Error("mj_compile failed");
  }

  const data = new mj.PhysicsData(model);

  // Run simulation.
  mj.mj_forward(model, data);
  mj.mj_stepN(model, data, 240);

  // Typed-array views into WASM memory.
  const qpos = data.qpos();
  const geomXpos = data.geom_xpos();

  // Consume immediately or copy if you need persistence.
  console.log("time", data.time());
  console.log("qpos[0]", qpos.length ? qpos[0] : undefined);
  console.log("first geom world xyz", geomXpos.slice(0, 3));

  // Cleanup in reverse ownership order.
  data.delete();
  model.delete();
  spec.delete();
}
```

## Limitations and Next Steps

- The bridge depends on a separate USD annotation extraction stage.
- The current TinyUSDZ web path remains geometry-first and does not yet provide full
  automatic `mjcPhysics` import.
- If you need complete physics schema ingestion, extend the extraction layer first, then
  incrementally map each schema family (`Scene`, `Joint`, `Collision`, `Material`, then
  `Actuator`/`Tendon`/`Equality`) to `MjSpec` APIs.
