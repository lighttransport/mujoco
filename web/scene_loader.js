// Builds a MuJoCo model from a JSON scene description by translating each
// entry into the corresponding mjSpec API call.
//
// Returns { model, data, geomInfos } where geomInfos is an array (one per
// MuJoCo geom, index 0 = first geom) of { type, size, rgba, isStatic }
// that the renderer can use to create matching Three.js meshes.

const GEOM_TYPES = {
  plane:     "GEOM_PLANE",
  hfield:    "GEOM_HFIELD",
  sphere:    "GEOM_SPHERE",
  capsule:   "GEOM_CAPSULE",
  ellipsoid: "GEOM_ELLIPSOID",
  cylinder:  "GEOM_CYLINDER",
  box:       "GEOM_BOX",
};

const JOINT_TYPES = {
  free:  "JNT_FREE",
  ball:  "JNT_BALL",
  slide: "JNT_SLIDE",
  hinge: "JNT_HINGE",
};

function pad3(arr) {
  const a = arr || [0, 0, 0];
  return [a[0] || 0, a[1] || 0, a[2] || 0];
}

function addGeom(mj, parent, g) {
  const geom = mj.MjsGeom.add(parent, g.name || "");
  const typeConst = GEOM_TYPES[g.type];
  if (typeConst === undefined) {
    throw new Error(`Unknown geom type: "${g.type}"`);
  }
  mj.MjsGeom.setType(geom, mj[typeConst]);

  const s = pad3(g.size);
  mj.MjsGeom.setSize(geom, s[0], s[1], s[2]);

  if (g.pos) {
    const p = pad3(g.pos);
    mj.MjsGeom.setPos(geom, p[0], p[1], p[2]);
  }
  if (g.quat) {
    mj.MjsGeom.setQuat(geom, g.quat[0], g.quat[1], g.quat[2], g.quat[3]);
  }
  if (g.rgba) {
    mj.MjsGeom.setRGBA(geom, g.rgba[0], g.rgba[1], g.rgba[2], g.rgba[3] ?? 1);
  }
  if (g.mass != null) {
    mj.MjsGeom.setMass(geom, g.mass);
  }
  if (g.friction) {
    const f = pad3(g.friction);
    mj.MjsGeom.setFriction(geom, f[0], f[1], f[2]);
  }
  if (g.contype != null) mj.MjsGeom.setConType(geom, g.contype);
  if (g.conaffinity != null) mj.MjsGeom.setConAffinity(geom, g.conaffinity);
  if (g.condim != null) mj.MjsGeom.setCondim(geom, g.condim);

  return geom;
}

function addJoint(mj, parent, j) {
  if (typeof j === "string") {
    // Shorthand: "free", "ball", etc.
    if (j === "free") {
      mj.MjsJoint.addFree(parent);
      return;
    }
    j = { type: j };
  }

  const jnt = mj.MjsJoint.add(parent, j.name || "");
  const typeConst = JOINT_TYPES[j.type];
  if (typeConst === undefined) {
    throw new Error(`Unknown joint type: "${j.type}"`);
  }
  mj.MjsJoint.setType(jnt, mj[typeConst]);

  if (j.axis) {
    const a = pad3(j.axis);
    mj.MjsJoint.setAxis(jnt, a[0], a[1], a[2]);
  }
  if (j.range) {
    mj.MjsJoint.setRange(jnt, j.range[0], j.range[1]);
  }
  if (j.damping != null) {
    mj.MjsJoint.setDamping(jnt, j.damping);
  }
}

function addBody(mj, parentBody, b) {
  const body = mj.MjsBody.add(parentBody, b.name || "");

  if (b.pos) {
    const p = pad3(b.pos);
    mj.MjsBody.setPos(body, p[0], p[1], p[2]);
  }
  if (b.quat) {
    mj.MjsBody.setQuat(body, b.quat[0], b.quat[1], b.quat[2], b.quat[3]);
  }
  if (b.mass != null) {
    mj.MjsBody.setMass(body, b.mass);
  }

  // Joint (string shorthand or object or array)
  if (b.joint) {
    if (Array.isArray(b.joint)) {
      for (const j of b.joint) addJoint(mj, body, j);
    } else {
      addJoint(mj, body, b.joint);
    }
  }
  if (b.joints) {
    for (const j of b.joints) addJoint(mj, body, j);
  }

  // Geoms
  if (b.geoms) {
    for (const g of b.geoms) addGeom(mj, body, g);
  }
  if (b.geom) {
    addGeom(mj, body, b.geom);
  }

  // Nested child bodies
  if (b.bodies) {
    for (const child of b.bodies) addBody(mj, body, child);
  }

  return body;
}

export function buildModelFromJSON(mj, config) {
  const spec = new mj.MjSpec();

  if (config.name) spec.setModelName(config.name);

  const dt = config.timestep ?? 0.002;
  spec.setTimestep(dt);

  const g = config.gravity ?? [0, 0, -9.81];
  spec.setGravity(g[0], g[1], g[2]);

  const world = spec.worldBody();

  // Lights
  if (config.lights) {
    for (const l of config.lights) {
      const light = mj.MjsLight.add(world, l.name || "");
      if (l.pos) mj.MjsLight.setPos(light, l.pos[0], l.pos[1], l.pos[2]);
      if (l.dir) mj.MjsLight.setDir(light, l.dir[0], l.dir[1], l.dir[2]);
      if (l.diffuse) mj.MjsLight.setDiffuse(light, l.diffuse[0], l.diffuse[1], l.diffuse[2]);
    }
  }

  // Collect geom info for the renderer (in MuJoCo geom order).
  const geomInfos = [];

  // Bodies (top-level bodies are children of worldbody)
  if (config.bodies) {
    for (const b of config.bodies) {
      addBody(mj, world, b);
    }
  }

  // Compile
  const model = spec.compile();
  spec.delete();
  const data = new mj.PhysicsData(model);

  // Build geomInfos from the JSON config (flattened, DFS order matches MuJoCo).
  function collectGeoms(bodyDef) {
    const geoms = bodyDef.geoms || (bodyDef.geom ? [bodyDef.geom] : []);
    const hasJoint = !!(bodyDef.joint || bodyDef.joints);
    for (const g of geoms) {
      geomInfos.push({
        type: g.type,
        size: g.size || [0, 0, 0],
        rgba: g.rgba || [0.5, 0.5, 0.5, 1],
        isStatic: !hasJoint,
      });
    }
    if (bodyDef.bodies) {
      for (const child of bodyDef.bodies) collectGeoms(child);
    }
  }
  if (config.bodies) {
    for (const b of config.bodies) collectGeoms(b);
  }

  return { model, data, geomInfos };
}
