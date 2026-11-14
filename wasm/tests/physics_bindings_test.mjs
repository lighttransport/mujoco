// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import assert from 'node:assert/strict';
import path from 'node:path';
import process from 'node:process';
import {fileURLToPath, pathToFileURL} from 'node:url';

const testDir = path.dirname(fileURLToPath(import.meta.url));
const distDir = path.resolve(
    process.env.MUJOCO_PHYSICS_DIST || path.join(testDir, '..', 'dist'));
const modulePath = path.join(distDir, 'mujoco_physics.js');
const wasmPath = path.join(distDir, 'mujoco_physics.wasm');
const loadMuJoCo = (await import(pathToFileURL(modulePath).href)).default;
const mj = await loadMuJoCo({
  locateFile: (filename) => filename.endsWith('.wasm') ? wasmPath :
    path.join(distDir, filename),
});

const spec = new mj.MjSpec();
let model;
let data;

try {
  const world = spec.worldBody();
  mj.MjsBody.addSite(world, 'anchor', 0, 0, 0);

  const body = mj.MjsBody.add(world, 'link');
  mj.MjsBody.setMass(body, 1);
  mj.MjsBody.setIPos(body, 0.2, 0, 0);
  mj.MjsBody.setDiagInertia(body, 0.02, 0.02, 0.02);

  const joint = mj.MjsJoint.add(body, 'hinge');
  mj.MjsJoint.setType(joint, mj.JNT_HINGE);
  mj.MjsJoint.setAxis(joint, 0, 1, 0);
  mj.MjsJoint.setSpringYield(joint, 1e-3);
  mj.MjsBody.addSite(body, 'tip', 0.5, 0, 0);

  const tendon = spec.addSpatialTendon('distance', 0.1, 1.0, true);
  mj.MjsTendon.wrapSite(tendon, 'anchor');
  mj.MjsTendon.wrapSite(tendon, 'tip');
  mj.MjsTendon.setSpringYield(tendon, 2e-3);

  model = spec.compile();
  data = new mj.PhysicsData(model);
  mj.mj_forward(model, data);

  assert.equal(model.ntendon(), 1);
  assert.equal(model.nv(), 1);

  const originJacobian = Array.from(data.jacBody(1));
  const comJacobian = Array.from(data.jacBodyCom(1));
  assert.equal(comJacobian.length, 6 * model.nv());
  assert.ok(comJacobian.every(Number.isFinite));
  assert.notDeepEqual(comJacobian, originJacobian);
  assert.ok(Math.abs(comJacobian[2] + 0.2) < 1e-12);

  // Plasticity diagnostics (MuJoCo v4+)
  // 1 yielding hinge (nq=1) + 1 yielding tendon (2 plastic dofs each).
  assert.equal(model.nplastic(), 3);
  const springYields = Array.from(model.jnt_springyield());
  assert.equal(springYields.length, 1);
  // hinge yield is authored in degrees by default and compiled to radians
  assert.ok(Math.abs(springYields[0] - (1e-3 * Math.PI) / 180) < 1e-12);
  const tendonYield = Array.from(model.tendon_springyield());
  assert.equal(tendonYield.length, 1);
  assert.ok(Math.abs(tendonYield[0] - 2e-3) < 1e-12);
  assert.deepEqual(Array.from(model.tendon_type()), [mj.TENDON_SPATIAL]);
  assert.equal(model.flex_yield().length, 0);
  assert.equal(data.plastic().length, 3);
  assert.ok(Array.from(data.plastic()).every(Number.isFinite));
  // the yielding hinge's slot starts from qpos_spring (default 0); tendon
  // slots initialize from tendon_lengthspring (finite, nonzero for this model)
  assert.equal(data.plastic()[0], 0);

  // --- Orientation readback (MuJoCo v4+) ---
  assert.equal(data.xiquat().length, 4 * model.nbody());
  assert.equal(data.geom_xquat().length, 4 * model.ngeom());
  assert.equal(data.site_xquat().length, 8);  // anchor + tip
  assert.ok(Array.from(data.xiquat()).every(Number.isFinite));
  assert.ok(Array.from(data.site_xquat()).every(Number.isFinite));

  // --- Surface-velocity contacts summary (MuJoCo v4+) ---
  assert.equal(data.contacts().length, 10 * data.ncon());
  assert.equal(data.contactsSurfaceVel().length, 3 * data.ncon());

  // --- Enablement flags (MuJoCo v4+) ---
  assert.equal(mj.mjENBL_SPECULATIVE, 1 << 7);

  // mj_resetData re-initializes the plastic state vector from model data.
  data.plastic()[0] = 123.5;
  mj.mj_resetData(model, data);
  assert.equal(data.plastic()[0], 0);
} finally {
  data?.delete();
  model?.delete();
  spec.delete();
}

console.log('Physics-only v4 bindings (plasticity, xiquat, surfacevel): PASS');
