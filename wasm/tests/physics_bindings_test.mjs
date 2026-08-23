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
  mj.MjsBody.addSite(body, 'tip', 0.5, 0, 0);

  const tendon = spec.addSpatialTendon('distance', 0.1, 1.0, true);
  mj.MjsTendon.wrapSite(tendon, 'anchor');
  mj.MjsTendon.wrapSite(tendon, 'tip');

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
} finally {
  data?.delete();
  model?.delete();
  spec.delete();
}

console.log('Physics-only COM Jacobian and spatial tendon bindings: PASS');
