// Loads a USD scene via tinyusdz WASM and builds:
//   - MuJoCo physics model with bounding-box proxy geoms
//   - Three.js visual meshes from actual USD geometry
//
// Returns { model, data, geomInfos, visualMeshes, geomToMeshMap }

import * as THREE from "three";
import { TinyUSDZLoader } from "tinyusdz/TinyUSDZLoader.js";
import { TinyUSDZLoaderUtils } from "tinyusdz/TinyUSDZLoaderUtils.js";

// Singleton loader — avoids re-initializing the tinyusdz WASM module.
let loader = null;
async function getLoader() {
  if (!loader) {
    loader = new TinyUSDZLoader();
    await loader.init({ useZstdCompressedWasm: false });
  }
  return loader;
}

// ---------------------------------------------------------------------------
// Scene-graph traversal: collect mesh nodes with accumulated world transforms
// ---------------------------------------------------------------------------

function flattenMeshNodes(node, parentWorld) {
  const local = node.localMatrix
    ? TinyUSDZLoaderUtils.toMatrix4(node.localMatrix)
    : new THREE.Matrix4();
  const world = new THREE.Matrix4().multiplyMatrices(parentWorld, local);

  const results = [];
  if (node.nodeType === "mesh" && node.contentId !== undefined) {
    results.push({
      name: node.primName || node.absPath || "",
      meshId: node.contentId,
      worldMatrix: world.clone(),
    });
  }
  if (node.children) {
    for (const child of node.children) {
      results.push(...flattenMeshNodes(child, world));
    }
  }
  return results;
}

// ---------------------------------------------------------------------------
// AABB from mesh vertex data (mesh-local space)
// ---------------------------------------------------------------------------

function computeLocalAABB(points) {
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < points.length; i += 3) {
    const x = points[i], y = points[i + 1], z = points[i + 2];
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
  }
  return {
    center: [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2],
    halfExtents: [
      Math.max((maxX - minX) / 2, 0.001),
      Math.max((maxY - minY) / 2, 0.001),
      Math.max((maxZ - minZ) / 2, 0.001),
    ],
  };
}

// ---------------------------------------------------------------------------
// Y-up ↔ Z-up helpers
// ---------------------------------------------------------------------------

function yUpToZUp(x, y, z) {
  // USD/Three.js Y-up  →  MuJoCo Z-up
  return [x, -z, y];
}

function yUpToZUpHalfExtents(hx, hy, hz) {
  return [hx, hz, hy]; // swap Y/Z (half-extents are positive, no sign flip)
}

// ---------------------------------------------------------------------------
// Main entry point
// ---------------------------------------------------------------------------

/**
 * Load a USD file and create a MuJoCo physics model + Three.js visual meshes.
 *
 * @param {Object} mj          - MuJoCo WASM module
 * @param {string} usdUrl      - URL to .usd / .usda / .usdc / .usdz file
 * @param {Object} [options]
 * @param {number} [options.timestep=0.002]
 * @param {number} [options.density=500]          kg/m³ for mass estimation
 * @param {string[]} [options.staticNames=[]]     mesh names treated as static
 * @param {Function} [options.classify]           (name, aabb) => 'static'|'dynamic'
 * @returns {Promise<{model, data, geomInfos, visualMeshes, geomToMeshMap}>}
 */
export async function loadUSDScene(mj, usdUrl, options = {}) {
  const timestep = options.timestep ?? 0.002;
  const density = options.density ?? 500;
  const staticNames = new Set(options.staticNames || []);
  const classify = options.classify || null;

  // --- Load USD ---
  const tinyLoader = await getLoader();
  const usdScene = await tinyLoader.loadAsync(usdUrl);
  const usdRoot = usdScene.getDefaultRootNode();

  // Detect up-axis (default Y)
  const upAxis =
    typeof usdScene.getUpAxis === "function" ? usdScene.getUpAxis() : "Y";
  const needsAxisConvert = upAxis === "Y";

  // --- Flatten mesh hierarchy ---
  const meshNodes = flattenMeshNodes(usdRoot, new THREE.Matrix4());
  if (meshNodes.length === 0) {
    throw new Error("USD scene contains no meshes");
  }

  // --- Pre-extract mesh data (copy from WASM heap immediately) ---
  const meshDataList = meshNodes.map((mn) => {
    const mesh = usdScene.getMesh(mn.meshId);
    const points = new Float32Array(mesh.points);
    const aabb = computeLocalAABB(points);
    return { mn, mesh, points, aabb };
  });

  // --- Build Three.js visual meshes ---
  const visualMeshes = meshDataList.map(({ mesh }) => {
    const geometry = TinyUSDZLoaderUtils.convertUsdMeshToThreeMesh(mesh);
    const material = new THREE.MeshStandardMaterial({
      color: 0x8899bb,
      roughness: 0.4,
      metalness: 0.1,
    });
    const visMesh = new THREE.Mesh(geometry, material);
    visMesh.castShadow = true;
    visMesh.receiveShadow = true;
    return visMesh;
  });

  // Offset each geometry so its AABB center is at the local origin.
  // This lets MuJoCo's body position (at AABB center) drive the mesh directly.
  for (let i = 0; i < meshDataList.length; i++) {
    const { aabb } = meshDataList[i];
    visualMeshes[i].geometry.translate(
      -aabb.center[0],
      -aabb.center[1],
      -aabb.center[2],
    );
  }

  // --- Build MuJoCo physics model ---
  const spec = new mj.MjSpec();
  spec.setModelName("usd_scene");
  spec.setTimestep(timestep);
  spec.setGravity(0, 0, -9.81);

  const world = spec.worldBody();

  // Ground plane (always present)
  const groundGeom = mj.MjsGeom.add(world, "ground");
  mj.MjsGeom.setType(groundGeom, mj.GEOM_PLANE);
  mj.MjsGeom.setSize(groundGeom, 5, 5, 0.1);
  mj.MjsGeom.setRGBA(groundGeom, 0.3, 0.3, 0.4, 1);

  const geomInfos = [];
  const geomToMeshMap = [];

  // Ground geom (index 0)
  geomInfos.push({ type: "plane", size: [5, 5, 0.1], rgba: [0.3, 0.3, 0.4, 1], isStatic: true });
  geomToMeshMap.push(-1);

  for (let i = 0; i < meshDataList.length; i++) {
    const { mn, aabb } = meshDataList[i];

    // AABB center in world space (USD coordinates)
    const localCenter = new THREE.Vector3(...aabb.center);
    const worldCenter = localCenter.applyMatrix4(mn.worldMatrix);

    // Convert to MuJoCo Z-up
    let pos, size;
    if (needsAxisConvert) {
      pos = yUpToZUp(worldCenter.x, worldCenter.y, worldCenter.z);
      size = yUpToZUpHalfExtents(...aabb.halfExtents);
    } else {
      pos = [worldCenter.x, worldCenter.y, worldCenter.z];
      size = [...aabb.halfExtents];
    }

    // Classify body as static or dynamic
    const name = mn.name;
    let isStatic = false;
    if (classify) {
      isStatic = classify(name, aabb) === "static";
    } else if (staticNames.has(name)) {
      isStatic = true;
    }

    if (isStatic) {
      // Static: geom on world body
      const geom = mj.MjsGeom.add(world, name + "_box");
      mj.MjsGeom.setType(geom, mj.GEOM_BOX);
      mj.MjsGeom.setSize(geom, size[0], size[1], size[2]);
      mj.MjsGeom.setPos(geom, pos[0], pos[1], pos[2]);
      mj.MjsGeom.setRGBA(geom, 0.5, 0.5, 0.6, 0.3);
    } else {
      // Dynamic: body + free joint + geom
      const body = mj.MjsBody.add(world, name);
      mj.MjsBody.setPos(body, pos[0], pos[1], pos[2]);
      mj.MjsJoint.addFree(body);

      const geom = mj.MjsGeom.add(body, name + "_box");
      mj.MjsGeom.setType(geom, mj.GEOM_BOX);
      mj.MjsGeom.setSize(geom, size[0], size[1], size[2]);
      // Mass from volume × density
      const volume = 8 * size[0] * size[1] * size[2];
      mj.MjsGeom.setMass(geom, Math.max(0.01, volume * density));
      mj.MjsGeom.setRGBA(geom, 0.5, 0.5, 0.6, 0.3);
    }

    geomInfos.push({
      type: "box",
      size,
      rgba: [0.5, 0.5, 0.6, 0.3],
      isStatic,
    });
    geomToMeshMap.push(i); // geom index → visual mesh index
  }

  // Compile
  const model = spec.compile();
  spec.delete();
  const data = new mj.PhysicsData(model);

  return { model, data, geomInfos, visualMeshes, geomToMeshMap };
}
