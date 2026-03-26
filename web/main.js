import * as THREE from "three";
import { buildModelFromJSON } from "./scene_loader.js";

const info = document.getElementById("info");

// Map MuJoCo geom type names to Three.js geometries.
function makeThreeGeom(type, size) {
  switch (type) {
    case "box":
      return new THREE.BoxGeometry(size[0] * 2, size[1] * 2, size[2] * 2);
    case "sphere":
      return new THREE.SphereGeometry(size[0], 24, 16);
    case "capsule":
      return new THREE.CapsuleGeometry(size[0], size[1] * 2, 12, 16);
    case "cylinder":
      return new THREE.CylinderGeometry(size[0], size[0], size[1] * 2, 24);
    case "ellipsoid":
      return new THREE.SphereGeometry(1, 24, 16).scale(size[0], size[1], size[2]);
    default:
      return new THREE.SphereGeometry(0.05, 12, 8);
  }
}

function rgbaToColor(rgba) {
  return new THREE.Color(rgba[0], rgba[1], rgba[2]);
}

async function main() {
  // --- Load MuJoCo physics WASM ---
  const { default: loadMujocoPhysics } = await import(
    /* @vite-ignore */ "./mujoco_physics.js"
  );
  const mj = await loadMujocoPhysics();
  info.textContent = "Loading scene...";

  // --- Load scene JSON ---
  const sceneUrl = new URLSearchParams(location.search).get("scene") || "./scene.json";
  const resp = await fetch(sceneUrl);
  if (!resp.ok) {
    info.textContent = `Failed to load ${sceneUrl}`;
    return;
  }
  const config = await resp.json();

  // --- Build MuJoCo model from JSON ---
  const { model, data, geomInfos } = buildModelFromJSON(mj, config);

  const dt = model.timestep();
  const ngeom = model.ngeom();
  info.textContent =
    `MuJoCo | nq=${model.nq()} nv=${model.nv()} ngeom=${ngeom} dt=${dt}`;

  // --- Three.js setup ---
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  document.body.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1a2e);
  scene.fog = new THREE.Fog(0x1a1a2e, 20, 40);

  const camera = new THREE.PerspectiveCamera(
    50, window.innerWidth / window.innerHeight, 0.1, 100
  );
  camera.position.set(4, 3, 4);
  camera.lookAt(0, 0.5, 0);

  // Lights
  scene.add(new THREE.AmbientLight(0x404060, 1.5));
  const dirLight = new THREE.DirectionalLight(0xffffff, 2);
  dirLight.position.set(5, 8, 4);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.set(1024, 1024);
  scene.add(dirLight);

  // Ground visual (always present, rendered as a grid)
  const groundMesh = new THREE.Mesh(
    new THREE.PlaneGeometry(20, 20),
    new THREE.MeshStandardMaterial({ color: 0x334455, roughness: 0.8 })
  );
  groundMesh.rotation.x = -Math.PI / 2;
  groundMesh.receiveShadow = true;
  scene.add(groundMesh);
  scene.add(new THREE.GridHelper(20, 40, 0x556677, 0x2a2a3e));

  // Create Three.js meshes from geomInfos.
  // Dynamic geoms get updated each frame; static geoms (planes, etc.) are skipped.
  const dynamicGeomIndices = [];
  const dynamicMeshes = [];

  for (let i = 0; i < geomInfos.length; i++) {
    const gi = geomInfos[i];
    if (gi.isStatic || gi.type === "plane") continue;

    const geo = makeThreeGeom(gi.type, gi.size);
    const mat = new THREE.MeshStandardMaterial({
      color: rgbaToColor(gi.rgba),
      roughness: 0.3,
      metalness: 0.1,
      transparent: gi.rgba[3] < 1,
      opacity: gi.rgba[3],
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.castShadow = true;
    scene.add(mesh);
    dynamicGeomIndices.push(i);
    dynamicMeshes.push(mesh);
  }

  window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  // --- Simulation loop ---
  const m4 = new THREE.Matrix4();
  const stepsPerFrame = Math.round((1 / 60) / dt);

  function animate() {
    mj.mj_stepN(model, data, stepsPerFrame);

    const gxpos = data.geom_xpos();
    const gxmat = data.geom_xmat();

    for (let k = 0; k < dynamicMeshes.length; k++) {
      const gi = dynamicGeomIndices[k];
      const px = gxpos[gi * 3 + 0];
      const py = gxpos[gi * 3 + 1];
      const pz = gxpos[gi * 3 + 2];
      // MuJoCo Z-up -> Three.js Y-up: (x, z, -y)
      dynamicMeshes[k].position.set(px, pz, -py);

      const o = gi * 9;
      m4.set(
        gxmat[o + 0], gxmat[o + 6], -gxmat[o + 3], 0,
        gxmat[o + 2], gxmat[o + 8], -gxmat[o + 5], 0,
        -gxmat[o + 1], -gxmat[o + 7], gxmat[o + 4], 0,
        0, 0, 0, 1
      );
      dynamicMeshes[k].setRotationFromMatrix(m4);
    }

    const t = data.time();
    info.textContent = `t=${t.toFixed(2)}s | bodies=${model.nbody()} geoms=${ngeom}`;

    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  }

  mj.mj_resetData(model, data);
  mj.mj_forward(model, data);
  requestAnimationFrame(animate);
}

main().catch((e) => {
  console.error(e);
  info.textContent = `Error: ${e.message}`;
});
