import * as THREE from "three";

const info = document.getElementById("info");

async function main() {
  // --- Load MuJoCo physics WASM ---
  // Loaded from public/ — skip vite's module graph.
  const { default: loadMujocoPhysics } = await import(
    /* @vite-ignore */ "./mujoco_physics.js"
  );
  const mj = await loadMujocoPhysics();
  info.textContent = "Building scene...";

  // --- Build scene procedurally via mjSpec ---
  const spec = new mj.MjSpec();
  spec.setModelName("three_demo");
  spec.setTimestep(0.002);
  spec.setGravity(0, 0, -9.81);

  const world = spec.worldBody();

  // Ground plane
  const groundGeom = mj.MjsGeom.add(world, "ground");
  mj.MjsGeom.setType(groundGeom, mj.GEOM_PLANE);
  mj.MjsGeom.setSize(groundGeom, 5, 5, 0.1);
  mj.MjsGeom.setRGBA(groundGeom, 0.3, 0.3, 0.4, 1);

  // Light
  const light = mj.MjsLight.add(world, "top");
  mj.MjsLight.setPos(light, 0, 0, 4);
  mj.MjsLight.setDir(light, 0, 0, -1);
  mj.MjsLight.setDiffuse(light, 0.8, 0.8, 0.8);

  // Falling box
  const boxBody = mj.MjsBody.add(world, "box");
  mj.MjsBody.setPos(boxBody, 0, 0, 2);
  mj.MjsJoint.addFree(boxBody);
  const boxGeom = mj.MjsGeom.add(boxBody, "box_geom");
  mj.MjsGeom.setType(boxGeom, mj.GEOM_BOX);
  mj.MjsGeom.setSize(boxGeom, 0.2, 0.2, 0.2);
  mj.MjsGeom.setRGBA(boxGeom, 0.2, 0.6, 1.0, 1);

  // Sphere on a pedestal
  const sphereBody = mj.MjsBody.add(world, "sphere");
  mj.MjsBody.setPos(sphereBody, 1.2, 0, 1.5);
  mj.MjsJoint.addFree(sphereBody);
  const sphereGeom = mj.MjsGeom.add(sphereBody, "sphere_geom");
  mj.MjsGeom.setType(sphereGeom, mj.GEOM_SPHERE);
  mj.MjsGeom.setSize(sphereGeom, 0.15, 0, 0);
  mj.MjsGeom.setRGBA(sphereGeom, 1.0, 0.4, 0.2, 1);

  // Capsule
  const capBody = mj.MjsBody.add(world, "capsule");
  mj.MjsBody.setPos(capBody, -1.0, 0.5, 1.8);
  mj.MjsJoint.addFree(capBody);
  const capGeom = mj.MjsGeom.add(capBody, "cap_geom");
  mj.MjsGeom.setType(capGeom, mj.GEOM_CAPSULE);
  mj.MjsGeom.setSize(capGeom, 0.1, 0.3, 0);
  mj.MjsGeom.setRGBA(capGeom, 0.3, 1.0, 0.4, 1);

  // Compile
  const model = spec.compile();
  spec.delete();
  const data = new mj.PhysicsData(model);

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

  // Ground
  const groundMesh = new THREE.Mesh(
    new THREE.PlaneGeometry(20, 20),
    new THREE.MeshStandardMaterial({ color: 0x334455, roughness: 0.8 })
  );
  groundMesh.rotation.x = -Math.PI / 2;
  groundMesh.receiveShadow = true;
  scene.add(groundMesh);
  scene.add(new THREE.GridHelper(20, 40, 0x556677, 0x2a2a3e));

  // Create Three.js meshes for each dynamic geom.
  // geom 0 is the ground plane (static), geoms 1..ngeom-1 are dynamic.
  const geomMeshes = [];
  const geomColors = [
    null,                     // 0: ground (handled separately)
    new THREE.Color(0x3399ff), // 1: box
    new THREE.Color(0xff6633), // 2: sphere
    new THREE.Color(0x44ff66), // 3: capsule
  ];
  const geomGeometries = [
    null,
    new THREE.BoxGeometry(0.4, 0.4, 0.4),
    new THREE.SphereGeometry(0.15, 24, 16),
    new THREE.CapsuleGeometry(0.1, 0.6, 12, 16),
  ];

  for (let i = 1; i < ngeom; i++) {
    const geo = geomGeometries[i] || new THREE.SphereGeometry(0.1, 16, 12);
    const mat = new THREE.MeshStandardMaterial({
      color: geomColors[i] || 0xcccccc, roughness: 0.3, metalness: 0.1
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.castShadow = true;
    scene.add(mesh);
    geomMeshes.push(mesh);
  }

  window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  // --- Simulation loop ---
  // MuJoCo uses Z-up, Three.js uses Y-up.
  // MuJoCo geom_xmat is a 3x3 rotation matrix (row-major).
  const m4 = new THREE.Matrix4();
  const stepsPerFrame = Math.round((1 / 60) / dt);

  function animate() {
    mj.mj_stepN(model, data, stepsPerFrame);

    const gxpos = data.geom_xpos();
    const gxmat = data.geom_xmat();

    for (let i = 0; i < geomMeshes.length; i++) {
      const gi = i + 1; // skip geom 0 (ground)
      const px = gxpos[gi * 3 + 0];
      const py = gxpos[gi * 3 + 1];
      const pz = gxpos[gi * 3 + 2];
      // MuJoCo Z-up → Three.js Y-up: (x, z, -y)
      geomMeshes[i].position.set(px, pz, -py);

      // Convert MuJoCo row-major 3x3 rotation to Three.js 4x4 (with axis swap)
      const o = gi * 9;
      // MuJoCo mat columns → Three.js columns with Z-up→Y-up
      m4.set(
        gxmat[o + 0], gxmat[o + 6], -gxmat[o + 3], 0,
        gxmat[o + 2], gxmat[o + 8], -gxmat[o + 5], 0,
        -gxmat[o + 1], -gxmat[o + 7], gxmat[o + 4], 0,
        0, 0, 0, 1
      );
      geomMeshes[i].setRotationFromMatrix(m4);
    }

    const t = data.time();
    const qpos = data.qpos();
    info.textContent =
      `t=${t.toFixed(2)}s | box z=${qpos[2].toFixed(3)}`;

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
