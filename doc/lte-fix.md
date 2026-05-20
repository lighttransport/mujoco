# LTE fix: NaN/Inf in thin hull mesh–mesh collision (GJK/EPA)

Investigation and fix for `Nan, Inf or huge value in QACC` instability caused by
**thin (nearly-planar) convex hull mesh–mesh collisions**.

Related upstream report: [google-deepmind/mujoco#1593](https://github.com/google-deepmind/mujoco/issues/1593)
(the upstream thread is a generic capsule-robot RL-divergence question, but it is the
canonical "Nan/Inf/huge value in QACC" symptom; here we localize the mesh-collision cause).

- **Status:** fixed locally on the fork (branch `tinyusdz`). The bug is also present in
  upstream `main` — these are genuine latent bugs, not a fork regression.
- **MuJoCo version:** 3.x (native CCD path).

---

## 1. Symptom

During simulation/training with thin convex-hull meshes in contact, MuJoCo intermittently
warns:

```
Got MuJoCo Warning: Nan, Inf or huge value in QACC at DOF 0. The simulation is unstable.
```

The contact's witness points become non-finite (or huge), which flows into the contact
position/normal → constraint Jacobian → force → `qacc`.

## 2. Code path

For `mjGEOM_MESH` × `mjGEOM_MESH` the default collision path is **native CCD**:

```
mjc_Convex                       (src/engine/engine_collision_convex.c)
  └─ mjc_penetration             (src/engine/engine_collision_convex.c)
      └─ mjc_ccd                 (src/engine/engine_collision_gjk.c)
          ├─ gjk()               broad overlap + initial simplex
          └─ epa()               penetration depth + witness points
              ├─ attachFace → projectOriginPlane()   (face normal / origin projection)
              └─ epaWitness → triAffineCoord()        (final witness points)
```

The witness points `status.x1 / status.x2` are turned into the contact in
`mjc_penetration`:

```c
con->pos   = 0.5 * (x1 + x2);
con->frame = normalize(x1 - x2);
```

## 3. Root cause

Two **divide-by-zero guards that are missing** (they exist in sibling code, but not here).
For a thin hull, EPA can select a degenerate *sliver* face — a triangle whose plane is
well-defined but whose area is ~0. Both functions then divide by that ~0 area and produce
`Inf`/`NaN`/huge values.

### 3.1 `triAffineCoord` — PRIMARY (`engine_collision_gjk.c`)

Computes barycentric coordinates of a point on a face as `lambda[i] = C3i / M_max`, where
`M_max` is the largest projected minor (≈ twice the triangle's projected area). There was
**no guard** against `M_max ≈ 0`:

```c
lambda[0] = C31 / M_max;   // M_max → 0 for a sliver face ⇒ Inf/NaN/huge
lambda[1] = C32 / M_max;
lambda[2] = C33 / M_max;
```

`triAffineCoord` is called by `epaWitness`, which produces the **final** witness points.
NaN/huge `lambda` ⇒ NaN/huge `x1/x2` ⇒ NaN/huge `con->pos` & `con->frame`. The mesh–mesh
multi-contact path (`polygonClip`) reuses `dir = x2 - x1`, so it inherits the same NaN.

### 3.2 `projectOriginPlane` — SECONDARY (`engine_collision_gjk.c`)

Branches 1 and 2 guard `if (nn == 0) return 1;` before dividing, but the **third branch**
divided `nv / nn` with no check:

```c
// branch 3 (last resort)
cross3(n, diff31, diff32);
nv = dot3(n, v3);
nn = dot3(n, n);
scl3(res, n, nv / nn);   // nn can be 0 for a degenerate face ⇒ Inf/NaN
return 0;
```

This yields Inf/NaN `face->v` / `face->dist2`.

> Note: `mju_normalize3` already guards zero-length vectors (returns `[1,0,0]`), so the
> normalization at `mjc_penetration` is **not** the source — the NaN arrives pre-formed in
> the witness points.

## 4. Fix

Four changes (3 source edits + tests).

### A. Guard `triAffineCoord` — `src/engine/engine_collision_gjk.c`

Centroid fallback when the projected triangle is degenerate. The penetration depth comes
from `face->dist2` independently, and `lambda` only interpolates witness points, so the
centroid is a bounded, correct fallback:

```c
  // guard against a degenerate (near-zero-area) projected triangle, which would
  // otherwise divide by ~0 and yield Inf/NaN witness points (see epaWitness).
  // depth comes from face->dist2 independently; lambda only blends witness points.
  if (mju_abs(M_max) < mjMINVAL) {
    lambda[0] = lambda[1] = lambda[2] = 1.0/3.0;
    return;
  }

  // compute affine coordinates
  lambda[0] = C31 / M_max;
  lambda[1] = C32 / M_max;
  lambda[2] = C33 / M_max;
```

### B. Guard `projectOriginPlane` third branch — `src/engine/engine_collision_gjk.c`

Mirror the guard already used in branches 1 and 2:

```c
  cross3(n, diff31, diff32);
  nv = dot3(n, v3);
  nn = dot3(n, n);
  if (nn == 0) return 1;   // ADDED: matches branches 1 and 2
  scl3(res, n, nv / nn);
  return 0;
```

### C. Defensive contact rejection — `src/engine/engine_collision_convex.c`

`mjc_penetration` is the single chokepoint where every native-CCD contact is materialized.
Screen each witness with `mju_isBad()` (catches NaN, Inf, **and** huge-finite `> mjMAXVAL`)
and drop bad ones, compacting the contact array. This is the catch-all guarantee that no
non-finite contact reaches the constraint solver, regardless of source:

```c
    int nwitness = status.nx;
    int ngood = 0;
    for (int i = 0; i < nwitness; i++) {
      mjtNum pos[3], frame[3];
      pos[0] = 0.5*(status.x1[3*i + 0] + status.x2[3*i + 0]);
      pos[1] = 0.5*(status.x1[3*i + 1] + status.x2[3*i + 1]);
      pos[2] = 0.5*(status.x1[3*i + 2] + status.x2[3*i + 2]);
      mji_sub3(frame, status.x1 + 3*i, status.x2 + 3*i);
      mju_normalize3(frame);

      // reject any witness that produced a non-finite/huge contact (defensive):
      // degenerate thin hulls can yield NaN/Inf witness points in EPA
      if (mju_isBad(margin + dist) ||
          mju_isBad(pos[0]) || mju_isBad(pos[1]) || mju_isBad(pos[2]) ||
          mju_isBad(frame[0]) || mju_isBad(frame[1]) || mju_isBad(frame[2])) {
        continue;
      }

      con->dist = margin + dist;
      mji_copy3(con->pos, pos);
      mji_copy3(con->frame, frame);
      mji_zero3(con->frame + 3);
      con++;
      ngood++;
    }
    return ngood;
```

A legitimate coincident witness (`x1 == x2`) still yields a valid frame via
`mju_normalize3`'s zero-length guard and is **kept** — only genuinely bad values are dropped.

### D. Tests — `test/engine/engine_collision_gjk_test.cc`

- `ThinMeshContactsAreFinite` — invariant sweep over ~990 thin / finely-faceted hull
  configs (sides × scale × thickness ratio × tilt × penetration axis × depth), asserting
  every reported `dist`/`dir`/`pos` is finite (`mju_isBad`).
- `ThinMeshNoNanWitness` — focused thin-plate case documenting the issue.

## 5. Verification

```bash
# build (see §6 about the USD configure issue first)
cmake --build build --target engine_collision_gjk_test engine_collision_convex_test -j

# GJK suite (inline-XML tests; run from anywhere)
./build/bin/engine_collision_gjk_test            # 43/43 pass

# convex suite loads engine/testdata/...; run from the test/ dir
( cd test && ../build/bin/engine_collision_convex_test )   # 3/3 pass
```

**Honest caveat on reproducibility.** With *well-formed, double-precision* hulls the
unpatched code stayed finite across all 990 swept configs — MuJoCo's other degeneracy
guards (e.g. EPA `BAD_NORMAL` at `mjMINVAL`) catch most cases first, and clean hulls do not
produce sub-`1e-15` EPA face areas (sub-micron meshes do not even compile). So:

- A and B are **defensive hardening of objectively-missing guards** (matching sibling code).
- C is the **robust catch-all** that prevents any non-finite contact from reaching the solver.
- The sweep test is a **forward-looking invariant guard**, not a pre-fix reproducer.

The degeneracy that triggers this in practice most likely comes from the **actual mesh
data** (near-collinear / duplicate convex-hull vertices from CAD/scan/decimation). With such
a mesh in hand, C can be confirmed to fire and the test tightened into a true reproducer.

## 6. Build / environment notes

- The fork's `build/` CMake dir fails at the *generate* step because
  `test/experimental/CMakeLists.txt` (gated on `MUJOCO_WITH_USD=ON`) references an imported
  `usd` target whose include path `/opt/USD_inst/20.05-py3/include` does not exist.
  Reconfigure with USD off to build/test:

  ```bash
  cmake -S . -B build -DMUJOCO_WITH_USD=OFF
  ```

  (Re-enable USD once the install path is restored.)

- **WASM build** (`build-wasm-physics/`) uses the same engine source — rebuild it to pick up
  this fix.

## 7. Immediate workarounds (no rebuild)

- Disable native CCD per model — routes to libccd/MPR, which does not use `triAffineCoord`
  (single-contact, slower, lower manifold quality):

  ```xml
  <option>
    <flag nativeccd="disable"/>
  </option>
  ```

- Remove the geometric pathology — give thin plates real thickness, or model them as
  `type="box"` primitives (analytic, robust).

## 8. Files changed

| File | Change |
| --- | --- |
| `src/engine/engine_collision_gjk.c` | A: `triAffineCoord` `M_max` guard; B: `projectOriginPlane` `nn` guard |
| `src/engine/engine_collision_convex.c` | C: `mju_isBad` contact rejection in `mjc_penetration` |
| `test/engine/engine_collision_gjk_test.cc` | D: `ThinMeshContactsAreFinite`, `ThinMeshNoNanWitness` |
