# How MuJoCo handles collision ("hull") meshes

A reference for how MuJoCo treats collision geometry — both the **engine internals**
(meshes are collided as their *convex hull*) and the **modeling convention** used
throughout [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
(`group 2` = visual, `group 3` = collision). Written with USD/tinyusdz → MJCF import in
mind; see the [USD → MJCF mapping](#7-usd--mjcf-import-mapping) section and the existing
[`usd-physics.md`](usd-physics.md).

All claims below are cited to source/doc lines in this repository.

---

## 1. TL;DR — the mental model

1. **`group` does NOT control collision.** It is a *visualization* (and inertia-selection)
   tag only. Whether two geoms collide is decided entirely by the `contype`/`conaffinity`
   bitmasks. Putting a collision geom in `group 3` merely **hides it in the viewer by
   default** — it does not enable or disable physics.
2. **Collision is filtered by `contype`/`conaffinity`.** Two geoms can collide iff
   `(contype1 & conaffinity2) || (contype2 & conaffinity1)`. Default is `1`/`1`
   (everything collides). Visual-only geoms set both to `0` to opt out.
3. **Meshes collide as their convex hull.** Even if a mesh is concave (and rendered
   concave), MuJoCo replaces it with the qhull-computed convex hull for collision.
4. **Concave collision = many convex pieces.** MuJoCo does *not* auto-decompose. A concave
   collision shape must be supplied as multiple `<geom type="mesh">`, one per convex piece
   (V-HACD / CoACD), or approximated with primitives.

---

## 2. The two-geom convention (visual vs collision)

Menagerie models split each body's geometry into two sets, organized with `<default>`
classes. This pattern is essentially universal — of the models checked, 84 define a
`visual` class, 86 a `collision` class, and 93 use `group="3"`.

```xml
<default>
  <default class="visual">
    <geom type="mesh" contype="0" conaffinity="0" group="2"/>
  </default>
  <default class="collision">
    <geom group="3"/>          <!-- inherits contype=conaffinity=1 → collides -->
  </default>
</default>
```

Then on each body:

```xml
<body name="link1">
  <geom mesh="link1_visual" class="visual"/>   <!-- detailed, no collision, shown -->
  <geom mesh="link1_c"      class="collision"/> <!-- simplified, collides, hidden -->
</body>
```

**Why split them:**
- **Performance/stability** — collision uses a coarse, low-vertex (often primitive) shape;
  rendering uses the detailed art mesh.
- **Clean inertia** — visual geoms are excluded from inferred mass/inertia (see
  `inertiagrouprange`, §3).
- **Clean viewer** — collision geoms sit in an invisible group (3) so the rendered model
  looks like the visual meshes; press the group-toggle keys to inspect collision shapes.

This exact block appears in `franka_emika_panda/panda.xml`, `unitree_go2/go2.xml`,
`universal_robots_ur5e/ur5e.xml`, `anybotics_anymal_c/anymal_c.xml`,
`shadow_hand/right_hand.xml`, `robotiq_2f85/2f85.xml`, `aloha/aloha.xml`,
`unitree_g1/g1.xml`, and most others.

---

## 3. `group` semantics — visualization + inertia only

The `group` integer has **no runtime effect on physics**. From
[`doc/XMLreference.rst:2583`](XMLreference.rst) (geom `group`, default `0`):

> *"This attribute specifies an integer group to which the geom belongs. The only effect on
> the physics is at compile time, when body masses and inertias are inferred from geoms
> selected based on their group; see `inertiagrouprange` … At runtime this attribute is used
> by the visualizer to enable and disable the rendering of entire geom groups. By default,
> groups 0, 1 and 2 are visible, while all other groups are invisible."*

This is borne out by the source:

- `mjModel.geom_group` is commented *"group for visibility"*
  (`include/mujoco/mjmodel.h:884`).
- It is read **only** by the visualizer
  (`src/engine/engine_vis_visualize.c:866` and `:1292`:
  `if (!vopt->geomgroup[... m->geom_group[i] ...])`), and for inertia-group selection in the
  compiler.
- There are **zero** references to `geom_group` in the collision driver
  (`src/engine/engine_collision_driver.c`).

Consequences:
- `group 2` (visual) is **visible** by default; `group 3` (collision) is **invisible** by
  default. That is the only reason collision geoms "disappear" in the viewer.
- In `simulate`, toggle geom group visibility with number keys, and press **`H`** to render
  the convex hulls actually used for collision.
- `inertiagrouprange` (`doc/XMLreference.rst:867`, default `"0 5"`) selects which groups
  feed inferred inertia — useful precisely *"in models that have redundant sets of geoms for
  collision and visualization."*

---

## 4. What actually filters collisions: `contype` / `conaffinity`

Each geom carries two 32-bit bitmasks. Two geoms are considered for collision iff:

```
(contype1 & conaffinity2) || (contype2 & conaffinity1)
```

From [`doc/XMLreference.rst:2548`](XMLreference.rst):

> *"This attribute and the next specify 32-bit integer bitmasks used for contact filtering of
> dynamically generated contact pairs … Two geoms can collide if the contype of one geom is
> compatible with the conaffinity of the other geom or vice versa. Compatible means that the
> two bitmasks have a common bit set to 1."*

Default is `contype = conaffinity = 1`, so everything collides unless told otherwise. The
filter is implemented in `src/engine/engine_collision_driver.c:99-102`:

```c
static int filterBitmask(int contype1, int conaffinity1,
                         int contype2, int conaffinity2) {
  return !(contype1 & conaffinity2) && !(contype2 & conaffinity1);
}
```
(returns `1` = *filter out / no collision*). It is applied in broad-phase and pair tests at
`engine_collision_driver.c:209, 472, 819, 1592`.

**Disabling collision** (the menagerie "visual" pattern):

```xml
<geom type="mesh" contype="0" conaffinity="0" group="2"/>
```

`0 & x == 0` for all `x`, so the geom collides with nothing. Geoms with
`contype = conaffinity = 0` that are not referenced elsewhere are even **discarded at
compile time** (with `disableflags`, `doc/XMLreference.rst:~815`); if such a geom was used
for inertia, an explicit `<inertial>` is synthesized.

Other per-contact controls (all per-geom, merged per pair):
- `condim` — contact dimensionality `1` (frictionless), `3` (regular), `4` (+torsion),
  `6` (+rolling).
- `margin` / `gap` — detect contact when `dist < margin`; generate force only when
  `dist < margin - gap`.
- `priority` — when two geoms differ in priority, the higher one's parameters win; equal
  priority merges (`condim` = max, `friction` = max, `solref`/`solimp` mixed by `solmix`).
- `friction`, `solref`, `solimp` — Coulomb friction and solver stiffness/impedance.

Param-merge logic lives in `engine_collision_driver.c:~1400-1493`. For full control over a
specific pair, use an explicit `<pair>` (overrides, stored in `mjModel.pair_*`); use
`<exclude>` to suppress a body pair entirely.

---

## 5. Mesh → convex hull pipeline

> *"collision detection is limited to convex geoms … Meshes specified by the user can be
> non-convex, and are rendered as such. For collision purposes however they are replaced
> with their convex hulls (visualized with the 'H' key in simulate), computed by the qhull
> library."* — [`doc/computation/index.rst:1549`](computation/index.rst)

### Compile time
- `mjCMesh::MakeGraph()` (`src/user/user_mesh.cc:1708`) runs qhull with options
  `"qhull Qt"` (`:1713`); if `maxhullvert` is set it appends `TA<maxhullvert-4>` to cap the
  hull vertex count (`:1715`). It then triangulates and computes vertex neighbors, storing an
  adjacency graph used by the narrow phase.
- The hull/mesh data is stored in `mjModel`
  (`include/mujoco/mjmodel.h:~1036-1068`): `mesh_vert`, `mesh_face`, `mesh_graph`,
  and the polygon tables `mesh_poly*`.
- `<mesh maxhullvert>` (`doc/XMLreference.rst:1317`, default `-1` = unlimited; values must be
  `> 3`) caps hull complexity for speed.
- `<mesh inertia>` (`doc/XMLreference.rst:~1289`) controls inertia from geometry:
  `legacy` (current default; overcounts non-convex volume), `convex` (hull; the future
  default), `exact` (watertight non-convex), `shell` (surface mass).

### Run time
- Narrow-phase convex collision (GJK/EPA, MPR) queries the mesh via a support function that
  hill-climbs over the hull adjacency graph
  (`src/engine/engine_collision_convex.c:~358-442`); it falls back to an exhaustive vertex
  search when there is no graph or the mesh has fewer than `mjMESH_HILLCLIMB_MIN` (10)
  vertices.

### Implication for concave shapes
MuJoCo never produces a concave collider from a single mesh. To collide a concave shape
accurately you must either:
1. supply a **convex decomposition** — multiple `<geom type="mesh">`, one per convex piece
   (e.g. V-HACD/CoACD), or
2. approximate it with **primitives** (box/capsule/cylinder/sphere/plane/hfield), or
3. use an **SDF plugin** (the one non-convex collision path; see `plugin/sdf`).

---

## 6. Collision strategies seen in Menagerie

| Strategy | What it is | Example models |
|---|---|---|
| **Mesh hull** | Collision geom references a mesh (collided as its hull); often the same or a simplified mesh | `shadow_hand`, `robotiq_2f85`, `franka_emika_panda` (main links), `unitree_g1` |
| **Convex decomposition** | One link's collision is several convex mesh pieces | `franka_emika_panda` link5 → `link5_collision_{0,1,2}.obj` (V-HACD, per its README) |
| **Primitive proxies** | box/capsule/cylinder/sphere instead of meshes; fastest and most stable | `unitree_go2`, `universal_robots_ur5e`, `anybotics_anymal_c` |

Hybrids are common: a robot may use mesh hulls for big links and primitives for contact-
critical parts. Contact regions (feet, gripper pads) are frequently tuned with a subclass
that sets `priority="1"`, `condim="6"`, and custom `friction`/`solimp` — e.g.
`anybotics_anymal_c` `foot` and `robotiq_2f85` `pad_box1`/`pad_box2`.

Naming is **not** standardized: some models name collision meshes (`*_c`,
`*_collision_*.obj`); others reuse the visual mesh name and distinguish solely by `class`/
`group`.

---

## 7. USD → MJCF import mapping

Practical mapping for a USD/tinyusdz → MJCF importer. The guiding rule: emit **two geoms**
per prim with physics + visuals — a `class="visual"` geom (`contype=0 conaffinity=0
group=2`) and a `class="collision"` geom (`group=3`, default contype/conaffinity).

| USD concept | MJCF target | Notes for the importer |
|---|---|---|
| `UsdGeom` render mesh, no physics API | `<geom type="mesh" class="visual">` → `contype=0 conaffinity=0 group=2` | Purely visual; keep out of inertia (or rely on `inertiagrouprange`). |
| Prim with `UsdPhysicsCollisionAPI` (mesh) | `<geom type="mesh" class="collision">`, `group=3`, default contype/conaffinity | MuJoCo collides it **as the convex hull** — warn if the source mesh is concave. |
| `UsdPhysicsMeshCollisionAPI` `approximation = convexHull` | a single `<geom type="mesh">` | Exact match — MuJoCo always hulls a mesh. |
| `approximation = convexDecomposition` | **multiple** `<geom type="mesh">`, one per piece | Run/ingest the decomposition; MuJoCo will not decompose for you. |
| `approximation = boundingCube` / `boundingSphere` | `<geom type="box">` / `<geom type="sphere">` proxy | Compute extents from mesh bounds. |
| `approximation = none` (exact triangle soup) | not collidable as-is | Accept the hull approximation, or emit an SDF plugin / decomposition. Document the fidelity loss. |
| Separate visual + collision prims under one Xform/body | two geoms in one `<body>` (visual + collision classes) | This is exactly the menagerie pattern. |
| USD physics collision groups / filtered pairs | `contype`/`conaffinity` bit assignment, and/or `<pair>` / `<exclude>` | Map group membership → bits; map explicit include/exclude → `<pair>`/`<exclude>`. |
| Physics material (static/dynamic friction, restitution) | `friction`, `solref`/`solimp`, `priority`, `condim` | Per-geom, or per-`<pair>` for an exact pair. MuJoCo has no direct restitution; approximate via `solref`. |
| `physics:approximation`-driven `maxhullvert` needs | `<mesh maxhullvert="N">` | Cap hull complexity for performance parity. |

Relevant in-repo code: USD decoder/import under `src/experimental/usd`,
`plugin/usd_decoder/usd_decoder.cc`, and the web loader `web/usd_loader.js`.

---

## 8. References

Source (this repo):
- `include/mujoco/mjmodel.h:884` — `geom_group` ("group for visibility");
  `:~1036-1068` — mesh/hull fields (`mesh_vert/face/graph`, `mesh_poly*`).
- `src/engine/engine_collision_driver.c:99-102` — `filterBitmask`;
  `:~1400-1493` — per-pair parameter merge.
- `src/engine/engine_collision_convex.c:~358-442` — mesh support via hull-graph hill-climb.
- `src/user/user_mesh.cc:1708` — `mjCMesh::MakeGraph` (qhull `"qhull Qt"`, `TA<maxhullvert>`).
- `src/engine/engine_vis_visualize.c:866` — group used only for render visibility.

Docs:
- `doc/XMLreference.rst:2548` (contype/conaffinity), `:2583` (geom `group`),
  `:1317` (`maxhullvert`), `:~1289` (mesh `inertia`), `:867` (`inertiagrouprange`).
- `doc/computation/index.rst:1549` — meshes collided as convex hull (qhull).
