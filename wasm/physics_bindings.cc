// Copyright 2025 DeepMind Technologies Limited
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

#ifdef __EMSCRIPTEN__

#include <emscripten/bind.h>
#include <emscripten/emscripten.h>
#include <emscripten/val.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <mujoco/mjspec.h>
#include "mujoco/mujoco.h"

namespace {

// Route mju_warning / mju_error directly to JS console.{warn,error}.
// Otherwise MuJoCo falls back to vfprintf via newlib stdio, which on
// `-s FILESYSTEM=0` ends up driving `fd_write` with a partly-initialized
// FILE struct (fd=0). The glue's per-fd buffer table only allocates
// slots for stdout/stderr, so a write to fd=0 crashes with
// "Cannot read properties of null (reading 'push')".
EM_JS(void, lg_console_warn, (const char* msg), {
  console.warn('[mj]', UTF8ToString(msg));
});

EM_JS(void, lg_console_error, (const char* msg), {
  console.error('[mj]', UTF8ToString(msg));
});

void warning_to_console(const char* msg) { lg_console_warn(msg); }
void error_to_console(const char* msg) { lg_console_error(msg); }

struct InstallMujocoCallbacks {
  InstallMujocoCallbacks() {
    mju_user_warning = &warning_to_console;
    mju_user_error = &error_to_console;
  }
};
const InstallMujocoCallbacks kInstallMujocoCallbacks;

using emscripten::typed_memory_view;
using emscripten::val;

// ---------------------------------------------------------------------------
// PhysicsModel / PhysicsData wrappers
// ---------------------------------------------------------------------------

class PhysicsModel {
 public:
  explicit PhysicsModel(mjModel* model) : model_(model) {}

  ~PhysicsModel() {
    if (model_) {
      mj_deleteModel(model_);
    }
  }

  PhysicsModel(const PhysicsModel&) = delete;
  PhysicsModel& operator=(const PhysicsModel&) = delete;

  mjModel* get() const { return model_; }

  int nq() const { return model_->nq; }
  int nv() const { return model_->nv; }
  int na() const { return model_->na; }
  int nu() const { return model_->nu; }
  int nbody() const { return model_->nbody; }
  int ngeom() const { return model_->ngeom; }
  int njnt() const { return model_->njnt; }
  int neq() const { return model_->neq; }
  int ntendon() const { return model_->ntendon; }
  int nsensordata() const { return model_->nsensordata; }
  // Diagnostics: passive joint/dof properties + compiled joint ranges.
  val jnt_stiffness() const { return val(typed_memory_view(model_->njnt, model_->jnt_stiffness)); }
  val jnt_range() const { return val(typed_memory_view(model_->njnt * 2, model_->jnt_range)); }
  val dof_damping() const { return val(typed_memory_view(model_->nv, model_->dof_damping)); }
  val dof_armature() const { return val(typed_memory_view(model_->nv, model_->dof_armature)); }
  val dof_frictionloss() const { return val(typed_memory_view(model_->nv, model_->dof_frictionloss)); }
  double timestep() const { return model_->opt.timestep; }
  void setTimestep(double dt) { model_->opt.timestep = dt; }
  val gravity() const {
    return val(typed_memory_view(3, model_->opt.gravity));
  }

  // Solver contact-override parameters (mjModel.opt). When the
  // mjENBL_OVERRIDE enable bit is set, MuJoCo uses o_solref/o_solimp for
  // EVERY contact instead of the per-geom-derived values — a runtime,
  // rebuild-free lever for global contact compliance (softness). Returned as
  // typed_memory_views so JS can read/write the arrays in place.
  val o_solref() const {
    return val(typed_memory_view(mjNREF, model_->opt.o_solref));
  }
  val o_solimp() const {
    return val(typed_memory_view(mjNIMP, model_->opt.o_solimp));
  }
  double o_margin() const { return model_->opt.o_margin; }
  void setOMargin(double m) { model_->opt.o_margin = m; }
  int enableflags() const { return model_->opt.enableflags; }
  void setEnableFlags(int f) { model_->opt.enableflags = f; }

 private:
  mjModel* model_;
};

class PhysicsData {
 public:
  explicit PhysicsData(const PhysicsModel& model) : model_(model.get()) {
    data_ = mj_makeData(model_);
    if (!data_) {
      mju_error("Failed to create mjData");
    }
  }

  ~PhysicsData() {
    if (data_) {
      mj_deleteData(data_);
    }
  }

  PhysicsData(const PhysicsData&) = delete;
  PhysicsData& operator=(const PhysicsData&) = delete;

  mjData* get() const { return data_; }

  double time() const { return data_->time; }

  val qpos() const {
    return val(typed_memory_view(model_->nq, data_->qpos));
  }

  val qvel() const {
    return val(typed_memory_view(model_->nv, data_->qvel));
  }

  val act() const {
    return val(typed_memory_view(model_->na, data_->act));
  }

  val ctrl() const {
    return val(typed_memory_view(model_->nu, data_->ctrl));
  }

  val sensordata() const {
    return val(typed_memory_view(model_->nsensordata, data_->sensordata));
  }

  val xpos() const {
    return val(typed_memory_view(model_->nbody * 3, data_->xpos));
  }

  val xquat() const {
    return val(typed_memory_view(model_->nbody * 4, data_->xquat));
  }

  val geom_xpos() const {
    return val(typed_memory_view(model_->ngeom * 3, data_->geom_xpos));
  }

  val geom_xmat() const {
    return val(typed_memory_view(model_->ngeom * 9, data_->geom_xmat));
  }

  // Mocap targets, writeable: [x,y,z] / [w,x,y,z] per mocap body. The JS side
  // sets these each substep to drive kinematic (mocap) bodies.
  val mocap_pos() const {
    return val(typed_memory_view(model_->nmocap * 3, data_->mocap_pos));
  }
  val mocap_quat() const {
    return val(typed_memory_view(model_->nmocap * 4, data_->mocap_quat));
  }

  // Per-body external force/torque accumulator, layout [fx,fy,fz, tx,ty,tz]
  // per body in the world frame. Writeable: the JS side mutates it directly
  // (e.g. the interactive grab-spring) and mj_step consumes it. Caller is
  // responsible for zeroing it when the interaction ends.
  val xfrc_applied() const {
    return val(typed_memory_view(model_->nbody * 6, data_->xfrc_applied));
  }

  // Joint-space applied generalized force (nv), indexed by DOF address. Lets
  // callers do PD / torque control without compiled actuators: write a torque
  // per joint DOF each substep and mj_step consumes it. Zero it on release.
  val qfrc_applied() const {
    return val(typed_memory_view(model_->nv, data_->qfrc_applied));
  }

  // Bias force (gravity + Coriolis/centrifugal), nv, from the last forward
  // pass. Read-only feedforward term for gravity-compensated control.
  val qfrc_bias() const {
    return val(typed_memory_view(model_->nv, data_->qfrc_bias));
  }

  // Number of active contacts after the last mj_forward / mj_step.
  int ncon() const { return data_->ncon; }

  // Flat contact summary: 10 doubles per active contact —
  // [dist, px,py,pz, nx,ny,nz, geom1, geom2, normalForce]. `dist` < 0 means
  // penetration; (px,py,pz) is the world contact point; (nx,ny,nz) is the
  // contact-frame normal; geom1/geom2 are the colliding geom indices;
  // normalForce is the solver-computed normal-force magnitude (N) from
  // mj_contactForce — 0 when the contact isn't in the constraint set.
  // Backed by a member buffer so the returned view stays valid until the
  // next call.
  val contacts() const {
    const int n = data_->ncon;
    contact_buf_.resize(static_cast<std::size_t>(n) * 10);
    for (int i = 0; i < n; ++i) {
      const mjContact& c = data_->contact[i];
      double* row = contact_buf_.data() + static_cast<std::size_t>(i) * 10;
      row[0] = c.dist;
      row[1] = c.pos[0]; row[2] = c.pos[1]; row[3] = c.pos[2];
      row[4] = c.frame[0]; row[5] = c.frame[1]; row[6] = c.frame[2];
      row[7] = static_cast<double>(c.geom[0]);
      row[8] = static_cast<double>(c.geom[1]);
      mjtNum force[6] = {0};
      if (c.efc_address >= 0) {
        mj_contactForce(model_, data_, i, force);
      }
      row[9] = force[0];   // normal component, contact frame
    }
    return val(typed_memory_view(contact_buf_.size(), contact_buf_.data()));
  }

  // Diagonal of the joint-space inertia (mass) matrix, nv entries, from the
  // last forward pass. qM is stored sparse; model->dof_Madr[i] addresses the
  // diagonal element of DOF i. Lets callers scale PD gains by per-joint
  // inertia so impedance control is uniformly stable across heavy and
  // light joints (gains become natural-frequency units, inertia-independent).
  val qM_diag() const {
    const int nv = model_->nv;
    qm_diag_buf_.resize(static_cast<std::size_t>(nv));
    for (int i = 0; i < nv; ++i) {
      qm_diag_buf_[i] = data_->qM[model_->dof_Madr[i]];
    }
    return val(typed_memory_view(qm_diag_buf_.size(), qm_diag_buf_.data()));
  }

 private:
  mjModel* model_;
  mjData* data_ = nullptr;
  mutable std::vector<double> contact_buf_;
  mutable std::vector<double> qm_diag_buf_;
};

// ---------------------------------------------------------------------------
// Load from binary buffer
// ---------------------------------------------------------------------------

PhysicsModel* LoadModelFromArrayBuffer(const val& buffer) {
  val uint8_ctor = val::global("Uint8Array");
  val bytes = buffer.instanceof(uint8_ctor) ? buffer : uint8_ctor.new_(buffer);
  unsigned int length = bytes["length"].as<unsigned int>();
  if (length == 0) {
    mju_error("ArrayBuffer is empty");
  }

  std::uint8_t* tmp = static_cast<std::uint8_t*>(std::malloc(length));
  if (!tmp) {
    mju_error("Failed to allocate buffer for model data");
  }

  val heap = val::module_property("HEAPU8");
  val sub = bytes.call<val>("subarray", 0, length);
  heap.call<void>("set", sub, reinterpret_cast<uintptr_t>(tmp));

  mjModel* model = mj_loadModelBuffer(tmp, static_cast<int>(length));
  std::free(tmp);
  if (!model) {
    mju_error("Failed to load model from binary buffer");
  }
  return new PhysicsModel(model);
}

// ---------------------------------------------------------------------------
// Spec API wrappers
// ---------------------------------------------------------------------------

class SpecWrapper {
 public:
  SpecWrapper() : spec_(mj_makeSpec()), owned_(true) {}
  ~SpecWrapper() {
    if (spec_ && owned_) {
      mj_deleteSpec(spec_);
    }
  }

  SpecWrapper(const SpecWrapper&) = delete;
  SpecWrapper& operator=(const SpecWrapper&) = delete;

  mjSpec* get() const { return spec_; }

  void setModelName(const std::string& name) {
    mjs_setString(spec_->modelname, name.c_str());
  }

  void setTimestep(double dt) { spec_->option.timestep = dt; }
  double getTimestep() const { return spec_->option.timestep; }

  void setGravity(double x, double y, double z) {
    spec_->option.gravity[0] = x;
    spec_->option.gravity[1] = y;
    spec_->option.gravity[2] = z;
  }

  // Integrator: 0=Euler, 1=RK4, 2=implicit, 3=implicitfast. implicit(fast)
  // integrates actuator gains and joint damping implicitly, which is what
  // makes stiff position servos stable at the sim timestep — Euler integrates
  // them explicitly and a stiff servo then blows up.
  void setIntegrator(int which) {
    spec_->option.integrator = static_cast<mjtIntegrator>(which);
  }

  mjsBody* worldBody() {
    return mjs_findBody(spec_, "world");
  }

  // Mesh asset (procedural). Member function so embind doesn't need
  // to marshal a SpecWrapper handle as a free-function parameter
  // (cross-class type marshalling triggers a "parameter 0 has
  // unknown type" registration error in embind). Returns the new
  // mjsMesh* the caller then populates via MjsMesh.setUserVert /
  // setUserFace / setMaxHullVert.
  mjsMesh* addMesh(const std::string& name) {
    mjsMesh* m = mjs_addMesh(spec_, nullptr);
    if (!m) {
      mju_error("mjs_addMesh failed");
    }
    if (!name.empty()) {
      mjs_setName(m->element, name.c_str());
    }
    return m;
  }

  // Contact-exclusion pair: emit a <contact><exclude> between two named
  // bodies so authored collision filtering (UsdPhysicsFilteredPairsAPI) is
  // honored exactly, instead of being packed into contype/conaffinity bits.
  void addExclude(const std::string& bodyName1, const std::string& bodyName2) {
    mjsExclude* ex = mjs_addExclude(spec_);
    if (!ex) {
      mju_error("mjs_addExclude failed");
    }
    mjs_setString(ex->bodyname1, bodyName1.c_str());
    mjs_setString(ex->bodyname2, bodyName2.c_str());
  }

  // Position servo actuator on a joint — the MuJoCo `<position kp kv>`
  // expansion (gaintype FIXED with gainprm[0]=kp; biastype AFFINE with
  // biasprm=[0,-kp,-kv]). Generalized force = kp*(ctrl - qpos) - kv*qvel,
  // integrated implicitly by mj_step, so a joint can be servoed to a target
  // angle/length (write data.ctrl[i] = target) stably at the sim timestep —
  // unlike an explicit qfrc PD. Actuators are indexed by creation order, which
  // is the same order as data.ctrl, so the caller tracks the ctrl index itself.
  // forcerange optionally clamps |output| (0 = unlimited).
  void addPositionActuator(const std::string& jointName, double kp, double kv,
                           double forceRange) {
    mjsActuator* a = mjs_addActuator(spec_, nullptr);
    if (!a) {
      mju_error("mjs_addActuator failed");
    }
    a->trntype = mjTRN_JOINT;
    mjs_setString(a->target, jointName.c_str());
    a->gaintype = mjGAIN_FIXED;
    a->gainprm[0] = kp;
    a->biastype = mjBIAS_AFFINE;
    a->biasprm[0] = 0.0;
    a->biasprm[1] = -kp;
    a->biasprm[2] = -kv;
    if (forceRange > 0.0) {
      a->forcelimited = mjLIMITED_TRUE;
      a->forcerange[0] = -forceRange;
      a->forcerange[1] = forceRange;
    }
  }

  PhysicsModel* compile() {
    mjModel* m = mj_compile(spec_, nullptr);
    if (!m) {
      // Surface the real MuJoCo error string before aborting so the JS
      // console isn't left with just "mj_compile failed". The JS glue
      // treats mju_error as a fatal exit which tears down the wasm before
      // mjs_getError could be retrieved from JS afterwards.
      const char* err = mjs_getError(spec_);
      if (err && *err) {
        lg_console_error(err);
      }
      mju_error("mj_compile failed");
    }
    return new PhysicsModel(m);
  }

 private:
  mjSpec* spec_;
  bool owned_;
};

// Body helpers — thin wrappers so we can chain calls from JS.

mjsBody* addBody(mjsBody* parent, const std::string& name) {
  mjsBody* b = mjs_addBody(parent, nullptr);
  if (!b) {
    mju_error("mjs_addBody failed");
  }
  mjs_setName(b->element, name.c_str());
  return b;
}

void setBodyPos(mjsBody* body, double x, double y, double z) {
  body->pos[0] = x;
  body->pos[1] = y;
  body->pos[2] = z;
}

void setBodyQuat(mjsBody* body, double w, double x, double y, double z) {
  body->quat[0] = w;
  body->quat[1] = x;
  body->quat[2] = y;
  body->quat[3] = z;
}

void setBodyMass(mjsBody* body, double mass) {
  body->mass = mass;
}

// Mark a body as a mocap body (kinematically driven via data.mocap_pos/quat
// rather than a free joint). MuJoCo mocap bodies must have no joints.
void setBodyMocap(mjsBody* body, bool mocap) {
  body->mocap = mocap ? 1 : 0;
}

// Inertial-frame position (CoM in body frame). When set together with
// non-zero body mass and inertia, the MuJoCo compiler uses these values
// directly instead of deriving inertia from geom volumes.
void setBodyIPos(mjsBody* body, double x, double y, double z) {
  body->ipos[0] = x;
  body->ipos[1] = y;
  body->ipos[2] = z;
}

// Inertial-frame orientation. Identity default is (1,0,0,0).
void setBodyIQuat(mjsBody* body, double w, double x, double y, double z) {
  body->iquat[0] = w;
  body->iquat[1] = x;
  body->iquat[2] = y;
  body->iquat[3] = z;
}

// Diagonal inertia tensor in the inertial frame (kg·m^2 per axis).
void setBodyDiagInertia(mjsBody* body, double ixx, double iyy, double izz) {
  body->inertia[0] = ixx;
  body->inertia[1] = iyy;
  body->inertia[2] = izz;
}

// Geom helpers

mjsGeom* addGeom(mjsBody* body, const std::string& name) {
  mjsGeom* g = mjs_addGeom(body, nullptr);
  if (!g) {
    mju_error("mjs_addGeom failed");
  }
  if (!name.empty()) {
    mjs_setName(g->element, name.c_str());
  }
  return g;
}

void setGeomType(mjsGeom* g, int type) {
  g->type = static_cast<mjtGeom>(type);
}

void setGeomSize(mjsGeom* g, double s0, double s1, double s2) {
  g->size[0] = s0;
  g->size[1] = s1;
  g->size[2] = s2;
}

void setGeomPos(mjsGeom* g, double x, double y, double z) {
  g->pos[0] = x;
  g->pos[1] = y;
  g->pos[2] = z;
}

void setGeomQuat(mjsGeom* g, double w, double x, double y, double z) {
  g->quat[0] = w;
  g->quat[1] = x;
  g->quat[2] = y;
  g->quat[3] = z;
}

void setGeomMass(mjsGeom* g, double mass) {
  g->mass = mass;
}

void setGeomRGBA(mjsGeom* g, float r, float ga, float b, float a) {
  g->rgba[0] = r;
  g->rgba[1] = ga;
  g->rgba[2] = b;
  g->rgba[3] = a;
}

void setGeomFriction(mjsGeom* g, double slide, double roll, double spin) {
  g->friction[0] = slide;
  g->friction[1] = roll;
  g->friction[2] = spin;
}

void setGeomConType(mjsGeom* g, int contype) { g->contype = contype; }
void setGeomConAffinity(mjsGeom* g, int conaffinity) { g->conaffinity = conaffinity; }
void setGeomCondim(mjsGeom* g, int condim) { g->condim = condim; }

// Bind a `<geom type="mesh">` to the named MjsMesh asset (the same
// name passed to addMesh). MuJoCo computes a convex hull from the
// referenced mesh's user-supplied vertices at compile time.
void setGeomMeshName(mjsGeom* g, const std::string& name) {
  mjs_setString(g->meshname, name.c_str());
}

// Mesh helpers — procedural mesh assets for mesh-typed colliders
// authored from JS-side data (USD mesh points/indices).
// `addMesh` itself lives on SpecWrapper (above) — see the comment
// there for the embind-marshalling rationale.

// Populate the mesh's user vertex buffer from a flat (x,y,z,…) array
// living in the WASM heap. `ptr` is the byte offset of a Float32Array
// view's underlying buffer; `n` is the element count. JS side
// allocates via `Module._malloc`, writes via a typed-array view over
// `Module.HEAPF32.buffer`, calls this, then `Module._free`. We pass
// raw heap pointers rather than `val` (which trips an LTO/closure
// funcref crash on the physics-only build) and rather than
// `register_vector<float>` (the per-element marshalling cost is
// prohibitive for meshes with thousands of vertices).
void setMeshUserVertPtr(mjsMesh* mesh, uintptr_t ptr, int n) {
  const float* data = reinterpret_cast<const float*>(ptr);
  mjs_setFloat(mesh->uservert, data, n);
}

void setMeshUserFacePtr(mjsMesh* mesh, uintptr_t ptr, int n) {
  const int* data = reinterpret_cast<const int*>(ptr);
  mjs_setInt(mesh->userface, data, n);
}

// Optional cap on the convex-hull vertex count MuJoCo computes
// (default 0 = no limit). Useful for pathological high-poly meshes
// where the full convex hull would be too expensive to maintain
// each step.
void setMeshMaxHullVert(mjsMesh* mesh, int n) {
  mesh->maxhullvert = n;
}

// Joint helpers

mjsJoint* addJoint(mjsBody* body, const std::string& name) {
  mjsJoint* j = mjs_addJoint(body, nullptr);
  if (!j) {
    mju_error("mjs_addJoint failed");
  }
  if (!name.empty()) {
    mjs_setName(j->element, name.c_str());
  }
  return j;
}

mjsJoint* addFreeJoint(mjsBody* body) {
  mjsJoint* j = mjs_addFreeJoint(body);
  if (!j) {
    mju_error("mjs_addFreeJoint failed");
  }
  return j;
}

void setJointType(mjsJoint* j, int type) {
  j->type = static_cast<mjtJoint>(type);
}

void setJointAxis(mjsJoint* j, double x, double y, double z) {
  j->axis[0] = x;
  j->axis[1] = y;
  j->axis[2] = z;
}

void setJointRange(mjsJoint* j, double lo, double hi) {
  j->limited = 1;
  j->range[0] = lo;
  j->range[1] = hi;
}

void setJointDamping(mjsJoint* j, double d) {
  j->damping[0] = d;
}

// Joint stiffness: degree-0 coefficient of the polynomial spring law.
// MuJoCo applies a restoring torque/force toward j->springref with this
// constant. PhysX/Newton authoring is a scalar; we map directly to the
// first polynomial coefficient (the others stay zero, matching MJCF
// `<joint stiffness=...>`).
void setJointStiffness(mjsJoint* j, double k) {
  j->stiffness[0] = k;
}

// Joint armature: rotational inertia added on top of the body inertia
// projected through the joint axis. Required to make MuJoCo behave like
// PhysX/Newton at small dt for high-mass-ratio chains.
void setJointArmature(mjsJoint* j, double a) {
  j->armature = a;
}

// Joint friction loss: coulomb-friction torque/force opposing motion.
void setJointFrictionLoss(mjsJoint* j, double f) {
  j->frictionloss = f;
}

// Light helpers

mjsLight* addLight(mjsBody* body, const std::string& name) {
  mjsLight* l = mjs_addLight(body, nullptr);
  if (!l) {
    mju_error("mjs_addLight failed");
  }
  if (!name.empty()) {
    mjs_setName(l->element, name.c_str());
  }
  return l;
}

void setLightPos(mjsLight* l, double x, double y, double z) {
  l->pos[0] = x;
  l->pos[1] = y;
  l->pos[2] = z;
}

void setLightDir(mjsLight* l, double x, double y, double z) {
  l->dir[0] = x;
  l->dir[1] = y;
  l->dir[2] = z;
}

void setLightDiffuse(mjsLight* l, float r, float g, float b) {
  l->diffuse[0] = r;
  l->diffuse[1] = g;
  l->diffuse[2] = b;
}

// Simulation wrappers

void mj_forward_wrapper(const PhysicsModel& model, PhysicsData& data) {
  mj_forward(model.get(), data.get());
}

void mj_step_wrapper(const PhysicsModel& model, PhysicsData& data) {
  mj_step(model.get(), data.get());
}

void mj_step_n_wrapper(const PhysicsModel& model, PhysicsData& data, int nstep) {
  for (int i = 0; i < nstep; ++i) {
    mj_step(model.get(), data.get());
  }
}

void mj_resetData_wrapper(const PhysicsModel& model, PhysicsData& data) {
  mj_resetData(model.get(), data.get());
}

}  // namespace

EMSCRIPTEN_BINDINGS(mujoco_physics_wasm) {
  // --- PhysicsModel ---
  emscripten::class_<PhysicsModel>("PhysicsModel")
      .function("nq", &PhysicsModel::nq)
      .function("nv", &PhysicsModel::nv)
      .function("na", &PhysicsModel::na)
      .function("nu", &PhysicsModel::nu)
      .function("nbody", &PhysicsModel::nbody)
      .function("ngeom", &PhysicsModel::ngeom)
      .function("njnt", &PhysicsModel::njnt)
      .function("neq", &PhysicsModel::neq)
      .function("ntendon", &PhysicsModel::ntendon)
      .function("jnt_stiffness", &PhysicsModel::jnt_stiffness)
      .function("jnt_range", &PhysicsModel::jnt_range)
      .function("dof_damping", &PhysicsModel::dof_damping)
      .function("dof_armature", &PhysicsModel::dof_armature)
      .function("dof_frictionloss", &PhysicsModel::dof_frictionloss)
      .function("nsensordata", &PhysicsModel::nsensordata)
      .function("timestep", &PhysicsModel::timestep)
      .function("setTimestep", &PhysicsModel::setTimestep)
      .function("gravity", &PhysicsModel::gravity)
      .function("o_solref", &PhysicsModel::o_solref)
      .function("o_solimp", &PhysicsModel::o_solimp)
      .function("o_margin", &PhysicsModel::o_margin)
      .function("setOMargin", &PhysicsModel::setOMargin)
      .function("enableflags", &PhysicsModel::enableflags)
      .function("setEnableFlags", &PhysicsModel::setEnableFlags);

  // Enable bit that makes the solver use opt.o_solref/o_solimp/o_margin for
  // all contacts (runtime contact-compliance override).
  emscripten::constant("mjENBL_OVERRIDE", static_cast<int>(mjENBL_OVERRIDE));

  // --- PhysicsData ---
  emscripten::class_<PhysicsData>("PhysicsData")
      .constructor<const PhysicsModel&>()
      .function("time", &PhysicsData::time)
      .function("qpos", &PhysicsData::qpos)
      .function("qvel", &PhysicsData::qvel)
      .function("act", &PhysicsData::act)
      .function("ctrl", &PhysicsData::ctrl)
      .function("sensordata", &PhysicsData::sensordata)
      .function("xpos", &PhysicsData::xpos)
      .function("xquat", &PhysicsData::xquat)
      .function("geom_xpos", &PhysicsData::geom_xpos)
      .function("geom_xmat", &PhysicsData::geom_xmat)
      .function("mocap_pos", &PhysicsData::mocap_pos)
      .function("mocap_quat", &PhysicsData::mocap_quat)
      .function("xfrc_applied", &PhysicsData::xfrc_applied)
      .function("qfrc_applied", &PhysicsData::qfrc_applied)
      .function("qfrc_bias", &PhysicsData::qfrc_bias)
      .function("qM_diag", &PhysicsData::qM_diag)
      .function("ncon", &PhysicsData::ncon)
      .function("contacts", &PhysicsData::contacts);

  // --- Load from binary ---
  emscripten::function("loadModelFromArrayBuffer", &LoadModelFromArrayBuffer,
                       emscripten::allow_raw_pointers());

  // --- Spec (procedural model building) ---
  emscripten::class_<SpecWrapper>("MjSpec")
      .constructor<>()
      .function("setModelName", &SpecWrapper::setModelName)
      .function("setIntegrator", &SpecWrapper::setIntegrator)
      .function("setTimestep", &SpecWrapper::setTimestep)
      .function("getTimestep", &SpecWrapper::getTimestep)
      .function("setGravity", &SpecWrapper::setGravity)
      .function("worldBody", &SpecWrapper::worldBody, emscripten::allow_raw_pointers())
      .function("addMesh", &SpecWrapper::addMesh, emscripten::allow_raw_pointers())
      .function("addExclude", &SpecWrapper::addExclude)
      .function("addPositionActuator", &SpecWrapper::addPositionActuator)
      .function("compile", &SpecWrapper::compile, emscripten::allow_raw_pointers());

  // --- Body ---
  emscripten::class_<mjsBody>("MjsBody")
      .class_function("add", &addBody, emscripten::allow_raw_pointers())
      .class_function("setPos", &setBodyPos, emscripten::allow_raw_pointers())
      .class_function("setQuat", &setBodyQuat, emscripten::allow_raw_pointers())
      .class_function("setMass", &setBodyMass, emscripten::allow_raw_pointers())
      .class_function("setIPos", &setBodyIPos, emscripten::allow_raw_pointers())
      .class_function("setIQuat", &setBodyIQuat, emscripten::allow_raw_pointers())
      .class_function("setDiagInertia", &setBodyDiagInertia, emscripten::allow_raw_pointers())
      .class_function("setMocap", &setBodyMocap, emscripten::allow_raw_pointers());

  // --- Geom ---
  emscripten::class_<mjsGeom>("MjsGeom")
      .class_function("add", &addGeom, emscripten::allow_raw_pointers())
      .class_function("setType", &setGeomType, emscripten::allow_raw_pointers())
      .class_function("setSize", &setGeomSize, emscripten::allow_raw_pointers())
      .class_function("setPos", &setGeomPos, emscripten::allow_raw_pointers())
      .class_function("setQuat", &setGeomQuat, emscripten::allow_raw_pointers())
      .class_function("setMass", &setGeomMass, emscripten::allow_raw_pointers())
      .class_function("setRGBA", &setGeomRGBA, emscripten::allow_raw_pointers())
      .class_function("setFriction", &setGeomFriction, emscripten::allow_raw_pointers())
      .class_function("setConType", &setGeomConType, emscripten::allow_raw_pointers())
      .class_function("setConAffinity", &setGeomConAffinity, emscripten::allow_raw_pointers())
      .class_function("setCondim", &setGeomCondim, emscripten::allow_raw_pointers())
      .class_function("setMeshName", &setGeomMeshName, emscripten::allow_raw_pointers());

  // --- Mesh asset (for `<geom type="mesh">`). MuJoCo computes the
  //     convex hull from the user-supplied vertices at compile time;
  //     the resulting geom uses true mesh-vs-* collision (per the
  //     mjGEOM_MESH branch of mj_collision), no AABB / OBB proxy.
  //     The constructor lives on MjSpec (`spec.addMesh(name)`) since
  //     a free function taking SpecWrapper& as parameter 0 trips up
  //     embind's parameter-type registration. ---
  emscripten::class_<mjsMesh>("MjsMesh")
      .class_function("setUserVertPtr", &setMeshUserVertPtr,
                      emscripten::allow_raw_pointers())
      .class_function("setUserFacePtr", &setMeshUserFacePtr,
                      emscripten::allow_raw_pointers())
      .class_function("setMaxHullVert", &setMeshMaxHullVert,
                      emscripten::allow_raw_pointers());

  // --- Joint ---
  emscripten::class_<mjsJoint>("MjsJoint")
      .class_function("add", &addJoint, emscripten::allow_raw_pointers())
      .class_function("addFree", &addFreeJoint, emscripten::allow_raw_pointers())
      .class_function("setType", &setJointType, emscripten::allow_raw_pointers())
      .class_function("setAxis", &setJointAxis, emscripten::allow_raw_pointers())
      .class_function("setRange", &setJointRange, emscripten::allow_raw_pointers())
      .class_function("setDamping", &setJointDamping, emscripten::allow_raw_pointers())
      .class_function("setStiffness", &setJointStiffness, emscripten::allow_raw_pointers())
      .class_function("setArmature", &setJointArmature, emscripten::allow_raw_pointers())
      .class_function("setFrictionLoss", &setJointFrictionLoss, emscripten::allow_raw_pointers());

  // --- Light ---
  emscripten::class_<mjsLight>("MjsLight")
      .class_function("add", &addLight, emscripten::allow_raw_pointers())
      .class_function("setPos", &setLightPos, emscripten::allow_raw_pointers())
      .class_function("setDir", &setLightDir, emscripten::allow_raw_pointers())
      .class_function("setDiffuse", &setLightDiffuse, emscripten::allow_raw_pointers());

  // --- Geom type constants ---
  emscripten::constant("GEOM_PLANE", static_cast<int>(mjGEOM_PLANE));
  emscripten::constant("GEOM_HFIELD", static_cast<int>(mjGEOM_HFIELD));
  emscripten::constant("GEOM_SPHERE", static_cast<int>(mjGEOM_SPHERE));
  emscripten::constant("GEOM_CAPSULE", static_cast<int>(mjGEOM_CAPSULE));
  emscripten::constant("GEOM_ELLIPSOID", static_cast<int>(mjGEOM_ELLIPSOID));
  emscripten::constant("GEOM_CYLINDER", static_cast<int>(mjGEOM_CYLINDER));
  emscripten::constant("GEOM_BOX", static_cast<int>(mjGEOM_BOX));
  emscripten::constant("GEOM_MESH", static_cast<int>(mjGEOM_MESH));

  // --- Joint type constants ---
  emscripten::constant("JNT_FREE", static_cast<int>(mjJNT_FREE));
  emscripten::constant("JNT_BALL", static_cast<int>(mjJNT_BALL));
  emscripten::constant("JNT_SLIDE", static_cast<int>(mjJNT_SLIDE));
  emscripten::constant("JNT_HINGE", static_cast<int>(mjJNT_HINGE));

  // --- Simulation functions ---
  emscripten::function("mj_forward", &mj_forward_wrapper);
  emscripten::function("mj_step", &mj_step_wrapper);
  emscripten::function("mj_stepN", &mj_step_n_wrapper);
  emscripten::function("mj_resetData", &mj_resetData_wrapper);
}

#endif  // __EMSCRIPTEN__
