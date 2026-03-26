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
#include <emscripten/val.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <mujoco/mjspec.h>
#include "mujoco/mujoco.h"

namespace {

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
  int nsensordata() const { return model_->nsensordata; }
  double timestep() const { return model_->opt.timestep; }
  void setTimestep(double dt) { model_->opt.timestep = dt; }
  val gravity() const {
    return val(typed_memory_view(3, model_->opt.gravity));
  }

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

 private:
  mjModel* model_;
  mjData* data_ = nullptr;
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

  mjsBody* worldBody() {
    return mjs_findBody(spec_, "world");
  }

  PhysicsModel* compile() {
    mjModel* m = mj_compile(spec_, nullptr);
    if (!m) {
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
      .function("nsensordata", &PhysicsModel::nsensordata)
      .function("timestep", &PhysicsModel::timestep)
      .function("setTimestep", &PhysicsModel::setTimestep)
      .function("gravity", &PhysicsModel::gravity);

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
      .function("geom_xmat", &PhysicsData::geom_xmat);

  // --- Load from binary ---
  emscripten::function("loadModelFromArrayBuffer", &LoadModelFromArrayBuffer,
                       emscripten::allow_raw_pointers());

  // --- Spec (procedural model building) ---
  emscripten::class_<SpecWrapper>("MjSpec")
      .constructor<>()
      .function("setModelName", &SpecWrapper::setModelName)
      .function("setTimestep", &SpecWrapper::setTimestep)
      .function("getTimestep", &SpecWrapper::getTimestep)
      .function("setGravity", &SpecWrapper::setGravity)
      .function("worldBody", &SpecWrapper::worldBody, emscripten::allow_raw_pointers())
      .function("compile", &SpecWrapper::compile, emscripten::allow_raw_pointers());

  // --- Body ---
  emscripten::class_<mjsBody>("MjsBody")
      .class_function("add", &addBody, emscripten::allow_raw_pointers())
      .class_function("setPos", &setBodyPos, emscripten::allow_raw_pointers())
      .class_function("setQuat", &setBodyQuat, emscripten::allow_raw_pointers())
      .class_function("setMass", &setBodyMass, emscripten::allow_raw_pointers());

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
      .class_function("setCondim", &setGeomCondim, emscripten::allow_raw_pointers());

  // --- Joint ---
  emscripten::class_<mjsJoint>("MjsJoint")
      .class_function("add", &addJoint, emscripten::allow_raw_pointers())
      .class_function("addFree", &addFreeJoint, emscripten::allow_raw_pointers())
      .class_function("setType", &setJointType, emscripten::allow_raw_pointers())
      .class_function("setAxis", &setJointAxis, emscripten::allow_raw_pointers())
      .class_function("setRange", &setJointRange, emscripten::allow_raw_pointers())
      .class_function("setDamping", &setJointDamping, emscripten::allow_raw_pointers());

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
