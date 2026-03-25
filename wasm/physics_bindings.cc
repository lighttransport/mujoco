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

#include "mujoco/mujoco.h"

namespace {

using emscripten::typed_memory_view;
using emscripten::val;

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
  int nsensordata() const { return model_->nsensordata; }
  double timestep() const { return model_->opt.timestep; }

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

 private:
  mjModel* model_;
  mjData* data_ = nullptr;
};

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

  // Use Emscripten's HEAPU8.set for an efficient bulk copy from JS to WASM heap.
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
  emscripten::class_<PhysicsModel>("PhysicsModel")
      .function("nq", &PhysicsModel::nq)
      .function("nv", &PhysicsModel::nv)
      .function("na", &PhysicsModel::na)
      .function("nu", &PhysicsModel::nu)
      .function("nsensordata", &PhysicsModel::nsensordata)
      .function("timestep", &PhysicsModel::timestep);

  emscripten::class_<PhysicsData>("PhysicsData")
      .constructor<const PhysicsModel&>()
      .function("time", &PhysicsData::time)
      .function("qpos", &PhysicsData::qpos)
      .function("qvel", &PhysicsData::qvel)
      .function("act", &PhysicsData::act)
      .function("ctrl", &PhysicsData::ctrl)
      .function("sensordata", &PhysicsData::sensordata);

  emscripten::function("loadModelFromArrayBuffer", &LoadModelFromArrayBuffer,
                       emscripten::allow_raw_pointers());
  emscripten::function("mj_forward", &mj_forward_wrapper);
  emscripten::function("mj_step", &mj_step_wrapper);
  emscripten::function("mj_stepN", &mj_step_n_wrapper);
  emscripten::function("mj_resetData", &mj_resetData_wrapper);
}

#endif  // __EMSCRIPTEN__
