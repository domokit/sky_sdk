// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/reactor_gles.h"

#include <algorithm>

#include "flutter/fml/trace_event.h"
#include "fml/closure.h"
#include "fml/logging.h"
#include "impeller/base/validation.h"

namespace impeller {

// static
std::optional<ReactorGLES::GLStorage> ReactorGLES::CreateGLHandle(
    const ProcTableGLES& gl,
    HandleType type) {
  GLStorage handle = GLStorage{.handle = GL_NONE};
  switch (type) {
    case HandleType::kUnknown:
      return std::nullopt;
    case HandleType::kTexture:
      gl.GenTextures(1u, &handle.handle);
      return handle;
    case HandleType::kBuffer:
      gl.GenBuffers(1u, &handle.handle);
      return handle;
    case HandleType::kProgram:
      return GLStorage{.handle = gl.CreateProgram()};
    case HandleType::kRenderBuffer:
      gl.GenRenderbuffers(1u, &handle.handle);
      return handle;
    case HandleType::kFrameBuffer:
      gl.GenFramebuffers(1u, &handle.handle);
      return handle;
    case HandleType::kFence:
      return GLStorage{.sync = gl.FenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0)};
  }
  return std::nullopt;
}

// static
bool ReactorGLES::CollectGLHandle(const ProcTableGLES& gl,
                                  HandleType type,
                                  ReactorGLES::GLStorage handle) {
  switch (type) {
    case HandleType::kUnknown:
      return false;
    case HandleType::kTexture:
      gl.DeleteTextures(1u, &handle.handle);
      return true;
    case HandleType::kBuffer:
      gl.DeleteBuffers(1u, &handle.handle);
      return true;
    case HandleType::kProgram:
      gl.DeleteProgram(handle.handle);
      return true;
    case HandleType::kRenderBuffer:
      gl.DeleteRenderbuffers(1u, &handle.handle);
      return true;
    case HandleType::kFrameBuffer:
      gl.DeleteFramebuffers(1u, &handle.handle);
      return true;
    case HandleType::kFence:
      gl.DeleteSync(handle.sync);
      break;
  }
  return false;
}

ReactorGLES::ReactorGLES(std::unique_ptr<ProcTableGLES> gl)
    : proc_table_(std::move(gl)) {
  if (!proc_table_ || !proc_table_->IsValid()) {
    VALIDATION_LOG << "Proc table was invalid.";
    return;
  }
  can_set_debug_labels_ = proc_table_->GetDescription()->HasDebugExtension();
  is_valid_ = true;
}

ReactorGLES::~ReactorGLES() {
  if (CanReactOnCurrentThread()) {
    for (auto& handle : handles_) {
      if (handle.second.name.has_value()) {
        CollectGLHandle(*proc_table_, handle.first.GetType(),
                        handle.second.name.value());
      }
    }
    proc_table_->Flush();
  }
}

bool ReactorGLES::IsValid() const {
  return is_valid_;
}

bool ReactorGLES::CanSetDebugLabels() const {
  return can_set_debug_labels_;
}

ReactorGLES::WorkerID ReactorGLES::AddWorker(std::weak_ptr<Worker> worker) {
  Lock lock(workers_mutex_);
  auto id = WorkerID{};
  workers_[id] = std::move(worker);
  return id;
}

bool ReactorGLES::RemoveWorker(WorkerID worker) {
  Lock lock(workers_mutex_);
  return workers_.erase(worker) == 1;
}

bool ReactorGLES::HasPendingOperations() const {
  auto thread_id = std::this_thread::get_id();
  Lock ops_lock(ops_mutex_);
  auto it = ops_.find(thread_id);
  return it != ops_.end() ? !it->second.empty() : false;
}

const ProcTableGLES& ReactorGLES::GetProcTable() const {
  FML_DCHECK(IsValid());
  return *proc_table_;
}

std::optional<ReactorGLES::GLStorage> ReactorGLES::GetHandle(
    const HandleGLES& handle) const {
  if (handle.untracked_id_.has_value()) {
    return ReactorGLES::GLStorage{.integer = handle.untracked_id_.value()};
  }

  ReaderLock handles_lock(handles_mutex_);
  if (auto found = handles_.find(handle); found != handles_.end()) {
    if (found->second.pending_collection) {
      VALIDATION_LOG
          << "Attempted to acquire a handle that was pending collection.";
      return std::nullopt;
    }
    std::optional<ReactorGLES::GLStorage> name = found->second.name;
    if (!name.has_value()) {
      VALIDATION_LOG << "Attempt to acquire a handle outside of an operation.";
      return std::nullopt;
    }
    return name;
  }
  VALIDATION_LOG << "Attempted to acquire an invalid GL handle.";
  return std::nullopt;
}

std::optional<GLuint> ReactorGLES::GetGLHandle(const HandleGLES& handle) const {
  if (handle.GetType() == HandleType::kFence) {
    return std::nullopt;
  }
  std::optional<ReactorGLES::GLStorage> gl_handle = GetHandle(handle);
  if (gl_handle.has_value()) {
    return gl_handle->handle;
  }
  return std::nullopt;
}

std::optional<GLsync> ReactorGLES::GetGLFence(const HandleGLES& handle) const {
  if (handle.GetType() != HandleType::kFence) {
    return std::nullopt;
  }
  std::optional<ReactorGLES::GLStorage> gl_handle = GetHandle(handle);
  if (gl_handle.has_value()) {
    return gl_handle->sync;
  }
  return std::nullopt;
}

bool ReactorGLES::AddOperation(Operation operation, bool defer) {
  if (!operation) {
    return false;
  }
  auto thread_id = std::this_thread::get_id();
  {
    Lock ops_lock(ops_mutex_);
    ops_[thread_id].emplace_back(std::move(operation));
  }
  // Attempt a reaction if able but it is not an error if this isn't possible.
  if (!defer) {
    [[maybe_unused]] auto result = React();
  }
  return true;
}

bool ReactorGLES::RegisterCleanupCallback(const HandleGLES& handle,
                                          const fml::closure& callback) {
  if (handle.IsDead()) {
    return false;
  }
  FML_DCHECK(!handle.untracked_id_.has_value());
  WriterLock handles_lock(handles_mutex_);
  if (auto found = handles_.find(handle); found != handles_.end()) {
    found->second.callback = fml::ScopedCleanupClosure(callback);
    return true;
  }
  return false;
}

HandleGLES ReactorGLES::CreateUntrackedHandle(HandleType type) {
  FML_DCHECK(CanReactOnCurrentThread());
  auto new_handle = HandleGLES::Create(type);
  std::optional<ReactorGLES::GLStorage> gl_handle =
      CreateGLHandle(GetProcTable(), type);
  if (gl_handle.has_value()) {
    new_handle.untracked_id_ = gl_handle.value().integer;
  }
  return new_handle;
}

HandleGLES ReactorGLES::CreateHandle(HandleType type, GLuint external_handle) {
  if (type == HandleType::kUnknown) {
    return HandleGLES::DeadHandle();
  }
  auto new_handle = HandleGLES::Create(type);
  if (new_handle.IsDead()) {
    return HandleGLES::DeadHandle();
  }

  std::optional<ReactorGLES::GLStorage> gl_handle;
  if (external_handle != GL_NONE) {
    gl_handle = ReactorGLES::GLStorage{.handle = external_handle};
  } else if (CanReactOnCurrentThread()) {
    gl_handle = CreateGLHandle(GetProcTable(), type);
  }

  WriterLock handles_lock(handles_mutex_);
  handles_[new_handle] = LiveHandle{gl_handle};
  return new_handle;
}

void ReactorGLES::CollectHandle(HandleGLES handle) {
  if (handle.untracked_id_.has_value()) {
    LiveHandle live_handle(GLStorage{.integer = handle.untracked_id_.value()});
    live_handle.pending_collection = true;
    WriterLock handles_lock(handles_mutex_);
    handles_[handle] = std::move(live_handle);
  } else {
    WriterLock handles_lock(handles_mutex_);
    if (auto found = handles_.find(handle); found != handles_.end()) {
      if (!found->second.pending_collection) {
        handles_to_collect_count_ += 1;
      }
      found->second.pending_collection = true;
    }
  }
}

bool ReactorGLES::React() {
  if (!CanReactOnCurrentThread()) {
    return false;
  }
  TRACE_EVENT0("impeller", "ReactorGLES::React");
  while (HasPendingOperations()) {
    if (!ReactOnce()) {
      return false;
    }
  }
  return true;
}

static DebugResourceType ToDebugResourceType(HandleType type) {
  switch (type) {
    case HandleType::kUnknown:
      FML_UNREACHABLE();
    case HandleType::kTexture:
      return DebugResourceType::kTexture;
    case HandleType::kBuffer:
      return DebugResourceType::kBuffer;
    case HandleType::kProgram:
      return DebugResourceType::kProgram;
    case HandleType::kRenderBuffer:
      return DebugResourceType::kRenderBuffer;
    case HandleType::kFrameBuffer:
      return DebugResourceType::kFrameBuffer;
    case HandleType::kFence:
      return DebugResourceType::kFence;
  }
  FML_UNREACHABLE();
}

bool ReactorGLES::ReactOnce() {
  if (!IsValid()) {
    return false;
  }
  TRACE_EVENT0("impeller", __FUNCTION__);
  return ConsolidateHandles() && FlushOps();
}

bool ReactorGLES::ConsolidateHandles() {
  TRACE_EVENT0("impeller", __FUNCTION__);
  const auto& gl = GetProcTable();
  std::vector<std::tuple<HandleGLES, std::optional<GLStorage>>>
      handles_to_delete;
  std::vector<std::tuple<DebugResourceType, GLint, std::string>>
      handles_to_name;
  {
    WriterLock handles_lock(handles_mutex_);
    handles_to_delete.reserve(handles_to_collect_count_);
    handles_to_collect_count_ = 0;
    for (auto& handle : handles_) {
      // Collect dead handles.
      if (handle.second.pending_collection) {
        handles_to_delete.emplace_back(
            std::make_tuple(handle.first, handle.second.name));
        continue;
      }
      // Create live handles.
      if (!handle.second.name.has_value()) {
        auto gl_handle = CreateGLHandle(gl, handle.first.GetType());
        if (!gl_handle) {
          VALIDATION_LOG << "Could not create GL handle.";
          return false;
        }
        handle.second.name = gl_handle;
      }
      // Set pending debug labels.
      if (handle.second.pending_debug_label.has_value() &&
          handle.first.GetType() != HandleType::kFence) {
        handles_to_name.emplace_back(std::make_tuple(
            ToDebugResourceType(handle.first.GetType()),
            handle.second.name.value().handle,
            std::move(handle.second.pending_debug_label.value())));
        handle.second.pending_debug_label = std::nullopt;
      }
    }
    for (const auto& handle_to_delete : handles_to_delete) {
      handles_.erase(std::get<0>(handle_to_delete));
    }
  }

  for (const auto& handle : handles_to_name) {
    gl.SetDebugLabel(std::get<0>(handle), std::get<1>(handle),
                     std::get<2>(handle));
  }
  for (const auto& handle : handles_to_delete) {
    const std::optional<GLStorage>& storage = std::get<1>(handle);
    // This could be false if the handle was created and collected without
    // use. We still need to get rid of map entry.
    if (storage.has_value()) {
      CollectGLHandle(gl, std::get<0>(handle).GetType(), storage.value());
    }
  }

  return true;
}

bool ReactorGLES::FlushOps() {
  TRACE_EVENT0("impeller", __FUNCTION__);

#ifdef IMPELLER_DEBUG
  // glDebugMessageControl sometimes must be called before glPushDebugGroup:
  // https://github.com/flutter/flutter/issues/135715#issuecomment-1740153506
  SetupDebugGroups();
#endif

  // Do NOT hold the ops or handles locks while performing operations in case
  // the ops enqueue more ops.
  decltype(ops_)::mapped_type ops;
  auto thread_id = std::this_thread::get_id();
  {
    Lock ops_lock(ops_mutex_);
    std::swap(ops_[thread_id], ops);
  }
  for (const auto& op : ops) {
    TRACE_EVENT0("impeller", "ReactorGLES::Operation");
    op(*this);
  }
  return true;
}

void ReactorGLES::SetupDebugGroups() {
  // Setup of a default active debug group: Filter everything in.
  if (proc_table_->DebugMessageControlKHR.IsAvailable()) {
    proc_table_->DebugMessageControlKHR(GL_DONT_CARE,  // source
                                        GL_DONT_CARE,  // type
                                        GL_DONT_CARE,  // severity
                                        0,             // count
                                        nullptr,       // ids
                                        GL_TRUE);      // enabled
  }
}

void ReactorGLES::SetDebugLabel(const HandleGLES& handle,
                                std::string_view label) {
  FML_DCHECK(handle.GetType() != HandleType::kFence);
  if (!can_set_debug_labels_) {
    return;
  }
  if (handle.IsDead()) {
    return;
  }
  if (handle.untracked_id_.has_value()) {
    FML_DCHECK(CanReactOnCurrentThread());
    const auto& gl = GetProcTable();
    gl.SetDebugLabel(ToDebugResourceType(handle.GetType()),
                     handle.untracked_id_.value(), label);
  } else {
    WriterLock handles_lock(handles_mutex_);
    if (auto found = handles_.find(handle); found != handles_.end()) {
      found->second.pending_debug_label = label;
    }
  }
}

bool ReactorGLES::CanReactOnCurrentThread() const {
  std::vector<WorkerID> dead_workers;
  Lock lock(workers_mutex_);
  for (const auto& worker : workers_) {
    auto worker_ptr = worker.second.lock();
    if (!worker_ptr) {
      dead_workers.push_back(worker.first);
      continue;
    }
    if (worker_ptr->CanReactorReactOnCurrentThreadNow(*this)) {
      return true;
    }
  }
  for (const auto& worker_id : dead_workers) {
    workers_.erase(worker_id);
  }
  return false;
}

}  // namespace impeller
