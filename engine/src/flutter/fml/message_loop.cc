// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/message_loop.h"

#include <memory>
#include <utility>

#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/fml/message_loop_impl.h"
#include "flutter/fml/task_runner.h"

namespace fml {

static thread_local std::unique_ptr<MessageLoop> tls_message_loop;

MessageLoop& MessageLoop::GetCurrent() {
  auto* loop = tls_message_loop.get();
  FML_CHECK(loop != nullptr)
      << "MessageLoop::EnsureInitializedForCurrentThread was not called on "
         "this thread prior to message loop use.";
  return *loop;
}

void MessageLoop::EnsureInitializedForCurrentThread() {
  if (tls_message_loop.get() != nullptr) {
    // Already initialized.
    return;
  }
  tls_message_loop.reset(new MessageLoop());
}

bool MessageLoop::IsInitializedForCurrentThread() {
  return tls_message_loop.get() != nullptr;
}

MessageLoop::MessageLoop()
    : loop_(MessageLoopImpl::Create()),
      task_runner_(fml::MakeRefCounted<fml::TaskRunner>(loop_)) {
  FML_CHECK(loop_);
  FML_CHECK(task_runner_);
}

MessageLoop::~MessageLoop() = default;

void MessageLoop::Run() {
  loop_->DoRun();
}

void MessageLoop::Terminate() {
  loop_->DoTerminate();
}

fml::RefPtr<fml::TaskRunner> MessageLoop::GetTaskRunner() const {
  return task_runner_;
}

fml::RefPtr<MessageLoopImpl> MessageLoop::GetLoopImpl() const {
  return loop_;
}

void MessageLoop::AddTaskObserver(intptr_t key, const fml::closure& callback) {
  loop_->AddTaskObserver(key, callback);
}

void MessageLoop::RemoveTaskObserver(intptr_t key) {
  loop_->RemoveTaskObserver(key);
}

void MessageLoop::RunExpiredTasksNow() {
  loop_->RunExpiredTasksNow();
}

TaskQueueId MessageLoop::GetCurrentTaskQueueId() {
  auto* loop = tls_message_loop.get();
  FML_CHECK(loop != nullptr)
      << "MessageLoop::EnsureInitializedForCurrentThread was not called on "
         "this thread prior to message loop use.";
  return loop->GetLoopImpl()->GetTaskQueueId();
}

}  // namespace fml
