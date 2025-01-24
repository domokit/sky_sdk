// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/dart_message_handler.h"

#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/dart/runtime/include/dart_native_api.h"
#include "third_party/dart/runtime/include/dart_tools_api.h"
#include "tonic/common/macros.h"
#include "tonic/dart_state.h"
#include "tonic/logging/dart_error.h"

namespace tonic {

DartMessageHandler::DartMessageHandler()
    : handled_first_message_(false),
      isolate_exited_(false),
      isolate_had_uncaught_exception_error_(false),
      isolate_had_fatal_error_(false),
      isolate_last_error_(kNoError),
      task_dispatcher_(nullptr) {}

DartMessageHandler::~DartMessageHandler() {
  task_dispatcher_ = nullptr;
}

void DartMessageHandler::Initialize(TaskDispatcher dispatcher) {
  // Only can be called once.
  TONIC_CHECK(!task_dispatcher_ && dispatcher);
  task_dispatcher_ = dispatcher;
  Dart_SetMessageNotifyCallback(MessageNotifyCallback);
}

void DartMessageHandler::OnMessage(DartState* dart_state) {
  auto task_dispatcher_ = dart_state->message_handler().task_dispatcher_;

  // Schedule a task to run on the message loop thread.
  auto weak_dart_state = dart_state->GetWeakPtr();
  task_dispatcher_([weak_dart_state]() {
    if (auto dart_state = weak_dart_state.lock()) {
      if (!dart_state->IsShuttingDown()) {
        dart_state->message_handler().OnHandleMessage(dart_state.get());
      }
    }
  });
}

void DartMessageHandler::UnhandledError(Dart_Handle error) {
  TONIC_DCHECK(Dart_CurrentIsolate());
  TONIC_DCHECK(Dart_IsError(error));

  isolate_last_error_ = GetErrorHandleType(error);
  // Remember that we had an uncaught exception error.
  isolate_had_uncaught_exception_error_ = true;
  if (Dart_IsFatalError(error)) {
    isolate_had_fatal_error_ = true;
    // Stop handling messages.
    Dart_SetMessageNotifyCallback(nullptr);
    // Shut down the isolate.
    Dart_ShutdownIsolate();
  }
}

void DartMessageHandler::OnHandleMessage(DartState* dart_state) {
  if (isolate_had_fatal_error_) {
    // Don't handle any more messages.
    return;
  }

  DartIsolateScope scope(dart_state->isolate());
  DartApiScope dart_api_scope;
  Dart_Handle result = Dart_Null();
  bool error = false;

  // On the first message, check if we should pause on isolate start.
  if (!handled_first_message()) {
    set_handled_first_message(true);
    if (Dart_ShouldPauseOnStart()) {
      // Mark that we are paused on isolate start.
      Dart_SetPausedOnStart(true);
    }
  }

  if (Dart_IsPausedOnStart()) {
    // We are paused on isolate start. Only handle service messages until we are
    // requested to resume.
    if (Dart_HasServiceMessages()) {
      bool resume = Dart_HandleServiceMessages();
      if (!resume) {
        return;
      }
      Dart_SetPausedOnStart(false);
      // We've resumed, handle normal messages that are in the queue.
      result = Dart_HandleMessage();
      error = CheckAndHandleError(result);
      dart_state->MessageEpilogue(result);
      if (!Dart_CurrentIsolate()) {
        isolate_exited_ = true;
        return;
      }
    }
  } else if (Dart_IsPausedOnExit()) {
    // We are paused on isolate exit. Only handle service messages until we are
    // requested to resume.
    if (Dart_HasServiceMessages()) {
      bool resume = Dart_HandleServiceMessages();
      if (!resume) {
        return;
      }
      Dart_SetPausedOnExit(false);
    }
  } else {
    // We are processing messages normally.
    result = Dart_HandleMessage();
    // If the Dart program has set a return code, then it is intending to shut
    // down by way of a fatal error, and so there is no need to emit a log
    // message.
    if (dart_state->has_set_return_code() && Dart_IsError(result) &&
        Dart_IsFatalError(result)) {
      error = true;
    } else {
      error = CheckAndHandleError(result);
    }
    dart_state->MessageEpilogue(result);
    if (!Dart_CurrentIsolate()) {
      isolate_exited_ = true;
      return;
    }
  }

  if (error) {
    UnhandledError(result);
  } else if (!Dart_HasLivePorts()) {
    // The isolate has no live ports and would like to exit.
    if (!Dart_IsPausedOnExit() && Dart_ShouldPauseOnExit()) {
      // Mark that we are paused on exit.
      Dart_SetPausedOnExit(true);
    } else {
      isolate_exited_ = true;
    }
  }
}

void DartMessageHandler::MessageNotifyCallback(Dart_Isolate dest_isolate) {
  auto dart_state = DartState::From(dest_isolate);
  TONIC_CHECK(dart_state);
  dart_state->message_handler().OnMessage(dart_state);
}

}  // namespace tonic
