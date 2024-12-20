// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_DART_STATE_H_
#define LIB_TONIC_DART_STATE_H_

#include <atomic>
#include <functional>
#include <memory>

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/common/macros.h"
#include "tonic/dart_persistent_value.h"
#include "tonic/scopes/dart_api_scope.h"
#include "tonic/scopes/dart_isolate_scope.h"

namespace tonic {
class DartClassLibrary;
class DartMessageHandler;
class FileLoader;

// DartState represents the state associated with a given Dart isolate. The
// lifetime of this object is controlled by the DartVM. If you want to hold a
// reference to a DartState instance, please hold a std::weak_ptr<DartState>.
//
// DartState is analogous to gin::PerIsolateData and JSC::ExecState.
class DartState : public std::enable_shared_from_this<DartState> {
 public:
  class Scope {
   public:
    explicit Scope(DartState* dart_state);
    explicit Scope(std::shared_ptr<DartState> dart_state);
    ~Scope();

   private:
    DartIsolateScope scope_;
    DartApiScope api_scope_;
  };

  explicit DartState(
      int dirfd = -1,
      std::function<void(Dart_Handle)> message_epilogue = nullptr);
  virtual ~DartState();

  static DartState* From(Dart_Isolate isolate);
  static DartState* Current();

  std::weak_ptr<DartState> GetWeakPtr();

  Dart_Isolate isolate() { return isolate_; }
  void SetIsolate(Dart_Isolate isolate);

  // TODO(https://github.com/flutter/flutter/issues/50997): Work around until we
  // drop the need for Dart_New in tonic.
  Dart_PersistentHandle private_constructor_name() {
    return private_constructor_name_.Get();
  }

  DartClassLibrary& class_library() { return *class_library_; }
  DartMessageHandler& message_handler() { return *message_handler_; }
  FileLoader& file_loader() { return *file_loader_; }

  void MessageEpilogue(Dart_Handle message_result) {
    if (message_epilogue_) {
      message_epilogue_(message_result);
    }
  }
  void SetReturnCode(uint32_t return_code);
  void SetReturnCodeCallback(std::function<void(uint32_t)> callback);
  bool has_set_return_code() const { return has_set_return_code_; }

  void SetIsShuttingDown() { is_shutting_down_ = true; }
  bool IsShuttingDown() { return is_shutting_down_; }

  virtual void DidSetIsolate();

  static Dart_Handle HandleLibraryTag(Dart_LibraryTag tag,
                                      Dart_Handle library,
                                      Dart_Handle url);

 protected:
  Dart_Isolate isolate() const { return isolate_; }

 private:
  Dart_Isolate isolate_;
  DartPersistentValue private_constructor_name_;
  std::unique_ptr<DartClassLibrary> class_library_;
  std::unique_ptr<DartMessageHandler> message_handler_;
  std::unique_ptr<FileLoader> file_loader_;
  std::function<void(Dart_Handle)> message_epilogue_;
  std::function<void(uint32_t)> set_return_code_callback_;
  bool has_set_return_code_;
  std::atomic<bool> is_shutting_down_;

 protected:
  TONIC_DISALLOW_COPY_AND_ASSIGN(DartState);
};

}  // namespace tonic

#endif  // LIB_TONIC_DART_STATE_H_
