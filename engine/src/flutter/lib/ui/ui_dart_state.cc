// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/ui_dart_state.h"

#include <iostream>
#include <utility>

#include "flutter/fml/message_loop.h"
#include "flutter/lib/ui/window/platform_configuration.h"
#include "flutter/lib/ui/window/platform_message.h"
#include "third_party/tonic/converter/dart_converter.h"
#include "third_party/tonic/dart_message_handler.h"

#if defined(FML_OS_ANDROID)
#include <android/log.h>
#elif defined(FML_OS_IOS)
extern "C" {
// Cannot import the syslog.h header directly because of macro collision.
extern void syslog(int, const char*, ...);
}
#endif

using tonic::ToDart;

namespace flutter {

UIDartState::Context::Context(const TaskRunners& task_runners)
    : task_runners(task_runners) {}

UIDartState::Context::Context(
    const TaskRunners& task_runners,
    fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> snapshot_delegate,
    fml::WeakPtr<IOManager> io_manager,
    fml::RefPtr<SkiaUnrefQueue> unref_queue,
    fml::WeakPtr<ImageDecoder> image_decoder,
    fml::WeakPtr<ImageGeneratorRegistry> image_generator_registry,
    std::string advisory_script_uri,
    std::string advisory_script_entrypoint,
    bool deterministic_rendering_enabled,
    std::shared_ptr<fml::ConcurrentTaskRunner> concurrent_task_runner,
    bool enable_impeller,
    impeller::RuntimeStageBackend runtime_stage_backend)
    : task_runners(task_runners),
      snapshot_delegate(std::move(snapshot_delegate)),
      io_manager(std::move(io_manager)),
      unref_queue(std::move(unref_queue)),
      image_decoder(std::move(image_decoder)),
      image_generator_registry(std::move(image_generator_registry)),
      advisory_script_uri(std::move(advisory_script_uri)),
      advisory_script_entrypoint(std::move(advisory_script_entrypoint)),
      deterministic_rendering_enabled(deterministic_rendering_enabled),
      concurrent_task_runner(std::move(concurrent_task_runner)),
      enable_impeller(enable_impeller),
      runtime_stage_backend(runtime_stage_backend) {}

UIDartState::UIDartState(
    TaskObserverAdd add_callback,
    TaskObserverRemove remove_callback,
    std::string logger_prefix,
    UnhandledExceptionCallback unhandled_exception_callback,
    LogMessageCallback log_message_callback,
    std::shared_ptr<IsolateNameServer> isolate_name_server,
    bool is_root_isolate,
    const UIDartState::Context& context)
    : add_callback_(std::move(add_callback)),
      remove_callback_(std::move(remove_callback)),
      logger_prefix_(std::move(logger_prefix)),
      is_root_isolate_(is_root_isolate),
      unhandled_exception_callback_(std::move(unhandled_exception_callback)),
      log_message_callback_(std::move(log_message_callback)),
      isolate_name_server_(std::move(isolate_name_server)),
      context_(context) {
  AddOrRemoveTaskObserver(true /* add */);
}

UIDartState::~UIDartState() {
  AddOrRemoveTaskObserver(false /* remove */);
}

const std::string& UIDartState::GetAdvisoryScriptURI() const {
  return context_.advisory_script_uri;
}

bool UIDartState::IsDeterministicRenderingEnabled() const {
  return context_.deterministic_rendering_enabled;
}

bool UIDartState::IsImpellerEnabled() const {
  return context_.enable_impeller;
}

impeller::RuntimeStageBackend UIDartState::GetRuntimeStageBackend() const {
  return context_.runtime_stage_backend;
}

void UIDartState::DidSetIsolate() {
  main_port_ = Dart_GetMainPortId();
  std::ostringstream debug_name;
  // main.dart$main-1234
  debug_name << context_.advisory_script_uri << "$"
             << context_.advisory_script_entrypoint << "-" << main_port_;
  SetDebugName(debug_name.str());
}

void UIDartState::ThrowIfUIOperationsProhibited() {
  if (!UIDartState::Current()->IsRootIsolate()) {
    Dart_EnterScope();
    Dart_ThrowException(
        tonic::ToDart("UI actions are only available on root isolate."));
  }
}

void UIDartState::SetDebugName(const std::string& debug_name) {
  debug_name_ = debug_name;
  if (platform_configuration_) {
    platform_configuration_->client()->UpdateIsolateDescription(debug_name_,
                                                                main_port_);
  }
}

UIDartState* UIDartState::Current() {
  return static_cast<UIDartState*>(DartState::Current());
}

void UIDartState::SetPlatformConfiguration(
    std::unique_ptr<PlatformConfiguration> platform_configuration) {
  FML_DCHECK(IsRootIsolate());
  platform_configuration_ = std::move(platform_configuration);
  if (platform_configuration_) {
    platform_configuration_->client()->UpdateIsolateDescription(debug_name_,
                                                                main_port_);
  }
}

void UIDartState::SetPlatformMessageHandler(
    std::weak_ptr<PlatformMessageHandler> handler) {
  FML_DCHECK(!IsRootIsolate());
  platform_message_handler_ = std::move(handler);
}

const TaskRunners& UIDartState::GetTaskRunners() const {
  return context_.task_runners;
}

fml::WeakPtr<IOManager> UIDartState::GetIOManager() const {
  return context_.io_manager;
}

fml::RefPtr<flutter::SkiaUnrefQueue> UIDartState::GetSkiaUnrefQueue() const {
  return context_.unref_queue;
}

std::shared_ptr<fml::ConcurrentTaskRunner>
UIDartState::GetConcurrentTaskRunner() const {
  return context_.concurrent_task_runner;
}

void UIDartState::ScheduleMicrotask(Dart_Handle closure) {
  if (tonic::CheckAndHandleError(closure) || !Dart_IsClosure(closure)) {
    return;
  }

  microtask_queue_.ScheduleMicrotask(closure);
}

void UIDartState::FlushMicrotasksNow() {
  microtask_queue_.RunMicrotasks();
}

bool UIDartState::HasPendingMicrotasks() {
  return microtask_queue_.HasMicrotasks();
}

void UIDartState::AddOrRemoveTaskObserver(bool add) {
  auto task_runner = context_.task_runners.GetUITaskRunner();
  if (!task_runner) {
    // This may happen in case the isolate has no thread affinity (for example,
    // the service isolate).
    return;
  }
  FML_DCHECK(add_callback_ && remove_callback_);
  if (add) {
    add_callback_(reinterpret_cast<intptr_t>(this),
                  [this]() { this->FlushMicrotasksNow(); });
  } else {
    remove_callback_(reinterpret_cast<intptr_t>(this));
  }
}

fml::TaskRunnerAffineWeakPtr<SnapshotDelegate>
UIDartState::GetSnapshotDelegate() const {
  return context_.snapshot_delegate;
}

fml::WeakPtr<ImageDecoder> UIDartState::GetImageDecoder() const {
  return context_.image_decoder;
}

fml::WeakPtr<ImageGeneratorRegistry> UIDartState::GetImageGeneratorRegistry()
    const {
  return context_.image_generator_registry;
}

std::shared_ptr<IsolateNameServer> UIDartState::GetIsolateNameServer() const {
  return isolate_name_server_;
}

tonic::DartErrorHandleType UIDartState::GetLastError() {
  tonic::DartErrorHandleType error = message_handler().isolate_last_error();
  if (error == tonic::kNoError) {
    error = microtask_queue_.GetLastError();
  }
  return error;
}

void UIDartState::LogMessage(const std::string& tag,
                             const std::string& message) const {
  if (log_message_callback_) {
    log_message_callback_(tag, message);
  } else {
    // Fall back to previous behavior if unspecified.
#if defined(FML_OS_ANDROID)
    __android_log_print(ANDROID_LOG_INFO, tag.c_str(), "%.*s",
                        static_cast<int>(message.size()), message.c_str());
#elif defined(FML_OS_IOS)
    std::stringstream stream;
    if (!tag.empty()) {
      stream << tag << ": ";
    }
    stream << message;
    std::string log = stream.str();
    syslog(1 /* LOG_ALERT */, "%.*s", static_cast<int>(log.size()),
           log.c_str());
#else
    if (!tag.empty()) {
      std::cout << tag << ": ";
    }
    std::cout << message << std::endl;
#endif
  }
}

Dart_Handle UIDartState::HandlePlatformMessage(
    std::unique_ptr<PlatformMessage> message) {
  if (platform_configuration_) {
    platform_configuration_->client()->HandlePlatformMessage(
        std::move(message));
  } else {
    std::shared_ptr<PlatformMessageHandler> handler =
        platform_message_handler_.lock();
    if (handler) {
      handler->HandlePlatformMessage(std::move(message));
    } else {
      return tonic::ToDart(
          "No platform channel handler registered for background isolate.");
    }
  }

  return Dart_Null();
}

int64_t UIDartState::GetRootIsolateToken() const {
  return IsRootIsolate() ? reinterpret_cast<int64_t>(this) : 0;
}

Dart_Isolate UIDartState::CreatePlatformIsolate(Dart_Handle entry_point,
                                                char** error) {
  FML_UNREACHABLE();
  return nullptr;
}

}  // namespace flutter
