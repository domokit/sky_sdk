// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/platform_view_ios.h"
#include <memory>

#include <utility>

#include "flutter/common/task_runners.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/trace_event.h"
#include "flutter/shell/common/shell_io_manager.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewController_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/vsync_waiter_ios.h"

FLUTTER_ASSERT_ARC

namespace flutter {

PlatformViewIOS::AccessibilityBridgeManager::AccessibilityBridgeManager(
    const std::function<void(bool)>& set_semantics_enabled)
    : AccessibilityBridgeManager(set_semantics_enabled, nullptr) {}

PlatformViewIOS::AccessibilityBridgeManager::AccessibilityBridgeManager(
    const std::function<void(bool)>& set_semantics_enabled,
    AccessibilityBridge* bridge)
    : accessibility_bridge_(bridge), set_semantics_enabled_(set_semantics_enabled) {
  if (bridge) {
    set_semantics_enabled_(true);
  }
}

void PlatformViewIOS::AccessibilityBridgeManager::Set(std::unique_ptr<AccessibilityBridge> bridge) {
  accessibility_bridge_ = std::move(bridge);
  set_semantics_enabled_(true);
}

void PlatformViewIOS::AccessibilityBridgeManager::Clear() {
  set_semantics_enabled_(false);
  accessibility_bridge_.reset();
}

PlatformViewIOS::PlatformViewIOS(PlatformView::Delegate& delegate,
                                 const std::shared_ptr<IOSContext>& context,
                                 __weak FlutterPlatformViewsController* platform_views_controller,
                                 const flutter::TaskRunners& task_runners)
    : PlatformView(delegate, task_runners),
      ios_context_(context),
      platform_views_controller_(platform_views_controller),
      accessibility_bridge_([this](bool enabled) { PlatformView::SetSemanticsEnabled(enabled); }),
      platform_message_handler_(
          new PlatformMessageHandlerIos(task_runners.GetPlatformTaskRunner())) {}

PlatformViewIOS::PlatformViewIOS(
    PlatformView::Delegate& delegate,
    IOSRenderingAPI rendering_api,
    __weak FlutterPlatformViewsController* platform_views_controller,
    const flutter::TaskRunners& task_runners,
    const std::shared_ptr<fml::ConcurrentTaskRunner>& worker_task_runner,
    const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch)
    : PlatformViewIOS(delegate,
                      IOSContext::Create(rendering_api,
                                         delegate.OnPlatformViewGetSettings().enable_impeller
                                             ? IOSRenderingBackend::kImpeller
                                             : IOSRenderingBackend::kSkia,
                                         is_gpu_disabled_sync_switch),
                      platform_views_controller,
                      task_runners) {}

PlatformViewIOS::~PlatformViewIOS() = default;

// |PlatformView|
void PlatformViewIOS::HandlePlatformMessage(std::unique_ptr<flutter::PlatformMessage> message) {
  platform_message_handler_->HandlePlatformMessage(std::move(message));
}

FlutterViewController* PlatformViewIOS::GetOwnerViewController() const {
  return owner_controller_;
}

void PlatformViewIOS::SetOwnerViewController(__weak FlutterViewController* owner_controller) {
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());
  std::lock_guard<std::mutex> guard(ios_surface_mutex_);
  if (ios_surface_ || !owner_controller) {
    NotifyDestroyed();
    ios_surface_.reset();
    accessibility_bridge_.Clear();
  }
  owner_controller_ = owner_controller;

  // Add an observer that will clear out the owner_controller_ ivar and
  // the accessibility_bridge_ in case the view controller is deleted.
  dealloc_view_controller_observer_.reset([[NSNotificationCenter defaultCenter]
      addObserverForName:FlutterViewControllerWillDealloc
                  object:owner_controller_
                   queue:[NSOperationQueue mainQueue]
              usingBlock:^(NSNotification* note) {
                // Implicit copy of 'this' is fine.
                accessibility_bridge_.Clear();
                owner_controller_ = nil;
              }]);

  if (owner_controller_ && owner_controller_.isViewLoaded) {
    this->attachView();
  }
  // Do not call `NotifyCreated()` here - let FlutterViewController take care
  // of that when its Viewport is sized.  If `NotifyCreated()` is called here,
  // it can occasionally get invoked before the viewport is sized resulting in
  // a framebuffer that will not be able to completely attach.
}

void PlatformViewIOS::attachView() {
  FML_DCHECK(owner_controller_);
  FML_DCHECK(owner_controller_.isViewLoaded) << "FlutterViewController's view should be loaded "
                                                "before attaching to PlatformViewIOS.";
  FlutterView* flutter_view = static_cast<FlutterView*>(owner_controller_.view);
  CALayer* ca_layer = flutter_view.layer;
  ios_surface_ = IOSSurface::Create(ios_context_, ca_layer);
  FML_DCHECK(ios_surface_ != nullptr);

  if (accessibility_bridge_) {
    accessibility_bridge_.Set(std::make_unique<AccessibilityBridge>(
        owner_controller_, this, owner_controller_.platformViewsController));
  }
}

PointerDataDispatcherMaker PlatformViewIOS::GetDispatcherMaker() {
  return [](DefaultPointerDataDispatcher::Delegate& delegate) {
    return std::make_unique<SmoothPointerDataDispatcher>(delegate);
  };
}

void PlatformViewIOS::RegisterExternalTexture(int64_t texture_id,
                                              NSObject<FlutterTexture>* texture) {
  RegisterTexture(ios_context_->CreateExternalTexture(texture_id, texture));
}

// |PlatformView|
std::unique_ptr<Surface> PlatformViewIOS::CreateRenderingSurface() {
  FML_DCHECK(task_runners_.GetRasterTaskRunner()->RunsTasksOnCurrentThread());
  std::lock_guard<std::mutex> guard(ios_surface_mutex_);
  if (!ios_surface_) {
    FML_DLOG(INFO) << "Could not CreateRenderingSurface, this PlatformViewIOS "
                      "has no ViewController.";
    return nullptr;
  }
  return ios_surface_->CreateGPUSurface(ios_context_->GetMainContext().get());
}

// |PlatformView|
std::shared_ptr<ExternalViewEmbedder> PlatformViewIOS::CreateExternalViewEmbedder() {
  return std::make_shared<IOSExternalViewEmbedder>(platform_views_controller_, ios_context_);
}

// |PlatformView|
sk_sp<GrDirectContext> PlatformViewIOS::CreateResourceContext() const {
  return ios_context_->CreateResourceContext();
}

// |PlatformView|
std::shared_ptr<impeller::Context> PlatformViewIOS::GetImpellerContext() const {
  return ios_context_->GetImpellerContext();
}

// |PlatformView|
void PlatformViewIOS::SetSemanticsEnabled(bool enabled) {
  if (!owner_controller_) {
    FML_LOG(WARNING) << "Could not set semantics to enabled, this "
                        "PlatformViewIOS has no ViewController.";
    return;
  }
  if (enabled && !accessibility_bridge_) {
    accessibility_bridge_.Set(std::make_unique<AccessibilityBridge>(
        owner_controller_, this, owner_controller_.platformViewsController));
  } else if (!enabled && accessibility_bridge_) {
    accessibility_bridge_.Clear();
  } else {
    PlatformView::SetSemanticsEnabled(enabled);
  }
}

// |shell:PlatformView|
void PlatformViewIOS::SetAccessibilityFeatures(int32_t flags) {
  PlatformView::SetAccessibilityFeatures(flags);
}

// |PlatformView|
void PlatformViewIOS::UpdateSemantics(flutter::SemanticsNodeUpdates update,
                                      flutter::CustomAccessibilityActionUpdates actions) {
  FML_DCHECK(owner_controller_);
  if (accessibility_bridge_) {
    accessibility_bridge_.get()->UpdateSemantics(std::move(update), actions);
    [[NSNotificationCenter defaultCenter] postNotificationName:FlutterSemanticsUpdateNotification
                                                        object:owner_controller_];
  }
}

// |PlatformView|
std::unique_ptr<VsyncWaiter> PlatformViewIOS::CreateVSyncWaiter() {
  return std::make_unique<VsyncWaiterIOS>(task_runners_);
}

void PlatformViewIOS::OnPreEngineRestart() const {
  if (accessibility_bridge_) {
    accessibility_bridge_.get()->clearState();
  }
  if (!owner_controller_) {
    return;
  }
  [owner_controller_.platformViewsController reset];
  [owner_controller_.restorationPlugin reset];
}

std::unique_ptr<std::vector<std::string>> PlatformViewIOS::ComputePlatformResolvedLocales(
    const std::vector<std::string>& supported_locale_data) {
  size_t localeDataLength = 3;
  NSMutableArray<NSString*>* supported_locale_identifiers =
      [NSMutableArray arrayWithCapacity:supported_locale_data.size() / localeDataLength];
  for (size_t i = 0; i < supported_locale_data.size(); i += localeDataLength) {
    NSDictionary<NSString*, NSString*>* dict = @{
      NSLocaleLanguageCode : [NSString stringWithUTF8String:supported_locale_data[i].c_str()]
          ?: @"",
      NSLocaleCountryCode : [NSString stringWithUTF8String:supported_locale_data[i + 1].c_str()]
          ?: @"",
      NSLocaleScriptCode : [NSString stringWithUTF8String:supported_locale_data[i + 2].c_str()]
          ?: @""
    };
    [supported_locale_identifiers addObject:[NSLocale localeIdentifierFromComponents:dict]];
  }
  NSArray<NSString*>* result =
      [NSBundle preferredLocalizationsFromArray:supported_locale_identifiers];

  // Output format should be either empty or 3 strings for language, country, and script.
  std::unique_ptr<std::vector<std::string>> out = std::make_unique<std::vector<std::string>>();

  if (result != nullptr && [result count] > 0) {
    NSLocale* locale = [NSLocale localeWithLocaleIdentifier:[result firstObject]];
    NSString* languageCode = [locale languageCode];
    out->emplace_back(languageCode == nullptr ? "" : languageCode.UTF8String);
    NSString* countryCode = [locale countryCode];
    out->emplace_back(countryCode == nullptr ? "" : countryCode.UTF8String);
    NSString* scriptCode = [locale scriptCode];
    out->emplace_back(scriptCode == nullptr ? "" : scriptCode.UTF8String);
  }
  return out;
}

PlatformViewIOS::ScopedObserver::ScopedObserver() {}

PlatformViewIOS::ScopedObserver::~ScopedObserver() {
  if (observer_) {
    [[NSNotificationCenter defaultCenter] removeObserver:observer_];
  }
}

void PlatformViewIOS::ScopedObserver::reset(id<NSObject> observer) {
  if (observer != observer_) {
    if (observer_) {
      [[NSNotificationCenter defaultCenter] removeObserver:observer_];
    }
    observer_ = observer;
  }
}

}  // namespace flutter
