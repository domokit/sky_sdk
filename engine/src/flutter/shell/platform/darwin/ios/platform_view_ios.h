// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_PLATFORM_VIEW_IOS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_PLATFORM_VIEW_IOS_H_

#include <memory>

#include "flutter/fml/closure.h"
#include "flutter/fml/macros.h"
#include "flutter/shell/common/platform_view.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterTexture.h"
#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterViewController.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterView.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/accessibility_bridge.h"
#import "flutter/shell/platform/darwin/ios/ios_context.h"
#import "flutter/shell/platform/darwin/ios/ios_external_view_embedder.h"
#import "flutter/shell/platform/darwin/ios/ios_surface.h"
#import "flutter/shell/platform/darwin/ios/platform_message_handler_ios.h"
#import "flutter/shell/platform/darwin/ios/rendering_api_selection.h"

@class FlutterViewController;

namespace flutter {

/**
 * A bridge connecting the platform agnostic shell and the iOS embedding.
 *
 * The shell provides and requests for UI related data and this PlatformView subclass fulfills
 * it with iOS specific capabilities. As an example, the iOS embedding (the `FlutterEngine` and the
 * `FlutterViewController`) sends pointer data to the shell and receives the shell's request for a
 * Skia GrDirectContext and supplies it.
 *
 * Despite the name "view", this class is unrelated to UIViews on iOS and doesn't have the same
 * lifecycle. It's a long lived bridge owned by the `FlutterEngine` and can be attached and
 * detached sequentially to multiple `FlutterViewController`s and `FlutterView`s.
 */
class PlatformViewIOS final : public PlatformView {
 public:
  PlatformViewIOS(PlatformView::Delegate& delegate,
                  const std::shared_ptr<IOSContext>& context,
                  __weak FlutterPlatformViewsController* platform_views_controller,
                  const flutter::TaskRunners& task_runners);

  explicit PlatformViewIOS(
      PlatformView::Delegate& delegate,
      IOSRenderingAPI rendering_api,
      __weak FlutterPlatformViewsController* platform_views_controller,
      const flutter::TaskRunners& task_runners,
      const std::shared_ptr<fml::ConcurrentTaskRunner>& worker_task_runner,
      const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch);

  ~PlatformViewIOS() override;

  /**
   * Returns the `FlutterViewController` currently attached to the `FlutterEngine` owning
   * this PlatformViewIOS.
   */
  FlutterViewController* GetOwnerViewController() const __attribute__((cf_audited_transfer));

  /**
   * Updates the `FlutterViewController` currently attached to the `FlutterEngine` owning
   * this PlatformViewIOS. This should be updated when the `FlutterEngine`
   * is given a new `FlutterViewController`.
   */
  void SetOwnerViewController(__weak FlutterViewController* owner_controller);

  /**
   * Called one time per `FlutterViewController` when the `FlutterViewController`'s
   * UIView is first loaded.
   *
   * Can be used to perform late initialization after `FlutterViewController`'s
   * init.
   */
  void attachView();

  /**
   * Called through when an external texture such as video or camera is
   * given to the `FlutterEngine` or `FlutterViewController`.
   */
  void RegisterExternalTexture(int64_t id, NSObject<FlutterTexture>* texture);

  // |PlatformView|
  PointerDataDispatcherMaker GetDispatcherMaker() override;

  // |PlatformView|
  void SetSemanticsEnabled(bool enabled) override;

  // |PlatformView|
  void HandlePlatformMessage(std::unique_ptr<flutter::PlatformMessage> message) override;

  // |PlatformView|
  std::unique_ptr<Surface> CreateRenderingSurface() override;

  // |PlatformView|
  std::shared_ptr<ExternalViewEmbedder> CreateExternalViewEmbedder() override;

  // |PlatformView|
  sk_sp<GrDirectContext> CreateResourceContext() const override;

  // |PlatformView|
  std::shared_ptr<impeller::Context> GetImpellerContext() const override;

  // |PlatformView|
  void SetAccessibilityFeatures(int32_t flags) override;

  // |PlatformView|
  void UpdateSemantics(flutter::SemanticsNodeUpdates update,
                       flutter::CustomAccessibilityActionUpdates actions) override;

  // |PlatformView|
  std::unique_ptr<VsyncWaiter> CreateVSyncWaiter() override;

  // |PlatformView|
  void OnPreEngineRestart() const override;

  // |PlatformView|
  std::unique_ptr<std::vector<std::string>> ComputePlatformResolvedLocales(
      const std::vector<std::string>& supported_locale_data) override;

  /** Accessor for the `IOSContext` associated with the platform view. */
  const std::shared_ptr<IOSContext>& GetIosContext() { return ios_context_; }

  std::shared_ptr<PlatformMessageHandlerIos> GetPlatformMessageHandlerIos() const {
    return platform_message_handler_;
  }

  std::shared_ptr<PlatformMessageHandler> GetPlatformMessageHandler() const override {
    return platform_message_handler_;
  }

 private:
  /// Smart pointer for use with objective-c observers.
  /// This guarantees we remove the observer.
  class ScopedObserver {
   public:
    ScopedObserver();
    ~ScopedObserver();
    void reset(id<NSObject> observer);
    ScopedObserver(const ScopedObserver&) = delete;
    ScopedObserver& operator=(const ScopedObserver&) = delete;

   private:
    id<NSObject> observer_ = nil;
  };

  /// Wrapper that guarantees we communicate clearing Accessibility
  /// information to Dart.
  class AccessibilityBridgeManager {
   public:
    explicit AccessibilityBridgeManager(const std::function<void(bool)>& set_semantics_enabled);
    AccessibilityBridgeManager(const std::function<void(bool)>& set_semantics_enabled,
                               AccessibilityBridge* bridge);
    explicit operator bool() const noexcept { return static_cast<bool>(accessibility_bridge_); }
    AccessibilityBridge* get() const noexcept { return accessibility_bridge_.get(); }
    void Set(std::unique_ptr<AccessibilityBridge> bridge);
    void Clear();

   private:
    FML_DISALLOW_COPY_AND_ASSIGN(AccessibilityBridgeManager);
    std::unique_ptr<AccessibilityBridge> accessibility_bridge_;
    std::function<void(bool)> set_semantics_enabled_;
  };

  __weak FlutterViewController* owner_controller_;
  // Since the `ios_surface_` is created on the platform thread but
  // used on the raster thread we need to protect it with a mutex.
  std::mutex ios_surface_mutex_;
  std::unique_ptr<IOSSurface> ios_surface_;
  std::shared_ptr<IOSContext> ios_context_;
  __weak FlutterPlatformViewsController* platform_views_controller_;
  AccessibilityBridgeManager accessibility_bridge_;
  ScopedObserver dealloc_view_controller_observer_;
  std::vector<std::string> platform_resolved_locale_;
  std::shared_ptr<PlatformMessageHandlerIos> platform_message_handler_;

  FML_DISALLOW_COPY_AND_ASSIGN(PlatformViewIOS);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_PLATFORM_VIEW_IOS_H_
