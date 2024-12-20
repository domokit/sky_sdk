// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fuchsia/ui/composition/cpp/fidl.h>
#include <fuchsia/ui/composition/cpp/fidl_test_base.h>
#include <fuchsia/ui/input/cpp/fidl.h>
#include <fuchsia/ui/input3/cpp/fidl.h>
#include <fuchsia/ui/input3/cpp/fidl_test_base.h>
#include <fuchsia/ui/views/cpp/fidl.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/async-loop/default.h>
#include <lib/async/default.h>
#include <lib/fidl/cpp/binding_set.h>
#include <lib/zx/eventpair.h>

#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "flutter/flow/embedded_views.h"
#include "flutter/lib/ui/window/platform_message.h"
#include "flutter/lib/ui/window/pointer_data.h"
#include "flutter/lib/ui/window/viewport_metrics.h"
#include "flutter/shell/common/context_options.h"
#include "flutter/shell/platform/fuchsia/flutter/platform_view.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "fakes/focuser.h"
#include "fakes/platform_message.h"
#include "fakes/touch_source.h"
#include "fakes/view_ref_focused.h"
#include "flutter/shell/platform/fuchsia/flutter/surface.h"
#include "flutter/shell/platform/fuchsia/flutter/task_runner_adapter.h"
#include "platform/assert.h"
#include "pointer_event_utility.h"

namespace flutter_runner::testing {
namespace {

class MockExternalViewEmbedder : public flutter::ExternalViewEmbedder {
 public:
  flutter::DlCanvas* GetRootCanvas() override { return nullptr; }

  void CancelFrame() override {}
  void BeginFrame(GrDirectContext* context,
                  const fml::RefPtr<fml::RasterThreadMerger>&
                      raster_thread_merger) override {}

  void PrepareFlutterView(SkISize frame_size,
                          double device_pixel_ratio) override {}

  void SubmitFlutterView(
      int64_t flutter_view_id,
      GrDirectContext* context,
      const std::shared_ptr<impeller::AiksContext>& aiks_context,
      std::unique_ptr<flutter::SurfaceFrame> frame) override {}

  void PrerollCompositeEmbeddedView(
      int64_t view_id,
      std::unique_ptr<flutter::EmbeddedViewParams> params) override {}

  flutter::DlCanvas* CompositeEmbeddedView(int64_t view_id) override {
    return nullptr;
  }
};

class MockPlatformViewDelegate : public flutter::PlatformView::Delegate {
 public:
  void Reset() {
    message_ = nullptr;
    metrics_ = flutter::ViewportMetrics{};
    semantics_features_ = 0;
    semantics_enabled_ = false;
    pointer_packets_.clear();
  }

  // |flutter::PlatformView::Delegate|
  void OnPlatformViewCreated(std::unique_ptr<flutter::Surface> surface) {
    ASSERT_EQ(surface_.get(), nullptr);

    surface_ = std::move(surface);
  }
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewDestroyed() {}
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewScheduleFrame() {}
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewAddView(int64_t view_id,
                             const flutter::ViewportMetrics& viewport_metrics,
                             AddViewCallback callback) override {}
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewRemoveView(int64_t view_id,
                                RemoveViewCallback callback) override {}
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewSetNextFrameCallback(const fml::closure& closure) {}
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewSetViewportMetrics(
      int64_t view_id,
      const flutter::ViewportMetrics& metrics) {
    metrics_ = metrics;
  }
  // |flutter::PlatformView::Delegate|
  const flutter::Settings& OnPlatformViewGetSettings() const {
    return settings_;
  }
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewDispatchPlatformMessage(
      std::unique_ptr<flutter::PlatformMessage> message) {
    message_ = std::move(message);
  }
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewDispatchPointerDataPacket(
      std::unique_ptr<flutter::PointerDataPacket> packet) {
    pointer_packets_.push_back(std::move(packet));
  }
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewDispatchKeyDataPacket(
      std::unique_ptr<flutter::KeyDataPacket> packet,
      std::function<void(bool)> callback) {}
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewDispatchSemanticsAction(int32_t id,
                                             flutter::SemanticsAction action,
                                             fml::MallocMapping args) {}
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewSetSemanticsEnabled(bool enabled) {
    semantics_enabled_ = enabled;
  }
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewSetAccessibilityFeatures(int32_t flags) {
    semantics_features_ = flags;
  }
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewRegisterTexture(
      std::shared_ptr<flutter::Texture> texture) {}
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewUnregisterTexture(int64_t texture_id) {}
  // |flutter::PlatformView::Delegate|
  void OnPlatformViewMarkTextureFrameAvailable(int64_t texture_id) {}
  // |flutter::PlatformView::Delegate|
  std::unique_ptr<std::vector<std::string>> ComputePlatformViewResolvedLocale(
      const std::vector<std::string>& supported_locale_data) {
    return nullptr;
  }
  // |flutter::PlatformView::Delegate|
  void LoadDartDeferredLibrary(
      intptr_t loading_unit_id,
      std::unique_ptr<const fml::Mapping> snapshot_data,
      std::unique_ptr<const fml::Mapping> snapshot_instructions) {}
  // |flutter::PlatformView::Delegate|
  void LoadDartDeferredLibraryError(intptr_t loading_unit_id,
                                    const std::string error_message,
                                    bool transient) {}
  // |flutter::PlatformView::Delegate|
  void UpdateAssetResolverByType(
      std::unique_ptr<flutter::AssetResolver> updated_asset_resolver,
      flutter::AssetResolver::AssetResolverType type) {}

  flutter::Surface* surface() const { return surface_.get(); }
  flutter::PlatformMessage* message() const { return message_.get(); }
  const flutter::ViewportMetrics& metrics() const { return metrics_; }
  int32_t semantics_features() const { return semantics_features_; }
  bool semantics_enabled() const { return semantics_enabled_; }
  const std::vector<std::unique_ptr<flutter::PointerDataPacket>>&
  pointer_packets() const {
    return pointer_packets_;
  }
  std::vector<std::unique_ptr<flutter::PointerDataPacket>>
  TakePointerDataPackets() {
    auto tmp = std::move(pointer_packets_);
    pointer_packets_.clear();
    return tmp;
  }

 private:
  std::unique_ptr<flutter::Surface> surface_;
  std::unique_ptr<flutter::PlatformMessage> message_;
  flutter::ViewportMetrics metrics_;
  std::vector<std::unique_ptr<flutter::PointerDataPacket>> pointer_packets_;
  int32_t semantics_features_ = 0;
  bool semantics_enabled_ = false;
  flutter::Settings settings_;
};

class MockResponse : public flutter::PlatformMessageResponse {
 public:
  MOCK_METHOD(void, Complete, (std::unique_ptr<fml::Mapping> data), (override));
  MOCK_METHOD(void, CompleteEmpty, (), (override));
};

class TestPlatformMessageResponse : public flutter::PlatformMessageResponse {
 public:
  TestPlatformMessageResponse() {}
  void Complete(std::unique_ptr<fml::Mapping> data) override {
    result_string = std::string(
        reinterpret_cast<const char*>(data->GetMapping()), data->GetSize());
    is_complete_ = true;
  }
  void CompleteEmpty() override { is_complete_ = true; }
  std::string result_string;
  FML_DISALLOW_COPY_AND_ASSIGN(TestPlatformMessageResponse);
};

class MockKeyboard : public fuchsia::ui::input3::testing::Keyboard_TestBase {
 public:
  explicit MockKeyboard(
      fidl::InterfaceRequest<fuchsia::ui::input3::Keyboard> keyboard)
      : keyboard_(this, std::move(keyboard)) {}
  ~MockKeyboard() = default;

  void AddListener(fuchsia::ui::views::ViewRef view_ref,
                   fuchsia::ui::input3::KeyboardListenerHandle listener,
                   AddListenerCallback callback) override {
    FML_CHECK(!listener_.is_bound());

    listener_ = listener.Bind();
    view_ref_ = std::move(view_ref);

    callback();
  }

  void NotImplemented_(const std::string& name) override { FAIL(); }

  fidl::Binding<fuchsia::ui::input3::Keyboard> keyboard_;
  fuchsia::ui::input3::KeyboardListenerPtr listener_{};
  fuchsia::ui::views::ViewRef view_ref_{};

  FML_DISALLOW_COPY_AND_ASSIGN(MockKeyboard);
};

class MockChildViewWatcher
    : public fuchsia::ui::composition::testing::ChildViewWatcher_TestBase {
 public:
  explicit MockChildViewWatcher(
      fidl::InterfaceRequest<fuchsia::ui::composition::ChildViewWatcher>
          request)
      : binding_(this, std::move(request)) {}
  ~MockChildViewWatcher() = default;

  // |fuchsia::ui::composition::ChildViewWatcher|
  void GetStatus(GetStatusCallback callback) override {
    // GetStatus only returns once as per flatland.fidl comments
    if (get_status_returned_) {
      return;
    }
    callback(fuchsia::ui::composition::ChildViewStatus::CONTENT_HAS_PRESENTED);
    get_status_returned_ = true;
  }

  // |fuchsia::ui::composition::ChildViewWatcher|
  void GetViewRef(GetViewRefCallback callback) override {
    // GetViewRef only returns once as per flatland.fidl comments
    ASSERT_FALSE(control_ref_.reference);
    fuchsia::ui::views::ViewRefControl view_ref_control;
    fuchsia::ui::views::ViewRef view_ref;
    auto status = zx::eventpair::create(
        /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
    ZX_ASSERT(status == ZX_OK);
    view_ref_control.reference.replace(
        ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
        &view_ref_control.reference);
    view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);
    control_ref_ = std::move(view_ref_control);
    callback(std::move(view_ref));
  }

  void NotImplemented_(const std::string& name) override { FAIL(); }

  fidl::Binding<fuchsia::ui::composition::ChildViewWatcher> binding_;
  fuchsia::ui::views::ViewRefControl control_ref_;
  bool get_status_returned_ = false;

  FML_DISALLOW_COPY_AND_ASSIGN(MockChildViewWatcher);
};

class MockParentViewportWatcher
    : public fuchsia::ui::composition::testing::ParentViewportWatcher_TestBase {
 public:
  explicit MockParentViewportWatcher() : binding_(this, handle_.NewRequest()) {}
  ~MockParentViewportWatcher() = default;

  // |fuchsia::ui::composition::ParentViewportWatcher|
  void GetStatus(GetStatusCallback callback) override {
    // GetStatus only returns once as per flatland.fidl comments
    if (get_status_returned_) {
      return;
    }
    callback(
        fuchsia::ui::composition::ParentViewportStatus::CONNECTED_TO_DISPLAY);
    get_status_returned_ = true;
  }

  // |fuchsia::ui::composition::ParentViewportWatcher|
  void GetLayout(GetLayoutCallback callback) override {
    if (layout_changed_) {
      callback(std::move(layout_));
    } else {
      FML_CHECK(!pending_callback_valid_);
      pending_layout_callback_ = std::move(callback);
      pending_callback_valid_ = true;
    }
  }

  void SetLayout(uint32_t logical_size_x,
                 uint32_t logical_size_y,
                 float DPR = 1.0) {
    ::fuchsia::math::SizeU logical_size;
    logical_size.width = logical_size_x;
    logical_size.height = logical_size_y;
    layout_.set_logical_size(logical_size);
    layout_.set_device_pixel_ratio({DPR, DPR});

    if (pending_callback_valid_) {
      pending_layout_callback_(std::move(layout_));
      pending_callback_valid_ = false;
    } else {
      layout_changed_ = true;
    }
  }

  fuchsia::ui::composition::ParentViewportWatcherHandle GetHandle() {
    FML_CHECK(handle_);  // You can only get the handle once.
    return std::move(handle_);
  }

  void NotImplemented_(const std::string& name) override { FAIL(); }

  fuchsia::ui::composition::ParentViewportWatcherHandle handle_;
  fidl::Binding<fuchsia::ui::composition::ParentViewportWatcher> binding_;

  fuchsia::ui::composition::LayoutInfo layout_;
  bool layout_changed_ = false;
  GetLayoutCallback pending_layout_callback_;
  bool pending_callback_valid_ = false;

  bool get_status_returned_ = false;

  FML_DISALLOW_COPY_AND_ASSIGN(MockParentViewportWatcher);
};

// Used to construct partial instances of PlatformView for testing.  The
// PlatformView constructor has many parameters, not all of which need to
// be filled out for each test.  The builder allows you to initialize only
// those that matter to your specific test.  Not all builder methods are
// provided: if you find some that are missing, feel free to add them.
class PlatformViewBuilder {
 public:
  PlatformViewBuilder(flutter::PlatformView::Delegate& delegate,
                      flutter::TaskRunners task_runners)
      : delegate_(delegate), task_runners_(task_runners) {
    fuchsia::ui::views::ViewRefControl view_ref_control;
    fuchsia::ui::views::ViewRef view_ref;
    auto status = zx::eventpair::create(
        /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
    ZX_ASSERT(status == ZX_OK);
    view_ref_control.reference.replace(
        ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
        &view_ref_control.reference);
    view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);
    auto view_ref_pair =
        std::make_pair(std::move(view_ref_control), std::move(view_ref));
    view_ref_pair_ = std::move(view_ref_pair);
  }

  PlatformViewBuilder& SetExternalViewEmbedder(
      std::shared_ptr<flutter::ExternalViewEmbedder> embedder) {
    external_external_view_embedder_ = embedder;
    return *this;
  }

  PlatformViewBuilder& SetImeService(
      fuchsia::ui::input::ImeServiceHandle ime_service) {
    ime_service_ = std::move(ime_service);
    return *this;
  }

  PlatformViewBuilder& SetKeyboard(
      fuchsia::ui::input3::KeyboardHandle keyboard) {
    keyboard_ = std::move(keyboard);
    return *this;
  }

  PlatformViewBuilder& SetTouchSource(
      fuchsia::ui::pointer::TouchSourceHandle touch_source) {
    touch_source_ = std::move(touch_source);
    return *this;
  }

  PlatformViewBuilder& SetMouseSource(
      fuchsia::ui::pointer::MouseSourceHandle mouse_source) {
    mouse_source_ = std::move(mouse_source);
    return *this;
  }

  PlatformViewBuilder& SetFocuser(fuchsia::ui::views::FocuserHandle focuser) {
    focuser_ = std::move(focuser);
    return *this;
  }

  PlatformViewBuilder& SetViewRefFocused(
      fuchsia::ui::views::ViewRefFocusedHandle view_ref_focused) {
    view_ref_focused_ = std::move(view_ref_focused);
    return *this;
  }

  PlatformViewBuilder& SetPointerInjectorRegistry(
      fuchsia::ui::pointerinjector::RegistryHandle pointerinjector_registry) {
    pointerinjector_registry_ = std::move(pointerinjector_registry);
    return *this;
  }

  PlatformViewBuilder& SetEnableWireframeCallback(
      OnEnableWireframeCallback callback) {
    wireframe_enabled_callback_ = std::move(callback);
    return *this;
  }

  PlatformViewBuilder& SetParentViewportWatcher(
      fuchsia::ui::composition::ParentViewportWatcherHandle
          parent_viewport_watcher) {
    parent_viewport_watcher_ = std::move(parent_viewport_watcher);
    return *this;
  }

  PlatformViewBuilder& SetCreateViewCallback(OnCreateViewCallback callback) {
    on_create_view_callback_ = std::move(callback);
    return *this;
  }

  PlatformViewBuilder& SetDestroyViewCallback(OnDestroyViewCallback callback) {
    on_destroy_view_callback_ = std::move(callback);
    return *this;
  }

  PlatformViewBuilder& SetUpdateViewCallback(OnUpdateViewCallback callback) {
    on_update_view_callback_ = std::move(callback);
    return *this;
  }

  PlatformViewBuilder& SetCreateSurfaceCallback(
      OnCreateSurfaceCallback callback) {
    on_create_surface_callback_ = std::move(callback);
    return *this;
  }

  PlatformViewBuilder& SetShaderWarmupCallback(
      OnShaderWarmupCallback callback) {
    on_shader_warmup_callback_ = std::move(callback);
    return *this;
  }

  // Once Build is called, the instance is no longer usable.
  PlatformView Build() {
    EXPECT_FALSE(std::exchange(built_, true))
        << "Build() was already called, this builder is good for one use only.";
    return PlatformView(
        delegate_, task_runners_, std::move(view_ref_pair_.second),
        external_external_view_embedder_, std::move(ime_service_),
        std::move(keyboard_), std::move(touch_source_),
        std::move(mouse_source_), std::move(focuser_),
        std::move(view_ref_focused_), std::move(parent_viewport_watcher_),
        std::move(pointerinjector_registry_),
        std::move(wireframe_enabled_callback_),
        std::move(on_create_view_callback_),
        std::move(on_update_view_callback_),
        std::move(on_destroy_view_callback_),
        std::move(on_create_surface_callback_),
        std::move(on_semantics_node_update_callback_),
        std::move(on_request_announce_callback_),
        std::move(on_shader_warmup_callback_), [](auto...) {}, [](auto...) {},
        nullptr);
  }

 private:
  PlatformViewBuilder() = delete;

  flutter::PlatformView::Delegate& delegate_;
  flutter::TaskRunners task_runners_;
  std::pair<fuchsia::ui::views::ViewRefControl, fuchsia::ui::views::ViewRef>
      view_ref_pair_;

  std::shared_ptr<flutter::ExternalViewEmbedder>
      external_external_view_embedder_;
  fuchsia::ui::input::ImeServiceHandle ime_service_;
  fuchsia::ui::input3::KeyboardHandle keyboard_;
  fuchsia::ui::pointer::TouchSourceHandle touch_source_;
  fuchsia::ui::pointer::MouseSourceHandle mouse_source_;
  fuchsia::ui::views::ViewRefFocusedHandle view_ref_focused_;
  fuchsia::ui::views::FocuserHandle focuser_;
  fuchsia::ui::pointerinjector::RegistryHandle pointerinjector_registry_;
  fit::closure on_session_listener_error_callback_;
  OnEnableWireframeCallback wireframe_enabled_callback_;
  fuchsia::ui::composition::ParentViewportWatcherHandle
      parent_viewport_watcher_;
  OnCreateViewCallback on_create_view_callback_;
  OnDestroyViewCallback on_destroy_view_callback_;
  OnUpdateViewCallback on_update_view_callback_;
  OnCreateSurfaceCallback on_create_surface_callback_;
  OnSemanticsNodeUpdateCallback on_semantics_node_update_callback_;
  OnRequestAnnounceCallback on_request_announce_callback_;
  OnShaderWarmupCallback on_shader_warmup_callback_;

  bool built_{false};
};

std::string ToString(const fml::Mapping& mapping) {
  return std::string(mapping.GetMapping(),
                     mapping.GetMapping() + mapping.GetSize());
}

// Stolen from pointer_data_packet_converter_unittests.cc.
void UnpackPointerPacket(std::vector<flutter::PointerData>& output,  // NOLINT
                         std::unique_ptr<flutter::PointerDataPacket> packet) {
  for (size_t i = 0; i < packet->GetLength(); i++) {
    flutter::PointerData pointer_data = packet->GetPointerData(i);
    output.push_back(pointer_data);
  }
  packet.reset();
}

}  // namespace

class PlatformViewTests : public ::testing::Test {
 protected:
  PlatformViewTests() : loop_(&kAsyncLoopConfigAttachToCurrentThread) {}

  async_dispatcher_t* dispatcher() { return loop_.dispatcher(); }

  void RunLoopUntilIdle() {
    loop_.RunUntilIdle();
    loop_.ResetQuit();
  }

  void RunLoopOnce() {
    loop_.Run(zx::time::infinite(), true);
    loop_.ResetQuit();
  }

  fuchsia::ui::input3::KeyEvent MakeEvent(
      fuchsia::ui::input3::KeyEventType event_type,
      std::optional<fuchsia::ui::input3::Modifiers> modifiers,
      fuchsia::input::Key key) {
    fuchsia::ui::input3::KeyEvent event;
    event.set_timestamp(++event_timestamp_);
    event.set_type(event_type);
    if (modifiers.has_value()) {
      event.set_modifiers(modifiers.value());
    }
    event.set_key(key);
    return event;
  }

  fuchsia::ui::composition::ChildViewWatcherPtr MakeChildViewWatcher() {
    fuchsia::ui::composition::ChildViewWatcherPtr ptr;
    auto watcher = std::make_unique<MockChildViewWatcher>(
        ptr.NewRequest(loop_.dispatcher()));
    child_view_watchers_.push_back(std::move(watcher));
    return ptr;
  }

 private:
  async::Loop loop_;

  uint64_t event_timestamp_{42};

  std::vector<std::unique_ptr<MockChildViewWatcher>> child_view_watchers_;

  FML_DISALLOW_COPY_AND_ASSIGN(PlatformViewTests);
};

// This test makes sure that the PlatformView always completes a platform
// message request, even for error conditions or if the request is malformed.
TEST_F(PlatformViewTests, InvalidPlatformMessageRequest) {
  MockPlatformViewDelegate delegate;
  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners", nullptr, nullptr, nullptr, nullptr);

  FakeViewRefFocused vrf;
  fidl::BindingSet<fuchsia::ui::views::ViewRefFocused> vrf_bindings;
  auto vrf_handle = vrf_bindings.AddBinding(&vrf);

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetViewRefFocused(std::move(vrf_handle))
                           .Build();

  // Cast platform_view to its base view so we can have access to the public
  // "HandlePlatformMessage" function.
  auto base_view = static_cast<flutter::PlatformView*>(&platform_view);
  EXPECT_TRUE(base_view);

  // Invalid platform channel.
  auto response1 = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(response1->WithMessage(
      "flutter/invalid", "{\"method\":\"Invalid.invalidMethod\"}"));

  // Invalid json.
  auto response2 = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(
      response2->WithMessage("flutter/platform_views", "{Invalid JSON"));

  // Invalid method.
  auto response3 = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(response3->WithMessage(
      "flutter/platform_views", "{\"method\":\"View.focus.invalidMethod\"}"));

  // Missing arguments.
  auto response4 = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(response4->WithMessage(
      "flutter/platform_views", "{\"method\":\"View.update\"}"));
  auto response5 = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(
      response5->WithMessage("flutter/platform_views",
                             "{\"method\":\"View.update\",\"args\":{"
                             "\"irrelevantField\":\"irrelevantValue\"}}"));

  // Wrong argument types.
  auto response6 = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(response6->WithMessage(
      "flutter/platform_views",
      "{\"method\":\"View.update\",\"args\":{\"viewId\":false,\"hitTestable\":"
      "123,\"focusable\":\"yes\"}}"));

  // Run the event loop and check our responses.
  RunLoopUntilIdle();
  response1->ExpectCompleted("");
  response2->ExpectCompleted("");
  response3->ExpectCompleted("");
  response4->ExpectCompleted("");
  response5->ExpectCompleted("");
  response6->ExpectCompleted("");
}

// This test makes sure that the PlatformView correctly returns a Surface
// instance that can surface the provided gr_context and external_view_embedder.
TEST_F(PlatformViewTests, CreateSurfaceTest) {
  MockPlatformViewDelegate delegate;

  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners",  // label
                           nullptr,         // platform
                           flutter_runner::CreateFMLTaskRunner(
                               async_get_default_dispatcher()),  // raster
                           nullptr,                              // ui
                           nullptr                               // io
      );

  // Test create surface callback function.
  sk_sp<GrDirectContext> gr_context = GrDirectContext::MakeMock(
      nullptr,
      flutter::MakeDefaultContextOptions(flutter::ContextType::kRender));
  std::shared_ptr<MockExternalViewEmbedder> external_view_embedder =
      std::make_shared<MockExternalViewEmbedder>();
  auto CreateSurfaceCallback = [&external_view_embedder, gr_context]() {
    return std::make_unique<flutter_runner::Surface>(
        "PlatformViewTest", external_view_embedder, gr_context.get());
  };

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetCreateSurfaceCallback(CreateSurfaceCallback)
                           .SetExternalViewEmbedder(external_view_embedder)
                           .Build();
  platform_view.NotifyCreated();

  RunLoopUntilIdle();

  EXPECT_EQ(gr_context.get(), delegate.surface()->GetContext());
  EXPECT_EQ(external_view_embedder.get(),
            platform_view.CreateExternalViewEmbedder().get());
}

// This test makes sure that the PlatformView correctly registers Scenic
// MetricsEvents sent to it via FIDL, correctly parses the metrics it receives,
// and calls the SetViewportMetrics callback with the appropriate parameters.
TEST_F(PlatformViewTests, SetViewportMetrics) {
  constexpr float kDPR = 2;
  constexpr uint32_t width = 640;
  constexpr uint32_t height = 480;

  MockPlatformViewDelegate delegate;
  EXPECT_EQ(delegate.metrics(), flutter::ViewportMetrics());

  MockParentViewportWatcher watcher;
  flutter::TaskRunners task_runners("test_runners", nullptr, nullptr, nullptr,
                                    nullptr);
  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetParentViewportWatcher(watcher.GetHandle())
                           .Build();
  RunLoopUntilIdle();
  EXPECT_EQ(delegate.metrics(), flutter::ViewportMetrics());

  watcher.SetLayout(width, height, kDPR);
  RunLoopUntilIdle();
  EXPECT_EQ(delegate.metrics(),
            flutter::ViewportMetrics(kDPR, std::round(width * kDPR),
                                     std::round(height * kDPR), -1.0, 0));
}

// This test makes sure that the PlatformView correctly registers semantics
// settings changes applied to it and calls the SetSemanticsEnabled /
// SetAccessibilityFeatures callbacks with the appropriate parameters.
TEST_F(PlatformViewTests, ChangesAccessibilitySettings) {
  MockPlatformViewDelegate delegate;
  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners", nullptr, nullptr, nullptr, nullptr);

  EXPECT_FALSE(delegate.semantics_enabled());
  EXPECT_EQ(delegate.semantics_features(), 0);

  auto platform_view =
      PlatformViewBuilder(delegate, std::move(task_runners)).Build();

  RunLoopUntilIdle();

  platform_view.SetSemanticsEnabled(true);

  EXPECT_TRUE(delegate.semantics_enabled());
  EXPECT_EQ(delegate.semantics_features(),
            static_cast<int32_t>(
                flutter::AccessibilityFeatureFlag::kAccessibleNavigation));

  platform_view.SetSemanticsEnabled(false);

  EXPECT_FALSE(delegate.semantics_enabled());
  EXPECT_EQ(delegate.semantics_features(), 0);
}

// This test makes sure that the PlatformView forwards messages on the
// "flutter/platform_views" channel for EnableWireframe.
TEST_F(PlatformViewTests, EnableWireframeTest) {
  MockPlatformViewDelegate delegate;
  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners", nullptr, nullptr, nullptr, nullptr);

  // Test wireframe callback function. If the message sent to the platform
  // view was properly handled and parsed, this function should be called,
  // setting |wireframe_enabled| to true.
  bool wireframe_enabled = false;
  auto EnableWireframeCallback = [&wireframe_enabled](bool should_enable) {
    wireframe_enabled = should_enable;
  };

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetEnableWireframeCallback(EnableWireframeCallback)
                           .Build();

  // Cast platform_view to its base view so we can have access to the public
  // "HandlePlatformMessage" function.
  auto base_view = static_cast<flutter::PlatformView*>(&platform_view);
  EXPECT_TRUE(base_view);

  // JSON for the message to be passed into the PlatformView.
  const uint8_t txt[] =
      "{"
      "    \"method\":\"View.enableWireframe\","
      "    \"args\": {"
      "       \"enable\":true"
      "    }"
      "}";

  std::unique_ptr<flutter::PlatformMessage> message =
      std::make_unique<flutter::PlatformMessage>(
          "flutter/platform_views", fml::MallocMapping::Copy(txt, sizeof(txt)),
          fml::RefPtr<flutter::PlatformMessageResponse>());
  base_view->HandlePlatformMessage(std::move(message));

  RunLoopUntilIdle();

  EXPECT_TRUE(wireframe_enabled);
}

// This test makes sure that the PlatformView forwards messages on the
// "flutter/platform_views" channel for Createview.
TEST_F(PlatformViewTests, CreateViewTest) {
  MockPlatformViewDelegate delegate;
  const uint64_t view_id = 42;
  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners",  // label
                           flutter_runner::CreateFMLTaskRunner(
                               async_get_default_dispatcher()),  // platform
                           nullptr,                              // raster
                           nullptr,                              // ui
                           nullptr                               // io
      );

  // Test wireframe callback function. If the message sent to the platform
  // view was properly handled and parsed, this function should be called,
  // setting |wireframe_enabled| to true.
  bool create_view_called = false;
  auto CreateViewCallback =
      [&create_view_called, this](
          int64_t view_id, flutter_runner::ViewCallback on_view_created,
          flutter_runner::ViewCreatedCallback on_view_bound, bool hit_testable,
          bool focusable) {
        create_view_called = true;
        on_view_created();
        fuchsia::ui::composition::ContentId content_id;
        on_view_bound(std::move(content_id), MakeChildViewWatcher());
      };

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetCreateViewCallback(CreateViewCallback)
                           .Build();

  // Cast platform_view to its base view so we can have access to the public
  // "HandlePlatformMessage" function.
  auto base_view = static_cast<flutter::PlatformView*>(&platform_view);
  EXPECT_TRUE(base_view);

  // JSON for the message to be passed into the PlatformView.
  std::ostringstream create_view_message;
  create_view_message << "{" << "  \"method\":\"View.create\","
                      << "  \"args\":{" << "    \"viewId\":" << view_id << ","
                      << "    \"hitTestable\":true," << "    \"focusable\":true"
                      << "  }" << "}";

  std::string create_view_call = create_view_message.str();
  std::unique_ptr<flutter::PlatformMessage> message =
      std::make_unique<flutter::PlatformMessage>(
          "flutter/platform_views",
          fml::MallocMapping::Copy(create_view_call.c_str(),
                                   create_view_call.size()),
          fml::RefPtr<flutter::PlatformMessageResponse>());
  base_view->HandlePlatformMessage(std::move(message));

  RunLoopUntilIdle();

  EXPECT_TRUE(create_view_called);

  // Platform view forwards the 'View.viewConnected' message on the
  // 'flutter/platform_views' channel when a view gets created.
  std::ostringstream view_connected_expected_out;
  view_connected_expected_out << "{" << "\"method\":\"View.viewConnected\","
                              << "\"args\":{" << "  \"viewId\":" << view_id
                              << "  }" << "}";

  ASSERT_NE(delegate.message(), nullptr);
  EXPECT_EQ(view_connected_expected_out.str(),
            ToString(delegate.message()->data()));
}

// This test makes sure that the PlatformView forwards messages on the
// "flutter/platform_views" channel for UpdateView.
TEST_F(PlatformViewTests, UpdateViewTest) {
  MockPlatformViewDelegate delegate;
  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners", nullptr, nullptr, nullptr, nullptr);

  std::optional<SkRect> occlusion_hint_for_test;
  std::optional<bool> hit_testable_for_test;
  std::optional<bool> focusable_for_test;
  auto UpdateViewCallback = [&occlusion_hint_for_test, &hit_testable_for_test,
                             &focusable_for_test](
                                int64_t view_id, SkRect occlusion_hint,
                                bool hit_testable, bool focusable) {
    occlusion_hint_for_test = occlusion_hint;
    hit_testable_for_test = hit_testable;
    focusable_for_test = focusable;
  };

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetUpdateViewCallback(UpdateViewCallback)
                           .Build();

  // Cast platform_view to its base view so we can have access to the public
  // "HandlePlatformMessage" function.
  auto base_view = static_cast<flutter::PlatformView*>(&platform_view);
  EXPECT_TRUE(base_view);

  // Send a basic message.
  const uint8_t json[] =
      "{"
      "    \"method\":\"View.update\","
      "    \"args\": {"
      "       \"viewId\":42,"
      "       \"hitTestable\":true,"
      "       \"focusable\":true"
      "    }"
      "}";
  std::unique_ptr<flutter::PlatformMessage> message =
      std::make_unique<flutter::PlatformMessage>(
          "flutter/platform_views",
          fml::MallocMapping::Copy(json, sizeof(json)),
          fml::RefPtr<flutter::PlatformMessageResponse>());
  base_view->HandlePlatformMessage(std::move(message));

  RunLoopUntilIdle();
  ASSERT_TRUE(occlusion_hint_for_test.has_value());
  ASSERT_TRUE(hit_testable_for_test.has_value());
  ASSERT_TRUE(focusable_for_test.has_value());
  EXPECT_EQ(occlusion_hint_for_test.value(), SkRect::MakeEmpty());
  EXPECT_EQ(hit_testable_for_test.value(), true);
  EXPECT_EQ(focusable_for_test.value(), true);

  // Reset for the next message.
  occlusion_hint_for_test.reset();
  hit_testable_for_test.reset();
  focusable_for_test.reset();

  // Send another basic message.
  const uint8_t json_false[] =
      "{"
      "    \"method\":\"View.update\","
      "    \"args\": {"
      "       \"viewId\":42,"
      "       \"hitTestable\":false,"
      "       \"focusable\":false"
      "    }"
      "}";
  std::unique_ptr<flutter::PlatformMessage> message_false =
      std::make_unique<flutter::PlatformMessage>(
          "flutter/platform_views",
          fml::MallocMapping::Copy(json_false, sizeof(json_false)),
          fml::RefPtr<flutter::PlatformMessageResponse>());
  base_view->HandlePlatformMessage(std::move(message_false));
  RunLoopUntilIdle();
  ASSERT_TRUE(occlusion_hint_for_test.has_value());
  ASSERT_TRUE(hit_testable_for_test.has_value());
  ASSERT_TRUE(focusable_for_test.has_value());
  EXPECT_EQ(occlusion_hint_for_test.value(), SkRect::MakeEmpty());
  EXPECT_EQ(hit_testable_for_test.value(), false);
  EXPECT_EQ(focusable_for_test.value(), false);

  // Reset for the next message.
  occlusion_hint_for_test.reset();
  hit_testable_for_test.reset();
  focusable_for_test.reset();

  // Send a message including an occlusion hint.
  const uint8_t json_occlusion_hint[] =
      "{"
      "    \"method\":\"View.update\","
      "    \"args\": {"
      "       \"viewId\":42,"
      "       \"hitTestable\":true,"
      "       \"focusable\":true,"
      "       \"viewOcclusionHintLTRB\":[0.1,0.2,0.3,0.4]"
      "    }"
      "}";
  std::unique_ptr<flutter::PlatformMessage> message_occlusion_hint =
      std::make_unique<flutter::PlatformMessage>(
          "flutter/platform_views",
          fml::MallocMapping::Copy(json_occlusion_hint,
                                   sizeof(json_occlusion_hint)),
          fml::RefPtr<flutter::PlatformMessageResponse>());
  base_view->HandlePlatformMessage(std::move(message_occlusion_hint));
  RunLoopUntilIdle();
  ASSERT_TRUE(occlusion_hint_for_test.has_value());
  ASSERT_TRUE(hit_testable_for_test.has_value());
  ASSERT_TRUE(focusable_for_test.has_value());
  EXPECT_EQ(occlusion_hint_for_test.value(),
            SkRect::MakeLTRB(0.1, 0.2, 0.3, 0.4));
  EXPECT_EQ(hit_testable_for_test.value(), true);
  EXPECT_EQ(focusable_for_test.value(), true);
}

// This test makes sure that the PlatformView forwards messages on the
// "flutter/platform_views" channel for DestroyView.
TEST_F(PlatformViewTests, DestroyViewTest) {
  MockPlatformViewDelegate delegate;
  const uint64_t view_id = 42;

  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners",  // label
                           flutter_runner::CreateFMLTaskRunner(
                               async_get_default_dispatcher()),  // platform
                           nullptr,                              // raster
                           nullptr,                              // ui
                           nullptr                               // io
      );

  bool destroy_view_called = false;

  auto on_destroy_view = [&destroy_view_called](
                             int64_t view_id,
                             flutter_runner::ViewIdCallback on_view_unbound) {
    destroy_view_called = true;
    fuchsia::ui::composition::ContentId content_id;
    on_view_unbound(std::move(content_id));
  };

  bool create_view_called = false;
  auto on_create_view = [&create_view_called, this](
                            int64_t view_id,
                            flutter_runner::ViewCallback on_view_created,
                            flutter_runner::ViewCreatedCallback on_view_bound,
                            bool hit_testable, bool focusable) {
    create_view_called = true;
    on_view_created();
    fuchsia::ui::composition::ContentId content_id;
    on_view_bound(std::move(content_id), MakeChildViewWatcher());
  };

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetCreateViewCallback(on_create_view)
                           .SetDestroyViewCallback(on_destroy_view)
                           .Build();

  // Cast platform_view to its base view so we can have access to the public
  // "HandlePlatformMessage" function.
  auto base_view = static_cast<flutter::PlatformView*>(&platform_view);
  EXPECT_TRUE(base_view);

  std::ostringstream create_message;
  create_message << "{" << "    \"method\":\"View.create\","
                 << "    \"args\": {" << "       \"viewId\":" << view_id << ","
                 << "       \"hitTestable\":true,"
                 << "       \"focusable\":true" << "    }" << "}";

  auto create_response = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(create_response->WithMessage(
      "flutter/platform_views", create_message.str()));
  RunLoopUntilIdle();

  delegate.Reset();

  // JSON for the message to be passed into the PlatformView.
  std::ostringstream dispose_message;
  dispose_message << "{" << "    \"method\":\"View.dispose\","
                  << "    \"args\": {" << "       \"viewId\":" << view_id
                  << "    }" << "}";

  std::string dispose_view_call = dispose_message.str();
  std::unique_ptr<flutter::PlatformMessage> message =
      std::make_unique<flutter::PlatformMessage>(
          "flutter/platform_views",
          fml::MallocMapping::Copy(dispose_view_call.c_str(),
                                   dispose_view_call.size()),
          fml::RefPtr<flutter::PlatformMessageResponse>());
  base_view->HandlePlatformMessage(std::move(message));

  RunLoopUntilIdle();

  EXPECT_TRUE(destroy_view_called);

  // Platform view forwards the 'View.viewDisconnected' message on the
  // 'flutter/platform_views' channel when a view gets destroyed.
  std::ostringstream view_disconnected_expected_out;
  view_disconnected_expected_out
      << "{" << "\"method\":\"View.viewDisconnected\"," << "\"args\":{"
      << "  \"viewId\":" << view_id << "  }" << "}";

  ASSERT_NE(delegate.message(), nullptr);
  EXPECT_EQ(view_disconnected_expected_out.str(),
            ToString(delegate.message()->data()));
}

// This test makes sure that the PlatformView forwards messages on the
// "flutter/platform_views" channel for View.focus.getCurrent and
// View.focus.getNext.
TEST_F(PlatformViewTests, GetFocusStatesTest) {
  MockPlatformViewDelegate delegate;
  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners", nullptr, nullptr, nullptr, nullptr);

  FakeViewRefFocused vrf;
  fidl::BindingSet<fuchsia::ui::views::ViewRefFocused> vrf_bindings;
  auto vrf_handle = vrf_bindings.AddBinding(&vrf);

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetViewRefFocused(std::move(vrf_handle))
                           .Build();

  // Cast platform_view to its base view so we can have access to the public
  // "HandlePlatformMessage" function.
  auto base_view = static_cast<flutter::PlatformView*>(&platform_view);
  EXPECT_TRUE(base_view);

  std::vector<bool> vrf_states{false, true,  true, false,
                               true,  false, true, true};

  for (std::size_t i = 0; i < vrf_states.size(); ++i) {
    // View.focus.getNext should complete with the next focus state.
    auto response1 = FakePlatformMessageResponse::Create();
    base_view->HandlePlatformMessage(response1->WithMessage(
        "flutter/platform_views", "{\"method\":\"View.focus.getNext\"}"));
    // Duplicate View.focus.getNext requests should complete empty.
    auto response2 = FakePlatformMessageResponse::Create();
    base_view->HandlePlatformMessage(response2->WithMessage(
        "flutter/platform_views", "{\"method\":\"View.focus.getNext\"}"));

    // Post watch events and make sure the hanging get is invoked each time.
    RunLoopUntilIdle();
    EXPECT_EQ(vrf.times_watched, i + 1);

    // Dispatch the next vrf event.
    vrf.ScheduleCallback(vrf_states[i]);
    RunLoopUntilIdle();

    // Make sure View.focus.getCurrent completes with the current focus state.
    auto response3 = FakePlatformMessageResponse::Create();
    base_view->HandlePlatformMessage(response3->WithMessage(
        "flutter/platform_views", "{\"method\":\"View.focus.getCurrent\"}"));
    // Duplicate View.focus.getCurrent are allowed.
    auto response4 = FakePlatformMessageResponse::Create();
    base_view->HandlePlatformMessage(response4->WithMessage(
        "flutter/platform_views", "{\"method\":\"View.focus.getCurrent\"}"));

    // Run event loop and check our results.
    RunLoopUntilIdle();
    response1->ExpectCompleted(vrf_states[i] ? "[true]" : "[false]");
    response2->ExpectCompleted("[null]");
    response3->ExpectCompleted(vrf_states[i] ? "[true]" : "[false]");
    response4->ExpectCompleted(vrf_states[i] ? "[true]" : "[false]");
  }
}

// This test makes sure that the PlatformView forwards messages on the
// "flutter/platform_views" channel for View.focus.request.
TEST_F(PlatformViewTests, RequestFocusTest) {
  MockPlatformViewDelegate delegate;
  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners",  // label
                           flutter_runner::CreateFMLTaskRunner(
                               async_get_default_dispatcher()),  // platform
                           nullptr,                              // raster
                           nullptr,                              // ui
                           nullptr                               // io
      );

  FakeFocuser focuser;
  fidl::BindingSet<fuchsia::ui::views::Focuser> focuser_bindings;
  auto focuser_handle = focuser_bindings.AddBinding(&focuser);

  bool create_view_called = false;
  auto on_create_view = [&create_view_called, this](
                            int64_t view_id,
                            flutter_runner::ViewCallback on_view_created,
                            flutter_runner::ViewCreatedCallback on_view_bound,
                            bool hit_testable, bool focusable) {
    create_view_called = true;
    on_view_created();
    fuchsia::ui::composition::ContentId content_id;
    on_view_bound(std::move(content_id), MakeChildViewWatcher());
  };

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetFocuser(std::move(focuser_handle))
                           .SetCreateViewCallback(on_create_view)
                           .Build();

  // Cast platform_view to its base view so we can have access to the public
  // "HandlePlatformMessage" function.
  auto base_view = static_cast<flutter::PlatformView*>(&platform_view);
  EXPECT_TRUE(base_view);

  uint64_t view_id = 42;

  std::ostringstream create_message;
  create_message << "{" << "    \"method\":\"View.create\","
                 << "    \"args\": {" << "       \"viewId\":" << view_id << ","
                 << "       \"hitTestable\":true,"
                 << "       \"focusable\":true" << "    }" << "}";

  // Dispatch the plaform message request.
  auto create_response = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(create_response->WithMessage(
      "flutter/platform_views", create_message.str()));

  RunLoopUntilIdle();

  // JSON for the message to be passed into the PlatformView.
  std::ostringstream focus_message;
  focus_message << "{" << "    \"method\":\"View.focus.requestById\","
                << "    \"args\": {" << "       \"viewId\":" << view_id
                << "    }" << "}";

  // Dispatch the plaform message request.
  auto focus_response = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(focus_response->WithMessage(
      "flutter/platform_views", focus_message.str()));
  RunLoopUntilIdle();

  focus_response->ExpectCompleted("[0]");
  EXPECT_TRUE(focuser.request_focus_called());
}

// This test tries to set focus on a view without creating it first
TEST_F(PlatformViewTests, RequestFocusNeverCreatedTest) {
  MockPlatformViewDelegate delegate;
  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners",  // label
                           flutter_runner::CreateFMLTaskRunner(
                               async_get_default_dispatcher()),  // platform
                           nullptr,                              // raster
                           nullptr,                              // ui
                           nullptr                               // io
      );

  FakeFocuser focuser;
  fidl::BindingSet<fuchsia::ui::views::Focuser> focuser_bindings;
  auto focuser_handle = focuser_bindings.AddBinding(&focuser);

  bool create_view_called = false;
  auto on_create_view = [&create_view_called, this](
                            int64_t view_id,
                            flutter_runner::ViewCallback on_view_created,
                            flutter_runner::ViewCreatedCallback on_view_bound,
                            bool hit_testable, bool focusable) {
    create_view_called = true;
    on_view_created();
    fuchsia::ui::composition::ContentId content_id;
    on_view_bound(std::move(content_id), MakeChildViewWatcher());
  };

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetFocuser(std::move(focuser_handle))
                           .SetCreateViewCallback(on_create_view)
                           .Build();

  auto base_view = static_cast<flutter::PlatformView*>(&platform_view);
  EXPECT_TRUE(base_view);

  uint64_t view_id = 42;

  std::ostringstream focus_message;
  focus_message << "{" << "    \"method\":\"View.focus.requestById\","
                << "    \"args\": {" << "       \"viewId\":" << view_id
                << "    }" << "}";

  // Dispatch the plaform message request.
  auto focus_response = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(focus_response->WithMessage(
      "flutter/platform_views", focus_message.str()));
  RunLoopUntilIdle();

  focus_response->ExpectCompleted("[1]");
  EXPECT_FALSE(focuser.request_focus_called());
}

TEST_F(PlatformViewTests, RequestFocusDisposedTest) {
  MockPlatformViewDelegate delegate;
  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners",  // label
                           flutter_runner::CreateFMLTaskRunner(
                               async_get_default_dispatcher()),  // platform
                           nullptr,                              // raster
                           nullptr,                              // ui
                           nullptr                               // io
      );

  FakeFocuser focuser;
  fidl::BindingSet<fuchsia::ui::views::Focuser> focuser_bindings;
  auto focuser_handle = focuser_bindings.AddBinding(&focuser);

  bool create_view_called = false;
  auto on_create_view = [&create_view_called, this](
                            int64_t view_id,
                            flutter_runner::ViewCallback on_view_created,
                            flutter_runner::ViewCreatedCallback on_view_bound,
                            bool hit_testable, bool focusable) {
    create_view_called = true;
    on_view_created();
    fuchsia::ui::composition::ContentId content_id;
    on_view_bound(std::move(content_id), MakeChildViewWatcher());
  };

  bool destroy_view_called = false;

  auto on_destroy_view = [&destroy_view_called](
                             int64_t view_id,
                             flutter_runner::ViewIdCallback on_view_unbound) {
    destroy_view_called = true;
    fuchsia::ui::composition::ContentId content_id;
    on_view_unbound(std::move(content_id));
  };

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetFocuser(std::move(focuser_handle))
                           .SetCreateViewCallback(on_create_view)
                           .SetDestroyViewCallback(on_destroy_view)
                           .Build();

  auto base_view = static_cast<flutter::PlatformView*>(&platform_view);
  EXPECT_TRUE(base_view);

  uint64_t view_id = 42;

  // Create a new view
  std::ostringstream create_message;
  create_message << "{" << "    \"method\":\"View.create\","
                 << "    \"args\": {" << "       \"viewId\":" << view_id << ","
                 << "       \"hitTestable\":true,"
                 << "       \"focusable\":true" << "    }" << "}";

  auto create_response = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(create_response->WithMessage(
      "flutter/platform_views", create_message.str()));
  RunLoopUntilIdle();

  EXPECT_FALSE(destroy_view_called);
  // Dispose of the view
  std::ostringstream dispose_message;
  dispose_message << "{" << "    \"method\":\"View.dispose\","
                  << "    \"args\": {" << "       \"viewId\":" << view_id
                  << "    }" << "}";

  auto dispose_response = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(dispose_response->WithMessage(
      "flutter/platform_views", dispose_message.str()));
  RunLoopUntilIdle();
  EXPECT_TRUE(destroy_view_called);

  // Request focus on newly disposed view
  std::ostringstream focus_message;
  focus_message << "{" << "    \"method\":\"View.focus.requestById\","
                << "    \"args\": {" << "       \"viewId\":" << view_id
                << "    }" << "}";

  auto focus_response = FakePlatformMessageResponse::Create();
  base_view->HandlePlatformMessage(focus_response->WithMessage(
      "flutter/platform_views", focus_message.str()));
  RunLoopUntilIdle();

  // Expect it to fail
  focus_response->ExpectCompleted("[1]");
  EXPECT_FALSE(focuser.request_focus_called());
}

// Makes sure that OnKeyEvent is dispatched as a platform message.
TEST_F(PlatformViewTests, OnKeyEvent) {
  struct EventFlow {
    fuchsia::ui::input3::KeyEvent event;
    fuchsia::ui::input3::KeyEventStatus expected_key_event_status;
    std::string expected_platform_message;
  };

  MockPlatformViewDelegate delegate;
  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners", nullptr, nullptr, nullptr, nullptr);

  fuchsia::ui::input3::KeyboardHandle keyboard_service;
  MockKeyboard keyboard(keyboard_service.NewRequest());

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetKeyboard(std::move(keyboard_service))
                           .Build();
  RunLoopUntilIdle();

  std::vector<EventFlow> events;
  // Press A.  Get 'a'.
  // The HID usage for the key A is 0x70004, or 458756.
  events.emplace_back(EventFlow{
      MakeEvent(fuchsia::ui::input3::KeyEventType::PRESSED, std::nullopt,
                fuchsia::input::Key::A),
      fuchsia::ui::input3::KeyEventStatus::HANDLED,
      R"({"type":"keydown","keymap":"fuchsia","hidUsage":458756,"codePoint":97,"modifiers":0})",
  });
  // Release A. Get 'a' release.
  events.emplace_back(EventFlow{
      MakeEvent(fuchsia::ui::input3::KeyEventType::RELEASED, std::nullopt,
                fuchsia::input::Key::A),
      fuchsia::ui::input3::KeyEventStatus::HANDLED,
      R"({"type":"keyup","keymap":"fuchsia","hidUsage":458756,"codePoint":97,"modifiers":0})",
  });
  // Press CAPS_LOCK.  Modifier now active.
  events.emplace_back(EventFlow{
      MakeEvent(fuchsia::ui::input3::KeyEventType::PRESSED,
                fuchsia::ui::input3::Modifiers::CAPS_LOCK,
                fuchsia::input::Key::CAPS_LOCK),
      fuchsia::ui::input3::KeyEventStatus::HANDLED,
      R"({"type":"keydown","keymap":"fuchsia","hidUsage":458809,"codePoint":0,"modifiers":1})",
  });
  // Press A.  Get 'A'.
  events.emplace_back(EventFlow{
      MakeEvent(fuchsia::ui::input3::KeyEventType::PRESSED, std::nullopt,
                fuchsia::input::Key::A),
      fuchsia::ui::input3::KeyEventStatus::HANDLED,
      R"({"type":"keydown","keymap":"fuchsia","hidUsage":458756,"codePoint":65,"modifiers":1})",
  });
  // Release CAPS_LOCK.
  events.emplace_back(EventFlow{
      MakeEvent(fuchsia::ui::input3::KeyEventType::RELEASED,
                fuchsia::ui::input3::Modifiers::CAPS_LOCK,
                fuchsia::input::Key::CAPS_LOCK),
      fuchsia::ui::input3::KeyEventStatus::HANDLED,
      R"({"type":"keyup","keymap":"fuchsia","hidUsage":458809,"codePoint":0,"modifiers":1})",
  });
  // Press A again.  This time get 'A'.
  // CAPS_LOCK is latched active even if it was just released.
  events.emplace_back(EventFlow{
      MakeEvent(fuchsia::ui::input3::KeyEventType::PRESSED, std::nullopt,
                fuchsia::input::Key::A),
      fuchsia::ui::input3::KeyEventStatus::HANDLED,
      R"({"type":"keydown","keymap":"fuchsia","hidUsage":458756,"codePoint":65,"modifiers":1})",
  });

  for (const auto& event : events) {
    fuchsia::ui::input3::KeyEvent e;
    event.event.Clone(&e);
    fuchsia::ui::input3::KeyEventStatus key_event_status{0u};
    keyboard.listener_->OnKeyEvent(
        std::move(e),
        [&key_event_status](fuchsia::ui::input3::KeyEventStatus status) {
          key_event_status = status;
        });
    RunLoopUntilIdle();

    ASSERT_NOTNULL(delegate.message());
    EXPECT_EQ(event.expected_platform_message,
              ToString(delegate.message()->data()));
    EXPECT_EQ(event.expected_key_event_status, key_event_status);
  }
}

TEST_F(PlatformViewTests, OnShaderWarmup) {
  MockPlatformViewDelegate delegate;
  flutter::TaskRunners task_runners =
      flutter::TaskRunners("test_runners", nullptr, nullptr, nullptr, nullptr);

  uint64_t width = 200;
  uint64_t height = 100;
  std::vector<std::string> shaders = {"foo.skp", "bar.skp", "baz.skp"};

  OnShaderWarmupCallback on_shader_warmup_callback =
      [&](const std::vector<std::string>& shaders_in,
          std::function<void(uint32_t)> completion_callback, uint64_t width_in,
          uint64_t height_in) {
        ASSERT_EQ(shaders.size(), shaders_in.size());
        for (size_t i = 0; i < shaders_in.size(); i++) {
          ASSERT_EQ(shaders[i], shaders_in[i]);
        }
        ASSERT_EQ(width, width_in);
        ASSERT_EQ(height, height_in);

        completion_callback(shaders_in.size());
      };

  auto platform_view = PlatformViewBuilder(delegate, std::move(task_runners))
                           .SetShaderWarmupCallback(on_shader_warmup_callback)
                           .Build();

  std::ostringstream shaders_array_ostream;
  shaders_array_ostream << "[ ";
  for (auto it = shaders.begin(); it != shaders.end(); ++it) {
    shaders_array_ostream << "\"" << *it << "\"";
    if (std::next(it) != shaders.end()) {
      shaders_array_ostream << ", ";
    }
  }
  shaders_array_ostream << "]";

  std::string shaders_array_string = shaders_array_ostream.str();

  // Create initial view for testing.
  std::ostringstream warmup_shaders_ostream;
  warmup_shaders_ostream << "{" << "  \"method\":\"WarmupSkps\","
                         << "  \"args\":{"
                         << "    \"shaders\":" << shaders_array_string << ","
                         << "    \"width\":" << width << ","
                         << "    \"height\":" << height << "  }" << "}\n";
  std::string warmup_shaders_string = warmup_shaders_ostream.str();

  fml::RefPtr<TestPlatformMessageResponse> response(
      new TestPlatformMessageResponse);
  static_cast<flutter::PlatformView*>(&platform_view)
      ->HandlePlatformMessage(std::make_unique<flutter::PlatformMessage>(
          "fuchsia/shader_warmup",
          fml::MallocMapping::Copy(warmup_shaders_string.c_str(),
                                   warmup_shaders_string.size()),
          response));
  RunLoopUntilIdle();
  ASSERT_TRUE(response->is_complete());

  std::ostringstream expected_result_ostream;
  expected_result_ostream << "[" << shaders.size() << "]";
  std::string expected_result_string = expected_result_ostream.str();
  EXPECT_EQ(expected_result_string, response->result_string);
}

TEST_F(PlatformViewTests, TouchSourceLogicalToPhysicalConversion) {
  constexpr uint32_t width = 640;
  constexpr uint32_t height = 480;
  constexpr std::array<std::array<float, 2>, 2> kRect = {
      {{0, 0}, {width, height}}};
  constexpr std::array<float, 9> kIdentity = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  constexpr fuchsia::ui::pointer::TouchInteractionId kIxnOne = {
      .device_id = 0u, .pointer_id = 1u, .interaction_id = 2u};

  MockPlatformViewDelegate delegate;
  flutter::TaskRunners task_runners("test_runners", nullptr, nullptr, nullptr,
                                    nullptr);

  MockParentViewportWatcher viewport_watcher;
  FakeTouchSource touch_server;
  fidl::BindingSet<fuchsia::ui::pointer::TouchSource> touch_bindings;
  auto touch_handle = touch_bindings.AddBinding(&touch_server);
  auto platform_view =
      PlatformViewBuilder(delegate, std::move(task_runners))
          .SetParentViewportWatcher(viewport_watcher.GetHandle())
          .SetTouchSource(std::move(touch_handle))
          .Build();
  RunLoopUntilIdle();
  EXPECT_EQ(delegate.pointer_packets().size(), 0u);

  viewport_watcher.SetLayout(width, height);
  RunLoopUntilIdle();
  EXPECT_EQ(delegate.metrics(),
            flutter::ViewportMetrics(1, width, height, -1, 0));

  // Inject
  std::vector<fuchsia::ui::pointer::TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(/* in nanoseconds */ 1111789u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kIxnOne, fuchsia::ui::pointer::EventPhase::ADD,
                     {width / 2, height / 2})
          .AddResult(
              {.interaction = kIxnOne,
               .status = fuchsia::ui::pointer::TouchInteractionStatus::GRANTED})
          .BuildAsVector();
  touch_server.ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  // Unpack
  std::vector<std::unique_ptr<flutter::PointerDataPacket>> packets =
      delegate.TakePointerDataPackets();
  ASSERT_EQ(packets.size(), 1u);
  std::vector<flutter::PointerData> flutter_events;
  UnpackPointerPacket(flutter_events, std::move(packets[0]));

  // Examine phases
  ASSERT_EQ(flutter_events.size(), 2u);
  EXPECT_EQ(flutter_events[0].change, flutter::PointerData::Change::kAdd);
  EXPECT_EQ(flutter_events[1].change, flutter::PointerData::Change::kDown);

  // Examine coordinates
  EXPECT_EQ(flutter_events[0].physical_x, width / 2);
  EXPECT_EQ(flutter_events[0].physical_y, height / 2);
  EXPECT_EQ(flutter_events[1].physical_x, width / 2);
  EXPECT_EQ(flutter_events[1].physical_y, height / 2);
}

}  // namespace flutter_runner::testing
