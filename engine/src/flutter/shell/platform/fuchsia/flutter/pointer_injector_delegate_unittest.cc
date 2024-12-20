// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fuchsia/ui/pointerinjector/cpp/fidl.h>
#include <fuchsia/ui/views/cpp/fidl.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/async-loop/default.h>
#include <lib/async/cpp/task.h>
#include <lib/zx/eventpair.h>

#include <lib/fidl/cpp/binding_set.h>

#include "pointer_injector_delegate.h"
#include "tests/fakes/mock_injector_registry.h"
#include "tests/fakes/platform_message.h"

namespace flutter_runner::testing {

using fup_DeviceType = fuchsia::ui::pointerinjector::DeviceType;
using fup_DispatchPolicy = fuchsia::ui::pointerinjector::DispatchPolicy;
using fup_EventPhase = fuchsia::ui::pointerinjector::EventPhase;
using fup_RegistryHandle = fuchsia::ui::pointerinjector::RegistryHandle;
using fuv_ViewRef = fuchsia::ui::views::ViewRef;

namespace {

// clang-format off
  static constexpr std::array<float, 9> kIdentityMatrix = {
    1, 0, 0, // column one
    0, 1, 0, // column two
    0, 0, 1, // column three
  };
// clang-format on

rapidjson::Value ParsePlatformMessage(std::string json) {
  rapidjson::Document document;
  document.Parse(json);
  if (document.HasParseError() || !document.IsObject()) {
    FML_LOG(ERROR) << "Could not parse document";
    return rapidjson::Value();
  }
  return document.GetObject();
}

zx_koid_t ExtractKoid(const zx::object_base& object) {
  zx_info_handle_basic_t info{};
  if (object.get_info(ZX_INFO_HANDLE_BASIC, &info, sizeof(info), nullptr,
                      nullptr) != ZX_OK) {
    return ZX_KOID_INVALID;  // no info
  }

  return info.koid;
}

zx_koid_t ExtractKoid(const fuv_ViewRef& view_ref) {
  return ExtractKoid(view_ref.reference);
}

class PlatformMessageBuilder {
 public:
  PlatformMessageBuilder& SetViewId(uint64_t view_id) {
    view_id_ = view_id;
    return *this;
  }

  PlatformMessageBuilder& SetPointerX(float x) {
    pointer_x_ = x;
    return *this;
  }

  PlatformMessageBuilder& SetPointerY(float y) {
    pointer_y_ = y;
    return *this;
  }

  PlatformMessageBuilder& SetPhase(int phase) {
    phase_ = phase;
    return *this;
  }

  PlatformMessageBuilder& SetPointerId(int pointer_id) {
    pointer_id_ = pointer_id;
    return *this;
  }

  PlatformMessageBuilder& SetTraceFlowId(int trace_flow_id) {
    trace_flow_id_ = trace_flow_id;
    return *this;
  }

  PlatformMessageBuilder& SetLogicalWidth(float width) {
    width_ = width;
    return *this;
  }

  PlatformMessageBuilder& SetLogicalHeight(float height) {
    height_ = height;
    return *this;
  }

  PlatformMessageBuilder& SetTimestamp(int timestamp) {
    timestamp_ = timestamp;
    return *this;
  }

  rapidjson::Value Build() {
    std::ostringstream message;
    message << "{" << "    \"method\":\""
            << PointerInjectorDelegate::kPointerInjectorMethodPrefix << "\","
            << "    \"args\": {" << "        \"viewId\":" << view_id_ << ","
            << "        \"x\":" << pointer_x_ << ","
            << "        \"y\":" << pointer_y_ << ","
            << "        \"phase\":" << phase_ << ","
            << "        \"pointerId\":" << pointer_id_ << ","
            << "        \"traceFlowId\":" << trace_flow_id_ << ","
            << "        \"viewRef\":" << view_ref_.reference.get() << ","
            << "        \"logicalWidth\":" << width_ << ","
            << "        \"logicalHeight\":" << height_ << ","
            << "        \"timestamp\":" << timestamp_ << "   }" << "}";
    return ParsePlatformMessage(message.str());
  }

 private:
  uint64_t view_id_ = 0;
  float pointer_x_ = 0.f, pointer_y_ = 0.f;
  int phase_ = 1, pointer_id_ = 0, trace_flow_id_ = 0;
  fuv_ViewRef view_ref_;
  float width_ = 0.f, height_ = 0.f;
  int timestamp_ = 0;
};

}  // namespace

class PointerInjectorDelegateTest : public ::testing::Test,
                                    public ::testing::WithParamInterface<bool> {
 protected:
  PointerInjectorDelegateTest()
      : loop_(&kAsyncLoopConfigAttachToCurrentThread) {}

  // TODO(fxbug.dev/104285): Replace the RunLoop methods with the one provided
  // by the sdk.
  void RunLoopUntilIdle() { loop_.RunUntilIdle(); }

  bool RunGivenLoopWithTimeout(async::Loop* loop, zx::duration timeout) {
    // This cannot be a local variable because the delayed task below can
    // execute after this function returns.
    auto canceled = std::make_shared<bool>(false);
    bool timed_out = false;
    async::PostDelayedTask(
        loop->dispatcher(),
        [loop, canceled, &timed_out] {
          if (*canceled) {
            return;
          }
          timed_out = true;
          loop->Quit();
        },
        timeout);
    loop->Run();
    loop->ResetQuit();

    if (!timed_out) {
      *canceled = true;
    }
    return timed_out;
  }

  bool RunLoopWithTimeoutOrUntil(fit::function<bool()> condition,
                                 zx::duration timeout,
                                 zx::duration step) {
    const zx::time timeout_deadline = zx::deadline_after(timeout);

    while (zx::clock::get_monotonic() < timeout_deadline &&
           loop_.GetState() == ASYNC_LOOP_RUNNABLE) {
      if (condition()) {
        loop_.ResetQuit();
        return true;
      }

      if (step == zx::duration::infinite()) {
        // Performs a single unit of work, possibly blocking until there is work
        // to do or the timeout deadline arrives.
        loop_.Run(timeout_deadline, true);
      } else {
        // Performs work until the step deadline arrives.
        RunGivenLoopWithTimeout(&loop_, step);
      }
    }

    loop_.ResetQuit();
    return condition();
  }

  void RunLoopUntil(fit::function<bool()> condition,
                    zx::duration step = zx::msec(10)) {
    RunLoopWithTimeoutOrUntil(std::move(condition), zx::duration::infinite(),
                              step);
  }

  void SetUp() override {
    fuchsia::ui::views::ViewRefControl view_ref_control;
    fuchsia::ui::views::ViewRef view_ref;
    auto status = zx::eventpair::create(
        /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
    ASSERT_EQ(status, ZX_OK);
    view_ref_control.reference.replace(
        ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
        &view_ref_control.reference);
    view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

    host_view_ref_ = std::move(view_ref);

    fup_RegistryHandle registry;
    registry_ = std::make_unique<MockInjectorRegistry>(registry.NewRequest());

    fuv_ViewRef host_view_ref_clone;
    fidl::Clone(host_view_ref_, &host_view_ref_clone);

    pointer_injector_delegate_ = std::make_unique<PointerInjectorDelegate>(
        std::move(registry), std::move(host_view_ref_clone));
  }

  void CreateView(uint64_t view_id,
                  std::optional<fuv_ViewRef> view_ref = std::nullopt) {
    fuv_ViewRef ref;
    if (view_ref.has_value()) {
      ref = std::move(*view_ref);
    } else {
      fuchsia::ui::views::ViewRefControl view_ref_control;
      fuchsia::ui::views::ViewRef view_ref;
      auto status = zx::eventpair::create(
          /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
      ASSERT_EQ(status, ZX_OK);
      view_ref_control.reference.replace(
          ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
          &view_ref_control.reference);
      view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

      ref = std::move(view_ref);
    }
    pointer_injector_delegate_->OnCreateView(view_id, std::move(ref));
  }

  std::unique_ptr<PointerInjectorDelegate> pointer_injector_delegate_;
  std::unique_ptr<MockInjectorRegistry> registry_;
  fuv_ViewRef host_view_ref_;

 private:
  async::Loop loop_;
};

TEST_P(PointerInjectorDelegateTest, IncorrectPlatformMessage_ShouldFail) {
  const uint64_t view_id = 1;

  // Create a view.
  CreateView(view_id);

  // A platform message in incorrect JSON format should fail.
  {
    auto response = FakePlatformMessageResponse::Create();

    EXPECT_FALSE(pointer_injector_delegate_->HandlePlatformMessage(
        ParsePlatformMessage("{Incorrect Json}"), response));
  }

  // |PointerInjectorDelegate| only handles "View.Pointerinjector.inject"
  // platform messages.
  {
    auto response = FakePlatformMessageResponse::Create();

    EXPECT_FALSE(pointer_injector_delegate_->HandlePlatformMessage(
        ParsePlatformMessage("{\"method\":\"View.focus.getCurrent\"}"),
        response));
  }

  // A platform message with no args should fail.
  {
    auto response = FakePlatformMessageResponse::Create();

    EXPECT_FALSE(pointer_injector_delegate_->HandlePlatformMessage(
        ParsePlatformMessage("{\"method\":\"View.Pointerinjector.inject\"}"),
        response));
  }
}

TEST_P(PointerInjectorDelegateTest, ViewsReceiveInjectedEvents) {
  const uint64_t num_events = 150;

  // Inject |num_events| platform messages for view 1.
  {
    const uint64_t view_id = 1;

    CreateView(view_id);

    fuchsia::ui::views::ViewRefControl view_ref_control;
    fuchsia::ui::views::ViewRef view_ref;
    auto status = zx::eventpair::create(
        /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
    ASSERT_EQ(status, ZX_OK);
    view_ref_control.reference.replace(
        ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
        &view_ref_control.reference);
    view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

    for (size_t i = 0; i < num_events; i++) {
      auto response = FakePlatformMessageResponse::Create();

      EXPECT_TRUE(pointer_injector_delegate_->HandlePlatformMessage(
          PlatformMessageBuilder().SetViewId(view_id).Build(), response));

      response->ExpectCompleted("[0]");
    }
  }

  // Inject |num_events| platform messages for view 2.
  {
    const uint64_t view_id = 2;

    CreateView(view_id);

    fuchsia::ui::views::ViewRefControl view_ref_control;
    fuchsia::ui::views::ViewRef view_ref;
    auto status = zx::eventpair::create(
        /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
    ASSERT_EQ(status, ZX_OK);
    view_ref_control.reference.replace(
        ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
        &view_ref_control.reference);
    view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

    for (size_t i = 0; i < num_events; i++) {
      auto response = FakePlatformMessageResponse::Create();

      EXPECT_TRUE(pointer_injector_delegate_->HandlePlatformMessage(
          PlatformMessageBuilder().SetViewId(view_id).Build(), response));

      response->ExpectCompleted("[0]");
    }
  }

  // The mock Pointerinjector registry server receives |num_events| pointer
  // events from |f.u.p.Device.Inject| calls for each view.
  RunLoopUntil(
      [this] { return registry_->num_events_received() == 2 * num_events; });

  // The mock Pointerinjector registry server receives a
  // |f.u.p.Registry.Register| call for each view.
  EXPECT_TRUE(registry_->num_register_calls() == 2);
}

TEST_P(PointerInjectorDelegateTest,
       ViewsDontReceivePointerEventsBeforeCreation) {
  const uint64_t num_events = 150;
  const uint64_t view_id_1 = 1;

  // Inject |num_events| platform messages for |view_id_1|.
  {
    fuchsia::ui::views::ViewRefControl view_ref_control;
    fuchsia::ui::views::ViewRef view_ref;
    auto status = zx::eventpair::create(
        /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
    ASSERT_EQ(status, ZX_OK);
    view_ref_control.reference.replace(
        ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
        &view_ref_control.reference);
    view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

    for (size_t i = 0; i < num_events; i++) {
      auto response = FakePlatformMessageResponse::Create();

      // The platform message is *silently* accepted for non-existent views, in
      // order to cleanly handle the lifecycle case where the child view is
      // forcibly killed. By doing so, products avoid "MissingPluginException"
      // log spam.
      EXPECT_TRUE(pointer_injector_delegate_->HandlePlatformMessage(
          PlatformMessageBuilder().SetViewId(view_id_1).Build(), response));
    }
  }

  const uint64_t view_id_2 = 2;

  // Inject |num_events| platform messages for |view_id_2|.
  {
    fuchsia::ui::views::ViewRefControl view_ref_control;
    fuchsia::ui::views::ViewRef view_ref;
    auto status = zx::eventpair::create(
        /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
    ASSERT_EQ(status, ZX_OK);
    view_ref_control.reference.replace(
        ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
        &view_ref_control.reference);
    view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

    for (size_t i = 0; i < num_events; i++) {
      auto response = FakePlatformMessageResponse::Create();

      // The platform message is *silently* accepted for non-existent views, in
      // order to cleanly handle the lifecycle case where the child view is
      // forcibly killed. By doing so, products avoid "MissingPluginException"
      // log spam.
      EXPECT_TRUE(pointer_injector_delegate_->HandlePlatformMessage(
          PlatformMessageBuilder().SetViewId(view_id_2).Build(), response));
    }
  }

  RunLoopUntilIdle();

  // The views do not receive any pointer events till they get created.
  EXPECT_TRUE(registry_->num_events_received() == 0);
}

// PointerInjectorDelegate should generate a correct |f.u.p.Config| from a
// platform message.
TEST_P(PointerInjectorDelegateTest, ValidRegistrationConfigTest) {
  const uint64_t view_id = 1;

  const float x = 2.f, y = 2.f, width = 5.f, height = 5.f;
  const int phase = 2, pointer_id = 5, trace_flow_id = 5, timestamp = 10;

  auto response = FakePlatformMessageResponse::Create();

  fuchsia::ui::views::ViewRefControl view_ref_control;
  fuchsia::ui::views::ViewRef view_ref;
  auto status = zx::eventpair::create(
      /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
  ZX_ASSERT(status == ZX_OK);
  view_ref_control.reference.replace(
      ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
      &view_ref_control.reference);
  view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

  // Create the view.
  fuv_ViewRef view_ref_clone;
  fidl::Clone(view_ref, &view_ref_clone);
  CreateView(view_id, std::move(view_ref_clone));

  // Inject a platform message.
  EXPECT_TRUE(pointer_injector_delegate_->HandlePlatformMessage(
      PlatformMessageBuilder()
          .SetViewId(view_id)
          .SetPointerX(x)
          .SetPointerY(y)
          .SetPhase(phase)
          .SetPointerId(pointer_id)
          .SetTraceFlowId(trace_flow_id)
          .SetLogicalWidth(width)
          .SetLogicalHeight(height)
          .SetTimestamp(timestamp)
          .Build(),
      response));

  response->ExpectCompleted("[0]");

  // The mock Pointerinjector registry server receives a pointer event from
  // |f.u.p.Device.Inject| call for the view.
  RunLoopUntil([this] { return registry_->num_events_received() == 1; });

  // The mock Pointerinjector registry server receives a
  // |f.u.p.Registry.Register| call for the view.
  ASSERT_TRUE(registry_->num_register_calls() == 1);

  const auto& config = registry_->config();

  ASSERT_TRUE(config.has_device_id());
  EXPECT_EQ(config.device_id(), 1u);

  ASSERT_TRUE(config.has_device_type());
  EXPECT_EQ(config.device_type(), fup_DeviceType::TOUCH);

  ASSERT_TRUE(config.has_dispatch_policy());
  EXPECT_EQ(config.dispatch_policy(), fup_DispatchPolicy::EXCLUSIVE_TARGET);

  ASSERT_TRUE(config.has_context());
  ASSERT_TRUE(config.context().is_view());
  EXPECT_EQ(ExtractKoid(config.context().view()), ExtractKoid(host_view_ref_));

  ASSERT_TRUE(config.has_target());
  ASSERT_TRUE(config.target().is_view());
  EXPECT_EQ(ExtractKoid(config.target().view()), ExtractKoid(view_ref));

  ASSERT_TRUE(config.has_viewport());
  ASSERT_TRUE(config.viewport().has_viewport_to_context_transform());
  EXPECT_EQ(config.viewport().viewport_to_context_transform(), kIdentityMatrix);

  std::array<std::array<float, 2>, 2> extents{{{0, 0}, {width, height}}};
  ASSERT_TRUE(config.viewport().has_extents());
  EXPECT_EQ(config.viewport().extents(), extents);
}

// PointerInjectorDelegate generates a correct f.u.p.Event from the platform
// message.
TEST_P(PointerInjectorDelegateTest, ValidPointerEventTest) {
  const uint64_t view_id = 1;

  const float x = 2.f, y = 2.f, width = 5.f, height = 5.f;
  const int phase = 2, pointer_id = 5, trace_flow_id = 5, timestamp = 10;

  auto response = FakePlatformMessageResponse::Create();

  fuchsia::ui::views::ViewRefControl view_ref_control;
  fuchsia::ui::views::ViewRef view_ref;
  auto status = zx::eventpair::create(
      /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
  ZX_ASSERT(status == ZX_OK);
  view_ref_control.reference.replace(
      ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
      &view_ref_control.reference);
  view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

  // Create the view.
  fuv_ViewRef view_ref_clone;
  fidl::Clone(view_ref, &view_ref_clone);
  CreateView(view_id, std::move(view_ref_clone));

  // Inject a platform message.
  EXPECT_TRUE(pointer_injector_delegate_->HandlePlatformMessage(
      PlatformMessageBuilder()
          .SetViewId(view_id)
          .SetPointerX(x)
          .SetPointerY(y)
          .SetPhase(phase)
          .SetPointerId(pointer_id)
          .SetTraceFlowId(trace_flow_id)
          .SetLogicalWidth(width)
          .SetLogicalHeight(height)
          .SetTimestamp(timestamp)
          .Build(),
      response));

  response->ExpectCompleted("[0]");

  // The mock Pointerinjector registry server receives a pointer event from
  // |f.u.p.Device.Inject| call for the view.
  RunLoopUntil([this] { return registry_->num_events_received() == 1; });

  // The mock Pointerinjector registry server receives a
  // |f.u.p.Registry.Register| call for the view.
  ASSERT_TRUE(registry_->num_register_calls() == 1);

  const auto& events = registry_->events();

  ASSERT_EQ(events.size(), 1u);

  const auto& event = events[0];

  ASSERT_TRUE(event.has_timestamp());
  EXPECT_EQ(event.timestamp(), timestamp);

  ASSERT_TRUE(event.has_trace_flow_id());
  EXPECT_EQ(event.trace_flow_id(), static_cast<uint64_t>(trace_flow_id));

  ASSERT_TRUE(event.has_data());
  ASSERT_TRUE(event.data().is_pointer_sample());

  const auto& pointer_sample = event.data().pointer_sample();

  ASSERT_TRUE(pointer_sample.has_pointer_id());
  ASSERT_TRUE(pointer_sample.has_phase());
  ASSERT_TRUE(pointer_sample.has_position_in_viewport());
  EXPECT_EQ(pointer_sample.pointer_id(), static_cast<uint32_t>(pointer_id));
  EXPECT_EQ(pointer_sample.phase(), static_cast<fup_EventPhase>(phase));
  EXPECT_THAT(pointer_sample.position_in_viewport(),
              ::testing::ElementsAre(x, y));
}

TEST_P(PointerInjectorDelegateTest, DestroyedViewsDontGetPointerEvents) {
  const uint64_t view_id = 1, num_events = 150;

  fuchsia::ui::views::ViewRefControl view_ref_control;
  fuchsia::ui::views::ViewRef view_ref;
  auto status = zx::eventpair::create(
      /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
  ZX_ASSERT(status == ZX_OK);
  view_ref_control.reference.replace(
      ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
      &view_ref_control.reference);
  view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

  // Create the view.
  CreateView(view_id);

  // Inject |num_events| platform messages.
  for (size_t i = 0; i < num_events; i++) {
    auto response = FakePlatformMessageResponse::Create();

    EXPECT_TRUE(pointer_injector_delegate_->HandlePlatformMessage(
        PlatformMessageBuilder().SetViewId(view_id).Build(), response));

    response->ExpectCompleted("[0]");
  }

  // Destroy the view.
  pointer_injector_delegate_->OnDestroyView(view_id);

  // The view does not receive |num_events| pointer events as it gets destroyed
  // before all the pointer events could be dispatched.
  const zx::duration timeout = zx::sec(1), step = zx::msec(10);
  EXPECT_FALSE(RunLoopWithTimeoutOrUntil(
      [this] { return registry_->num_events_received() == num_events; },
      timeout, step));

  EXPECT_LT(registry_->num_events_received(), num_events);
}

TEST_P(PointerInjectorDelegateTest, ViewsGetPointerEventsInFIFO) {
  const uint64_t view_id = 1, num_events = 150;

  fuchsia::ui::views::ViewRefControl view_ref_control;
  fuchsia::ui::views::ViewRef view_ref;
  auto status = zx::eventpair::create(
      /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
  ZX_ASSERT(status == ZX_OK);
  view_ref_control.reference.replace(
      ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
      &view_ref_control.reference);
  view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

  // Create the view.
  CreateView(view_id);

  // Inject |num_events| platform messages.
  for (size_t i = 0; i < num_events; i++) {
    auto response = FakePlatformMessageResponse::Create();

    EXPECT_TRUE(pointer_injector_delegate_->HandlePlatformMessage(
        PlatformMessageBuilder()
            .SetViewId(view_id)
            .SetPointerId(static_cast<uint32_t>(i))
            .Build(),
        response));

    response->ExpectCompleted("[0]");
  }

  // The mock Pointerinjector registry server receives |num_events| pointer
  // events from |f.u.p.Device.Inject| call for the view.
  RunLoopUntil(
      [this] { return registry_->num_events_received() == num_events; });

  // The mock Pointerinjector registry server receives a
  // |f.u.p.Registry.Register| call for the view.
  ASSERT_TRUE(registry_->num_register_calls() == 1);

  auto& events = registry_->events();

  // The view should receive the pointer events in a FIFO order. As we injected
  // platform messages with an increasing |pointer_id|, the received pointer
  // events should also have the |pointer_id| in an increasing order.
  for (size_t i = 0; i < events.size() - 1; i++) {
    ASSERT_TRUE(events[i].has_data());
    ASSERT_TRUE(events[i + 1].has_data());
    ASSERT_TRUE(events[i].data().is_pointer_sample());
    ASSERT_TRUE(events[i + 1].data().is_pointer_sample());

    const auto& pointer_sample_1 = events[i].data().pointer_sample();
    const auto& pointer_sample_2 = events[i + 1].data().pointer_sample();

    ASSERT_TRUE(pointer_sample_1.has_pointer_id());
    ASSERT_TRUE(pointer_sample_2.has_pointer_id());

    EXPECT_TRUE(pointer_sample_1.pointer_id() < pointer_sample_2.pointer_id());
  }
}

TEST_P(PointerInjectorDelegateTest, DeviceRetriesRegisterWhenClosed) {
  const uint64_t view_id = 1;
  const int pointer_id = 1;
  fuchsia::ui::views::ViewRefControl view_ref_control;
  fuchsia::ui::views::ViewRef view_ref;
  auto status = zx::eventpair::create(
      /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
  ZX_ASSERT(status == ZX_OK);
  view_ref_control.reference.replace(
      ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
      &view_ref_control.reference);
  view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

  auto response = FakePlatformMessageResponse::Create();
  auto response_2 = FakePlatformMessageResponse::Create();

  // Create the view.
  fuv_ViewRef view_ref_clone;
  fidl::Clone(view_ref, &view_ref_clone);
  CreateView(view_id, std::move(view_ref_clone));

  EXPECT_TRUE(pointer_injector_delegate_->HandlePlatformMessage(
      PlatformMessageBuilder()
          .SetViewId(view_id)
          .SetPointerId(pointer_id)
          .Build(),
      response));

  response->ExpectCompleted("[0]");

  // The mock Pointerinjector registry server receives a pointer event from
  // |f.u.p.Device.Inject| call for the view.
  RunLoopUntil([this] { return registry_->num_events_received() == 1; });

  // The mock Pointerinjector registry server receives a
  // |f.u.p.Registry.Register| call for the view.
  ASSERT_TRUE(registry_->num_register_calls() == 1);

  // Close the device channel.
  registry_->ClearBindings();
  RunLoopUntilIdle();

  EXPECT_TRUE(pointer_injector_delegate_->HandlePlatformMessage(
      PlatformMessageBuilder()
          .SetViewId(view_id)
          .SetPointerId(pointer_id)
          .Build(),
      response_2));

  response_2->ExpectCompleted("[0]");

  // The mock Pointerinjector registry server receives a pointer event from
  // |f.u.p.Device.Inject| call for the view.
  RunLoopUntil([this] { return registry_->num_events_received() == 2; });

  // The device tries to register again as the channel got closed.
  ASSERT_TRUE(registry_->num_register_calls() == 2);
}

INSTANTIATE_TEST_SUITE_P(PointerInjectorDelegateParameterizedTest,
                         PointerInjectorDelegateTest,
                         ::testing::Bool());

}  // namespace flutter_runner::testing
