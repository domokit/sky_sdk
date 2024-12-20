// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fuchsia/ui/pointer/cpp/fidl.h>
#include <gtest/gtest.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/async-loop/default.h>
#include <lib/fidl/cpp/binding_set.h>

#include <array>
#include <optional>
#include <vector>

#include "flutter/fml/logging.h"
#include "flutter/fml/macros.h"
#include "pointer_delegate.h"
#include "tests/fakes/mouse_source.h"
#include "tests/fakes/touch_source.h"
#include "tests/pointer_event_utility.h"

namespace flutter_runner::testing {

using fup_EventPhase = fuchsia::ui::pointer::EventPhase;
using fup_TouchEvent = fuchsia::ui::pointer::TouchEvent;
using fup_TouchIxnId = fuchsia::ui::pointer::TouchInteractionId;
using fup_TouchIxnResult = fuchsia::ui::pointer::TouchInteractionResult;
using fup_TouchIxnStatus = fuchsia::ui::pointer::TouchInteractionStatus;
using fup_TouchPointerSample = fuchsia::ui::pointer::TouchPointerSample;
using fup_TouchResponse = fuchsia::ui::pointer::TouchResponse;
using fup_TouchResponseType = fuchsia::ui::pointer::TouchResponseType;
using fup_ViewParameters = fuchsia::ui::pointer::ViewParameters;
using fup_MouseEvent = fuchsia::ui::pointer::MouseEvent;

constexpr std::array<std::array<float, 2>, 2> kRect = {{{0, 0}, {20, 20}}};
constexpr std::array<float, 9> kIdentity = {1, 0, 0, 0, 1, 0, 0, 0, 1};
constexpr fup_TouchIxnId kIxnOne = {.device_id = 1u,
                                    .pointer_id = 1u,
                                    .interaction_id = 2u};

constexpr uint32_t kMouseDeviceId = 123;
constexpr std::array<int64_t, 2> kNoScrollInPhysicalPixelDelta = {0, 0};
const bool kNotPrecisionScroll = false;
const bool kPrecisionScroll = true;

// Fixture to exercise Flutter runner's implementation for
// fuchsia.ui.pointer.TouchSource.
class PointerDelegateTest : public ::testing::Test {
 protected:
  PointerDelegateTest() : loop_(&kAsyncLoopConfigAttachToCurrentThread) {
    touch_source_ = std::make_unique<FakeTouchSource>();
    mouse_source_ = std::make_unique<FakeMouseSource>();
    pointer_delegate_ = std::make_unique<flutter_runner::PointerDelegate>(
        touch_source_bindings_.AddBinding(touch_source_.get()),
        mouse_source_bindings_.AddBinding(mouse_source_.get()));
  }

  void RunLoopUntilIdle() { loop_.RunUntilIdle(); }

  std::unique_ptr<FakeTouchSource> touch_source_;
  std::unique_ptr<FakeMouseSource> mouse_source_;
  std::unique_ptr<flutter_runner::PointerDelegate> pointer_delegate_;

 private:
  async::Loop loop_;
  fidl::BindingSet<fuchsia::ui::pointer::TouchSource> touch_source_bindings_;
  fidl::BindingSet<fuchsia::ui::pointer::MouseSource> mouse_source_bindings_;

  FML_DISALLOW_COPY_AND_ASSIGN(PointerDelegateTest);
};

TEST_F(PointerDelegateTest, Data_FuchsiaTimeVersusFlutterTime) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  // Fuchsia ADD -> Flutter ADD+DOWN
  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(/* in nanoseconds */ 1111789u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kIxnOne, fup_EventPhase::ADD, {10.f, 10.f})
          .AddResult(
              {.interaction = kIxnOne, .status = fup_TouchIxnStatus::GRANTED})
          .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers->size(), 2u);
  EXPECT_EQ((*pointers)[0].time_stamp, /* in microseconds */ 1111u);
  EXPECT_EQ((*pointers)[1].time_stamp, /* in microseconds */ 1111u);
}

TEST_F(PointerDelegateTest, Phase_FlutterPhasesAreSynthesized) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  // Fuchsia ADD -> Flutter ADD+DOWN
  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kIxnOne, fup_EventPhase::ADD, {10.f, 10.f})
          .AddResult(
              {.interaction = kIxnOne, .status = fup_TouchIxnStatus::GRANTED})
          .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers->size(), 2u);
  EXPECT_EQ((*pointers)[0].change, flutter::PointerData::Change::kAdd);
  EXPECT_EQ((*pointers)[1].change, flutter::PointerData::Change::kDown);

  // Fuchsia CHANGE -> Flutter MOVE
  events = TouchEventBuilder::New()
               .AddTime(2222000u)
               .AddSample(kIxnOne, fup_EventPhase::CHANGE, {10.f, 10.f})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers->size(), 1u);
  EXPECT_EQ((*pointers)[0].change, flutter::PointerData::Change::kMove);

  // Fuchsia REMOVE -> Flutter UP+REMOVE
  events = TouchEventBuilder::New()
               .AddTime(3333000u)
               .AddSample(kIxnOne, fup_EventPhase::REMOVE, {10.f, 10.f})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers->size(), 2u);
  EXPECT_EQ((*pointers)[0].change, flutter::PointerData::Change::kUp);
  EXPECT_EQ((*pointers)[1].change, flutter::PointerData::Change::kRemove);
}

TEST_F(PointerDelegateTest, Phase_FuchsiaCancelBecomesFlutterCancel) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  // Fuchsia ADD -> Flutter ADD+DOWN
  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kIxnOne, fup_EventPhase::ADD, {10.f, 10.f})
          .AddResult(
              {.interaction = kIxnOne, .status = fup_TouchIxnStatus::GRANTED})
          .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers->size(), 2u);
  EXPECT_EQ((*pointers)[0].change, flutter::PointerData::Change::kAdd);
  EXPECT_EQ((*pointers)[1].change, flutter::PointerData::Change::kDown);

  // Fuchsia CANCEL -> Flutter CANCEL
  events = TouchEventBuilder::New()
               .AddTime(2222000u)
               .AddSample(kIxnOne, fup_EventPhase::CANCEL, {10.f, 10.f})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers->size(), 1u);
  EXPECT_EQ((*pointers)[0].change, flutter::PointerData::Change::kCancel);
}

TEST_F(PointerDelegateTest, Coordinates_CorrectMapping) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  // Fuchsia ADD event, with a view parameter that maps the viewport identically
  // to the view. Then the center point of the viewport should map to the center
  // of the view, (10.f, 10.f).
  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(2222000u)
          .AddViewParameters(/*view*/ {{{0, 0}, {20, 20}}},
                             /*viewport*/ {{{0, 0}, {20, 20}}},
                             /*matrix*/ {1, 0, 0, 0, 1, 0, 0, 0, 1})
          .AddSample(kIxnOne, fup_EventPhase::ADD, {10.f, 10.f})
          .AddResult(
              {.interaction = kIxnOne, .status = fup_TouchIxnStatus::GRANTED})
          .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers->size(), 2u);  // ADD - DOWN
  EXPECT_FLOAT_EQ((*pointers)[0].physical_x, 10.f);
  EXPECT_FLOAT_EQ((*pointers)[0].physical_y, 10.f);
  pointers = {};

  // Fuchsia CHANGE event, with a view parameter that translates the viewport by
  // (10, 10) within the view. Then the minimal point in the viewport (its
  // origin) should map to the center of the view, (10.f, 10.f).
  events = TouchEventBuilder::New()
               .AddTime(2222000u)
               .AddViewParameters(/*view*/ {{{0, 0}, {20, 20}}},
                                  /*viewport*/ {{{0, 0}, {20, 20}}},
                                  /*matrix*/ {1, 0, 0, 0, 1, 0, 10, 10, 1})
               .AddSample(kIxnOne, fup_EventPhase::CHANGE, {0.f, 0.f})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers->size(), 1u);  // MOVE
  EXPECT_FLOAT_EQ((*pointers)[0].physical_x, 10.f);
  EXPECT_FLOAT_EQ((*pointers)[0].physical_y, 10.f);

  // Fuchsia CHANGE event, with a view parameter that scales the viewport by
  // (0.5, 0.5) within the view. Then the maximal point in the viewport should
  // map to the center of the view, (10.f, 10.f).
  events = TouchEventBuilder::New()
               .AddTime(2222000u)
               .AddViewParameters(/*view*/ {{{0, 0}, {20, 20}}},
                                  /*viewport*/ {{{0, 0}, {20, 20}}},
                                  /*matrix*/ {0.5f, 0, 0, 0, 0.5f, 0, 0, 0, 1})
               .AddSample(kIxnOne, fup_EventPhase::CHANGE, {20.f, 20.f})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers->size(), 1u);  // MOVE
  EXPECT_FLOAT_EQ((*pointers)[0].physical_x, 10.f);
  EXPECT_FLOAT_EQ((*pointers)[0].physical_y, 10.f);
}

TEST_F(PointerDelegateTest, Coordinates_DownEventClampedToView) {
  const float kSmallDiscrepancy = -0.00003f;

  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kIxnOne, fup_EventPhase::ADD, {10.f, kSmallDiscrepancy})
          .AddResult(
              {.interaction = kIxnOne, .status = fup_TouchIxnStatus::GRANTED})
          .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers->size(), 2u);

  const auto& add_event = (*pointers)[0];
  EXPECT_FLOAT_EQ(add_event.physical_x, 10.f);
  EXPECT_FLOAT_EQ(add_event.physical_y, kSmallDiscrepancy);

  const auto& down_event = (*pointers)[1];
  EXPECT_FLOAT_EQ(down_event.physical_x, 10.f);
  EXPECT_EQ(down_event.physical_y, 0.f);
}

TEST_F(PointerDelegateTest, Protocol_FirstResponseIsEmpty) {
  bool called = false;
  pointer_delegate_->WatchLoop(
      [&called](std::vector<flutter::PointerData> events) { called = true; });
  RunLoopUntilIdle();  // Server gets Watch call.

  EXPECT_FALSE(called);  // No events yet received to forward to client.
  // Server sees an initial "response" from client, which is empty, by contract.
  const auto responses = touch_source_->UploadedResponses();
  ASSERT_TRUE(responses.has_value());
  ASSERT_EQ(responses->size(), 0u);
}

TEST_F(PointerDelegateTest, Protocol_ResponseMatchesEarlierEvents) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  // Fuchsia view parameter only. Empty response.
  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .BuildAsVector();

  // Fuchsia ptr 1 ADD sample. Yes response.
  fup_TouchEvent e1 =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample({.device_id = 0u, .pointer_id = 1u, .interaction_id = 3u},
                     fup_EventPhase::ADD, {10.f, 10.f})
          .Build();
  events.emplace_back(std::move(e1));

  // Fuchsia ptr 2 ADD sample. Yes response.
  fup_TouchEvent e2 =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddSample({.device_id = 0u, .pointer_id = 2u, .interaction_id = 3u},
                     fup_EventPhase::ADD, {5.f, 5.f})
          .Build();
  events.emplace_back(std::move(e2));

  // Fuchsia ptr 3 ADD sample. Yes response.
  fup_TouchEvent e3 =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddSample({.device_id = 0u, .pointer_id = 3u, .interaction_id = 3u},
                     fup_EventPhase::ADD, {1.f, 1.f})
          .Build();
  events.emplace_back(std::move(e3));
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  const auto responses = touch_source_->UploadedResponses();
  ASSERT_TRUE(responses.has_value());
  ASSERT_EQ(responses.value().size(), 4u);
  // Event 0 did not carry a sample, so no response.
  EXPECT_FALSE(responses.value()[0].has_response_type());
  // Events 1-3 had a sample, must have a response.
  EXPECT_TRUE(responses.value()[1].has_response_type());
  EXPECT_EQ(responses.value()[1].response_type(), fup_TouchResponseType::YES);
  EXPECT_TRUE(responses.value()[2].has_response_type());
  EXPECT_EQ(responses.value()[2].response_type(), fup_TouchResponseType::YES);
  EXPECT_TRUE(responses.value()[3].has_response_type());
  EXPECT_EQ(responses.value()[3].response_type(), fup_TouchResponseType::YES);
}

TEST_F(PointerDelegateTest, Protocol_LateGrant) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  // Fuchsia ADD, no grant result - buffer it.
  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kIxnOne, fup_EventPhase::ADD, {10.f, 10.f})
          .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  EXPECT_EQ(pointers.value().size(), 0u);
  pointers = {};

  // Fuchsia CHANGE, no grant result - buffer it.
  events = TouchEventBuilder::New()
               .AddTime(2222000u)
               .AddSample(kIxnOne, fup_EventPhase::CHANGE, {10.f, 10.f})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  EXPECT_EQ(pointers.value().size(), 0u);
  pointers = {};

  // Fuchsia result: ownership granted. Buffered pointers released.
  events = TouchEventBuilder::New()
               .AddTime(3333000u)
               .AddResult({kIxnOne, fup_TouchIxnStatus::GRANTED})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 3u);  // ADD - DOWN - MOVE
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kAdd);
  EXPECT_EQ(pointers.value()[1].change, flutter::PointerData::Change::kDown);
  EXPECT_EQ(pointers.value()[2].change, flutter::PointerData::Change::kMove);
  pointers = {};

  // Fuchsia CHANGE, grant result - release immediately.
  events = TouchEventBuilder::New()
               .AddTime(/* in nanoseconds */ 4444000u)
               .AddSample(kIxnOne, fup_EventPhase::CHANGE, {10.f, 10.f})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 1u);
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kMove);
  EXPECT_EQ(pointers.value()[0].time_stamp, /* in microseconds */ 4444u);
  pointers = {};
}

TEST_F(PointerDelegateTest, Protocol_LateGrantCombo) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  // Fuchsia ADD, no grant result - buffer it.
  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kIxnOne, fup_EventPhase::ADD, {10.f, 10.f})
          .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  EXPECT_EQ(pointers.value().size(), 0u);
  pointers = {};

  // Fuchsia CHANGE, no grant result - buffer it.
  events = TouchEventBuilder::New()
               .AddTime(2222000u)
               .AddSample(kIxnOne, fup_EventPhase::CHANGE, {10.f, 10.f})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  EXPECT_EQ(pointers.value().size(), 0u);
  pointers = {};

  // Fuchsia CHANGE, with grant result - release buffered events.
  events = TouchEventBuilder::New()
               .AddTime(3333000u)
               .AddSample(kIxnOne, fup_EventPhase::CHANGE, {10.f, 10.f})
               .AddResult({kIxnOne, fup_TouchIxnStatus::GRANTED})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 4u);  // ADD - DOWN - MOVE - MOVE
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kAdd);
  EXPECT_EQ(pointers.value()[0].time_stamp, 1111u);
  EXPECT_EQ(pointers.value()[1].change, flutter::PointerData::Change::kDown);
  EXPECT_EQ(pointers.value()[1].time_stamp, 1111u);
  EXPECT_EQ(pointers.value()[2].change, flutter::PointerData::Change::kMove);
  EXPECT_EQ(pointers.value()[2].time_stamp, 2222u);
  EXPECT_EQ(pointers.value()[3].change, flutter::PointerData::Change::kMove);
  EXPECT_EQ(pointers.value()[3].time_stamp, 3333u);
  pointers = {};
}

TEST_F(PointerDelegateTest, Protocol_EarlyGrant) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  // Fuchsia ADD, with grant result - release immediately.
  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kIxnOne, fup_EventPhase::ADD, {10.f, 10.f})
          .AddResult({kIxnOne, fup_TouchIxnStatus::GRANTED})
          .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 2u);
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kAdd);
  EXPECT_EQ(pointers.value()[1].change, flutter::PointerData::Change::kDown);
  pointers = {};

  // Fuchsia CHANGE, after grant result - release immediately.
  events = TouchEventBuilder::New()
               .AddTime(2222000u)
               .AddSample(kIxnOne, fup_EventPhase::CHANGE, {10.f, 10.f})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 1u);
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kMove);
  pointers = {};
}

TEST_F(PointerDelegateTest, Protocol_LateDeny) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  // Fuchsia ADD, no grant result - buffer it.
  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kIxnOne, fup_EventPhase::ADD, {10.f, 10.f})
          .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  EXPECT_EQ(pointers.value().size(), 0u);
  pointers = {};

  // Fuchsia CHANGE, no grant result - buffer it.
  events = TouchEventBuilder::New()
               .AddTime(2222000u)
               .AddSample(kIxnOne, fup_EventPhase::CHANGE, {10.f, 10.f})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  EXPECT_EQ(pointers.value().size(), 0u);
  pointers = {};

  // Fuchsia result: ownership denied. Buffered pointers deleted.
  events = TouchEventBuilder::New()
               .AddTime(3333000u)
               .AddResult({kIxnOne, fup_TouchIxnStatus::DENIED})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 0u);  // Do not release to client!
  pointers = {};
}

TEST_F(PointerDelegateTest, Protocol_LateDenyCombo) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  // Fuchsia ADD, no grant result - buffer it.
  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kIxnOne, fup_EventPhase::ADD, {10.f, 10.f})
          .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  EXPECT_EQ(pointers.value().size(), 0u);
  pointers = {};

  // Fuchsia CHANGE, no grant result - buffer it.
  events = TouchEventBuilder::New()
               .AddTime(2222000u)
               .AddSample(kIxnOne, fup_EventPhase::CHANGE, {10.f, 10.f})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  EXPECT_EQ(pointers.value().size(), 0u);
  pointers = {};

  // Fuchsia result: ownership denied. Buffered pointers deleted.
  events = TouchEventBuilder::New()
               .AddTime(3333000u)
               .AddSample(kIxnOne, fup_EventPhase::CANCEL, {10.f, 10.f})
               .AddResult({kIxnOne, fup_TouchIxnStatus::DENIED})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 0u);  // Do not release to client!
  pointers = {};
}

TEST_F(PointerDelegateTest, Protocol_PointersAreIndependent) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  constexpr fup_TouchIxnId kIxnTwo = {
      .device_id = 1u, .pointer_id = 2u, .interaction_id = 2u};

  // Fuchsia ptr1 ADD and ptr2 ADD, no grant result for either - buffer them.
  std::vector<fup_TouchEvent> events =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kIxnOne, fup_EventPhase::ADD, {10.f, 10.f})
          .BuildAsVector();
  fup_TouchEvent ptr2 =
      TouchEventBuilder::New()
          .AddTime(1111000u)
          .AddSample(kIxnTwo, fup_EventPhase::ADD, {15.f, 15.f})
          .Build();
  events.emplace_back(std::move(ptr2));
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  EXPECT_EQ(pointers.value().size(), 0u);
  pointers = {};

  // Server grants win to pointer 2.
  events = TouchEventBuilder::New()
               .AddTime(2222000u)
               .AddResult({kIxnTwo, fup_TouchIxnStatus::GRANTED})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  // Note: Fuchsia's device and pointer IDs (both 32 bit) are coerced together
  // to fit in Flutter's 64-bit device ID. However, Flutter's pointer_identifier
  // is not set by platform runner code - PointerDataCaptureConverter (PDPC)
  // sets it.
  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 2u);
  EXPECT_EQ(pointers.value()[0].pointer_identifier, 0);  // reserved for PDPC
  EXPECT_EQ(pointers.value()[0].device, (int64_t)((1ul << 32) | 2u));
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kAdd);
  EXPECT_EQ(pointers.value()[1].pointer_identifier, 0);  // reserved for PDPC
  EXPECT_EQ(pointers.value()[1].device, (int64_t)((1ul << 32) | 2u));
  EXPECT_EQ(pointers.value()[1].change, flutter::PointerData::Change::kDown);
  pointers = {};

  // Server grants win to pointer 1.
  events = TouchEventBuilder::New()
               .AddTime(3333000u)
               .AddResult({kIxnOne, fup_TouchIxnStatus::GRANTED})
               .BuildAsVector();
  touch_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 2u);
  EXPECT_EQ(pointers.value()[0].pointer_identifier, 0);  // reserved for PDPC
  EXPECT_EQ(pointers.value()[0].device, (int64_t)((1ul << 32) | 1u));
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kAdd);
  EXPECT_EQ(pointers.value()[1].pointer_identifier, 0);  // reserved for PDPC
  EXPECT_EQ(pointers.value()[1].device, (int64_t)((1ul << 32) | 1u));
  EXPECT_EQ(pointers.value()[1].change, flutter::PointerData::Change::kDown);
  pointers = {};
}

TEST_F(PointerDelegateTest, MouseWheel_TickBased) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  std::vector<fup_MouseEvent> events =
      MouseEventBuilder()
          .AddTime(1111789u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kMouseDeviceId, {10.f, 10.f}, {}, {0, 1},
                     kNoScrollInPhysicalPixelDelta, kNotPrecisionScroll)
          .AddMouseDeviceInfo(kMouseDeviceId, {0, 1, 2})
          .BuildAsVector();
  mouse_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 1u);
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kHover);
  EXPECT_EQ(pointers.value()[0].signal_kind,
            flutter::PointerData::SignalKind::kScroll);
  EXPECT_EQ(pointers.value()[0].kind, flutter::PointerData::DeviceKind::kMouse);
  EXPECT_EQ(pointers.value()[0].buttons, 0);
  EXPECT_EQ(pointers.value()[0].scroll_delta_x, 0);
  EXPECT_EQ(pointers.value()[0].scroll_delta_y, -20);
  pointers = {};

  // receive a horizontal scroll
  events = MouseEventBuilder()
               .AddTime(1111789u)
               .AddViewParameters(kRect, kRect, kIdentity)
               .AddSample(kMouseDeviceId, {10.f, 10.f}, {}, {1, 0},
                          kNoScrollInPhysicalPixelDelta, kNotPrecisionScroll)
               .AddMouseDeviceInfo(kMouseDeviceId, {0, 1, 2})
               .BuildAsVector();
  mouse_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 1u);
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kHover);
  EXPECT_EQ(pointers.value()[0].signal_kind,
            flutter::PointerData::SignalKind::kScroll);
  EXPECT_EQ(pointers.value()[0].kind, flutter::PointerData::DeviceKind::kMouse);
  EXPECT_EQ(pointers.value()[0].buttons, 0);
  EXPECT_EQ(pointers.value()[0].scroll_delta_x, 20);
  EXPECT_EQ(pointers.value()[0].scroll_delta_y, 0);
  pointers = {};
}

TEST_F(PointerDelegateTest, MouseWheel_PixelBased) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  std::vector<fup_MouseEvent> events =
      MouseEventBuilder()
          .AddTime(1111789u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kMouseDeviceId, {10.f, 10.f}, {}, {0, 1}, {0, 120},
                     kNotPrecisionScroll)
          .AddMouseDeviceInfo(kMouseDeviceId, {0, 1, 2})
          .BuildAsVector();
  mouse_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 1u);
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kHover);
  EXPECT_EQ(pointers.value()[0].signal_kind,
            flutter::PointerData::SignalKind::kScroll);
  EXPECT_EQ(pointers.value()[0].kind, flutter::PointerData::DeviceKind::kMouse);
  EXPECT_EQ(pointers.value()[0].buttons, 0);
  EXPECT_EQ(pointers.value()[0].scroll_delta_x, 0);
  EXPECT_EQ(pointers.value()[0].scroll_delta_y, -120);
  pointers = {};

  // receive a horizontal scroll
  events = MouseEventBuilder()
               .AddTime(1111789u)
               .AddViewParameters(kRect, kRect, kIdentity)
               .AddSample(kMouseDeviceId, {10.f, 10.f}, {}, {1, 0}, {120, 0},
                          kNotPrecisionScroll)
               .AddMouseDeviceInfo(kMouseDeviceId, {0, 1, 2})
               .BuildAsVector();
  mouse_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 1u);
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kHover);
  EXPECT_EQ(pointers.value()[0].signal_kind,
            flutter::PointerData::SignalKind::kScroll);
  EXPECT_EQ(pointers.value()[0].kind, flutter::PointerData::DeviceKind::kMouse);
  EXPECT_EQ(pointers.value()[0].buttons, 0);
  EXPECT_EQ(pointers.value()[0].scroll_delta_x, 120);
  EXPECT_EQ(pointers.value()[0].scroll_delta_y, 0);
  pointers = {};
}

TEST_F(PointerDelegateTest, MouseWheel_TouchpadPixelBased) {
  std::optional<std::vector<flutter::PointerData>> pointers;
  pointer_delegate_->WatchLoop(
      [&pointers](std::vector<flutter::PointerData> events) {
        pointers = std::move(events);
      });
  RunLoopUntilIdle();  // Server gets watch call.

  std::vector<fup_MouseEvent> events =
      MouseEventBuilder()
          .AddTime(1111789u)
          .AddViewParameters(kRect, kRect, kIdentity)
          .AddSample(kMouseDeviceId, {10.f, 10.f}, {}, {0, 1}, {0, 120},
                     kPrecisionScroll)
          .AddMouseDeviceInfo(kMouseDeviceId, {0, 1, 2})
          .BuildAsVector();
  mouse_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 1u);
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kHover);
  EXPECT_EQ(pointers.value()[0].signal_kind,
            flutter::PointerData::SignalKind::kScroll);
  EXPECT_EQ(pointers.value()[0].kind, flutter::PointerData::DeviceKind::kMouse);
  EXPECT_EQ(pointers.value()[0].buttons, 0);
  EXPECT_EQ(pointers.value()[0].scroll_delta_x, 0);
  EXPECT_EQ(pointers.value()[0].scroll_delta_y, -120);
  pointers = {};

  // receive a horizontal scroll
  events = MouseEventBuilder()
               .AddTime(1111789u)
               .AddViewParameters(kRect, kRect, kIdentity)
               .AddSample(kMouseDeviceId, {10.f, 10.f}, {}, {1, 0}, {120, 0},
                          kPrecisionScroll)
               .AddMouseDeviceInfo(kMouseDeviceId, {0, 1, 2})
               .BuildAsVector();
  mouse_source_->ScheduleCallback(std::move(events));
  RunLoopUntilIdle();

  ASSERT_TRUE(pointers.has_value());
  ASSERT_EQ(pointers.value().size(), 1u);
  EXPECT_EQ(pointers.value()[0].change, flutter::PointerData::Change::kHover);
  EXPECT_EQ(pointers.value()[0].signal_kind,
            flutter::PointerData::SignalKind::kScroll);
  EXPECT_EQ(pointers.value()[0].kind, flutter::PointerData::DeviceKind::kMouse);
  EXPECT_EQ(pointers.value()[0].buttons, 0);
  EXPECT_EQ(pointers.value()[0].scroll_delta_x, 120);
  EXPECT_EQ(pointers.value()[0].scroll_delta_y, 0);
  pointers = {};
}

}  // namespace flutter_runner::testing
