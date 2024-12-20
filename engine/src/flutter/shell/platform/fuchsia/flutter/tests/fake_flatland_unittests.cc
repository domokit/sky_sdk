// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "fakes/scenic/fake_flatland.h"

#include <lib/async-testing/test_loop.h>
#include <lib/async/dispatcher.h>

#include <string>

#include "flutter/fml/logging.h"
#include "fuchsia/ui/composition/cpp/fidl.h"
#include "gtest/gtest.h"

namespace flutter_runner::testing {
namespace {

std::string GetCurrentTestName() {
  return ::testing::UnitTest::GetInstance()->current_test_info()->name();
}

}  // namespace

class FakeFlatlandTest : public ::testing::Test {
 protected:
  FakeFlatlandTest() : flatland_subloop_(loop_.StartNewLoop()) {}
  ~FakeFlatlandTest() override = default;

  async::TestLoop& loop() { return loop_; }

  FakeFlatland& fake_flatland() { return fake_flatland_; }

  fuchsia::ui::composition::FlatlandPtr ConnectFlatland() {
    FML_CHECK(!fake_flatland_.is_flatland_connected());

    auto flatland_handle =
        fake_flatland_.ConnectFlatland(flatland_subloop_->dispatcher());
    return flatland_handle.Bind();
  }

 private:
  // Primary loop and subloop for the FakeFlatland instance to process its
  // messages.  The subloop allocates it's own zx_port_t, allowing us to use a
  // separate port for each end of the message channel, rather than sharing a
  // single one.  Dual ports allow messages and responses to be intermingled,
  // which is how production code behaves; this improves test realism.
  async::TestLoop loop_;
  std::unique_ptr<async::LoopInterface> flatland_subloop_;

  FakeFlatland fake_flatland_;
};

TEST_F(FakeFlatlandTest, Initialization) {
  EXPECT_EQ(fake_flatland().debug_name(), "");

  // Pump the loop one time; the flatland should retain its initial state.
  loop().RunUntilIdle();
  EXPECT_EQ(fake_flatland().debug_name(), "");
}

TEST_F(FakeFlatlandTest, DebugLabel) {
  const std::string debug_label = GetCurrentTestName();
  fuchsia::ui::composition::FlatlandPtr flatland = ConnectFlatland();

  // Set the flatland's debug name.  The `SetDebugName` hasn't been processed
  // yet, so the flatland's view of the debug name is still empty.
  flatland->SetDebugName(debug_label);
  EXPECT_EQ(fake_flatland().debug_name(), "");

  // Pump the loop; the contents of the initial `SetDebugName` should be
  // processed.
  loop().RunUntilIdle();
  EXPECT_EQ(fake_flatland().debug_name(), debug_label);
}

TEST_F(FakeFlatlandTest, Present) {
  fuchsia::ui::composition::FlatlandPtr flatland = ConnectFlatland();

  // Fire an OnNextFrameBegin event and verify the test receives it.
  constexpr uint64_t kOnNextFrameAdditionalPresentCredits = 10u;
  uint64_t on_next_frame_additional_present_credits = 0u;
  fuchsia::ui::composition::OnNextFrameBeginValues on_next_frame_begin_values;
  on_next_frame_begin_values.set_additional_present_credits(
      kOnNextFrameAdditionalPresentCredits);
  flatland.events().OnNextFrameBegin =
      [&on_next_frame_additional_present_credits](
          auto on_next_frame_begin_values) {
        static bool called_once = false;
        EXPECT_FALSE(called_once);

        on_next_frame_additional_present_credits =
            on_next_frame_begin_values.additional_present_credits();
        called_once = true;
      };
  fake_flatland().FireOnNextFrameBeginEvent(
      std::move(on_next_frame_begin_values));
  EXPECT_EQ(on_next_frame_additional_present_credits, 0u);
  loop().RunUntilIdle();
  EXPECT_EQ(on_next_frame_additional_present_credits,
            kOnNextFrameAdditionalPresentCredits);

  // Fire an OnFramePresented event and verify the test receives it.
  constexpr uint64_t kOnFramePresentedNumPresentsAllowed = 20u;
  uint64_t frame_presented_num_presents_allowed = 0u;
  flatland.events().OnFramePresented =
      [&frame_presented_num_presents_allowed](auto frame_presented_info) {
        static bool called_once = false;
        EXPECT_FALSE(called_once);

        frame_presented_num_presents_allowed =
            frame_presented_info.num_presents_allowed;
        called_once = true;
      };
  fake_flatland().FireOnFramePresentedEvent(
      fuchsia::scenic::scheduling::FramePresentedInfo{
          .actual_presentation_time = 0,
          .num_presents_allowed = kOnFramePresentedNumPresentsAllowed,
      });
  EXPECT_EQ(frame_presented_num_presents_allowed, 0u);
  loop().RunUntilIdle();
  EXPECT_EQ(frame_presented_num_presents_allowed,
            kOnFramePresentedNumPresentsAllowed);

  // Call Present and verify the fake handled it.
  constexpr int64_t kPresentRequestedTime = 42;
  int64_t present_requested_time = 0;
  fuchsia::ui::composition::PresentArgs present_args;
  present_args.set_requested_presentation_time(kPresentRequestedTime);
  fake_flatland().SetPresentHandler(
      [&present_requested_time](auto present_args) {
        static bool called_once = false;
        EXPECT_FALSE(called_once);

        present_requested_time = present_args.requested_presentation_time();
        called_once = true;
      });
  flatland->Present(std::move(present_args));
  EXPECT_EQ(present_requested_time, 0);
  loop().RunUntilIdle();
  EXPECT_EQ(present_requested_time, kPresentRequestedTime);
}

}  // namespace flutter_runner::testing
