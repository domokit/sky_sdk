// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "accessibility_bridge.h"

#include <fuchsia/ui/views/cpp/fidl.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/async-loop/default.h>
#include <lib/async/cpp/executor.h>
#include <lib/fidl/cpp/binding_set.h>
#include <lib/fidl/cpp/interface_request.h>
#include <lib/inspect/cpp/hierarchy.h>
#include <lib/inspect/cpp/inspector.h>
#include <lib/inspect/cpp/reader.h>
#include <lib/sys/cpp/testing/service_directory_provider.h>
#include <lib/zx/eventpair.h>
#include <zircon/status.h>
#include <zircon/types.h>

#include <memory>

#include "flutter/lib/ui/semantics/semantics_node.h"
#include "gtest/gtest.h"

#include "flutter_runner_fakes.h"

namespace flutter_runner_test {

namespace {

void ExpectNodeHasRole(
    const fuchsia::accessibility::semantics::Node& node,
    const std::unordered_map<uint32_t, fuchsia::accessibility::semantics::Role>
        roles_by_node_id) {
  ASSERT_TRUE(node.has_node_id());
  ASSERT_NE(roles_by_node_id.find(node.node_id()), roles_by_node_id.end());
  EXPECT_TRUE(node.has_role());
  EXPECT_EQ(node.role(), roles_by_node_id.at(node.node_id()));
}

}  // namespace

class AccessibilityBridgeTestDelegate {
 public:
  void SetSemanticsEnabled(bool enabled) { enabled_ = enabled; }
  void DispatchSemanticsAction(int32_t node_id,
                               flutter::SemanticsAction action) {
    actions.push_back(std::make_pair(node_id, action));
  }

  bool enabled() { return enabled_; }
  std::vector<std::pair<int32_t, flutter::SemanticsAction>> actions;

 private:
  bool enabled_;
};

class AccessibilityBridgeTest : public testing::Test {
 public:
  AccessibilityBridgeTest()
      : loop_(&kAsyncLoopConfigAttachToCurrentThread),
        services_provider_(loop_.dispatcher()),
        executor_(loop_.dispatcher()) {
    services_provider_.AddService(
        semantics_manager_.GetHandler(loop_.dispatcher()),
        SemanticsManager::Name_);
  }

  void RunLoopUntilIdle() {
    loop_.RunUntilIdle();
    loop_.ResetQuit();
  }

  void RunPromiseToCompletion(fpromise::promise<> promise) {
    bool done = false;
    executor_.schedule_task(
        std::move(promise).and_then([&done]() { done = true; }));
    while (loop_.GetState() == ASYNC_LOOP_RUNNABLE) {
      if (done) {
        loop_.ResetQuit();
        return;
      }

      loop_.Run(zx::deadline_after(zx::duration::infinite()), true);
    }
    loop_.ResetQuit();
  }

 protected:
  void SetUp() override {
    // Connect to SemanticsManager service.
    fuchsia::accessibility::semantics::SemanticsManagerHandle semantics_manager;
    zx_status_t semantics_status =
        services_provider_.service_directory()
            ->Connect<fuchsia::accessibility::semantics::SemanticsManager>(
                semantics_manager.NewRequest());
    if (semantics_status != ZX_OK) {
      FML_LOG(WARNING)
          << "fuchsia::accessibility::semantics::SemanticsManager connection "
             "failed: "
          << zx_status_get_string(semantics_status);
    }

    accessibility_delegate_.actions.clear();
    inspector_ = std::make_unique<inspect::Inspector>();
    flutter_runner::AccessibilityBridge::SetSemanticsEnabledCallback
        set_semantics_enabled_callback = [this](bool enabled) {
          accessibility_delegate_.SetSemanticsEnabled(enabled);
        };
    flutter_runner::AccessibilityBridge::DispatchSemanticsActionCallback
        dispatch_semantics_action_callback =
            [this](int32_t node_id, flutter::SemanticsAction action) {
              accessibility_delegate_.DispatchSemanticsAction(node_id, action);
            };

    fuchsia::ui::views::ViewRefControl view_ref_control;
    fuchsia::ui::views::ViewRef view_ref;
    auto status = zx::eventpair::create(
        /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
    ASSERT_EQ(status, ZX_OK);
    view_ref_control.reference.replace(
        ZX_DEFAULT_EVENTPAIR_RIGHTS & (~ZX_RIGHT_DUPLICATE),
        &view_ref_control.reference);
    view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);
    accessibility_bridge_ =
        std::make_unique<flutter_runner::AccessibilityBridge>(
            std::move(set_semantics_enabled_callback),
            std::move(dispatch_semantics_action_callback),
            std::move(semantics_manager), std::move(view_ref),
            inspector_->GetRoot().CreateChild("test_node"));

    RunLoopUntilIdle();
  }

  void TearDown() override { semantics_manager_.ResetTree(); }

  MockSemanticsManager semantics_manager_;
  AccessibilityBridgeTestDelegate accessibility_delegate_;
  std::unique_ptr<flutter_runner::AccessibilityBridge> accessibility_bridge_;
  // Required to verify inspect metrics.
  std::unique_ptr<inspect::Inspector> inspector_;

 private:
  async::Loop loop_;
  sys::testing::ServiceDirectoryProvider services_provider_;
  // Required to retrieve inspect metrics.
  async::Executor executor_;
};

TEST_F(AccessibilityBridgeTest, RegistersViewRef) {
  EXPECT_TRUE(semantics_manager_.RegisterViewCalled());
}

TEST_F(AccessibilityBridgeTest, EnableDisable) {
  EXPECT_FALSE(accessibility_delegate_.enabled());
  std::unique_ptr<fuchsia::accessibility::semantics::SemanticListener> listener(
      accessibility_bridge_.release());
  listener->OnSemanticsModeChanged(true, nullptr);
  EXPECT_TRUE(accessibility_delegate_.enabled());
}

TEST_F(AccessibilityBridgeTest, RequestAnnounce) {
  accessibility_bridge_->RequestAnnounce("message");
  RunLoopUntilIdle();

  auto& last_events = semantics_manager_.GetLastEvents();
  ASSERT_EQ(last_events.size(), 1u);
  ASSERT_TRUE(last_events[0].is_announce());
  EXPECT_EQ(last_events[0].announce().message(), "message");
}

TEST_F(AccessibilityBridgeTest, PopulatesIsKeyboardKeyAttribute) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.flags = static_cast<int>(flutter::SemanticsFlags::kIsKeyboardKey);

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(1U, semantics_manager_.LastUpdatedNodes().size());
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.node_id(), static_cast<unsigned int>(node0.id));
  EXPECT_TRUE(fuchsia_node.has_attributes());
  EXPECT_TRUE(fuchsia_node.attributes().is_keyboard_key());
}

TEST_F(AccessibilityBridgeTest, UpdatesNodeRoles) {
  flutter::SemanticsNodeUpdates updates;

  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.flags |= static_cast<int>(flutter::SemanticsFlags::kIsButton);
  node0.childrenInTraversalOrder = {1, 2, 3, 4, 5, 6, 7, 8};
  node0.childrenInHitTestOrder = {1, 2, 3, 4, 5, 6, 7, 8};
  updates.emplace(0, node0);

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.flags |= static_cast<int>(flutter::SemanticsFlags::kIsHeader);
  node1.childrenInTraversalOrder = {};
  node1.childrenInHitTestOrder = {};
  updates.emplace(1, node1);

  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.flags |= static_cast<int>(flutter::SemanticsFlags::kIsImage);
  node2.childrenInTraversalOrder = {};
  node2.childrenInHitTestOrder = {};
  updates.emplace(2, node2);

  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.flags |= static_cast<int>(flutter::SemanticsFlags::kIsTextField);
  node3.childrenInTraversalOrder = {};
  node3.childrenInHitTestOrder = {};
  updates.emplace(3, node3);

  flutter::SemanticsNode node4;
  node4.childrenInTraversalOrder = {};
  node4.childrenInHitTestOrder = {};
  node4.id = 4;
  node4.flags |= static_cast<int>(flutter::SemanticsFlags::kIsSlider);
  updates.emplace(4, node4);

  flutter::SemanticsNode node5;
  node5.childrenInTraversalOrder = {};
  node5.childrenInHitTestOrder = {};
  node5.id = 5;
  node5.flags |= static_cast<int>(flutter::SemanticsFlags::kIsLink);
  updates.emplace(5, node5);

  flutter::SemanticsNode node6;
  node6.childrenInTraversalOrder = {};
  node6.childrenInHitTestOrder = {};
  node6.id = 6;
  node6.flags |= static_cast<int>(flutter::SemanticsFlags::kHasCheckedState);
  node6.flags |=
      static_cast<int>(flutter::SemanticsFlags::kIsInMutuallyExclusiveGroup);
  updates.emplace(6, node6);

  flutter::SemanticsNode node7;
  node7.childrenInTraversalOrder = {};
  node7.childrenInHitTestOrder = {};
  node7.id = 7;
  node7.flags |= static_cast<int>(flutter::SemanticsFlags::kHasCheckedState);
  updates.emplace(7, node7);

  flutter::SemanticsNode node8;
  node8.childrenInTraversalOrder = {};
  node8.childrenInHitTestOrder = {};
  node8.id = 8;
  node8.flags |= static_cast<int>(flutter::SemanticsFlags::kHasToggledState);
  updates.emplace(7, node8);

  accessibility_bridge_->AddSemanticsNodeUpdate(std::move(updates), 1.f);
  RunLoopUntilIdle();

  std::unordered_map<uint32_t, fuchsia::accessibility::semantics::Role>
      roles_by_node_id = {
          {0u, fuchsia::accessibility::semantics::Role::BUTTON},
          {1u, fuchsia::accessibility::semantics::Role::HEADER},
          {2u, fuchsia::accessibility::semantics::Role::IMAGE},
          {3u, fuchsia::accessibility::semantics::Role::TEXT_FIELD},
          {4u, fuchsia::accessibility::semantics::Role::SLIDER},
          {5u, fuchsia::accessibility::semantics::Role::LINK},
          {6u, fuchsia::accessibility::semantics::Role::RADIO_BUTTON},
          {7u, fuchsia::accessibility::semantics::Role::CHECK_BOX},
          {8u, fuchsia::accessibility::semantics::Role::TOGGLE_SWITCH}};

  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(8u, semantics_manager_.LastUpdatedNodes().size());
  for (const auto& node : semantics_manager_.LastUpdatedNodes()) {
    ExpectNodeHasRole(node, roles_by_node_id);
  }

  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, DeletesChildrenTransitively) {
  // Test that when a node is deleted, so are its transitive children.
  flutter::SemanticsNode node2;
  node2.id = 2;

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.childrenInTraversalOrder = {2};
  node1.childrenInHitTestOrder = {2};

  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.childrenInTraversalOrder = {1};
  node0.childrenInHitTestOrder = {1};

  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
          {1, node1},
          {2, node2},
      },
      1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(3U, semantics_manager_.LastUpdatedNodes().size());
  EXPECT_EQ(0U, semantics_manager_.LastDeletedNodeIds().size());
  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());

  // Remove the children
  node0.childrenInTraversalOrder.clear();
  node0.childrenInHitTestOrder.clear();
  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
      },
      1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(1, semantics_manager_.DeleteCount());
  EXPECT_EQ(2, semantics_manager_.UpdateCount());
  EXPECT_EQ(2, semantics_manager_.CommitCount());
  EXPECT_EQ(1U, semantics_manager_.LastUpdatedNodes().size());
  ASSERT_EQ(std::vector<uint32_t>({1, 2}),
            semantics_manager_.LastDeletedNodeIds());
  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, PopulatesRoleButton) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.flags = static_cast<int>(flutter::SemanticsFlags::kIsButton);

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(1U, semantics_manager_.LastUpdatedNodes().size());
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.node_id(), static_cast<unsigned int>(node0.id));
  EXPECT_TRUE(fuchsia_node.has_role());
  EXPECT_EQ(fuchsia_node.role(),
            fuchsia::accessibility::semantics::Role::BUTTON);
}

TEST_F(AccessibilityBridgeTest, PopulatesRoleImage) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.flags = static_cast<int>(flutter::SemanticsFlags::kIsImage);

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(1U, semantics_manager_.LastUpdatedNodes().size());
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.node_id(), static_cast<unsigned int>(node0.id));
  EXPECT_TRUE(fuchsia_node.has_role());
  EXPECT_EQ(fuchsia_node.role(),
            fuchsia::accessibility::semantics::Role::IMAGE);
}

TEST_F(AccessibilityBridgeTest, PopulatesRoleSlider) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.actions |= static_cast<int>(flutter::SemanticsAction::kIncrease);

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(1U, semantics_manager_.LastUpdatedNodes().size());
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.node_id(), static_cast<unsigned int>(node0.id));
  EXPECT_TRUE(fuchsia_node.has_role());
  EXPECT_EQ(fuchsia_node.role(),
            fuchsia::accessibility::semantics::Role::SLIDER);
}

TEST_F(AccessibilityBridgeTest, PopulatesRoleHeader) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.flags = static_cast<int>(flutter::SemanticsFlags::kIsHeader);

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(1U, semantics_manager_.LastUpdatedNodes().size());
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.node_id(), static_cast<unsigned int>(node0.id));
  EXPECT_TRUE(fuchsia_node.has_role());
  EXPECT_EQ(fuchsia_node.role(),
            fuchsia::accessibility::semantics::Role::HEADER);
}

TEST_F(AccessibilityBridgeTest, PopulatesCheckedState) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  // HasCheckedState = true
  // IsChecked = true
  // IsSelected = false
  node0.flags |= static_cast<int>(flutter::SemanticsFlags::kHasCheckedState);
  node0.flags |= static_cast<int>(flutter::SemanticsFlags::kIsChecked);
  node0.value = "value";

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(1U, semantics_manager_.LastUpdatedNodes().size());
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.node_id(), static_cast<unsigned int>(node0.id));
  EXPECT_TRUE(fuchsia_node.has_states());
  const auto& states = fuchsia_node.states();
  EXPECT_TRUE(states.has_checked_state());
  EXPECT_EQ(states.checked_state(),
            fuchsia::accessibility::semantics::CheckedState::CHECKED);
  EXPECT_TRUE(states.has_selected());
  EXPECT_FALSE(states.selected());
  EXPECT_TRUE(states.has_value());
  EXPECT_EQ(states.value(), node0.value);

  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, PopulatesSelectedState) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  // HasCheckedState = false
  // IsChecked = false
  // IsSelected = true
  node0.flags = static_cast<int>(flutter::SemanticsFlags::kIsSelected);

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(1U, semantics_manager_.LastUpdatedNodes().size());
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.node_id(), static_cast<unsigned int>(node0.id));
  EXPECT_TRUE(fuchsia_node.has_states());
  const auto& states = fuchsia_node.states();
  EXPECT_TRUE(states.has_checked_state());
  EXPECT_EQ(states.checked_state(),
            fuchsia::accessibility::semantics::CheckedState::NONE);
  EXPECT_TRUE(states.has_selected());
  EXPECT_TRUE(states.selected());

  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, PopulatesToggledState) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.flags |= static_cast<int>(flutter::SemanticsFlags::kHasToggledState);
  node0.flags |= static_cast<int>(flutter::SemanticsFlags::kIsToggled);

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(1U, semantics_manager_.LastUpdatedNodes().size());
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.node_id(), static_cast<unsigned int>(node0.id));
  EXPECT_TRUE(fuchsia_node.has_states());
  const auto& states = fuchsia_node.states();
  EXPECT_TRUE(states.has_toggled_state());
  EXPECT_EQ(states.toggled_state(),
            fuchsia::accessibility::semantics::ToggledState::ON);

  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, PopulatesEnabledState) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  // HasEnabledState = true
  // IsEnabled = true
  node0.flags |= static_cast<int>(flutter::SemanticsFlags::kHasEnabledState);
  node0.flags |= static_cast<int>(flutter::SemanticsFlags::kIsEnabled);
  node0.value = "value";

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(1U, semantics_manager_.LastUpdatedNodes().size());
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.node_id(), static_cast<unsigned int>(node0.id));
  EXPECT_TRUE(fuchsia_node.has_states());
  const auto& states = fuchsia_node.states();
  EXPECT_TRUE(states.has_enabled_state());
  EXPECT_EQ(states.enabled_state(),
            fuchsia::accessibility::semantics::EnabledState::ENABLED);
  EXPECT_TRUE(states.has_value());
  EXPECT_EQ(states.value(), node0.value);

  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, ApplyViewPixelRatioToRoot) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.flags = static_cast<int>(flutter::SemanticsFlags::kIsSelected);

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.25f);
  RunLoopUntilIdle();
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.node_id(), static_cast<unsigned int>(node0.id));
  EXPECT_EQ(fuchsia_node.transform().matrix[0], 0.8f);
  EXPECT_EQ(fuchsia_node.transform().matrix[5], 0.8f);
  EXPECT_EQ(fuchsia_node.transform().matrix[10], 1.f);
}

TEST_F(AccessibilityBridgeTest, DoesNotPopulatesHiddenState) {
  // Flutter's notion of a hidden node is different from Fuchsia's hidden node.
  // This test make sures that this state does not get sent.
  flutter::SemanticsNode node0;
  node0.id = 0;
  // HasCheckedState = false
  // IsChecked = false
  // IsSelected = false
  // IsHidden = true
  node0.flags = static_cast<int>(flutter::SemanticsFlags::kIsHidden);

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(1u, semantics_manager_.LastUpdatedNodes().size());
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.node_id(), static_cast<unsigned int>(node0.id));
  EXPECT_TRUE(fuchsia_node.has_states());
  const auto& states = fuchsia_node.states();
  EXPECT_TRUE(states.has_checked_state());
  EXPECT_EQ(states.checked_state(),
            fuchsia::accessibility::semantics::CheckedState::NONE);
  EXPECT_TRUE(states.has_selected());
  EXPECT_FALSE(states.selected());
  EXPECT_FALSE(states.has_hidden());

  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, PopulatesActions) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.actions |= static_cast<int>(flutter::SemanticsAction::kTap);
  node0.actions |= static_cast<int>(flutter::SemanticsAction::kLongPress);
  node0.actions |= static_cast<int>(flutter::SemanticsAction::kShowOnScreen);
  node0.actions |= static_cast<int>(flutter::SemanticsAction::kIncrease);
  node0.actions |= static_cast<int>(flutter::SemanticsAction::kDecrease);

  accessibility_bridge_->AddSemanticsNodeUpdate({{0, node0}}, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(1u, semantics_manager_.LastUpdatedNodes().size());
  const auto& fuchsia_node = semantics_manager_.LastUpdatedNodes().at(0u);
  EXPECT_EQ(fuchsia_node.actions().size(), 5u);
  EXPECT_EQ(fuchsia_node.actions().at(0u),
            fuchsia::accessibility::semantics::Action::DEFAULT);
  EXPECT_EQ(fuchsia_node.actions().at(1u),
            fuchsia::accessibility::semantics::Action::SECONDARY);
  EXPECT_EQ(fuchsia_node.actions().at(2u),
            fuchsia::accessibility::semantics::Action::SHOW_ON_SCREEN);
  EXPECT_EQ(fuchsia_node.actions().at(3u),
            fuchsia::accessibility::semantics::Action::INCREMENT);
  EXPECT_EQ(fuchsia_node.actions().at(4u),
            fuchsia::accessibility::semantics::Action::DECREMENT);
}

TEST_F(AccessibilityBridgeTest, TruncatesLargeLabel) {
  // Test that labels which are too long are truncated.
  flutter::SemanticsNode node0;
  node0.id = 0;

  flutter::SemanticsNode node1;
  node1.id = 1;

  flutter::SemanticsNode bad_node;
  bad_node.id = 2;
  bad_node.label =
      std::string(fuchsia::accessibility::semantics::MAX_LABEL_SIZE + 1, '2');

  node0.childrenInTraversalOrder = {1, 2};
  node0.childrenInHitTestOrder = {1, 2};

  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
          {1, node1},
          {2, bad_node},
      },
      1.f);
  RunLoopUntilIdle();

  // Nothing to delete, but we should have broken
  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(3U, semantics_manager_.LastUpdatedNodes().size());
  auto trimmed_node =
      std::find_if(semantics_manager_.LastUpdatedNodes().begin(),
                   semantics_manager_.LastUpdatedNodes().end(),
                   [id = static_cast<uint32_t>(bad_node.id)](
                       fuchsia::accessibility::semantics::Node const& node) {
                     return node.node_id() == id;
                   });
  ASSERT_NE(trimmed_node, semantics_manager_.LastUpdatedNodes().end());
  ASSERT_TRUE(trimmed_node->has_attributes());
  EXPECT_EQ(
      trimmed_node->attributes().label(),
      std::string(fuchsia::accessibility::semantics::MAX_LABEL_SIZE, '2'));
  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, TruncatesLargeToolTip) {
  // Test that tooltips which are too long are truncated.
  flutter::SemanticsNode node0;
  node0.id = 0;

  flutter::SemanticsNode node1;
  node1.id = 1;

  flutter::SemanticsNode bad_node;
  bad_node.id = 2;
  bad_node.tooltip =
      std::string(fuchsia::accessibility::semantics::MAX_LABEL_SIZE + 1, '2');

  node0.childrenInTraversalOrder = {1, 2};
  node0.childrenInHitTestOrder = {1, 2};

  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
          {1, node1},
          {2, bad_node},
      },
      1.f);
  RunLoopUntilIdle();

  // Nothing to delete, but we should have broken
  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(3U, semantics_manager_.LastUpdatedNodes().size());
  auto trimmed_node =
      std::find_if(semantics_manager_.LastUpdatedNodes().begin(),
                   semantics_manager_.LastUpdatedNodes().end(),
                   [id = static_cast<uint32_t>(bad_node.id)](
                       fuchsia::accessibility::semantics::Node const& node) {
                     return node.node_id() == id;
                   });
  ASSERT_NE(trimmed_node, semantics_manager_.LastUpdatedNodes().end());
  ASSERT_TRUE(trimmed_node->has_attributes());
  EXPECT_EQ(
      trimmed_node->attributes().secondary_label(),
      std::string(fuchsia::accessibility::semantics::MAX_LABEL_SIZE, '2'));
  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, TruncatesLargeValue) {
  // Test that values which are too long are truncated.
  flutter::SemanticsNode node0;
  node0.id = 0;

  flutter::SemanticsNode node1;
  node1.id = 1;

  flutter::SemanticsNode bad_node;
  bad_node.id = 2;
  bad_node.value =
      std::string(fuchsia::accessibility::semantics::MAX_VALUE_SIZE + 1, '2');

  node0.childrenInTraversalOrder = {1, 2};
  node0.childrenInHitTestOrder = {1, 2};

  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
          {1, node1},
          {2, bad_node},
      },
      1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(3U, semantics_manager_.LastUpdatedNodes().size());
  auto trimmed_node =
      std::find_if(semantics_manager_.LastUpdatedNodes().begin(),
                   semantics_manager_.LastUpdatedNodes().end(),
                   [id = static_cast<uint32_t>(bad_node.id)](
                       fuchsia::accessibility::semantics::Node const& node) {
                     return node.node_id() == id;
                   });
  ASSERT_NE(trimmed_node, semantics_manager_.LastUpdatedNodes().end());
  ASSERT_TRUE(trimmed_node->has_states());
  EXPECT_EQ(
      trimmed_node->states().value(),
      std::string(fuchsia::accessibility::semantics::MAX_VALUE_SIZE, '2'));
  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, SplitsLargeUpdates) {
  // Test that labels which are too long are truncated.
  flutter::SemanticsNode node0;
  node0.id = 0;

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.label =
      std::string(fuchsia::accessibility::semantics::MAX_LABEL_SIZE, '1');

  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.label = "2";

  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.label = "3";

  flutter::SemanticsNode node4;
  node4.id = 4;
  node4.value =
      std::string(fuchsia::accessibility::semantics::MAX_VALUE_SIZE, '4');

  node0.childrenInTraversalOrder = {1, 2};
  node0.childrenInHitTestOrder = {1, 2};
  node1.childrenInTraversalOrder = {3, 4};
  node1.childrenInHitTestOrder = {3, 4};

  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
          {1, node1},
          {2, node2},
          {3, node3},
          {4, node4},
      },
      1.f);
  RunLoopUntilIdle();

  // Nothing to delete, but we should have broken into groups (4, 3, 2), (1, 0)
  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(2, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_EQ(2U, semantics_manager_.LastUpdatedNodes().size());
  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, HandlesCycles) {
  // Test that cycles don't cause fatal error.
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.childrenInTraversalOrder.push_back(0);
  node0.childrenInHitTestOrder.push_back(0);
  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
      },
      1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(1, semantics_manager_.UpdateCount());
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());

  node0.childrenInTraversalOrder = {0, 1};
  node0.childrenInHitTestOrder = {0, 1};
  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.childrenInTraversalOrder = {0};
  node1.childrenInHitTestOrder = {0};
  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
          {1, node1},
      },
      1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(0, semantics_manager_.DeleteCount());
  EXPECT_EQ(2, semantics_manager_.UpdateCount());
  EXPECT_EQ(2, semantics_manager_.CommitCount());
  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, BatchesLargeMessages) {
  // Tests that messages get batched appropriately.
  flutter::SemanticsNode node0;
  node0.id = 0;

  flutter::SemanticsNodeUpdates update;

  const int32_t child_nodes = 100;
  const int32_t leaf_nodes = 100;
  for (int32_t i = 1; i < child_nodes + 1; i++) {
    flutter::SemanticsNode node;
    node.id = i;
    node0.childrenInTraversalOrder.push_back(i);
    node0.childrenInHitTestOrder.push_back(i);
    for (int32_t j = 0; j < leaf_nodes; j++) {
      flutter::SemanticsNode leaf_node;
      int id = (i * child_nodes) + ((j + 1) * leaf_nodes);
      leaf_node.id = id;
      leaf_node.label = "A relatively simple label";
      node.childrenInTraversalOrder.push_back(id);
      node.childrenInHitTestOrder.push_back(id);
      update.insert(std::make_pair(id, std::move(leaf_node)));
    }
    update.insert(std::make_pair(i, std::move(node)));
  }

  update.insert(std::make_pair(0, std::move(node0)));

  // Make the semantics manager hold answering to this commit to test the flow
  // control. This means the second update will not be pushed until the first
  // one has processed.
  semantics_manager_.SetShouldHoldCommitResponse(true);
  accessibility_bridge_->AddSemanticsNodeUpdate(update, 1.f);
  RunLoopUntilIdle();

  EXPECT_EQ(0, semantics_manager_.DeleteCount());

  EXPECT_TRUE(6 <= semantics_manager_.UpdateCount() &&
              semantics_manager_.UpdateCount() <= 12);
  EXPECT_EQ(1, semantics_manager_.CommitCount());
  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());

  int next_update_count = semantics_manager_.UpdateCount() + 1;
  // Remove the children
  node0.childrenInTraversalOrder.clear();
  node0.childrenInHitTestOrder.clear();
  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
      },
      1.f);
  RunLoopUntilIdle();

  // Should still be 0, because the commit was not answered yet.
  EXPECT_EQ(0, semantics_manager_.DeleteCount());

  semantics_manager_.InvokeCommitCallback();
  RunLoopUntilIdle();

  EXPECT_EQ(1, semantics_manager_.DeleteCount());
  EXPECT_EQ(next_update_count, semantics_manager_.UpdateCount());
  EXPECT_EQ(2, semantics_manager_.CommitCount());
  EXPECT_FALSE(semantics_manager_.DeleteOverflowed());
  EXPECT_FALSE(semantics_manager_.UpdateOverflowed());
}

TEST_F(AccessibilityBridgeTest, HitTest) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.rect.setLTRB(0, 0, 100, 100);
  node0.flags |= static_cast<int32_t>(flutter::SemanticsFlags::kIsFocusable);

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.rect.setLTRB(10, 10, 20, 20);
  // Setting platform view id ensures this node is considered focusable.
  node1.platformViewId = 1u;

  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.rect.setLTRB(25, 10, 45, 20);
  // Setting label ensures this node is considered focusable.
  node2.label = "label";

  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.rect.setLTRB(10, 25, 20, 45);
  // Setting actions to a nonzero value ensures this node is considered
  // focusable.
  node3.actions = 1u;

  flutter::SemanticsNode node4;
  node4.id = 4;
  node4.rect.setLTRB(10, 10, 20, 20);
  node4.transform.setTranslate(20, 20, 0);
  node4.flags |= static_cast<int32_t>(flutter::SemanticsFlags::kIsFocusable);

  node0.childrenInTraversalOrder = {1, 2, 3, 4};
  node0.childrenInHitTestOrder = {1, 2, 3, 4};

  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
          {1, node1},
          {2, node2},
          {3, node3},
          {4, node4},
      },
      1.f);
  RunLoopUntilIdle();

  uint32_t hit_node_id;
  auto callback = [&hit_node_id](fuchsia::accessibility::semantics::Hit hit) {
    EXPECT_TRUE(hit.has_node_id());
    hit_node_id = hit.node_id();
  };

  // Nodes are:
  // ----------
  // | 1   2  |
  // | 3   4  |
  // ----------

  accessibility_bridge_->HitTest({1, 1}, callback);
  EXPECT_EQ(hit_node_id, 0u);
  accessibility_bridge_->HitTest({15, 15}, callback);
  EXPECT_EQ(hit_node_id, 1u);
  accessibility_bridge_->HitTest({30, 15}, callback);
  EXPECT_EQ(hit_node_id, 2u);
  accessibility_bridge_->HitTest({15, 30}, callback);
  EXPECT_EQ(hit_node_id, 3u);
  accessibility_bridge_->HitTest({30, 30}, callback);
  EXPECT_EQ(hit_node_id, 4u);
}

TEST_F(AccessibilityBridgeTest, HitTestWithPixelRatio) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.rect.setLTRB(0, 0, 100, 100);
  node0.flags |= static_cast<int32_t>(flutter::SemanticsFlags::kIsFocusable);

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.rect.setLTRB(10, 10, 20, 20);
  // Setting platform view id ensures this node is considered focusable.
  node1.platformViewId = 1u;

  node0.childrenInTraversalOrder = {1};
  node0.childrenInHitTestOrder = {1};

  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
          {1, node1},
      },
      // Pick a very small pixel ratio so that a point within the bounds of
      // the node's root-space coordinates will be well outside the "screen"
      // bounds of the node.
      .1f);
  RunLoopUntilIdle();

  uint32_t hit_node_id;
  auto callback = [&hit_node_id](fuchsia::accessibility::semantics::Hit hit) {
    EXPECT_TRUE(hit.has_node_id());
    hit_node_id = hit.node_id();
  };
  accessibility_bridge_->HitTest({15, 15}, callback);
  EXPECT_EQ(hit_node_id, 0u);
}

TEST_F(AccessibilityBridgeTest, HitTestUnfocusableChild) {
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.rect.setLTRB(0, 0, 100, 100);

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.rect.setLTRB(10, 10, 60, 60);

  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.rect.setLTRB(50, 50, 100, 100);
  node2.flags |= static_cast<int32_t>(flutter::SemanticsFlags::kIsFocusable);

  node0.childrenInTraversalOrder = {1, 2};
  node0.childrenInHitTestOrder = {1, 2};

  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
          {1, node1},
          {2, node2},
      },
      1.f);
  RunLoopUntilIdle();

  uint32_t hit_node_id;
  auto callback = [&hit_node_id](fuchsia::accessibility::semantics::Hit hit) {
    EXPECT_TRUE(hit.has_node_id());
    hit_node_id = hit.node_id();
  };

  accessibility_bridge_->HitTest({55, 55}, callback);
  EXPECT_EQ(hit_node_id, 2u);
}

TEST_F(AccessibilityBridgeTest, HitTestOverlapping) {
  // Tests that the first node in hit test order wins, even if a later node
  // would be able to receive the hit.
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.rect.setLTRB(0, 0, 100, 100);
  node0.flags |= static_cast<int32_t>(flutter::SemanticsFlags::kIsFocusable);

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.rect.setLTRB(0, 0, 100, 100);
  node1.flags |= static_cast<int32_t>(flutter::SemanticsFlags::kIsFocusable);

  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.rect.setLTRB(25, 10, 45, 20);
  node2.flags |= static_cast<int32_t>(flutter::SemanticsFlags::kIsFocusable);

  node0.childrenInTraversalOrder = {1, 2};
  node0.childrenInHitTestOrder = {2, 1};

  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
          {1, node1},
          {2, node2},
      },
      1.f);
  RunLoopUntilIdle();

  uint32_t hit_node_id;
  auto callback = [&hit_node_id](fuchsia::accessibility::semantics::Hit hit) {
    EXPECT_TRUE(hit.has_node_id());
    hit_node_id = hit.node_id();
  };

  accessibility_bridge_->HitTest({30, 15}, callback);
  EXPECT_EQ(hit_node_id, 2u);
}

TEST_F(AccessibilityBridgeTest, Actions) {
  flutter::SemanticsNode node0;
  node0.id = 0;

  flutter::SemanticsNode node1;
  node1.id = 1;

  node0.childrenInTraversalOrder = {1};
  node0.childrenInHitTestOrder = {1};

  accessibility_bridge_->AddSemanticsNodeUpdate(
      {
          {0, node0},
          {1, node1},
      },
      1.f);
  RunLoopUntilIdle();

  auto handled_callback = [](bool handled) { EXPECT_TRUE(handled); };
  auto unhandled_callback = [](bool handled) { EXPECT_FALSE(handled); };

  accessibility_bridge_->OnAccessibilityActionRequested(
      0u, fuchsia::accessibility::semantics::Action::DEFAULT, handled_callback);
  EXPECT_EQ(accessibility_delegate_.actions.size(), 1u);
  EXPECT_EQ(accessibility_delegate_.actions.back(),
            std::make_pair(0, flutter::SemanticsAction::kTap));

  accessibility_bridge_->OnAccessibilityActionRequested(
      0u, fuchsia::accessibility::semantics::Action::SECONDARY,
      handled_callback);
  EXPECT_EQ(accessibility_delegate_.actions.size(), 2u);
  EXPECT_EQ(accessibility_delegate_.actions.back(),
            std::make_pair(0, flutter::SemanticsAction::kLongPress));

  accessibility_bridge_->OnAccessibilityActionRequested(
      0u, fuchsia::accessibility::semantics::Action::SET_FOCUS,
      unhandled_callback);
  EXPECT_EQ(accessibility_delegate_.actions.size(), 2u);

  accessibility_bridge_->OnAccessibilityActionRequested(
      0u, fuchsia::accessibility::semantics::Action::SET_VALUE,
      unhandled_callback);
  EXPECT_EQ(accessibility_delegate_.actions.size(), 2u);

  accessibility_bridge_->OnAccessibilityActionRequested(
      0u, fuchsia::accessibility::semantics::Action::SHOW_ON_SCREEN,
      handled_callback);
  EXPECT_EQ(accessibility_delegate_.actions.size(), 3u);
  EXPECT_EQ(accessibility_delegate_.actions.back(),
            std::make_pair(0, flutter::SemanticsAction::kShowOnScreen));

  accessibility_bridge_->OnAccessibilityActionRequested(
      2u, fuchsia::accessibility::semantics::Action::DEFAULT,
      unhandled_callback);
  EXPECT_EQ(accessibility_delegate_.actions.size(), 3u);

  accessibility_bridge_->OnAccessibilityActionRequested(
      0u, fuchsia::accessibility::semantics::Action::INCREMENT,
      handled_callback);
  EXPECT_EQ(accessibility_delegate_.actions.size(), 4u);
  EXPECT_EQ(accessibility_delegate_.actions.back(),
            std::make_pair(0, flutter::SemanticsAction::kIncrease));

  accessibility_bridge_->OnAccessibilityActionRequested(
      0u, fuchsia::accessibility::semantics::Action::DECREMENT,
      handled_callback);
  EXPECT_EQ(accessibility_delegate_.actions.size(), 5u);
  EXPECT_EQ(accessibility_delegate_.actions.back(),
            std::make_pair(0, flutter::SemanticsAction::kDecrease));
}

#if !FLUTTER_RELEASE
TEST_F(AccessibilityBridgeTest, InspectData) {
  flutter::SemanticsNodeUpdates updates;
  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.label = "node0";
  node0.hint = "node0_hint";
  node0.value = "value";
  node0.flags |= static_cast<int>(flutter::SemanticsFlags::kIsButton);
  node0.childrenInTraversalOrder = {1};
  node0.childrenInHitTestOrder = {1};
  node0.rect.setLTRB(0, 0, 100, 100);
  updates.emplace(0, node0);

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.flags |= static_cast<int>(flutter::SemanticsFlags::kIsHeader);
  node1.childrenInTraversalOrder = {};
  node1.childrenInHitTestOrder = {};
  updates.emplace(1, node1);

  accessibility_bridge_->AddSemanticsNodeUpdate(std::move(updates), 1.f);
  RunLoopUntilIdle();

  fpromise::result<inspect::Hierarchy> hierarchy;
  ASSERT_FALSE(hierarchy.is_ok());
  RunPromiseToCompletion(
      inspect::ReadFromInspector(*inspector_)
          .then([&hierarchy](fpromise::result<inspect::Hierarchy>& result) {
            hierarchy = std::move(result);
          }));
  ASSERT_TRUE(hierarchy.is_ok());

  auto tree_inspect_hierarchy = hierarchy.value().GetByPath({"test_node"});
  ASSERT_NE(tree_inspect_hierarchy, nullptr);
  // TODO(http://fxbug.dev/75841): Rewrite flutter engine accessibility bridge
  // tests using inspect matchers. The checks bellow verify that the tree was
  // built, and that it matches the format of the input tree. This will be
  // updated in the future when test matchers are available to verify individual
  // property values.
  const auto& root = tree_inspect_hierarchy->children();
  ASSERT_EQ(root.size(), 1u);
  EXPECT_EQ(root[0].name(), "semantic_tree_root");
  const auto& child = root[0].children();
  ASSERT_EQ(child.size(), 1u);
  EXPECT_EQ(child[0].name(), "node_1");
}
#endif  // !FLUTTER_RELEASE

}  // namespace flutter_runner_test
