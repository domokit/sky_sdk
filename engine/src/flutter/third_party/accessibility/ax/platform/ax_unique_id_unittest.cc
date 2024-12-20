// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_unique_id.h"

#include <memory>

#include "gtest/gtest.h"

namespace ui {

TEST(AXPlatformUniqueIdTest, IdsAreUnique) {
  AXUniqueId id1, id2;
  EXPECT_FALSE(id1 == id2);
  EXPECT_GT(id2.Get(), id1.Get());
}

static const int32_t kMaxId = 100;

class AXTestSmallBankUniqueId : public AXUniqueId {
 public:
  AXTestSmallBankUniqueId();
  ~AXTestSmallBankUniqueId() override;

 private:
  friend class AXUniqueId;
  BASE_DISALLOW_COPY_AND_ASSIGN(AXTestSmallBankUniqueId);
};

AXTestSmallBankUniqueId::AXTestSmallBankUniqueId() : AXUniqueId(kMaxId) {}
AXTestSmallBankUniqueId::~AXTestSmallBankUniqueId() = default;

TEST(AXPlatformUniqueIdTest, UnassignedIdsAreReused) {
  // Create a bank of ids that uses up all available ids.
  // Then remove an id and replace with a new one. Since it's the only
  // slot available, the id will end up having the same value, rather than
  // starting over at 1.
  std::unique_ptr<AXTestSmallBankUniqueId> ids[kMaxId];

  for (int i = 0; i < kMaxId; i++) {
    ids[i] = std::make_unique<AXTestSmallBankUniqueId>();
  }

  static int kIdToReplace = 10;
  int32_t expected_id = ids[kIdToReplace]->Get();

  // Delete one of the ids and replace with a new one.
  ids[kIdToReplace] = nullptr;
  ids[kIdToReplace] = std::make_unique<AXTestSmallBankUniqueId>();

  // Expect that the original Id gets reused.
  EXPECT_EQ(ids[kIdToReplace]->Get(), expected_id);
}

TEST(AXPlatformUniqueIdTest, DoesCreateCorrectId) {
  constexpr int kLargerThanMaxId = kMaxId * 2;
  std::unique_ptr<AXUniqueId> ids[kLargerThanMaxId];
  // Creates and releases to fill up the internal static counter.
  for (int i = 0; i < kLargerThanMaxId; i++) {
    ids[i] = std::make_unique<AXUniqueId>();
  }
  for (int i = 0; i < kLargerThanMaxId; i++) {
    ids[i].reset(nullptr);
  }
  // Creates an unique id whose max value is less than the internal
  // static counter.
  std::unique_ptr<AXTestSmallBankUniqueId> unique_id =
      std::make_unique<AXTestSmallBankUniqueId>();

  EXPECT_LE(unique_id->Get(), kMaxId);
}

}  // namespace ui
