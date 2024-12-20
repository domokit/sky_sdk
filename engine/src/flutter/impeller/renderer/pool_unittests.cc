// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"

#include "impeller/renderer/pool.h"

namespace impeller {
namespace testing {

namespace {
class Foobar {
 public:
  static std::shared_ptr<Foobar> Create() { return std::make_shared<Foobar>(); }

  size_t GetSize() const { return size_; }

  void SetSize(size_t size) { size_ = size; }

  void Reset() { is_reset_ = true; }

  bool GetIsReset() const { return is_reset_; }

  void SetIsReset(bool is_reset) { is_reset_ = is_reset; }

 private:
  size_t size_;
  bool is_reset_ = false;
};
}  // namespace

TEST(PoolTest, Simple) {
  Pool<Foobar> pool(1'000);
  {
    auto grabbed = pool.Grab();
    grabbed->SetSize(123);
    pool.Recycle(grabbed);
    EXPECT_EQ(pool.GetSize(), 123u);
  }
  auto grabbed = pool.Grab();
  EXPECT_EQ(grabbed->GetSize(), 123u);
  EXPECT_TRUE(grabbed->GetIsReset());
  EXPECT_EQ(pool.GetSize(), 0u);
}

TEST(PoolTest, Overload) {
  Pool<Foobar> pool(1'000);
  {
    std::vector<std::shared_ptr<Foobar>> values;
    values.reserve(20);
    for (int i = 0; i < 20; i++) {
      values.push_back(pool.Grab());
    }
    for (const auto& value : values) {
      value->SetSize(100);
      pool.Recycle(value);
    }
  }
  EXPECT_EQ(pool.GetSize(), 1'000u);
}

}  // namespace testing
}  // namespace impeller
