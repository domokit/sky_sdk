// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop::testing {

IMPELLER_DEFINE_HANDLE(FlagHandle);

class FlagObject final
    : public Object<FlagObject, IMPELLER_INTERNAL_HANDLE_NAME(FlagHandle)> {
 public:
  explicit FlagObject(bool& destruction_flag)
      : destruction_flag_(destruction_flag) {
    FML_CHECK(!destruction_flag_) << "Destruction flag must be cleared.";
  }

  ~FlagObject() {
    FML_CHECK(!destruction_flag_) << "Already destructed.";
    destruction_flag_ = true;
  }

 private:
  bool& destruction_flag_;
};

IMPELLER_DEFINE_HANDLE(TestHandle);

class TestObject final
    : public Object<TestObject, IMPELLER_INTERNAL_HANDLE_NAME(TestHandle)> {
 public:
  TestObject(int arg1, double arg2, char arg3)
      : arg1_(arg1), arg2_(arg2), arg3_(arg3) {}

  ~TestObject() = default;

  auto GetArg1() const { return arg1_; }

  auto GetArg2() const { return arg2_; }

  auto GetArg3() const { return arg3_; }

 private:
  int arg1_ = {};
  double arg2_ = {};
  char arg3_ = {};
};

TEST(InteropObjectTest, CanCreateScoped) {
  bool destructed = false;
  {
    auto object = Adopt(new FlagObject(destructed));  //
  }
  ASSERT_TRUE(destructed);

  destructed = false;
  {
    auto object = Ref(new FlagObject(destructed));
    // New objects start with retain count of 1.
    object->Release();
  }
  ASSERT_TRUE(destructed);
}

TEST(InteropObjectTest, CanCreate) {
  auto object = Create<TestObject>(1, 1.3, 'c');
  ASSERT_EQ(object->GetArg1(), 1);
  ASSERT_EQ(object->GetArg2(), 1.3);
  ASSERT_EQ(object->GetArg3(), 'c');
}

TEST(InteropObjectTest, CanCopyAssignMove) {
  auto o = Create<TestObject>(1, 2.3, 'd');
  ASSERT_EQ(o->GetRefCountForTests(), 1u);
  {
    auto o1 = o;  // NOLINT(performance-unnecessary-copy-initialization)
    ASSERT_EQ(o->GetRefCountForTests(), 2u);
    auto o2 = o;  // NOLINT(performance-unnecessary-copy-initialization)
    ASSERT_EQ(o->GetRefCountForTests(), 3u);
    auto o3 = o1;  // NOLINT(performance-unnecessary-copy-initialization)
    ASSERT_EQ(o->GetRefCountForTests(), 4u);
  }
  ASSERT_EQ(o->GetRefCountForTests(), 1u);

  {
    auto o1(o);  // NOLINT(performance-unnecessary-copy-initialization)
    ASSERT_EQ(o->GetRefCountForTests(), 2u);
    ASSERT_EQ(o1->GetRefCountForTests(), 2u);
  }

  auto move_o = std::move(o);
  ASSERT_EQ(move_o->GetRefCountForTests(), 1u);
}

}  // namespace impeller::interop::testing
