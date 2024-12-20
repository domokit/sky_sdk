// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file tests both ref_counted.h and ref_ptr.h (which the former includes).
// TODO(vtl): Possibly we could separate these tests out better, since a lot of
// it is actually testing |RefPtr|.

#include "flutter/fml/memory/ref_counted.h"

#include "flutter/fml/macros.h"
#include "gtest/gtest.h"

#if defined(__clang__)
#define ALLOW_PESSIMIZING_MOVE(code_line)                                   \
  _Pragma("clang diagnostic push")                                          \
      _Pragma("clang diagnostic ignored \"-Wpessimizing-move\"") code_line; \
  _Pragma("clang diagnostic pop")
#else
#define ALLOW_PESSIMIZING_MOVE(code_line) code_line;
#endif

#if defined(__clang__)
#define ALLOW_SELF_MOVE(code_line)                                   \
  _Pragma("clang diagnostic push")                                   \
      _Pragma("clang diagnostic ignored \"-Wself-move\"") code_line; \
  _Pragma("clang diagnostic pop")
#else
#define ALLOW_SELF_MOVE(code_line) code_line;
#endif

#if defined(__clang__)
#define ALLOW_SELF_ASSIGN_OVERLOADED(code_line)                        \
  _Pragma("clang diagnostic push")                                     \
      _Pragma("clang diagnostic ignored \"-Wself-assign-overloaded\"") \
          code_line;                                                   \
  _Pragma("clang diagnostic pop")
#else
#define ALLOW_SELF_ASSIGN_OVERLOADED(code_line) code_line;
#endif

namespace fml {
namespace {

class MyClass : public RefCountedThreadSafe<MyClass> {
 protected:
  MyClass(MyClass** created, bool* was_destroyed)
      : was_destroyed_(was_destroyed) {
    if (created) {
      *created = this;
    }
  }
  virtual ~MyClass() {
    if (was_destroyed_) {
      *was_destroyed_ = true;
    }
  }

 private:
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(MyClass);
  FML_FRIEND_MAKE_REF_COUNTED(MyClass);

  bool* was_destroyed_;

  FML_DISALLOW_COPY_AND_ASSIGN(MyClass);
};

class MySubclass final : public MyClass {
 private:
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(MySubclass);
  FML_FRIEND_MAKE_REF_COUNTED(MySubclass);

  MySubclass(MySubclass** created, bool* was_destroyed)
      : MyClass(nullptr, was_destroyed) {
    if (created) {
      *created = this;
    }
  }
  ~MySubclass() override {}

  FML_DISALLOW_COPY_AND_ASSIGN(MySubclass);
};

TEST(RefCountedTest, Constructors) {
  bool was_destroyed;

  {
    // Default.
    RefPtr<MyClass> r;
    EXPECT_TRUE(r.get() == nullptr);
    EXPECT_FALSE(r);
  }

  {
    // Nullptr.
    RefPtr<MyClass> r(nullptr);
    EXPECT_TRUE(r.get() == nullptr);
    EXPECT_FALSE(r);
  }

  {
    MyClass* created = nullptr;
    was_destroyed = false;
    // Adopt, then RVO.
    RefPtr<MyClass> r(MakeRefCounted<MyClass>(&created, &was_destroyed));
    EXPECT_TRUE(created);
    EXPECT_EQ(created, r.get());
    EXPECT_TRUE(r);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);

  {
    MyClass* created = nullptr;
    was_destroyed = false;
    // Adopt, then move.
    ALLOW_PESSIMIZING_MOVE(RefPtr<MyClass> r(
        std::move(MakeRefCounted<MyClass>(&created, &was_destroyed))))
    EXPECT_TRUE(created);
    EXPECT_EQ(created, r.get());
    EXPECT_TRUE(r);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);

  {
    MyClass* created = nullptr;
    was_destroyed = false;
    RefPtr<MyClass> r1(MakeRefCounted<MyClass>(&created, &was_destroyed));
    // Copy.
    // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
    RefPtr<MyClass> r2(r1);
    EXPECT_TRUE(created);
    EXPECT_EQ(created, r1.get());
    EXPECT_EQ(created, r2.get());
    EXPECT_TRUE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);

  {
    MyClass* created = nullptr;
    was_destroyed = false;
    RefPtr<MyClass> r1(MakeRefCounted<MyClass>(&created, &was_destroyed));
    // From raw pointer.
    RefPtr<MyClass> r2(created);
    EXPECT_TRUE(created);
    EXPECT_EQ(created, r1.get());
    EXPECT_EQ(created, r2.get());
    EXPECT_TRUE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);

  {
    MySubclass* created = nullptr;
    was_destroyed = false;
    // Adopt, then "move".
    RefPtr<MyClass> r(MakeRefCounted<MySubclass>(&created, &was_destroyed));
    EXPECT_TRUE(created);
    EXPECT_EQ(static_cast<MyClass*>(created), r.get());
    EXPECT_TRUE(r);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);

  {
    MySubclass* created = nullptr;
    was_destroyed = false;
    // Adopt, then "move".
    ALLOW_PESSIMIZING_MOVE(RefPtr<MyClass> r(
        std::move(MakeRefCounted<MySubclass>(&created, &was_destroyed))))
    EXPECT_TRUE(created);
    EXPECT_EQ(static_cast<MyClass*>(created), r.get());
    EXPECT_TRUE(r);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);

  {
    MySubclass* created = nullptr;
    was_destroyed = false;
    RefPtr<MySubclass> r1(MakeRefCounted<MySubclass>(&created, &was_destroyed));
    // "Copy".
    RefPtr<MyClass> r2(r1);
    EXPECT_TRUE(created);
    EXPECT_EQ(static_cast<MyClass*>(created), r1.get());
    EXPECT_EQ(static_cast<MyClass*>(created), r2.get());
    EXPECT_TRUE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);

  {
    MySubclass* created = nullptr;
    was_destroyed = false;
    RefPtr<MySubclass> r1(MakeRefCounted<MySubclass>(&created, &was_destroyed));
    // From raw pointer.
    RefPtr<MyClass> r2(created);
    EXPECT_TRUE(created);
    EXPECT_EQ(static_cast<MyClass*>(created), r1.get());
    EXPECT_EQ(static_cast<MyClass*>(created), r2.get());
    EXPECT_TRUE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);
}

TEST(RefCountedTest, NullAssignmentToNull) {
  RefPtr<MyClass> r1;
  // No-op null assignment using |nullptr|.
  r1 = nullptr;
  EXPECT_TRUE(r1.get() == nullptr);
  EXPECT_FALSE(r1);

  RefPtr<MyClass> r2;
  // No-op null assignment using copy constructor.
  r1 = r2;
  EXPECT_TRUE(r1.get() == nullptr);
  EXPECT_TRUE(r2.get() == nullptr);
  EXPECT_FALSE(r1);
  EXPECT_FALSE(r2);

  // No-op null assignment using move constructor.
  r1 = std::move(r2);
  EXPECT_TRUE(r1.get() == nullptr);
  // The clang linter flags the method called on the moved-from reference, but
  // this is testing the move implementation, so it is marked NOLINT.
  EXPECT_TRUE(r2.get() == nullptr);  // NOLINT(clang-analyzer-cplusplus.Move,
                                     // bugprone-use-after-move)
  EXPECT_FALSE(r1);
  EXPECT_FALSE(r2);

  RefPtr<MySubclass> r3;
  // No-op null assignment using "copy" constructor.
  r1 = r3;
  EXPECT_TRUE(r1.get() == nullptr);
  EXPECT_TRUE(r3.get() == nullptr);
  EXPECT_FALSE(r1);
  EXPECT_FALSE(r3);

  // No-op null assignment using "move" constructor.
  r1 = std::move(r3);
  EXPECT_TRUE(r1.get() == nullptr);
  EXPECT_TRUE(r3.get() == nullptr);  // NOLINT(bugprone-use-after-move)
  EXPECT_FALSE(r1);
  EXPECT_FALSE(r3);
}

TEST(RefCountedTest, NonNullAssignmentToNull) {
  bool was_destroyed;

  {
    MyClass* created = nullptr;
    was_destroyed = false;
    RefPtr<MyClass> r1(MakeRefCounted<MyClass>(&created, &was_destroyed));
    RefPtr<MyClass> r2;
    // Copy assignment (to null ref pointer).
    r2 = r1;
    EXPECT_EQ(created, r1.get());
    EXPECT_EQ(created, r2.get());
    EXPECT_TRUE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);

  {
    MyClass* created = nullptr;
    was_destroyed = false;
    RefPtr<MyClass> r1(MakeRefCounted<MyClass>(&created, &was_destroyed));
    RefPtr<MyClass> r2;
    // Move assignment (to null ref pointer).
    r2 = std::move(r1);
    // The clang linter flags the method called on the moved-from reference, but
    // this is testing the move implementation, so it is marked NOLINT.
    EXPECT_TRUE(r1.get() == nullptr);  // NOLINT(clang-analyzer-cplusplus.Move,
                                       // bugprone-use-after-move)
    EXPECT_EQ(created, r2.get());
    EXPECT_FALSE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);

  {
    MySubclass* created = nullptr;
    was_destroyed = false;
    RefPtr<MySubclass> r1(MakeRefCounted<MySubclass>(&created, &was_destroyed));
    RefPtr<MyClass> r2;
    // "Copy" assignment (to null ref pointer).
    r2 = r1;
    EXPECT_EQ(created, r1.get());
    EXPECT_EQ(static_cast<MyClass*>(created), r2.get());
    EXPECT_TRUE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);

  {
    MySubclass* created = nullptr;
    was_destroyed = false;
    RefPtr<MySubclass> r1(MakeRefCounted<MySubclass>(&created, &was_destroyed));
    RefPtr<MyClass> r2;
    // "Move" assignment (to null ref pointer).
    r2 = std::move(r1);
    EXPECT_TRUE(r1.get() == nullptr);  // NOLINT(bugprone-use-after-move)
    EXPECT_EQ(static_cast<MyClass*>(created), r2.get());
    EXPECT_FALSE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);
}

TEST(RefCountedTest, NullAssignmentToNonNull) {
  bool was_destroyed = false;
  RefPtr<MyClass> r1(MakeRefCounted<MyClass>(nullptr, &was_destroyed));
  // Null assignment (to non-null ref pointer) using |nullptr|.
  r1 = nullptr;
  EXPECT_TRUE(r1.get() == nullptr);
  EXPECT_FALSE(r1);
  EXPECT_TRUE(was_destroyed);

  was_destroyed = false;
  r1 = MakeRefCounted<MyClass>(nullptr, &was_destroyed);
  RefPtr<MyClass> r2;
  // Null assignment (to non-null ref pointer) using copy constructor.
  r1 = r2;
  EXPECT_TRUE(r1.get() == nullptr);
  EXPECT_TRUE(r2.get() == nullptr);
  EXPECT_FALSE(r1);
  EXPECT_FALSE(r2);
  EXPECT_TRUE(was_destroyed);

  was_destroyed = false;
  r1 = MakeRefCounted<MyClass>(nullptr, &was_destroyed);
  // Null assignment using move constructor.
  r1 = std::move(r2);
  EXPECT_TRUE(r1.get() == nullptr);
  // The clang linter flags the method called on the moved-from reference, but
  // this is testing the move implementation, so it is marked NOLINT.
  EXPECT_TRUE(r2.get() == nullptr);  // NOLINT(clang-analyzer-cplusplus.Move,
                                     // bugprone-use-after-move)
  EXPECT_FALSE(r1);
  EXPECT_FALSE(r2);
  EXPECT_TRUE(was_destroyed);

  was_destroyed = false;
  r1 = MakeRefCounted<MyClass>(nullptr, &was_destroyed);
  RefPtr<MySubclass> r3;
  // Null assignment (to non-null ref pointer) using "copy" constructor.
  r1 = r3;
  EXPECT_TRUE(r1.get() == nullptr);
  EXPECT_TRUE(r3.get() == nullptr);
  EXPECT_FALSE(r1);
  EXPECT_FALSE(r3);
  EXPECT_TRUE(was_destroyed);

  was_destroyed = false;
  r1 = MakeRefCounted<MyClass>(nullptr, &was_destroyed);
  // Null assignment (to non-null ref pointer) using "move" constructor.
  r1 = std::move(r3);
  EXPECT_TRUE(r1.get() == nullptr);
  EXPECT_TRUE(r3.get() == nullptr);  // NOLINT(bugprone-use-after-move)
  EXPECT_FALSE(r1);
  EXPECT_FALSE(r3);
  EXPECT_TRUE(was_destroyed);
}

TEST(RefCountedTest, NonNullAssignmentToNonNull) {
  bool was_destroyed1;
  bool was_destroyed2;

  {
    was_destroyed1 = false;
    was_destroyed2 = false;
    RefPtr<MyClass> r1(MakeRefCounted<MyClass>(nullptr, &was_destroyed1));
    RefPtr<MyClass> r2(MakeRefCounted<MyClass>(nullptr, &was_destroyed2));
    // Copy assignment (to non-null ref pointer).
    r2 = r1;
    EXPECT_EQ(r1.get(), r2.get());
    EXPECT_TRUE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed1);
    EXPECT_TRUE(was_destroyed2);
  }
  EXPECT_TRUE(was_destroyed1);

  {
    was_destroyed1 = false;
    was_destroyed2 = false;
    RefPtr<MyClass> r1(MakeRefCounted<MyClass>(nullptr, &was_destroyed1));
    RefPtr<MyClass> r2(MakeRefCounted<MyClass>(nullptr, &was_destroyed2));
    // Move assignment (to non-null ref pointer).
    r2 = std::move(r1);
    // The clang linter flags the method called on the moved-from reference, but
    // this is testing the move implementation, so it is marked NOLINT.
    EXPECT_TRUE(r1.get() == nullptr);  // NOLINT(clang-analyzer-cplusplus.Move,
                                       // bugprone-use-after-move)
    EXPECT_FALSE(r2.get() == nullptr);
    EXPECT_FALSE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed1);
    EXPECT_TRUE(was_destroyed2);
  }
  EXPECT_TRUE(was_destroyed1);

  {
    was_destroyed1 = false;
    was_destroyed2 = false;
    RefPtr<MySubclass> r1(MakeRefCounted<MySubclass>(nullptr, &was_destroyed1));
    RefPtr<MyClass> r2(MakeRefCounted<MyClass>(nullptr, &was_destroyed2));
    // "Copy" assignment (to non-null ref pointer).
    r2 = r1;
    EXPECT_EQ(r1.get(), r2.get());
    EXPECT_TRUE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed1);
    EXPECT_TRUE(was_destroyed2);
  }
  EXPECT_TRUE(was_destroyed1);

  {
    was_destroyed1 = false;
    was_destroyed2 = false;
    RefPtr<MySubclass> r1(MakeRefCounted<MySubclass>(nullptr, &was_destroyed1));
    RefPtr<MyClass> r2(MakeRefCounted<MyClass>(nullptr, &was_destroyed2));
    // Move assignment (to non-null ref pointer).
    r2 = std::move(r1);
    EXPECT_TRUE(r1.get() == nullptr);  // NOLINT(bugprone-use-after-move)
    EXPECT_FALSE(r2.get() == nullptr);
    EXPECT_FALSE(r1);
    EXPECT_TRUE(r2);
    EXPECT_FALSE(was_destroyed1);
    EXPECT_TRUE(was_destroyed2);
  }
  EXPECT_TRUE(was_destroyed1);
}

TEST(RefCountedTest, SelfAssignment) {
  bool was_destroyed;

  {
    MyClass* created = nullptr;
    was_destroyed = false;
    // This line is marked NOLINT because the clang linter does not reason about
    // the value of the reference count. In particular, the self-assignment
    // below is handled in the copy constructor by a refcount increment then
    // decrement. The linter sees only that the decrement might destroy the
    // object.
    RefPtr<MyClass> r(MakeRefCounted<MyClass>(  // NOLINT
        &created, &was_destroyed));
    // Copy.
    ALLOW_SELF_ASSIGN_OVERLOADED(r = r);
    EXPECT_EQ(created, r.get());
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);

  {
    MyClass* created = nullptr;
    was_destroyed = false;
    RefPtr<MyClass> r(MakeRefCounted<MyClass>(&created, &was_destroyed));
    // Move.
    ALLOW_SELF_MOVE(r = std::move(r))
    EXPECT_EQ(created, r.get());
    EXPECT_FALSE(was_destroyed);
  }
  EXPECT_TRUE(was_destroyed);
}

TEST(RefCountedTest, Swap) {
  MyClass* created1 = nullptr;
  static bool was_destroyed1;
  was_destroyed1 = false;
  RefPtr<MyClass> r1(MakeRefCounted<MyClass>(&created1, &was_destroyed1));
  EXPECT_TRUE(created1);
  EXPECT_EQ(created1, r1.get());

  MyClass* created2 = nullptr;
  static bool was_destroyed2;
  was_destroyed2 = false;
  RefPtr<MyClass> r2(MakeRefCounted<MyClass>(&created2, &was_destroyed2));
  EXPECT_TRUE(created2);
  EXPECT_EQ(created2, r2.get());
  EXPECT_NE(created1, created2);

  r1.swap(r2);
  EXPECT_EQ(created2, r1.get());
  EXPECT_EQ(created1, r2.get());
}

TEST(RefCountedTest, GetAndDereferenceOperators) {
  // Note: We check here that .get(), operator*, and operator-> are const, but
  // return non-const pointers/refs.

  MyClass* created = nullptr;
  const RefPtr<MyClass> r(MakeRefCounted<MyClass>(&created, nullptr));
  MyClass* ptr = r.get();  // Assign to non-const pointer.
  EXPECT_EQ(created, ptr);
  ptr = r.operator->();  // Assign to non-const pointer.
  EXPECT_EQ(created, ptr);
  MyClass& ref = *r;  // "Assign" to non-const reference.
  EXPECT_EQ(created, &ref);
}

// You can manually call |AddRef()| and |Release()| if you want.
TEST(RefCountedTest, AddRefRelease) {
  MyClass* created = nullptr;
  bool was_destroyed = false;
  {
    RefPtr<MyClass> r(MakeRefCounted<MyClass>(&created, &was_destroyed));
    EXPECT_EQ(created, r.get());
    created->AddRef();
  }
  EXPECT_FALSE(was_destroyed);
  created->Release();
  EXPECT_TRUE(was_destroyed);
}

TEST(RefCountedTest, Mix) {
  MySubclass* created = nullptr;
  bool was_destroyed = false;
  RefPtr<MySubclass> r1(MakeRefCounted<MySubclass>(&created, &was_destroyed));
  ASSERT_FALSE(was_destroyed);
  EXPECT_TRUE(created->HasOneRef());
  created->AssertHasOneRef();

  RefPtr<MySubclass> r2 = r1;
  ASSERT_FALSE(was_destroyed);
  EXPECT_FALSE(created->HasOneRef());

  r1 = nullptr;
  ASSERT_FALSE(was_destroyed);
  created->AssertHasOneRef();

  {
    RefPtr<MyClass> r3 = r2;
    EXPECT_FALSE(created->HasOneRef());
    {
      // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
      RefPtr<MyClass> r4(r3);
      r2 = nullptr;
      ASSERT_FALSE(was_destroyed);
      EXPECT_FALSE(created->HasOneRef());
    }
    ASSERT_FALSE(was_destroyed);
    EXPECT_TRUE(created->HasOneRef());
    created->AssertHasOneRef();

    r1 = RefPtr<MySubclass>(static_cast<MySubclass*>(r3.get()));
    ASSERT_FALSE(was_destroyed);
    EXPECT_FALSE(created->HasOneRef());
  }
  ASSERT_FALSE(was_destroyed);
  EXPECT_TRUE(created->HasOneRef());
  created->AssertHasOneRef();

  EXPECT_EQ(created, r1.get());

  r1 = nullptr;
  EXPECT_TRUE(was_destroyed);
}

class MyPublicClass : public RefCountedThreadSafe<MyPublicClass> {
 public:
  // Overloaded constructors work with |MakeRefCounted()|.
  MyPublicClass() : has_num_(false), num_(0) {}
  explicit MyPublicClass(int num) : has_num_(true), num_(num) {}

  ~MyPublicClass() {}

  bool has_num() const { return has_num_; }
  int num() const { return num_; }

 private:
  bool has_num_;
  int num_;

  FML_DISALLOW_COPY_AND_ASSIGN(MyPublicClass);
};

// You can also just keep constructors and destructors public. Make sure that
// works (mostly that it compiles).
TEST(RefCountedTest, PublicCtorAndDtor) {
  RefPtr<MyPublicClass> r1 = MakeRefCounted<MyPublicClass>();
  ASSERT_TRUE(r1);
  EXPECT_FALSE(r1->has_num());

  RefPtr<MyPublicClass> r2 = MakeRefCounted<MyPublicClass>(123);
  ASSERT_TRUE(r2);
  EXPECT_TRUE(r2->has_num());
  EXPECT_EQ(123, r2->num());
  EXPECT_NE(r1.get(), r2.get());

  r1 = r2;
  EXPECT_TRUE(r1->has_num());
  EXPECT_EQ(123, r1->num());
  EXPECT_EQ(r1.get(), r2.get());

  r2 = nullptr;
  EXPECT_FALSE(r2);
  EXPECT_TRUE(r1->has_num());
  EXPECT_EQ(123, r1->num());

  r1 = nullptr;
  EXPECT_FALSE(r1);
}

// The danger with having a public constructor or destructor is that certain
// things will compile. You should get some protection by assertions in Debug
// builds.
#ifndef NDEBUG
TEST(RefCountedTest, DebugChecks) {
  {
    MyPublicClass* p = new MyPublicClass();
    EXPECT_DEATH_IF_SUPPORTED(  // NOLINT(clang-analyzer-cplusplus.NewDeleteLeaks)
        delete p, "!adoption_required_");
  }

  {
    MyPublicClass* p = new MyPublicClass();
    EXPECT_DEATH_IF_SUPPORTED(  // NOLINT(clang-analyzer-cplusplus.NewDeleteLeaks)
        RefPtr<MyPublicClass> r(p), "!adoption_required_");
  }

  {
    RefPtr<MyPublicClass> r(MakeRefCounted<MyPublicClass>());
    EXPECT_DEATH_IF_SUPPORTED(delete r.get(), "destruction_started_");
  }
}
#endif

// TODO(vtl): Add (threaded) stress tests.

}  // namespace
}  // namespace fml
