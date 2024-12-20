// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/window_proc_delegate_manager.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

namespace {

#ifdef _WIN32
#define FLUTTER_NOINLINE __declspec(noinline)
#else
#define FLUTTER_NOINLINE __attribute__((noinline))
#endif

using TestWindowProcDelegate = std::function<std::optional<
    LRESULT>(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)>;

// A FlutterDesktopWindowProcCallback that forwards to a std::function provided
// as user_data.
FLUTTER_NOINLINE
bool TestWindowProcCallback(HWND hwnd,
                            UINT message,
                            WPARAM wparam,
                            LPARAM lparam,
                            void* user_data,
                            LRESULT* result) {
  TestWindowProcDelegate& delegate =
      *static_cast<TestWindowProcDelegate*>(user_data);
  auto delegate_result = delegate(hwnd, message, wparam, lparam);
  if (delegate_result) {
    *result = *delegate_result;
  }
  return delegate_result.has_value();
}

// Same as the above, but with a different address, to test multiple
// registration.
bool TestWindowProcCallback2(HWND hwnd,
                             UINT message,
                             WPARAM wparam,
                             LPARAM lparam,
                             void* user_data,
                             LRESULT* result) {
  return TestWindowProcCallback(hwnd, message, wparam, lparam, user_data,
                                result);
}

}  // namespace

TEST(WindowProcDelegateManagerTest, CallsCorrectly) {
  WindowProcDelegateManager manager;
  HWND dummy_hwnd;

  bool called = false;
  TestWindowProcDelegate delegate = [&called, &dummy_hwnd](
                                        HWND hwnd, UINT message, WPARAM wparam,
                                        LPARAM lparam) {
    called = true;
    EXPECT_EQ(hwnd, dummy_hwnd);
    EXPECT_EQ(message, 2);
    EXPECT_EQ(wparam, 3);
    EXPECT_EQ(lparam, 4);
    return std::optional<LRESULT>();
  };
  manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback, &delegate);
  auto result = manager.OnTopLevelWindowProc(dummy_hwnd, 2, 3, 4);

  EXPECT_TRUE(called);
  EXPECT_FALSE(result);
}

TEST(WindowProcDelegateManagerTest, ReplacementRegister) {
  WindowProcDelegateManager manager;

  bool called_a = false;
  TestWindowProcDelegate delegate_a =
      [&called_a](HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
        called_a = true;
        return std::optional<LRESULT>();
      };
  bool called_b = false;
  TestWindowProcDelegate delegate_b =
      [&called_b](HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
        called_b = true;
        return std::optional<LRESULT>();
      };
  manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback,
                                             &delegate_a);
  // The function pointer is the same, so this should replace, not add.
  manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback,
                                             &delegate_b);
  manager.OnTopLevelWindowProc(nullptr, 0, 0, 0);

  EXPECT_FALSE(called_a);
  EXPECT_TRUE(called_b);
}

TEST(WindowProcDelegateManagerTest, RegisterMultiple) {
  WindowProcDelegateManager manager;

  bool called_a = false;
  TestWindowProcDelegate delegate_a =
      [&called_a](HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
        called_a = true;
        return std::optional<LRESULT>();
      };
  bool called_b = false;
  TestWindowProcDelegate delegate_b =
      [&called_b](HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
        called_b = true;
        return std::optional<LRESULT>();
      };
  manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback,
                                             &delegate_a);
  // Function pointer is different, so both should be called.
  manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback2,
                                             &delegate_b);
  manager.OnTopLevelWindowProc(nullptr, 0, 0, 0);

  EXPECT_TRUE(called_a);
  EXPECT_TRUE(called_b);
}

TEST(WindowProcDelegateManagerTest, Ordered) {
  TestWindowProcDelegate delegate_1 = [](HWND hwnd, UINT message, WPARAM wparam,
                                         LPARAM lparam) { return 1; };
  TestWindowProcDelegate delegate_2 = [](HWND hwnd, UINT message, WPARAM wparam,
                                         LPARAM lparam) { return 2; };

  // Result should be 1 if delegate '1' is registered before delegate '2'.
  {
    WindowProcDelegateManager manager;
    manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback,
                                               &delegate_1);
    manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback2,
                                               &delegate_2);

    std::optional<LRESULT> result =
        manager.OnTopLevelWindowProc(nullptr, 0, 0, 0);

    EXPECT_EQ(result, 1);
  }

  // Result should be 2 if delegate '2' is registered before delegate '1'.
  {
    WindowProcDelegateManager manager;
    manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback2,
                                               &delegate_2);
    manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback,
                                               &delegate_1);

    std::optional<LRESULT> result =
        manager.OnTopLevelWindowProc(nullptr, 0, 0, 0);

    EXPECT_EQ(result, 2);
  }
}

TEST(WindowProcDelegateManagerTest, ConflictingDelegates) {
  WindowProcDelegateManager manager;

  bool called_a = false;
  TestWindowProcDelegate delegate_a =
      [&called_a](HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
        called_a = true;
        return std::optional<LRESULT>(1);
      };
  bool called_b = false;
  TestWindowProcDelegate delegate_b =
      [&called_b](HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
        called_b = true;
        return std::optional<LRESULT>(1);
      };
  manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback,
                                             &delegate_a);
  manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback2,
                                             &delegate_b);
  auto result = manager.OnTopLevelWindowProc(nullptr, 0, 0, 0);

  EXPECT_TRUE(result);
  // Exactly one of the handlers should be called since each will claim to have
  // handled the message. Which one is unspecified, since the calling order is
  // unspecified.
  EXPECT_TRUE(called_a || called_b);
  EXPECT_NE(called_a, called_b);
}

TEST(WindowProcDelegateManagerTest, Unregister) {
  WindowProcDelegateManager manager;

  bool called = false;
  TestWindowProcDelegate delegate = [&called](HWND hwnd, UINT message,
                                              WPARAM wparam, LPARAM lparam) {
    called = true;
    return std::optional<LRESULT>();
  };
  manager.RegisterTopLevelWindowProcDelegate(TestWindowProcCallback, &delegate);
  manager.UnregisterTopLevelWindowProcDelegate(TestWindowProcCallback);
  auto result = manager.OnTopLevelWindowProc(nullptr, 0, 0, 0);

  EXPECT_FALSE(result);
  EXPECT_FALSE(called);
}

}  // namespace testing
}  // namespace flutter
