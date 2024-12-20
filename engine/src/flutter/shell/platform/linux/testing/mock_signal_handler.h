// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_SIGNAL_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_SIGNAL_HANDLER_H_

#include <glib-object.h>
#include <glib.h>

#include "gmock/gmock.h"

// Expects a signal that has no arguments.
//
// MockSignalHandler timeout(timer, "timeout");
// EXPECT_SIGNAL(timeout).Times(3);
//
#define EXPECT_SIGNAL(mock) EXPECT_CALL(mock, Handler())

// Expects a signal that has 1 argument.
//
// MockSignalHandler1<int> name_changed(object, "name-changed");
// EXPECT_SIGNAL(name_changed, testing::StrEq("example"));
//
#define EXPECT_SIGNAL1(mock, a1) EXPECT_CALL(mock, Handler1(a1))

// Expects a signal that has 2 arguments.
//
// MockSignalHandler2<int, GObject*> child_added(parent, "children::add");
// EXPECT_SIGNAL2(child_added, testing::Eq(1), testing::A<GObject*>());
//
#define EXPECT_SIGNAL2(mock, a1, a2) EXPECT_CALL(mock, Handler2(a1, a2))

namespace flutter {
namespace testing {

class SignalHandler {
 public:
  SignalHandler(gpointer instance, const gchar* name, GCallback callback);
  virtual ~SignalHandler();

 private:
  gulong id_ = 0;
  gpointer instance_ = nullptr;
};

// A mock signal handler that has no arguments. Used with EXPECT_SIGNAL().
class MockSignalHandler : public SignalHandler {
 public:
  MockSignalHandler(gpointer instance, const gchar* name)
      : SignalHandler(instance, name, G_CALLBACK(OnSignal)) {}

  MOCK_METHOD(void, Handler, ());

 private:
  static void OnSignal(MockSignalHandler* mock) { mock->Handler(); }
};

// A mock signal handler that has 1 argument. Used with EXPECT_SIGNAL1().
template <typename A1>
class MockSignalHandler1 : public SignalHandler {
 public:
  MockSignalHandler1(gpointer instance, const gchar* name)
      : SignalHandler(instance, name, G_CALLBACK(OnSignal1)) {}

  MOCK_METHOD(void, Handler1, (A1 a1));

 private:
  static void OnSignal1(MockSignalHandler1* mock, A1 a1) { mock->Handler1(a1); }
};

// A mock signal handler that has 2 arguments. Used with EXPECT_SIGNAL2().
template <typename A1, typename A2>
class MockSignalHandler2 : public SignalHandler {
 public:
  MockSignalHandler2(gpointer instance, const gchar* name)
      : SignalHandler(instance, name, G_CALLBACK(OnSignal2)) {}

  MOCK_METHOD(void, Handler2, (A1 a1, A2 a2));

 private:
  static void OnSignal2(MockSignalHandler2* mock, A1 a1, A2 a2) {
    mock->Handler2(a1, a2);
  }
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_SIGNAL_HANDLER_H_
