// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/dart_vm.h"

#include "flutter/runtime/dart_vm_lifecycle.h"
#include "flutter/testing/fixture_test.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

using DartVMTest = FixtureTest;

TEST_F(DartVMTest, SimpleInitialization) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  auto vm = DartVMRef::Create(CreateSettingsForFixture());
  ASSERT_TRUE(vm);
}

TEST_F(DartVMTest, SimpleIsolateNameServer) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  auto vm = DartVMRef::Create(CreateSettingsForFixture());
  ASSERT_TRUE(vm);
  ASSERT_TRUE(vm.GetVMData());
  auto ns = vm->GetIsolateNameServer();
  ASSERT_EQ(ns->LookupIsolatePortByName("foobar").port_id, ILLEGAL_PORT);
  ASSERT_FALSE(ns->RemoveIsolateNameMapping("foobar"));
  Dart_PortEx correct_portex = {123, 456};
  ASSERT_TRUE(ns->RegisterIsolatePortWithName(correct_portex, "foobar"));
  ASSERT_FALSE(ns->RegisterIsolatePortWithName(correct_portex, "foobar"));
  Dart_PortEx response = ns->LookupIsolatePortByName("foobar");
  ASSERT_EQ(response.port_id, correct_portex.port_id);
  ASSERT_EQ(response.origin_id, correct_portex.origin_id);
  ASSERT_TRUE(ns->RemoveIsolateNameMapping("foobar"));
}

TEST_F(DartVMTest, OldGenHeapSize) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  auto settings = CreateSettingsForFixture();
  settings.old_gen_heap_size = 1024;
  auto vm = DartVMRef::Create(settings);
  // There is no way to introspect on the heap size so we just assert the vm was
  // created.
  ASSERT_TRUE(vm);
}

TEST_F(DartVMTest, DisableTimelineEventHandler) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  fml::tracing::TraceSetTimelineEventHandler(nullptr);
  auto settings = CreateSettingsForFixture();
  settings.enable_timeline_event_handler = false;
  auto vm = DartVMRef::Create(settings);
  ASSERT_FALSE(fml::tracing::TraceHasTimelineEventHandler());
}

TEST_F(DartVMTest, TraceGetTimelineMicrosDoesNotGetClockWhenSystraceIsEnabled) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  auto settings = CreateSettingsForFixture();
  settings.trace_systrace = true;
  auto vm = DartVMRef::Create(settings);
  ASSERT_EQ(-1, fml::tracing::TraceGetTimelineMicros());
}

}  // namespace testing
}  // namespace flutter
