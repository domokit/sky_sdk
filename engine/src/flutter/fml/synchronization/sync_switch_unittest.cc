// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/synchronization/sync_switch.h"

#include <thread>

#include "gtest/gtest.h"

using fml::SyncSwitch;

TEST(SyncSwitchTest, Basic) {
  SyncSwitch sync_switch;
  bool switch_value = false;
  sync_switch.Execute(SyncSwitch::Handlers()
                          .SetIfTrue([&] { switch_value = true; })
                          .SetIfFalse([&] { switch_value = false; }));
  EXPECT_FALSE(switch_value);
  sync_switch.SetSwitch(true);
  sync_switch.Execute(SyncSwitch::Handlers()
                          .SetIfTrue([&] { switch_value = true; })
                          .SetIfFalse([&] { switch_value = false; }));
  EXPECT_TRUE(switch_value);
}

TEST(SyncSwitchTest, NoopIfUndefined) {
  SyncSwitch sync_switch;
  bool switch_value = false;
  sync_switch.Execute(SyncSwitch::Handlers());
  EXPECT_FALSE(switch_value);
}

TEST(SyncSwitchTest, SharedLock) {
  SyncSwitch sync_switch;
  sync_switch.SetSwitch(true);
  bool switch_value1 = false;
  bool switch_value2 = false;

  std::thread thread1([&] {
    sync_switch.Execute(
        SyncSwitch::Handlers()
            .SetIfTrue([&] {
              switch_value1 = true;

              std::thread thread2([&]() {
                sync_switch.Execute(
                    SyncSwitch::Handlers()
                        .SetIfTrue([&] { switch_value2 = true; })
                        .SetIfFalse([&] { switch_value2 = false; }));
              });
              thread2.join();
            })
            .SetIfFalse([&] { switch_value1 = false; }));
  });
  thread1.join();
  EXPECT_TRUE(switch_value1);
  EXPECT_TRUE(switch_value2);
}
