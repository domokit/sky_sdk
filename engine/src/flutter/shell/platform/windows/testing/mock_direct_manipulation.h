// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_MOCK_DIRECT_MANIPULATION_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_MOCK_DIRECT_MANIPULATION_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/direct_manipulation.h"
#include "gmock/gmock.h"

namespace flutter {
namespace testing {

/// Mock for the |DirectManipulationOwner| base class.
class MockDirectManipulationOwner : public DirectManipulationOwner {
 public:
  explicit MockDirectManipulationOwner(FlutterWindow* window)
      : DirectManipulationOwner(window){};
  virtual ~MockDirectManipulationOwner() = default;

  MOCK_METHOD(void, SetContact, (UINT contact_id), (override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockDirectManipulationOwner);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_MOCK_DIRECT_MANIPULATION_H_
