// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_MOCK_WINDOWS_PROC_TABLE_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_MOCK_WINDOWS_PROC_TABLE_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/windows_proc_table.h"
#include "gmock/gmock.h"

namespace flutter {
namespace testing {

/// Mock for the |WindowsProcTable| base class.
class MockWindowsProcTable : public WindowsProcTable {
 public:
  MockWindowsProcTable() = default;
  virtual ~MockWindowsProcTable() = default;

  MOCK_METHOD(BOOL,
              GetPointerType,
              (UINT32 pointer_id, POINTER_INPUT_TYPE* pointer_type),
              (const, override));

  MOCK_METHOD(LRESULT,
              GetThreadPreferredUILanguages,
              (DWORD, PULONG, PZZWSTR, PULONG),
              (const, override));

  MOCK_METHOD(bool, GetHighContrastEnabled, (), (const, override));

  MOCK_METHOD(bool, DwmIsCompositionEnabled, (), (const, override));

  MOCK_METHOD(HRESULT, DwmFlush, (), (const, override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockWindowsProcTable);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_MOCK_WINDOWS_PROC_TABLE_H_
