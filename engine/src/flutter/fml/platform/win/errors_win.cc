// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/win/errors_win.h"

#include <Windows.h>

#include <sstream>

#include "flutter/fml/platform/win/wstring_conversion.h"

namespace fml {

std::string GetLastErrorMessage() {
  DWORD last_error = ::GetLastError();
  if (last_error == 0) {
    return {};
  }

  const DWORD flags = FORMAT_MESSAGE_ALLOCATE_BUFFER |
                      FORMAT_MESSAGE_FROM_SYSTEM |
                      FORMAT_MESSAGE_IGNORE_INSERTS;

  wchar_t* buffer = nullptr;
  size_t size = ::FormatMessage(
      flags,                                      // dwFlags
      NULL,                                       // lpSource
      last_error,                                 // dwMessageId
      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),  // dwLanguageId
      (LPWSTR)&buffer,                            // lpBuffer
      0,                                          // nSize
      NULL                                        // Arguments
  );

  std::wstring message(buffer, size);

  ::LocalFree(buffer);

  std::wstringstream stream;
  stream << message << " (" << last_error << ").";

  return WideStringToUtf8(stream.str());
}

}  // namespace fml
