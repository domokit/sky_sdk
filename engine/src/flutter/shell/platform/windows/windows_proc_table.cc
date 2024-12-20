// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/windows_proc_table.h"

#include <WinUser.h>
#include <dwmapi.h>

namespace flutter {

WindowsProcTable::WindowsProcTable() {
  user32_ = fml::NativeLibrary::Create("user32.dll");
  get_pointer_type_ =
      user32_->ResolveFunction<GetPointerType_*>("GetPointerType");
}

WindowsProcTable::~WindowsProcTable() {
  user32_ = nullptr;
}

BOOL WindowsProcTable::GetPointerType(UINT32 pointer_id,
                                      POINTER_INPUT_TYPE* pointer_type) const {
  if (!get_pointer_type_.has_value()) {
    return FALSE;
  }

  return get_pointer_type_.value()(pointer_id, pointer_type);
}

LRESULT WindowsProcTable::GetThreadPreferredUILanguages(DWORD flags,
                                                        PULONG count,
                                                        PZZWSTR languages,
                                                        PULONG length) const {
  return ::GetThreadPreferredUILanguages(flags, count, languages, length);
}

bool WindowsProcTable::GetHighContrastEnabled() const {
  HIGHCONTRAST high_contrast = {.cbSize = sizeof(HIGHCONTRAST)};
  if (!::SystemParametersInfoW(SPI_GETHIGHCONTRAST, sizeof(HIGHCONTRAST),
                               &high_contrast, 0)) {
    return false;
  }

  return high_contrast.dwFlags & HCF_HIGHCONTRASTON;
}

bool WindowsProcTable::DwmIsCompositionEnabled() const {
  BOOL composition_enabled;
  if (SUCCEEDED(::DwmIsCompositionEnabled(&composition_enabled))) {
    return composition_enabled;
  }

  return true;
}

HRESULT WindowsProcTable::DwmFlush() const {
  return ::DwmFlush();
}

}  // namespace flutter
