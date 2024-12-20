// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/win/dispatch_stub.h"

namespace base {
namespace win {
namespace test {

IFACEMETHODIMP DispatchStub::GetTypeInfoCount(UINT*) {
  return E_NOTIMPL;
}

IFACEMETHODIMP DispatchStub::GetTypeInfo(UINT, LCID, ITypeInfo**) {
  return E_NOTIMPL;
}

IFACEMETHODIMP DispatchStub::GetIDsOfNames(REFIID,
                                           LPOLESTR*,
                                           UINT,
                                           LCID,
                                           DISPID*) {
  return E_NOTIMPL;
}

IFACEMETHODIMP DispatchStub::Invoke(DISPID,
                                    REFIID,
                                    LCID,
                                    WORD,
                                    DISPPARAMS*,
                                    VARIANT*,
                                    EXCEPINFO*,
                                    UINT*) {
  return E_NOTIMPL;
}

}  // namespace test
}  // namespace win
}  // namespace base
