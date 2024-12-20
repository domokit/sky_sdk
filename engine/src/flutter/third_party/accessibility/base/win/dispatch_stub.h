// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_WIN_DISPATCH_STUB_H_
#define BASE_WIN_DISPATCH_STUB_H_

#include <wrl/client.h>
#include <wrl/implements.h>

namespace base {
namespace win {
namespace test {

// An unimplemented IDispatch subclass for testing purposes.
class DispatchStub
    : public Microsoft::WRL::RuntimeClass<
          Microsoft::WRL::RuntimeClassFlags<Microsoft::WRL::ClassicCom>,
          IDispatch> {
 public:
  DispatchStub() = default;
  DispatchStub(const DispatchStub&) = delete;
  DispatchStub& operator=(const DispatchStub&) = delete;

  // IDispatch:
  IFACEMETHODIMP GetTypeInfoCount(UINT*) override;
  IFACEMETHODIMP GetTypeInfo(UINT, LCID, ITypeInfo**) override;
  IFACEMETHODIMP GetIDsOfNames(REFIID, LPOLESTR*, UINT, LCID, DISPID*) override;
  IFACEMETHODIMP Invoke(DISPID,
                        REFIID,
                        LCID,
                        WORD,
                        DISPPARAMS*,
                        VARIANT*,
                        EXCEPINFO*,
                        UINT*) override;
};

}  // namespace test
}  // namespace win
}  // namespace base

#endif  // BASE_WIN_DISPATCH_STUB_H_
