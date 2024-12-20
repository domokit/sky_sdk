// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_DART_WRAPPER_H_
#define FLUTTER_LIB_UI_DART_WRAPPER_H_

#include "flutter/fml/memory/ref_counted.h"
#include "third_party/tonic/dart_wrappable.h"

namespace flutter {

template <typename T>
class RefCountedDartWrappable : public fml::RefCountedThreadSafe<T>,
                                public tonic::DartWrappable {
 public:
  virtual void RetainDartWrappableReference() const override {
    fml::RefCountedThreadSafe<T>::AddRef();
  }

  virtual void ReleaseDartWrappableReference() const override {
    fml::RefCountedThreadSafe<T>::Release();
  }
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_DART_WRAPPER_H_
