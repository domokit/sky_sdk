// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "export.h"

#include "third_party/skia/include/core/SkString.h"

SKWASM_EXPORT SkString* skString_allocate(size_t length) {
  return new SkString(length);
}

SKWASM_EXPORT char* skString_getData(SkString* string) {
  return string->data();
}

SKWASM_EXPORT int skString_getLength(SkString* string) {
  return string->size();
}

SKWASM_EXPORT void skString_free(SkString* string) {
  return delete string;
}

SKWASM_EXPORT std::u16string* skString16_allocate(size_t length) {
  std::u16string* string = new std::u16string();
  string->resize(length);
  return string;
}

SKWASM_EXPORT char16_t* skString16_getData(std::u16string* string) {
  return string->data();
}

SKWASM_EXPORT void skString16_free(std::u16string* string) {
  delete string;
}
