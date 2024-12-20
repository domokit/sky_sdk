// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_UTILS_DL_COMPARABLE_H_
#define FLUTTER_DISPLAY_LIST_UTILS_DL_COMPARABLE_H_

#include <memory>

namespace flutter {

// These templates implement deep pointer comparisons that compare not
// just the pointers to the objects, but also their contents (provided
// that the <T> class implements the == operator override).
// Any combination of shared_ptr<T> or T* are supported and null pointers
// are not equal to anything but another null pointer.

template <class T, class U>
bool Equals(const T* a, const U* b) {
  if (a == b) {
    return true;
  }
  if (!a || !b) {
    return false;
  }
  return *a == *b;
}

template <class T, class U>
bool Equals(std::shared_ptr<const T> a, const U* b) {
  return Equals(a.get(), b);
}

template <class T, class U>
bool Equals(std::shared_ptr<T> a, const U* b) {
  return Equals(a.get(), b);
}

template <class T, class U>
bool Equals(const T* a, std::shared_ptr<const U> b) {
  return Equals(a, b.get());
}

template <class T, class U>
bool Equals(const T* a, std::shared_ptr<U> b) {
  return Equals(a, b.get());
}

template <class T, class U>
bool Equals(std::shared_ptr<const T> a, std::shared_ptr<const U> b) {
  return Equals(a.get(), b.get());
}

template <class T, class U>
bool Equals(std::shared_ptr<T> a, std::shared_ptr<const U> b) {
  return Equals(a.get(), b.get());
}

template <class T, class U>
bool Equals(std::shared_ptr<const T> a, std::shared_ptr<U> b) {
  return Equals(a.get(), b.get());
}

template <class T, class U>
bool Equals(std::shared_ptr<T> a, std::shared_ptr<U> b) {
  return Equals(a.get(), b.get());
}

template <class T, class U>
bool NotEquals(const T* a, const U* b) {
  return !Equals(a, b);
}

template <class T, class U>
bool NotEquals(std::shared_ptr<const T> a, const U* b) {
  return !Equals(a.get(), b);
}

template <class T, class U>
bool NotEquals(std::shared_ptr<T> a, const U* b) {
  return !Equals(a.get(), b);
}

template <class T, class U>
bool NotEquals(const T* a, std::shared_ptr<const U> b) {
  return !Equals(a, b.get());
}

template <class T, class U>
bool NotEquals(const T* a, std::shared_ptr<U> b) {
  return !Equals(a, b.get());
}

template <class T, class U>
bool NotEquals(std::shared_ptr<const T> a, std::shared_ptr<const U> b) {
  return !Equals(a.get(), b.get());
}

template <class T, class U>
bool NotEquals(std::shared_ptr<T> a, std::shared_ptr<const U> b) {
  return !Equals(a.get(), b.get());
}

template <class T, class U>
bool NotEquals(std::shared_ptr<const T> a, std::shared_ptr<U> b) {
  return !Equals(a.get(), b.get());
}

template <class T, class U>
bool NotEquals(std::shared_ptr<T> a, std::shared_ptr<U> b) {
  return !Equals(a.get(), b.get());
}

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_UTILS_DL_COMPARABLE_H_
