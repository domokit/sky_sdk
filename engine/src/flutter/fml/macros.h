// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_MACROS_H_
#define FLUTTER_FML_MACROS_H_

#ifndef FML_USED_ON_EMBEDDER

#define FML_EMBEDDER_ONLY [[deprecated]]

#else  // FML_USED_ON_EMBEDDER

#define FML_EMBEDDER_ONLY

#endif  // FML_USED_ON_EMBEDDER

#define FML_DISALLOW_COPY(TypeName) TypeName(const TypeName&) = delete

#define FML_DISALLOW_ASSIGN(TypeName) \
  TypeName& operator=(const TypeName&) = delete

#define FML_DISALLOW_MOVE(TypeName) \
  TypeName(TypeName&&) = delete;    \
  TypeName& operator=(TypeName&&) = delete

#define FML_DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&) = delete;          \
  TypeName& operator=(const TypeName&) = delete

#define FML_DISALLOW_COPY_ASSIGN_AND_MOVE(TypeName) \
  TypeName(const TypeName&) = delete;               \
  TypeName(TypeName&&) = delete;                    \
  TypeName& operator=(const TypeName&) = delete;    \
  TypeName& operator=(TypeName&&) = delete

#define FML_DISALLOW_IMPLICIT_CONSTRUCTORS(TypeName) \
  TypeName() = delete;                               \
  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(TypeName)

#define FML_TEST_NAME(test_case_name, test_name) \
  test_case_name##_##test_name##_Test

#define FML_TEST_CLASS(test_case_name, test_name) \
  class FML_TEST_NAME(test_case_name, test_name)

#define FML_FRIEND_TEST(test_case_name, test_name) \
  friend FML_TEST_CLASS(test_case_name, test_name)

#endif  // FLUTTER_FML_MACROS_H_
