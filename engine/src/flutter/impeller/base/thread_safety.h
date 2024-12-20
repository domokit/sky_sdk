// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_BASE_THREAD_SAFETY_H_
#define FLUTTER_IMPELLER_BASE_THREAD_SAFETY_H_

#if defined(__clang__)
#define IPLR_THREAD_ANNOTATION_ATTRIBUTE__(x) __attribute__((x))
#else
#define IPLR_THREAD_ANNOTATION_ATTRIBUTE__(x)  // no-op
#endif

#define IPLR_CAPABILITY(x) IPLR_THREAD_ANNOTATION_ATTRIBUTE__(capability(x))

#define IPLR_SCOPED_CAPABILITY \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(scoped_lockable)

#define IPLR_GUARDED_BY(x) IPLR_THREAD_ANNOTATION_ATTRIBUTE__(guarded_by(x))

#define IPLR_PT_GUARDED_BY(x) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(pt_guarded_by(x))

#define IPLR_ACQUIRED_BEFORE(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(acquired_before(__VA_ARGS__))

#define IPLR_ACQUIRED_AFTER(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(acquired_after(__VA_ARGS__))

#define IPLR_REQUIRES(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(requires_capability(__VA_ARGS__))

#define IPLR_REQUIRES_SHARED(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(requires_shared_capability(__VA_ARGS__))

#define IPLR_ACQUIRE(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(acquire_capability(__VA_ARGS__))

#define IPLR_ACQUIRE_SHARED(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(acquire_shared_capability(__VA_ARGS__))

#define IPLR_RELEASE(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(release_capability(__VA_ARGS__))

#define IPLR_RELEASE_SHARED(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(release_shared_capability(__VA_ARGS__))

#define IPLR_RELEASE_GENERIC(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(release_generic_capability(__VA_ARGS__))

#define IPLR_TRY_ACQUIRE(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(try_acquire_capability(__VA_ARGS__))

#define IPLR_TRY_ACQUIRE_SHARED(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(try_acquire_shared_capability(__VA_ARGS__))

#define IPLR_EXCLUDES(...) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(locks_excluded(__VA_ARGS__))

#define IPLR_ASSERT_CAPABILITY(x) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(assert_capability(x))

#define IPLR_ASSERT_SHARED_CAPABILITY(x) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(assert_shared_capability(x))

#define IPLR_RETURN_CAPABILITY(x) \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(lock_returned(x))

#define IPLR_NO_THREAD_SAFETY_ANALYSIS \
  IPLR_THREAD_ANNOTATION_ATTRIBUTE__(no_thread_safety_analysis)

#endif  // FLUTTER_IMPELLER_BASE_THREAD_SAFETY_H_
