// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Provides a smart pointer class for intrusively reference-counted objects.

#ifndef FLUTTER_FML_MEMORY_REF_PTR_H_
#define FLUTTER_FML_MEMORY_REF_PTR_H_

#include <cstddef>
#include <functional>
#include <utility>

#include "flutter/fml/logging.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_ptr_internal.h"

namespace fml {

// A smart pointer class for intrusively reference-counted objects (e.g., those
// subclassing |RefCountedThreadSafe| -- see ref_counted.h).
//
// Such objects require *adoption* to obtain the first |RefPtr|, which is
// accomplished using |AdoptRef| (see below). (This is due to such objects being
// constructed with a reference count of 1. The adoption requirement is
// enforced, at least in Debug builds, by assertions.)
//
// E.g., if |Foo| is an intrusively reference-counted class:
//
//   // The |AdoptRef| may be put in a static factory method (e.g., if |Foo|'s
//   // constructor is private).
//   RefPtr<Foo> my_foo_ptr(AdoptRef(new Foo()));
//
//   // Now OK, since "my Foo" has been adopted ...
//   RefPtr<Foo> another_ptr_to_my_foo(my_foo_ptr.get());
//
//   // ... though this would preferable in this situation.
//   RefPtr<Foo> yet_another_ptr_to_my_foo(my_foo_ptr);
//
// Unlike Chromium's |scoped_refptr|, |RefPtr| is only explicitly constructible
// from a plain pointer (and not assignable). It is however implicitly
// constructible from |nullptr|. So:
//
//   RefPtr<Foo> foo(plain_ptr_to_adopted_foo);    // OK.
//   foo = plain_ptr_to_adopted_foo;               // Not OK (doesn't compile).
//   foo = RefPtr<Foo>(plain_ptr_to_adopted_foo);  // OK.
//   foo = nullptr;                                // OK.
//
// And if we have |void MyFunction(RefPtr<Foo> foo)|, calling it using
// |MyFunction(nullptr)| is also valid.
//
// Implementation note: For copy/move constructors/operator=s, we often have
// templated versions, so that the operation can be done on a |RefPtr<U>|, where
// |U| is a subclass of |T|. However, we also have non-templated versions with
// |U = T|, since the templated versions don't count as copy/move
// constructors/operator=s for the purposes of causing the default copy
// constructor/operator= to be deleted. E.g., if we didn't declare any
// non-templated versions, we'd get the default copy constructor/operator= (we'd
// only not get the default move constructor/operator= by virtue of having a
// destructor)! (In fact, it'd suffice to only declare a non-templated move
// constructor or move operator=, which would cause the copy
// constructor/operator= to be deleted, but for clarity we include explicit
// non-templated versions of everything.)
template <typename T>
class RefPtr final {
 public:
  RefPtr() : ptr_(nullptr) {}
  RefPtr(std::nullptr_t)  // NOLINT(google-explicit-constructor)
      : ptr_(nullptr) {}

  // Explicit constructor from a plain pointer (to an object that must have
  // already been adopted). (Note that in |T::T()|, references to |this| cannot
  // be taken, since the object being constructed will not have been adopted
  // yet.)
  template <typename U>
  explicit RefPtr(U* p) : ptr_(p) {
    if (ptr_) {
      ptr_->AddRef();
    }
  }

  // Copy constructor.
  RefPtr(const RefPtr<T>& r)  // NOLINT(google-explicit-constructor)
      : ptr_(r.ptr_) {
    if (ptr_) {
      ptr_->AddRef();
    }
  }

  template <typename U>
  RefPtr(const RefPtr<U>& r)  // NOLINT(google-explicit-constructor)
      : ptr_(r.ptr_) {
    if (ptr_) {
      ptr_->AddRef();
    }
  }

  // Move constructor.
  RefPtr(RefPtr<T>&& r) : ptr_(r.ptr_) {  // NOLINT(google-explicit-constructor)
    r.ptr_ = nullptr;
  }

  template <typename U>
  RefPtr(RefPtr<U>&& r) : ptr_(r.ptr_) {  // NOLINT(google-explicit-constructor)
    r.ptr_ = nullptr;
  }

  // Destructor.
  ~RefPtr() {
    if (ptr_) {
      // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDelete)
      ptr_->Release();
    }
  }

  T* get() const { return ptr_; }

  T& operator*() const {
    FML_DCHECK(ptr_);
    return *ptr_;
  }

  T* operator->() const {
    FML_DCHECK(ptr_);
    return ptr_;
  }

  // Copy assignment.
  RefPtr<T>& operator=(const RefPtr<T>& r) {
    // Handle self-assignment.
    if (r.ptr_ == ptr_) {
      return *this;
    }
    if (r.ptr_) {
      r.ptr_->AddRef();
    }
    T* old_ptr = ptr_;
    ptr_ = r.ptr_;
    if (old_ptr) {
      old_ptr->Release();
    }
    return *this;
  }

  template <typename U>
  RefPtr<T>& operator=(const RefPtr<U>& r) {
    if (reinterpret_cast<T*>(r.ptr_) == ptr_) {
      return *this;
    }
    if (r.ptr_) {
      r.ptr_->AddRef();
    }
    T* old_ptr = ptr_;
    ptr_ = r.ptr_;
    if (old_ptr) {
      old_ptr->Release();
    }
    return *this;
  }

  // Move assignment.
  // Note: Like |std::shared_ptr|, we support self-move and move assignment is
  // equivalent to |RefPtr<T>(std::move(r)).swap(*this)|.
  RefPtr<T>& operator=(RefPtr<T>&& r) {
    RefPtr<T>(std::move(r)).swap(*this);
    return *this;
  }

  template <typename U>
  RefPtr<T>& operator=(RefPtr<U>&& r) {
    RefPtr<T>(std::move(r)).swap(*this);
    return *this;
  }

  void swap(RefPtr<T>& r) {
    T* p = ptr_;
    ptr_ = r.ptr_;
    r.ptr_ = p;
  }

  // Returns a new |RefPtr<T>| with the same contents as this pointer. Useful
  // when a function takes a |RefPtr<T>&&| argument and the caller wants to
  // retain its reference (rather than moving it).
  RefPtr<T> Clone() const { return *this; }

  explicit operator bool() const { return !!ptr_; }

  template <typename U>
  bool operator==(const RefPtr<U>& rhs) const {
    return ptr_ == rhs.ptr_;
  }

  template <typename U>
  bool operator!=(const RefPtr<U>& rhs) const {
    return !operator==(rhs);
  }

  template <typename U>
  bool operator<(const RefPtr<U>& rhs) const {
    return ptr_ < rhs.ptr_;
  }

 private:
  template <typename U>
  friend class RefPtr;

  friend RefPtr<T> AdoptRef<T>(T*);

  enum AdoptTag { kAdopt };
  RefPtr(T* ptr, AdoptTag) : ptr_(ptr) { FML_DCHECK(ptr_); }

  T* ptr_;
};

// Adopts a newly-created |T|. Typically used in a static factory method, like:
//
//   // static
//   RefPtr<Foo> Foo::Create() {
//     return AdoptRef(new Foo());
//   }
template <typename T>
inline RefPtr<T> AdoptRef(T* ptr) {
#ifndef NDEBUG
  ptr->Adopt();
#endif
  return RefPtr<T>(ptr, RefPtr<T>::kAdopt);
}

// Constructs a |RefPtr<T>| from a plain pointer (to an object that must
// have already been adoped).  Avoids having to spell out the full type name.
//
//   Foo* foo = ...;
//   auto foo_ref = Ref(foo);
//
// (|foo_ref| will be of type |RefPtr<Foo>|.)
template <typename T>
inline RefPtr<T> Ref(T* ptr) {
  return RefPtr<T>(ptr);
}

// Creates an intrusively reference counted |T|, producing a |RefPtr<T>| (and
// performing the required adoption). Use like:
//
//   auto my_foo = MakeRefCounted<Foo>(ctor_arg1, ctor_arg2);
//
// (|my_foo| will be of type |RefPtr<Foo>|.)
template <typename T, typename... Args>
RefPtr<T> MakeRefCounted(Args&&... args) {
  return internal::MakeRefCountedHelper<T>::MakeRefCounted(
      std::forward<Args>(args)...);
}

}  // namespace fml

// Inject custom std::hash<> function object for |RefPtr<T>|.
namespace std {
template <typename T>
struct hash<fml::RefPtr<T>> {
  using argument_type = fml::RefPtr<T>;
  using result_type = std::size_t;

  result_type operator()(const argument_type& ptr) const {
    return std::hash<T*>()(ptr.get());
  }
};
}  // namespace std

#endif  // FLUTTER_FML_MEMORY_REF_PTR_H_
