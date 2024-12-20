// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_OBJECT_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_OBJECT_H_

#include <atomic>
#include <utility>

#include "flutter/fml/logging.h"

namespace impeller::interop {

class ObjectBase {
 public:
  ObjectBase() = default;

  virtual ~ObjectBase() = default;

  ObjectBase(const ObjectBase&) = delete;

  ObjectBase(ObjectBase&&) = delete;

  ObjectBase& operator=(const ObjectBase&) = delete;

  ObjectBase& operator=(ObjectBase&&) = delete;

  void Retain() { ref_count_++; }

  void Release() {
    if (ref_count_-- == 1u) {
      delete this;
    }
  }

  static void SafeRetain(void* ptr) {
    if (ptr) {
      reinterpret_cast<ObjectBase*>(ptr)->Retain();
    }
  }

  static void SafeRelease(void* ptr) {
    if (ptr) {
      reinterpret_cast<ObjectBase*>(ptr)->Release();
    }
  }

  uint64_t GetRefCountForTests() const { return ref_count_; }

 private:
  std::atomic_uint64_t ref_count_ = {1u};
};

template <typename Clasz, typename CSibling>
class Object : public ObjectBase {
 public:
  using InteropClass = Clasz;
  using InteropCSibling = CSibling;
};

enum class AdoptTag {
  kAdopted,
};

template <typename Object>
class ScopedObject final {
 public:
  ScopedObject() = default;

  ScopedObject(std::nullptr_t)  // NOLINT(google-explicit-constructor)
  {}

  explicit ScopedObject(Object* ptr, AdoptTag) : object_(ptr) {}

  explicit ScopedObject(Object* ptr) : object_(ptr) {
    if (object_) {
      object_->Retain();
    }
  }

  ~ScopedObject() { Release(); }

  ScopedObject(const ScopedObject& other) : ScopedObject(other.Get()) {}

  ScopedObject(ScopedObject&& other) { std::swap(object_, other.object_); }

  ScopedObject& operator=(const ScopedObject& other) {
    // Self assignment.
    if (object_ == other.object_) {
      return *this;
    }
    if (other.object_) {
      other.object_->Retain();
    }
    Release();
    FML_DCHECK(object_ == nullptr);
    object_ = other.object_;
    return *this;
  }

  ScopedObject& operator=(ScopedObject&& other) {
    std::swap(object_, other.object_);
    return *this;
  }

  Object* Get() const { return object_; }

  typename Object::InteropCSibling* GetC() const {
    return reinterpret_cast<typename Object::InteropCSibling*>(Get());
  }

  Object& operator*() const {
    FML_DCHECK(object_);
    return *object_;
  }

  Object* operator->() const {
    FML_DCHECK(object_);
    return object_;
  }

  explicit operator bool() const { return !!object_; }

  [[nodiscard]]
  typename Object::InteropCSibling* Leak() {
    auto to_leak = object_;
    object_ = nullptr;
    return reinterpret_cast<typename Object::InteropCSibling*>(to_leak);
  }

 private:
  Object* object_ = nullptr;

  void Release() {
    if (object_) {
      // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDelete)
      object_->Release();
      object_ = nullptr;
    }
  }
};

template <typename Object>
ScopedObject<Object> Ref(Object* object) {
  return ScopedObject<Object>{object};
}

template <typename Object>
ScopedObject<Object> Adopt(Object* object) {
  return ScopedObject<Object>{object, AdoptTag::kAdopted};
}

template <typename Object>
ScopedObject<Object> Adopt(typename Object::InteropCSibling* object) {
  return Adopt(reinterpret_cast<Object*>(object));
}

template <typename Object, typename... CtorArgs>
ScopedObject<Object> Create(CtorArgs&&... args) {
  return ScopedObject<Object>{new Object(std::forward<CtorArgs>(args)...),
                              AdoptTag::kAdopted};
}

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_OBJECT_H_
