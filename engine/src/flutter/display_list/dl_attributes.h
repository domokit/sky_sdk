// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_ATTRIBUTES_H_
#define FLUTTER_DISPLAY_LIST_DL_ATTRIBUTES_H_

#include <memory>

namespace flutter {

// ===========================================================================

// The base class for a family of DisplayList attribute classes.
//
// This class is designed to support the following properties for any
// attribute to facilitate the storage of the attribute in a DisplayList
// and for use in code that inspects or creates DisplayList objects:
//
// - Typed:
//     Even though most references and pointers are passed around as the
//     attribute's base class, a Type property can be queried using the |type|
//     method to determine which type of the attribute is being used. For
//     example, the Blend type of ColorFilter will return DlColorFilter::kBlend
//     from its type method.
//
// - Inspectable:
//     Any parameters required to full specify the action of the attribute are
//     provided on the type-specific classes.
//
// - Safely Downcast:
//     For the subclasses that have specific data to query, methods are
//     provided to safely downcast the reference for inspection. The down
//     casting method will either return a pointer to the instance with
//     its type-specific class type, or nullptr if it is executed on the
//     wrong type of instance.
//     (eg. DlColorFilter::asBlend() or DlMaskFilter::asBlur())
//
// - Immutable:
//     Neither the base class or any of the subclasses specify any mutation
//     methods. Instances are often passed around as const as a reminder,
//     but the classes have no mutation methods anyway.
//
// - Flat and Embeddable:
//     Bulk freed + bulk compared + zero memory fragmentation.
//
//     All of these classes are designed to be stored in the DisplayList
//     buffer allocated in-line with the rest of the data to avoid dangling
//     pointers that require explicit freeing when the DisplayList goes
//     away, or that fragment the memory needed to read the operations in
//     the DisplayList. Furthermore, the data in the classes can be bulk
//     compared using a |memcmp| when performing a |DisplayList::Equals|.
//
// - Passed by Pointer:
//     The data shared via the |DlOpReceiver::set<Attribute>| calls are stored
//     in the buffer itself and so their lifetime is controlled by the
//     DisplayList. That memory cannot be shared as by a |shared_ptr|
//     because the memory may be freed outside the control of the shared
//     pointer. Creating a shared version of the object would require a
//     new instantiation which we'd like to avoid on every dispatch call,
//     so a raw (const) pointer is shared in those dispatch calls instead,
//     with all of the responsibilities of non-ownership in the called method.
//
//     But, for methods that need to keep a copy of the data...
//
// - Shared_Ptr-able:
//     The classes support a method to return a |std::shared_ptr| version of
//     themselves, safely instantiating a new copy of the object into a
//     shared_ptr using |std::make_shared|. For those receiver objects
//     that may want to hold on to the contents of the object (typically
//     in a |current_attribute_| field), they can obtain a shared_ptr
//     copy safely and easily using the |shared| method.

// ===========================================================================

// |D| is the base type for the attribute
//     (i.e. DlColorFilter, etc.)
// |T| is the enum that describes the specific subclasses
//     (i.e DlColorFilterType, etc.)
template <class D, typename T>
class DlAttribute {
 public:
  // Return the recognized specific type of the attribute.
  virtual T type() const = 0;

  // Return the size of the instantiated data (typically used to allocate)
  // storage in the DisplayList buffer.
  virtual size_t size() const = 0;

  // Return a shared version of |this| attribute. The |shared_ptr| returned
  // will reference a copy of this object so that the lifetime of the shared
  // version is not tied to the storage of this particular instance.
  virtual std::shared_ptr<D> shared() const = 0;

  // Perform a content aware |==| comparison of the Attribute.
  bool operator==(D const& other) const {
    return type() == other.type() && equals_(other);
  }
  // Perform a content aware |!=| comparison of the Attribute.
  bool operator!=(D const& other) const { return !(*this == other); }

  virtual ~DlAttribute() = default;

 protected:
  // Virtual comparison method to support |==| and |!=|.
  virtual bool equals_(D const& other) const = 0;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_ATTRIBUTES_H_
