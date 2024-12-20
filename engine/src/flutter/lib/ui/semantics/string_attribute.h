// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_SEMANTICS_STRING_ATTRIBUTE_H_
#define FLUTTER_LIB_UI_SEMANTICS_STRING_ATTRIBUTE_H_

#include "flutter/lib/ui/dart_wrapper.h"
#include "third_party/tonic/dart_library_natives.h"

namespace flutter {

struct StringAttribute;

using StringAttributePtr = std::shared_ptr<flutter::StringAttribute>;
using StringAttributes = std::vector<StringAttributePtr>;

// When adding a new StringAttributeType, the classes in these file must be
// updated as well.
//  * engine/src/flutter/lib/ui/semantics.dart
//  * engine/src/flutter/lib/web_ui/lib/semantics.dart
//  * engine/src/flutter/shell/platform/android/io/flutter/view/AccessibilityBridge.java
//  * engine/src/flutter/shell/platform/embedder/embedder.h
//  * engine/src/flutter/lib/web_ui/test/engine/semantics/semantics_api_test.dart
//  * engine/src/flutter/testing/dart/semantics_test.dart

enum class StringAttributeType : int32_t {
  kSpellOut,
  kLocale,
};

//------------------------------------------------------------------------------
/// The c++ representation of the StringAttribute, this struct serves as an
/// abstract interface for the subclasses and should not be used directly.
struct StringAttribute {
  virtual ~StringAttribute() = default;
  int32_t start = -1;
  int32_t end = -1;
  StringAttributeType type;
};

//------------------------------------------------------------------------------
/// Indicates the string needs to be spelled out character by character when the
/// assistive technologies announce the string.
struct SpellOutStringAttribute : StringAttribute {};

//------------------------------------------------------------------------------
/// Indicates the string needs to be treated as a specific language when the
/// assistive technologies announce the string.
struct LocaleStringAttribute : StringAttribute {
  std::string locale;
};

//------------------------------------------------------------------------------
/// The peer class for all of the StringAttribute subclasses in semantics.dart.
class NativeStringAttribute
    : public RefCountedDartWrappable<NativeStringAttribute> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(NativeStringAttribute);

 public:
  ~NativeStringAttribute() override;

  //----------------------------------------------------------------------------
  /// The init method for SpellOutStringAttribute constructor
  static void initSpellOutStringAttribute(Dart_Handle string_attribute_handle,
                                          int32_t start,
                                          int32_t end);

  //----------------------------------------------------------------------------
  /// The init method for LocaleStringAttribute constructor
  static void initLocaleStringAttribute(Dart_Handle string_attribute_handle,
                                        int32_t start,
                                        int32_t end,
                                        std::string locale);

  //----------------------------------------------------------------------------
  /// Returns the c++ representataion of StringAttribute.
  const StringAttributePtr GetAttribute() const;

 private:
  NativeStringAttribute();
  StringAttributePtr attribute_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_SEMANTICS_STRING_ATTRIBUTE_H_
