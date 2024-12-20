// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/semantics/string_attribute.h"

#include "flutter/fml/logging.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/tonic/dart_args.h"
#include "third_party/tonic/dart_binding_macros.h"

#include <memory>
#include <utility>

namespace flutter {

IMPLEMENT_WRAPPERTYPEINFO(ui, NativeStringAttribute);

NativeStringAttribute::NativeStringAttribute() {}

NativeStringAttribute::~NativeStringAttribute() {}

void NativeStringAttribute::initSpellOutStringAttribute(
    Dart_Handle string_attribute_handle,
    int32_t start,
    int32_t end) {
  UIDartState::ThrowIfUIOperationsProhibited();
  auto native_string_attribute = fml::MakeRefCounted<NativeStringAttribute>();
  native_string_attribute->AssociateWithDartWrapper(string_attribute_handle);

  native_string_attribute->attribute_ =
      std::make_shared<SpellOutStringAttribute>();
  native_string_attribute->attribute_->start = start;
  native_string_attribute->attribute_->end = end;
  native_string_attribute->attribute_->type = StringAttributeType::kSpellOut;
}

void NativeStringAttribute::initLocaleStringAttribute(
    Dart_Handle string_attribute_handle,
    int32_t start,
    int32_t end,
    std::string locale) {
  UIDartState::ThrowIfUIOperationsProhibited();
  auto native_string_attribute = fml::MakeRefCounted<NativeStringAttribute>();
  native_string_attribute->AssociateWithDartWrapper(string_attribute_handle);

  auto locale_attribute = std::make_shared<LocaleStringAttribute>();
  locale_attribute->start = start;
  locale_attribute->end = end;
  locale_attribute->type = StringAttributeType::kLocale;
  locale_attribute->locale = std::move(locale);
  native_string_attribute->attribute_ = locale_attribute;
}

const StringAttributePtr NativeStringAttribute::GetAttribute() const {
  return attribute_;
}

}  // namespace flutter
