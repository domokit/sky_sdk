// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TYPOGRAPHER_BACKENDS_SKIA_TYPEFACE_SKIA_H_
#define FLUTTER_IMPELLER_TYPOGRAPHER_BACKENDS_SKIA_TYPEFACE_SKIA_H_

#include "impeller/base/backend_cast.h"
#include "impeller/typographer/typeface.h"
#include "third_party/skia/include/core/SkRefCnt.h"
#include "third_party/skia/include/core/SkTypeface.h"

namespace impeller {

class TypefaceSkia final : public Typeface,
                           public BackendCast<TypefaceSkia, Typeface> {
 public:
  explicit TypefaceSkia(sk_sp<SkTypeface> typeface);

  ~TypefaceSkia() override;

  // |Typeface|
  bool IsValid() const override;

  // |Comparable<Typeface>|
  std::size_t GetHash() const override;

  // |Comparable<Typeface>|
  bool IsEqual(const Typeface& other) const override;

  const sk_sp<SkTypeface>& GetSkiaTypeface() const;

 private:
  sk_sp<SkTypeface> typeface_;

  TypefaceSkia(const TypefaceSkia&) = delete;

  TypefaceSkia& operator=(const TypefaceSkia&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TYPOGRAPHER_BACKENDS_SKIA_TYPEFACE_SKIA_H_
