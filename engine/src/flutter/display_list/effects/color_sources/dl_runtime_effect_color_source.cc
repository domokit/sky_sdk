// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/color_sources/dl_runtime_effect_color_source.h"

namespace flutter {

DlRuntimeEffectColorSource::DlRuntimeEffectColorSource(
    sk_sp<DlRuntimeEffect> runtime_effect,
    std::vector<std::shared_ptr<DlColorSource>> samplers,
    std::shared_ptr<std::vector<uint8_t>> uniform_data)
    : runtime_effect_(std::move(runtime_effect)),
      samplers_(std::move(samplers)),
      uniform_data_(std::move(uniform_data)) {}

std::shared_ptr<DlColorSource> DlRuntimeEffectColorSource::shared() const {
  return std::make_shared<DlRuntimeEffectColorSource>(runtime_effect_,  //
                                                      samplers_,        //
                                                      uniform_data_);
}

bool DlRuntimeEffectColorSource::isUIThreadSafe() const {
  for (const auto& sampler : samplers_) {
    if (!sampler->isUIThreadSafe()) {
      return false;
    }
  }
  return true;
}

bool DlRuntimeEffectColorSource::equals_(DlColorSource const& other) const {
  FML_DCHECK(other.type() == DlColorSourceType::kRuntimeEffect);
  auto that = static_cast<DlRuntimeEffectColorSource const*>(&other);
  if (runtime_effect_ != that->runtime_effect_) {
    return false;
  }
  if (uniform_data_ != that->uniform_data_) {
    return false;
  }
  if (samplers_.size() != that->samplers_.size()) {
    return false;
  }
  for (size_t i = 0; i < samplers_.size(); i++) {
    if (samplers_[i] != that->samplers_[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace flutter
