// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_DL_RUNTIME_EFFECT_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_DL_RUNTIME_EFFECT_H_

#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/impeller/runtime_stage/runtime_stage.h"

#include "third_party/skia/include/core/SkRefCnt.h"
#include "third_party/skia/include/effects/SkRuntimeEffect.h"

namespace flutter {

class DlRuntimeEffect : public SkRefCnt {
 public:
  static sk_sp<DlRuntimeEffect> MakeSkia(
      const sk_sp<SkRuntimeEffect>& runtime_effect);

  static sk_sp<DlRuntimeEffect> MakeImpeller(
      std::shared_ptr<impeller::RuntimeStage> runtime_stage);

  virtual sk_sp<SkRuntimeEffect> skia_runtime_effect() const = 0;

  virtual std::shared_ptr<impeller::RuntimeStage> runtime_stage() const = 0;

 protected:
  DlRuntimeEffect();
  virtual ~DlRuntimeEffect();

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(DlRuntimeEffect);
};

class DlRuntimeEffectSkia final : public DlRuntimeEffect {
 public:
  explicit DlRuntimeEffectSkia(const sk_sp<SkRuntimeEffect>& runtime_effect);

  // |DlRuntimeEffect|
  sk_sp<SkRuntimeEffect> skia_runtime_effect() const override;

  // |DlRuntimeEffect|
  std::shared_ptr<impeller::RuntimeStage> runtime_stage() const override;

 private:
  DlRuntimeEffectSkia() = delete;
  // |DlRuntimeEffect|
  ~DlRuntimeEffectSkia() override;

  sk_sp<SkRuntimeEffect> skia_runtime_effect_;

  FML_DISALLOW_COPY_AND_ASSIGN(DlRuntimeEffectSkia);

  friend DlRuntimeEffect;
};

class DlRuntimeEffectImpeller final : public DlRuntimeEffect {
 public:
  explicit DlRuntimeEffectImpeller(
      std::shared_ptr<impeller::RuntimeStage> runtime_stage);

  // |DlRuntimeEffect|
  sk_sp<SkRuntimeEffect> skia_runtime_effect() const override;

  // |DlRuntimeEffect|
  std::shared_ptr<impeller::RuntimeStage> runtime_stage() const override;

 private:
  DlRuntimeEffectImpeller() = delete;
  // |DlRuntimeEffect|
  ~DlRuntimeEffectImpeller() override;

  std::shared_ptr<impeller::RuntimeStage> runtime_stage_;

  FML_DISALLOW_COPY_AND_ASSIGN(DlRuntimeEffectImpeller);

  friend DlRuntimeEffect;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_DL_RUNTIME_EFFECT_H_
