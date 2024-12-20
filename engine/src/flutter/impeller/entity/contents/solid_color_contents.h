// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_SOLID_COLOR_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_SOLID_COLOR_CONTENTS_H_

#include "impeller/entity/contents/color_source_contents.h"
#include "impeller/entity/contents/contents.h"
#include "impeller/geometry/color.h"

namespace impeller {

class SolidColorContents final : public ColorSourceContents {
 public:
  SolidColorContents();

  ~SolidColorContents() override;

  void SetColor(Color color);

  Color GetColor() const;

  // |ColorSourceContents|
  bool IsSolidColor() const override;

  // |Contents|
  bool IsOpaque(const Matrix& transform) const override;

  // |Contents|
  std::optional<Rect> GetCoverage(const Entity& entity) const override;

  // |Contents|
  bool Render(const ContentContext& renderer,
              const Entity& entity,
              RenderPass& pass) const override;

  std::optional<Color> AsBackgroundColor(const Entity& entity,
                                         ISize target_size) const override;

  // |Contents|
  [[nodiscard]] bool ApplyColorFilter(
      const ColorFilterProc& color_filter_proc) override;

 private:
  Color color_;

  SolidColorContents(const SolidColorContents&) = delete;

  SolidColorContents& operator=(const SolidColorContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_SOLID_COLOR_CONTENTS_H_
