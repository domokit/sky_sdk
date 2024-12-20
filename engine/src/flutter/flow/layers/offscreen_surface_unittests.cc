// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/offscreen_surface.h"

#include <memory>

#include "gtest/gtest.h"

namespace flutter::testing {

TEST(OffscreenSurfaceTest, EmptySurfaceIsInvalid) {
  auto surface = std::make_unique<OffscreenSurface>(nullptr, DlISize());
  ASSERT_FALSE(surface->IsValid());
}

TEST(OffscreenSurfaceTest, OnexOneSurfaceIsValid) {
  auto surface = std::make_unique<OffscreenSurface>(nullptr, DlISize(1, 1));
  ASSERT_TRUE(surface->IsValid());
}

TEST(OffscreenSurfaceTest, PaintSurfaceBlack) {
  auto surface = std::make_unique<OffscreenSurface>(nullptr, DlISize(1, 1));

  DlCanvas* canvas = surface->GetCanvas();
  canvas->Clear(DlColor::kBlack());
  canvas->Flush();

  auto raster_data = surface->GetRasterData(false);
  const uint32_t* actual =
      reinterpret_cast<const uint32_t*>(raster_data->data());

  // picking black as the color since byte ordering seems to matter.
  ASSERT_EQ(actual[0], 0xFF000000u);
}

}  // namespace flutter::testing
