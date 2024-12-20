// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/testing/dl_test_snippets.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_op_receiver.h"
#include "third_party/skia/include/core/SkFontMgr.h"
#include "third_party/skia/include/core/SkTypeface.h"
#include "txt/platform.h"

namespace flutter {
namespace testing {

sk_sp<DisplayList> GetSampleDisplayList() {
  DisplayListBuilder builder(SkRect::MakeWH(150, 100));
  builder.DrawRect(SkRect::MakeXYWH(10, 10, 80, 80), DlPaint(DlColor::kRed()));
  return builder.Build();
}

sk_sp<DisplayList> GetSampleNestedDisplayList() {
  DisplayListBuilder builder(SkRect::MakeWH(150, 100));
  DlPaint paint;
  for (int y = 10; y <= 60; y += 10) {
    for (int x = 10; x <= 60; x += 10) {
      paint.setColor(((x + y) % 20) == 10 ? DlColor(SK_ColorRED)
                                          : DlColor(SK_ColorBLUE));
      builder.DrawRect(SkRect::MakeXYWH(x, y, 80, 80), paint);
    }
  }
  DisplayListBuilder outer_builder(SkRect::MakeWH(150, 100));
  outer_builder.DrawDisplayList(builder.Build());
  return outer_builder.Build();
}

sk_sp<DisplayList> GetSampleDisplayList(int ops) {
  DisplayListBuilder builder(SkRect::MakeWH(150, 100));
  for (int i = 0; i < ops; i++) {
    builder.DrawColor(DlColor::kRed(), DlBlendMode::kSrc);
  }
  return builder.Build();
}

// ---------------
// Test Suite data
// ---------------

std::vector<DisplayListInvocationGroup> CreateAllAttributesOps() {
  return {
      {"SetAntiAlias",
       {
           {0, 8, 0, [](DlOpReceiver& r) { r.setAntiAlias(true); }},

           // Reset attribute to default as last entry
           {0, 0, 0, [](DlOpReceiver& r) { r.setAntiAlias(false); }},
       }},
      {"SetInvertColors",
       {
           {0, 8, 0, [](DlOpReceiver& r) { r.setInvertColors(true); }},

           // Reset attribute to default as last entry
           {0, 0, 0, [](DlOpReceiver& r) { r.setInvertColors(false); }},
       }},
      {"SetStrokeCap",
       {
           {0, 8, 0,
            [](DlOpReceiver& r) { r.setStrokeCap(DlStrokeCap::kRound); }},
           {0, 8, 0,
            [](DlOpReceiver& r) { r.setStrokeCap(DlStrokeCap::kSquare); }},

           // Reset attribute to default as last entry
           {0, 0, 0,
            [](DlOpReceiver& r) { r.setStrokeCap(DlStrokeCap::kButt); }},
       }},
      {"SetStrokeJoin",
       {
           {0, 8, 0,
            [](DlOpReceiver& r) { r.setStrokeJoin(DlStrokeJoin::kBevel); }},
           {0, 8, 0,
            [](DlOpReceiver& r) { r.setStrokeJoin(DlStrokeJoin::kRound); }},

           // Reset attribute to default as last entry
           {0, 0, 0,
            [](DlOpReceiver& r) { r.setStrokeJoin(DlStrokeJoin::kMiter); }},
       }},
      {"SetStyle",
       {
           {0, 8, 0,
            [](DlOpReceiver& r) { r.setDrawStyle(DlDrawStyle::kStroke); }},
           {0, 8, 0,
            [](DlOpReceiver& r) {
              r.setDrawStyle(DlDrawStyle::kStrokeAndFill);
            }},

           // Reset attribute to default as last entry
           {0, 0, 0,
            [](DlOpReceiver& r) { r.setDrawStyle(DlDrawStyle::kFill); }},
       }},
      {"SetStrokeWidth",
       {
           {0, 8, 0, [](DlOpReceiver& r) { r.setStrokeWidth(1.0); }},
           {0, 8, 0, [](DlOpReceiver& r) { r.setStrokeWidth(5.0); }},

           // Reset attribute to default as last entry
           {0, 0, 0, [](DlOpReceiver& r) { r.setStrokeWidth(0.0); }},
       }},
      {"SetStrokeMiter",
       {
           {0, 8, 0, [](DlOpReceiver& r) { r.setStrokeMiter(0.0); }},
           {0, 8, 0, [](DlOpReceiver& r) { r.setStrokeMiter(5.0); }},

           // Reset attribute to default as last entry
           {0, 0, 0, [](DlOpReceiver& r) { r.setStrokeMiter(4.0); }},
       }},
      {"SetColor",
       {
           {0, 4 + sizeof(DlColor), 0,
            [](DlOpReceiver& r) { r.setColor(DlColor(SK_ColorGREEN)); }},
           {0, 4 + sizeof(DlColor), 0,
            [](DlOpReceiver& r) { r.setColor(DlColor(SK_ColorBLUE)); }},

           // Reset attribute to default as last entry
           {0, 0, 0,
            [](DlOpReceiver& r) { r.setColor(DlColor(SK_ColorBLACK)); }},
       }},
      {"SetBlendMode",
       {
           {0, 8, 0,
            [](DlOpReceiver& r) { r.setBlendMode(DlBlendMode::kSrcIn); }},
           {0, 8, 0,
            [](DlOpReceiver& r) { r.setBlendMode(DlBlendMode::kDstIn); }},

           // Reset attribute to default as last entry
           {0, 0, 0,
            [](DlOpReceiver& r) { r.setBlendMode(DlBlendMode::kSrcOver); }},
       }},
      {"SetColorSource",
       {
           {0, 104, 0,
            [](DlOpReceiver& r) { r.setColorSource(kTestSource1.get()); }},
           {0, 176, 0,
            [](DlOpReceiver& r) { r.setColorSource(kTestSource2.get()); }},
           {0, 176, 0,
            [](DlOpReceiver& r) { r.setColorSource(kTestSource3.get()); }},
           {0, 184, 0,
            [](DlOpReceiver& r) { r.setColorSource(kTestSource4.get()); }},
           {0, 176, 0,
            [](DlOpReceiver& r) { r.setColorSource(kTestSource5.get()); }},

           // Reset attribute to default as last entry
           {0, 0, 0, [](DlOpReceiver& r) { r.setColorSource(nullptr); }},
       }},
      {"SetImageFilter",
       {
           {0, 32, 0,
            [](DlOpReceiver& r) { r.setImageFilter(&kTestBlurImageFilter1); }},
           {0, 32, 0,
            [](DlOpReceiver& r) { r.setImageFilter(&kTestBlurImageFilter2); }},
           {0, 32, 0,
            [](DlOpReceiver& r) { r.setImageFilter(&kTestBlurImageFilter3); }},
           {0, 32, 0,
            [](DlOpReceiver& r) { r.setImageFilter(&kTestBlurImageFilter4); }},
           {0, 24, 0,
            [](DlOpReceiver& r) {
              r.setImageFilter(&kTestDilateImageFilter1);
            }},
           {0, 24, 0,
            [](DlOpReceiver& r) {
              r.setImageFilter(&kTestDilateImageFilter2);
            }},
           {0, 24, 0,
            [](DlOpReceiver& r) {
              r.setImageFilter(&kTestDilateImageFilter3);
            }},
           {0, 24, 0,
            [](DlOpReceiver& r) { r.setImageFilter(&kTestErodeImageFilter1); }},
           {0, 24, 0,
            [](DlOpReceiver& r) { r.setImageFilter(&kTestErodeImageFilter2); }},
           {0, 24, 0,
            [](DlOpReceiver& r) { r.setImageFilter(&kTestErodeImageFilter3); }},
           {0, 88, 0,
            [](DlOpReceiver& r) {
              r.setImageFilter(&kTestMatrixImageFilter1);
            }},
           {0, 88, 0,
            [](DlOpReceiver& r) {
              r.setImageFilter(&kTestMatrixImageFilter2);
            }},
           {0, 88, 0,
            [](DlOpReceiver& r) {
              r.setImageFilter(&kTestMatrixImageFilter3);
            }},
           {0, 24, 0,
            [](DlOpReceiver& r) {
              r.setImageFilter(&kTestComposeImageFilter1);
            }},
           {0, 24, 0,
            [](DlOpReceiver& r) {
              r.setImageFilter(&kTestComposeImageFilter2);
            }},
           {0, 24, 0,
            [](DlOpReceiver& r) {
              r.setImageFilter(&kTestComposeImageFilter3);
            }},
           {0, 24, 0,
            [](DlOpReceiver& r) { r.setImageFilter(&kTestCFImageFilter1); }},
           {0, 24, 0,
            [](DlOpReceiver& r) { r.setImageFilter(&kTestCFImageFilter2); }},
           {0, 24, 0,
            [](DlOpReceiver& r) {
              r.setImageFilter(
                  kTestBlurImageFilter1
                      .makeWithLocalMatrix(DlMatrix::MakeTranslation({2, 2}))
                      .get());
            }},

           // Reset attribute to default as last entry
           {0, 0, 0, [](DlOpReceiver& r) { r.setImageFilter(nullptr); }},
       }},
      {"SetColorFilter",
       {
           {0, 40, 0,
            [](DlOpReceiver& r) {
              r.setColorFilter(kTestBlendColorFilter1.get());
            }},
           {0, 40, 0,
            [](DlOpReceiver& r) {
              r.setColorFilter(kTestBlendColorFilter2.get());
            }},
           {0, 40, 0,
            [](DlOpReceiver& r) {
              r.setColorFilter(kTestBlendColorFilter3.get());
            }},
           {0, 96, 0,
            [](DlOpReceiver& r) {
              r.setColorFilter(kTestMatrixColorFilter1.get());
            }},
           {0, 96, 0,
            [](DlOpReceiver& r) {
              r.setColorFilter(kTestMatrixColorFilter2.get());
            }},
           {0, 16, 0,
            [](DlOpReceiver& r) {
              r.setColorFilter(DlColorFilter::MakeSrgbToLinearGamma().get());
            }},
           {0, 16, 0,
            [](DlOpReceiver& r) {
              r.setColorFilter(DlColorFilter::MakeLinearToSrgbGamma().get());
            }},

           // Reset attribute to default as last entry
           {0, 0, 0, [](DlOpReceiver& r) { r.setColorFilter(nullptr); }},
       }},
      {"SetMaskFilter",
       {
           {0, 32, 0,
            [](DlOpReceiver& r) { r.setMaskFilter(&kTestMaskFilter1); }, 0u,
            2u},
           {0, 32, 0,
            [](DlOpReceiver& r) { r.setMaskFilter(&kTestMaskFilter2); }, 0u,
            2u},
           {0, 32, 0,
            [](DlOpReceiver& r) { r.setMaskFilter(&kTestMaskFilter3); }, 0u,
            2u},
           {0, 32, 0,
            [](DlOpReceiver& r) { r.setMaskFilter(&kTestMaskFilter4); }, 0u,
            2u},
           {0, 32, 0,
            [](DlOpReceiver& r) { r.setMaskFilter(&kTestMaskFilter5); }, 0u,
            2u},

           // Reset attribute to default as last entry
           {0, 0, 0, [](DlOpReceiver& r) { r.setMaskFilter(nullptr); }, 0u, 1u},
       }},
  };
}

std::vector<DisplayListInvocationGroup> CreateAllSaveRestoreOps() {
  return {
      {"Save(Layer)+Restore",
       {
           // There are many reasons that save and restore can elide content,
           // including whether or not there are any draw operations between
           // them, whether or not there are any state changes to restore, and
           // whether group rendering (opacity) optimizations can allow
           // attributes to be distributed to the children. To prevent those
           // cases we include at least one clip operation and 2 overlapping
           // rendering primitives between each save/restore pair.
           {5, 96, 2,
            [](DlOpReceiver& r) {
              r.save();
              r.clipRect(DlRect::MakeLTRB(0, 0, 25, 25),
                         DlCanvas::ClipOp::kIntersect, true);
              r.drawRect(DlRect::MakeLTRB(5, 5, 15, 15));
              r.drawRect(DlRect::MakeLTRB(10, 10, 20, 20));
              r.restore();
            }},
           {5, 120, 3,
            [](DlOpReceiver& r) {
              r.saveLayer(nullptr, SaveLayerOptions::kNoAttributes);
              r.clipRect(DlRect::MakeLTRB(0, 0, 25, 25),
                         DlCanvas::ClipOp::kIntersect, true);
              r.drawRect(DlRect::MakeLTRB(5, 5, 15, 15));
              r.drawRect(DlRect::MakeLTRB(10, 10, 20, 20));
              r.restore();
            }},
           {5, 120, 3,
            [](DlOpReceiver& r) {
              r.saveLayer(nullptr, SaveLayerOptions::kWithAttributes);
              r.clipRect(DlRect::MakeLTRB(0, 0, 25, 25),
                         DlCanvas::ClipOp::kIntersect, true);
              r.drawRect(DlRect::MakeLTRB(5, 5, 15, 15));
              r.drawRect(DlRect::MakeLTRB(10, 10, 20, 20));
              r.restore();
            }},
           // For saveLayer calls with bounds, we need at least one unclipped
           // draw command so that the bounds are not reduced in size to the
           // clip dimensions on the re-dispatch.
           {5, 120, 3,
            [](DlOpReceiver& r) {
              r.saveLayer(&kTestBounds, SaveLayerOptions::kNoAttributes);
              r.drawRect(kTestBounds);
              r.clipRect(DlRect::MakeLTRB(0, 0, 25, 25),
                         DlCanvas::ClipOp::kIntersect, true);
              r.drawRect(DlRect::MakeLTRB(10, 10, 20, 20));
              r.restore();
            }},
           {5, 120, 3,
            [](DlOpReceiver& r) {
              r.saveLayer(&kTestBounds, SaveLayerOptions::kWithAttributes);
              r.drawRect(kTestBounds);
              r.clipRect(DlRect::MakeLTRB(0, 0, 25, 25),
                         DlCanvas::ClipOp::kIntersect, true);
              r.drawRect(DlRect::MakeLTRB(10, 10, 20, 20));
              r.restore();
            }},
           {5, 152, 3,
            [](DlOpReceiver& r) {
              r.saveLayer(nullptr, SaveLayerOptions::kNoAttributes,
                          &kTestCFImageFilter1);
              r.clipRect(DlRect::MakeLTRB(0, 0, 25, 25),
                         DlCanvas::ClipOp::kIntersect, true);
              r.drawRect(DlRect::MakeLTRB(5, 5, 15, 15));
              r.drawRect(DlRect::MakeLTRB(10, 10, 20, 20));
              r.restore();
            }},
           {5, 152, 3,
            [](DlOpReceiver& r) {
              r.saveLayer(nullptr, SaveLayerOptions::kWithAttributes,
                          &kTestCFImageFilter1);
              r.clipRect(DlRect::MakeLTRB(0, 0, 25, 25),
                         DlCanvas::ClipOp::kIntersect, true);
              r.drawRect(DlRect::MakeLTRB(5, 5, 15, 15));
              r.drawRect(DlRect::MakeLTRB(10, 10, 20, 20));
              r.restore();
            }},
           {5, 152, 3,
            [](DlOpReceiver& r) {
              r.saveLayer(&kTestBounds, SaveLayerOptions::kNoAttributes,
                          &kTestCFImageFilter1);
              r.drawRect(kTestBounds);
              r.clipRect(DlRect::MakeLTRB(0, 0, 25, 25),
                         DlCanvas::ClipOp::kIntersect, true);
              r.drawRect(DlRect::MakeLTRB(10, 10, 20, 20));
              r.restore();
            }},
           {5, 152, 3,
            [](DlOpReceiver& r) {
              r.saveLayer(&kTestBounds, SaveLayerOptions::kWithAttributes,
                          &kTestCFImageFilter1);
              r.drawRect(kTestBounds);
              r.clipRect(DlRect::MakeLTRB(0, 0, 25, 25),
                         DlCanvas::ClipOp::kIntersect, true);
              r.drawRect(DlRect::MakeLTRB(10, 10, 20, 20));
              r.restore();
            }},
       }},
  };
}

std::vector<DisplayListInvocationGroup> CreateAllTransformOps() {
  return {
      {"Translate",
       {
           {1, 16, 0, [](DlOpReceiver& r) { r.translate(10, 10); }},
           {1, 16, 0, [](DlOpReceiver& r) { r.translate(10, 15); }},
           {1, 16, 0, [](DlOpReceiver& r) { r.translate(15, 10); }},
           {0, 0, 0, [](DlOpReceiver& r) { r.translate(0, 0); }},
       }},
      {"Scale",
       {
           {1, 16, 0, [](DlOpReceiver& r) { r.scale(2, 2); }},
           {1, 16, 0, [](DlOpReceiver& r) { r.scale(2, 3); }},
           {1, 16, 0, [](DlOpReceiver& r) { r.scale(3, 2); }},
           {0, 0, 0, [](DlOpReceiver& r) { r.scale(1, 1); }},
       }},
      {"Rotate",
       {
           {1, 8, 0, [](DlOpReceiver& r) { r.rotate(30); }},
           {1, 8, 0, [](DlOpReceiver& r) { r.rotate(45); }},
           {0, 0, 0, [](DlOpReceiver& r) { r.rotate(0); }},
           {0, 0, 0, [](DlOpReceiver& r) { r.rotate(360); }},
       }},
      {"Skew",
       {
           {1, 16, 0, [](DlOpReceiver& r) { r.skew(0.1, 0.1); }},
           {1, 16, 0, [](DlOpReceiver& r) { r.skew(0.1, 0.2); }},
           {1, 16, 0, [](DlOpReceiver& r) { r.skew(0.2, 0.1); }},
           {0, 0, 0, [](DlOpReceiver& r) { r.skew(0, 0); }},
       }},
      {"Transform2DAffine",
       {
           {1, 32, 0,
            [](DlOpReceiver& r) { r.transform2DAffine(0, 1, 12, 1, 0, 33); }},
           // r.transform(identity) is ignored
           {0, 0, 0,
            [](DlOpReceiver& r) { r.transform2DAffine(1, 0, 0, 0, 1, 0); }},
       }},
      {"TransformFullPerspective",
       {
           {1, 72, 0,
            [](DlOpReceiver& r) {
              r.transformFullPerspective(0, 1, 0, 12, 1, 0, 0, 33, 3, 2, 5, 29,
                                         0, 0, 0, 12);
            }},
           // r.transform(2D affine) is reduced to 2x3
           {1, 32, 0,
            [](DlOpReceiver& r) {
              r.transformFullPerspective(2, 1, 0, 4, 1, 3, 0, 5, 0, 0, 1, 0, 0,
                                         0, 0, 1);
            }},
           // r.transform(identity) is ignored
           {0, 0, 0,
            [](DlOpReceiver& r) {
              r.transformFullPerspective(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0,
                                         0, 0, 1);
            }},
       }},
  };
}

std::vector<DisplayListInvocationGroup> CreateAllClipOps() {
  return {
      {"ClipRect",
       {
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipRect(kTestBounds, DlCanvas::ClipOp::kIntersect, true);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipRect(kTestBounds.Shift(1, 1), DlCanvas::ClipOp::kIntersect,
                         true);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipRect(kTestBounds, DlCanvas::ClipOp::kIntersect, false);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipRect(kTestBounds, DlCanvas::ClipOp::kDifference, true);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipRect(kTestBounds, DlCanvas::ClipOp::kDifference, false);
            }},
       }},
      {"ClipOval",
       {
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipOval(kTestBounds, DlCanvas::ClipOp::kIntersect, true);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipOval(kTestBounds.Shift(1, 1), DlCanvas::ClipOp::kIntersect,
                         true);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipOval(kTestBounds, DlCanvas::ClipOp::kIntersect, false);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipOval(kTestBounds, DlCanvas::ClipOp::kDifference, true);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipOval(kTestBounds, DlCanvas::ClipOp::kDifference, false);
            }},
       }},
      {"ClipRRect",
       {
           {1, 56, 0,
            [](DlOpReceiver& r) {
              r.clipRoundRect(kTestRRect, DlCanvas::ClipOp::kIntersect, true);
            }},
           {1, 56, 0,
            [](DlOpReceiver& r) {
              r.clipRoundRect(kTestRRect.Shift(1, 1),
                              DlCanvas::ClipOp::kIntersect, true);
            }},
           {1, 56, 0,
            [](DlOpReceiver& r) {
              r.clipRoundRect(kTestRRect, DlCanvas::ClipOp::kIntersect, false);
            }},
           {1, 56, 0,
            [](DlOpReceiver& r) {
              r.clipRoundRect(kTestRRect, DlCanvas::ClipOp::kDifference, true);
            }},
           {1, 56, 0,
            [](DlOpReceiver& r) {
              r.clipRoundRect(kTestRRect, DlCanvas::ClipOp::kDifference, false);
            }},
       }},
      {"ClipPath",
       {
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipPath(kTestPath1, DlCanvas::ClipOp::kIntersect, true);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipPath(kTestPath2, DlCanvas::ClipOp::kIntersect, true);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipPath(kTestPath3, DlCanvas::ClipOp::kIntersect, true);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipPath(kTestPath1, DlCanvas::ClipOp::kIntersect, false);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipPath(kTestPath1, DlCanvas::ClipOp::kDifference, true);
            }},
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipPath(kTestPath1, DlCanvas::ClipOp::kDifference, false);
            }},
           // clipPath(rect) becomes clipRect
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipPath(kTestPathRect, DlCanvas::ClipOp::kIntersect, true);
            }},
           // clipPath(oval) becomes clipOval
           {1, 24, 0,
            [](DlOpReceiver& r) {
              r.clipPath(kTestPathOval, DlCanvas::ClipOp::kIntersect, true);
            }},
           // clipPath(rrect) becomes clipRRect
           {1, 56, 0,
            [](DlOpReceiver& r) {
              r.clipPath(kTestPathRRect, DlCanvas::ClipOp::kIntersect, true);
            }},
       }},
  };
}

std::vector<DisplayListInvocationGroup> CreateAllRenderingOps() {
  return {
      {"DrawPaint",
       {
           {1, 8, 1, [](DlOpReceiver& r) { r.drawPaint(); }},
       }},
      {"DrawColor",
       {
           {1, 32, 1,
            [](DlOpReceiver& r) {
              r.drawColor(DlColor(SK_ColorBLUE), DlBlendMode::kSrcIn);
            }},
           {1, 32, 1,
            [](DlOpReceiver& r) {
              r.drawColor(DlColor(SK_ColorBLUE), DlBlendMode::kDstOut);
            }},
           {1, 32, 1,
            [](DlOpReceiver& r) {
              r.drawColor(DlColor(SK_ColorCYAN), DlBlendMode::kSrcIn);
            }},
       }},
      {"DrawLine",
       {
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawLine({0, 0}, {10, 10});
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawLine({1, 0}, {10, 10});
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawLine({0, 1}, {10, 10});
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawLine({0, 0}, {20, 10});
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawLine({0, 0}, {10, 20});
            }},
       }},
      {"DrawDashedLine",
       {
           {1, 32, 1,
            [](DlOpReceiver& r) {
              r.drawDashedLine({0, 0}, {10, 10}, 4.0f, 2.0f);
            }},
           {1, 32, 1,
            [](DlOpReceiver& r) {
              r.drawDashedLine({1, 0}, {10, 10}, 4.0f, 2.0f);
            }},
           {1, 32, 1,
            [](DlOpReceiver& r) {
              r.drawDashedLine({0, 1}, {10, 10}, 4.0f, 2.0f);
            }},
           {1, 32, 1,
            [](DlOpReceiver& r) {
              r.drawDashedLine({0, 0}, {20, 10}, 4.0f, 2.0f);
            }},
           {1, 32, 1,
            [](DlOpReceiver& r) {
              r.drawDashedLine({0, 0}, {10, 20}, 4.0f, 2.0f);
            }},
           {1, 32, 1,
            [](DlOpReceiver& r) {
              r.drawDashedLine({0, 0}, {10, 10}, 5.0f, 2.0f);
            }},
           {1, 32, 1,
            [](DlOpReceiver& r) {
              r.drawDashedLine({0, 0}, {10, 10}, 4.0f, 3.0f);
            }},
       }},
      {"DrawRect",
       {
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawRect(DlRect::MakeLTRB(0, 0, 10, 10));
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawRect(DlRect::MakeLTRB(0, 1, 10, 10));
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawRect(DlRect::MakeLTRB(0, 0, 20, 10));
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawRect(DlRect::MakeLTRB(0, 0, 10, 20));
            }},
       }},
      {"DrawOval",
       {
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawOval(DlRect::MakeLTRB(0, 0, 10, 10));
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawOval(DlRect::MakeLTRB(0, 1, 10, 10));
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawOval(DlRect::MakeLTRB(0, 0, 20, 10));
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawOval(DlRect::MakeLTRB(0, 0, 10, 20));
            }},
       }},
      {"DrawCircle",
       {
           {1, 16, 1,
            [](DlOpReceiver& r) {
              r.drawCircle({0, 0}, 10);
            }},
           {1, 16, 1,
            [](DlOpReceiver& r) {
              r.drawCircle({0, 5}, 10);
            }},
           {1, 16, 1,
            [](DlOpReceiver& r) {
              r.drawCircle({0, 0}, 20);
            }},
       }},
      {"DrawRRect",
       {
           {1, 56, 1, [](DlOpReceiver& r) { r.drawRoundRect(kTestRRect); }},
           {1, 56, 1,
            [](DlOpReceiver& r) { r.drawRoundRect(kTestRRect.Shift(5, 5)); }},
       }},
      {"DrawDRRect",
       {
           {1, 104, 1,
            [](DlOpReceiver& r) {
              r.drawDiffRoundRect(kTestRRect, kTestInnerRRect);
            }},
           {1, 104, 1,
            [](DlOpReceiver& r) {
              r.drawDiffRoundRect(kTestRRect.Shift(5, 5),
                                  kTestInnerRRect.Shift(4, 4));
            }},
       }},
      {"DrawPath",
       {
           {1, 24, 1, [](DlOpReceiver& r) { r.drawPath(kTestPath1); }},
           {1, 24, 1, [](DlOpReceiver& r) { r.drawPath(kTestPath2); }},
           {1, 24, 1, [](DlOpReceiver& r) { r.drawPath(kTestPath3); }},
           // oval, rect and rrect paths are left as drawPath
           {1, 24, 1, [](DlOpReceiver& r) { r.drawPath(kTestPathRect); }},
           {1, 24, 1, [](DlOpReceiver& r) { r.drawPath(kTestPathOval); }},
           {1, 24, 1, [](DlOpReceiver& r) { r.drawPath(kTestPathRRect); }},
       }},
      {"DrawArc",
       {
           {1, 32, 1,
            [](DlOpReceiver& r) { r.drawArc(kTestBounds, 45, 270, false); }},
           {1, 32, 1,
            [](DlOpReceiver& r) {
              r.drawArc(kTestBounds.Shift(1, 1), 45, 270, false);
            }},
           {1, 32, 1,
            [](DlOpReceiver& r) { r.drawArc(kTestBounds, 30, 270, false); }},
           {1, 32, 1,
            [](DlOpReceiver& r) { r.drawArc(kTestBounds, 45, 260, false); }},
           {1, 32, 1,
            [](DlOpReceiver& r) { r.drawArc(kTestBounds, 45, 270, true); }},
       }},
      {"DrawPoints",
       {
           {1, 8 + TestPointCount * 8, 1,
            [](DlOpReceiver& r) {
              r.drawPoints(DlCanvas::PointMode::kPoints, TestPointCount,
                           ToDlPoints(kTestPoints));
            }},
           {1, 8 + (TestPointCount - 1) * 8, 1,
            [](DlOpReceiver& r) {
              r.drawPoints(DlCanvas::PointMode::kPoints, TestPointCount - 1,
                           ToDlPoints(kTestPoints));
            }},
           {1, 8 + TestPointCount * 8, 1,
            [](DlOpReceiver& r) {
              r.drawPoints(DlCanvas::PointMode::kLines, TestPointCount,
                           ToDlPoints(kTestPoints));
            }},
           {1, 8 + TestPointCount * 8, 1,
            [](DlOpReceiver& r) {
              r.drawPoints(DlCanvas::PointMode::kPolygon, TestPointCount,
                           ToDlPoints(kTestPoints));
            }},
       }},
      {"DrawVertices",
       {
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawVertices(kTestVertices1, DlBlendMode::kSrcIn);
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawVertices(kTestVertices1, DlBlendMode::kDstIn);
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawVertices(kTestVertices2, DlBlendMode::kSrcIn);
            }},
       }},
      {"DrawImage",
       {
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawImage(TestImage1, {10, 10}, kNearestSampling, false);
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawImage(TestImage1, {10, 10}, kNearestSampling, true);
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawImage(TestImage1, {20, 10}, kNearestSampling, false);
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawImage(TestImage1, {10, 20}, kNearestSampling, false);
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawImage(TestImage1, {10, 10}, kLinearSampling, false);
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawImage(TestImage2, {10, 10}, kNearestSampling, false);
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              auto dl_image = DlImage::Make(TestSkImage);
              r.drawImage(dl_image, {10, 10}, kNearestSampling, false);
            }},
       }},
      {"DrawImageRect",
       {
           {1, 56, 1,
            [](DlOpReceiver& r) {
              r.drawImageRect(TestImage1, DlRect::MakeLTRB(10, 10, 20, 20),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              kNearestSampling, false,
                              DlCanvas::SrcRectConstraint::kFast);
            }},
           {1, 56, 1,
            [](DlOpReceiver& r) {
              r.drawImageRect(TestImage1, DlRect::MakeLTRB(10, 10, 20, 20),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              kNearestSampling, true,
                              DlCanvas::SrcRectConstraint::kFast);
            }},
           {1, 56, 1,
            [](DlOpReceiver& r) {
              r.drawImageRect(TestImage1, DlRect::MakeLTRB(10, 10, 20, 20),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              kNearestSampling, false,
                              DlCanvas::SrcRectConstraint::kStrict);
            }},
           {1, 56, 1,
            [](DlOpReceiver& r) {
              r.drawImageRect(TestImage1, DlRect::MakeLTRB(10, 10, 25, 20),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              kNearestSampling, false,
                              DlCanvas::SrcRectConstraint::kFast);
            }},
           {1, 56, 1,
            [](DlOpReceiver& r) {
              r.drawImageRect(TestImage1, DlRect::MakeLTRB(10, 10, 20, 20),
                              DlRect::MakeLTRB(10, 10, 85, 80),
                              kNearestSampling, false,
                              DlCanvas::SrcRectConstraint::kFast);
            }},
           {1, 56, 1,
            [](DlOpReceiver& r) {
              r.drawImageRect(TestImage1, DlRect::MakeLTRB(10, 10, 20, 20),
                              DlRect::MakeLTRB(10, 10, 80, 80), kLinearSampling,
                              false, DlCanvas::SrcRectConstraint::kFast);
            }},
           {1, 56, 1,
            [](DlOpReceiver& r) {
              r.drawImageRect(TestImage2, DlRect::MakeLTRB(10, 10, 15, 15),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              kNearestSampling, false,
                              DlCanvas::SrcRectConstraint::kFast);
            }},
           {1, 56, 1,
            [](DlOpReceiver& r) {
              auto dl_image = DlImage::Make(TestSkImage);
              r.drawImageRect(dl_image, DlRect::MakeLTRB(10, 10, 15, 15),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              kNearestSampling, false,
                              DlCanvas::SrcRectConstraint::kFast);
            }},
       }},
      {"DrawImageNine",
       {
           {1, 48, 9,
            [](DlOpReceiver& r) {
              r.drawImageNine(TestImage1, DlIRect::MakeLTRB(10, 10, 20, 20),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              DlFilterMode::kNearest, false);
            }},
           {1, 48, 9,
            [](DlOpReceiver& r) {
              r.drawImageNine(TestImage1, DlIRect::MakeLTRB(10, 10, 20, 20),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              DlFilterMode::kNearest, true);
            }},
           {1, 48, 9,
            [](DlOpReceiver& r) {
              r.drawImageNine(TestImage1, DlIRect::MakeLTRB(10, 10, 25, 20),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              DlFilterMode::kNearest, false);
            }},
           {1, 48, 9,
            [](DlOpReceiver& r) {
              r.drawImageNine(TestImage1, DlIRect::MakeLTRB(10, 10, 20, 20),
                              DlRect::MakeLTRB(10, 10, 85, 80),
                              DlFilterMode::kNearest, false);
            }},
           {1, 48, 9,
            [](DlOpReceiver& r) {
              r.drawImageNine(TestImage1, DlIRect::MakeLTRB(10, 10, 20, 20),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              DlFilterMode::kLinear, false);
            }},
           {1, 48, 9,
            [](DlOpReceiver& r) {
              r.drawImageNine(TestImage2, DlIRect::MakeLTRB(10, 10, 15, 15),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              DlFilterMode::kNearest, false);
            }},
           {1, 48, 9,
            [](DlOpReceiver& r) {
              auto dl_image = DlImage::Make(TestSkImage);
              r.drawImageNine(dl_image, DlIRect::MakeLTRB(10, 10, 15, 15),
                              DlRect::MakeLTRB(10, 10, 80, 80),
                              DlFilterMode::kNearest, false);
            }},
       }},
      {"DrawAtlas",
       {
           {1, 48 + 32 + 8, 1,
            [](DlOpReceiver& r) {
              static SkRSXform xforms[] = {{1, 0, 0, 0}, {0, 1, 0, 0}};
              static DlRect texs[] = {DlRect::MakeLTRB(10, 10, 20, 20),
                                      DlRect::MakeLTRB(20, 20, 30, 30)};
              r.drawAtlas(TestImage1, xforms, texs, nullptr, 2,
                          DlBlendMode::kSrcIn, kNearestSampling, nullptr,
                          false);
            }},
           {1, 48 + 32 + 8, 1,
            [](DlOpReceiver& r) {
              static SkRSXform xforms[] = {{1, 0, 0, 0}, {0, 1, 0, 0}};
              static DlRect texs[] = {DlRect::MakeLTRB(10, 10, 20, 20),
                                      DlRect::MakeLTRB(20, 20, 30, 30)};
              r.drawAtlas(TestImage1, xforms, texs, nullptr, 2,
                          DlBlendMode::kSrcIn, kNearestSampling, nullptr, true);
            }},
           {1, 48 + 32 + 8, 1,
            [](DlOpReceiver& r) {
              static SkRSXform xforms[] = {{0, 1, 0, 0}, {0, 1, 0, 0}};
              static DlRect texs[] = {DlRect::MakeLTRB(10, 10, 20, 20),
                                      DlRect::MakeLTRB(20, 20, 30, 30)};
              r.drawAtlas(TestImage1, xforms, texs, nullptr, 2,
                          DlBlendMode::kSrcIn, kNearestSampling, nullptr,
                          false);
            }},
           {1, 48 + 32 + 8, 1,
            [](DlOpReceiver& r) {
              static SkRSXform xforms[] = {{1, 0, 0, 0}, {0, 1, 0, 0}};
              static DlRect texs[] = {DlRect::MakeLTRB(10, 10, 20, 20),
                                      DlRect::MakeLTRB(20, 25, 30, 30)};
              r.drawAtlas(TestImage1, xforms, texs, nullptr, 2,
                          DlBlendMode::kSrcIn, kNearestSampling, nullptr,
                          false);
            }},
           {1, 48 + 32 + 8, 1,
            [](DlOpReceiver& r) {
              static SkRSXform xforms[] = {{1, 0, 0, 0}, {0, 1, 0, 0}};
              static DlRect texs[] = {DlRect::MakeLTRB(10, 10, 20, 20),
                                      DlRect::MakeLTRB(20, 20, 30, 30)};
              r.drawAtlas(TestImage1, xforms, texs, nullptr, 2,
                          DlBlendMode::kSrcIn, kLinearSampling, nullptr, false);
            }},
           {1, 48 + 32 + 8, 1,
            [](DlOpReceiver& r) {
              static SkRSXform xforms[] = {{1, 0, 0, 0}, {0, 1, 0, 0}};
              static DlRect texs[] = {DlRect::MakeLTRB(10, 10, 20, 20),
                                      DlRect::MakeLTRB(20, 20, 30, 30)};
              r.drawAtlas(TestImage1, xforms, texs, nullptr, 2,
                          DlBlendMode::kDstIn, kNearestSampling, nullptr,
                          false);
            }},
           {1, 64 + 32 + 8, 1,
            [](DlOpReceiver& r) {
              static SkRSXform xforms[] = {{1, 0, 0, 0}, {0, 1, 0, 0}};
              static DlRect texs[] = {DlRect::MakeLTRB(10, 10, 20, 20),
                                      DlRect::MakeLTRB(20, 20, 30, 30)};
              static DlRect cull_rect = DlRect::MakeLTRB(0, 0, 200, 200);
              r.drawAtlas(TestImage2, xforms, texs, nullptr, 2,
                          DlBlendMode::kSrcIn, kNearestSampling, &cull_rect,
                          false);
            }},
           {1, 128, 1,
            [](DlOpReceiver& r) {
              static SkRSXform xforms[] = {{1, 0, 0, 0}, {0, 1, 0, 0}};
              static DlRect texs[] = {DlRect::MakeLTRB(10, 10, 20, 20),
                                      DlRect::MakeLTRB(20, 20, 30, 30)};
              static DlColor colors[] = {DlColor::kBlue(), DlColor::kGreen()};
              r.drawAtlas(TestImage1, xforms, texs, colors, 2,
                          DlBlendMode::kSrcIn, kNearestSampling, nullptr,
                          false);
            }},
           {1, 144, 1,
            [](DlOpReceiver& r) {
              static SkRSXform xforms[] = {{1, 0, 0, 0}, {0, 1, 0, 0}};
              static DlRect texs[] = {DlRect::MakeLTRB(10, 10, 20, 20),
                                      DlRect::MakeLTRB(20, 20, 30, 30)};
              static DlColor colors[] = {DlColor::kBlue(), DlColor::kGreen()};
              static DlRect cull_rect = DlRect::MakeLTRB(0, 0, 200, 200);
              r.drawAtlas(TestImage1, xforms, texs, colors, 2,
                          DlBlendMode::kSrcIn, kNearestSampling, &cull_rect,
                          false);
            }},
           {1, 48 + 32 + 8, 1,
            [](DlOpReceiver& r) {
              auto dl_image = DlImage::Make(TestSkImage);
              static SkRSXform xforms[] = {{1, 0, 0, 0}, {0, 1, 0, 0}};
              static DlRect texs[] = {DlRect::MakeLTRB(10, 10, 20, 20),
                                      DlRect::MakeLTRB(20, 20, 30, 30)};
              r.drawAtlas(dl_image, xforms, texs, nullptr, 2,
                          DlBlendMode::kSrcIn, kNearestSampling, nullptr,
                          false);
            }},
       }},
      {"DrawDisplayList",
       {
           {1, 16, 1,
            [](DlOpReceiver& r) { r.drawDisplayList(TestDisplayList1, 1.0); },
            1u},
           {1, 16, 1,
            [](DlOpReceiver& r) { r.drawDisplayList(TestDisplayList1, 0.5); },
            1u},
           {1, 16, 1,
            [](DlOpReceiver& r) { r.drawDisplayList(TestDisplayList2, 1.0); },
            1u},
           {1, 16, 1,
            [](DlOpReceiver& r) {
              r.drawDisplayList(MakeTestDisplayList(10, 10, SK_ColorRED), 1.0);
            },
            1u},
       }},
      {"DrawTextBlob",
       {
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawTextBlob(GetTestTextBlob(1), 10, 10);
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawTextBlob(GetTestTextBlob(1), 20, 10);
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawTextBlob(GetTestTextBlob(1), 10, 20);
            }},
           {1, 24, 1,
            [](DlOpReceiver& r) {
              r.drawTextBlob(GetTestTextBlob(2), 10, 10);
            }},
       }},
      {"DrawShadow",
       {
           {1, 48, 1,
            [](DlOpReceiver& r) {
              r.drawShadow(kTestPath1, DlColor(SK_ColorGREEN), 1.0, false, 1.0);
            }},
           {1, 48, 1,
            [](DlOpReceiver& r) {
              r.drawShadow(kTestPath2, DlColor(SK_ColorGREEN), 1.0, false, 1.0);
            }},
           {1, 48, 1,
            [](DlOpReceiver& r) {
              r.drawShadow(kTestPath1, DlColor(SK_ColorBLUE), 1.0, false, 1.0);
            }},
           {1, 48, 1,
            [](DlOpReceiver& r) {
              r.drawShadow(kTestPath1, DlColor(SK_ColorGREEN), 2.0, false, 1.0);
            }},
           {1, 48, 1,
            [](DlOpReceiver& r) {
              r.drawShadow(kTestPath1, DlColor(SK_ColorGREEN), 1.0, true, 1.0);
            }},
           {1, 48, 1,
            [](DlOpReceiver& r) {
              r.drawShadow(kTestPath1, DlColor(SK_ColorGREEN), 1.0, false, 2.5);
            }},
       }},
  };
}

std::vector<DisplayListInvocationGroup> CreateAllGroups() {
  std::vector<DisplayListInvocationGroup> result;
  auto all_attribute_ops = CreateAllAttributesOps();
  std::move(all_attribute_ops.begin(), all_attribute_ops.end(),
            std::back_inserter(result));
  auto all_save_restore_ops = CreateAllSaveRestoreOps();
  std::move(all_save_restore_ops.begin(), all_save_restore_ops.end(),
            std::back_inserter(result));
  auto all_transform_ops = CreateAllTransformOps();
  std::move(all_transform_ops.begin(), all_transform_ops.end(),
            std::back_inserter(result));
  auto all_clip_ops = CreateAllClipOps();
  std::move(all_clip_ops.begin(), all_clip_ops.end(),
            std::back_inserter(result));
  auto all_rendering_ops = CreateAllRenderingOps();
  std::move(all_rendering_ops.begin(), all_rendering_ops.end(),
            std::back_inserter(result));
  return result;
}

SkFont CreateTestFontOfSize(SkScalar scalar) {
  static constexpr const char* kTestFontFixture = "Roboto-Regular.ttf";
  auto mapping = flutter::testing::OpenFixtureAsSkData(kTestFontFixture);
  FML_CHECK(mapping);
  return SkFont{txt::GetDefaultFontManager()->makeFromData(mapping), scalar};
}

sk_sp<SkTextBlob> GetTestTextBlob(int index) {
  static std::map<int, sk_sp<SkTextBlob>> text_blobs;
  auto it = text_blobs.find(index);
  if (it != text_blobs.end()) {
    return it->second;
  }
  std::string text = "TestBlob" + std::to_string(index);
  sk_sp<SkTextBlob> blob =
      SkTextBlob::MakeFromText(text.c_str(), text.size(),
                               CreateTestFontOfSize(20), SkTextEncoding::kUTF8);
  text_blobs.insert(std::make_pair(index, blob));
  return blob;
}

}  // namespace testing
}  // namespace flutter
