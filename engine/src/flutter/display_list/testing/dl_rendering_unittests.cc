// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <utility>

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_op_flags.h"
#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/effects/color_filters/dl_matrix_color_filter.h"
#include "flutter/display_list/effects/dl_image_filter.h"
#include "flutter/display_list/skia/dl_sk_canvas.h"
#include "flutter/display_list/skia/dl_sk_conversions.h"
#include "flutter/display_list/skia/dl_sk_dispatcher.h"
#include "flutter/display_list/testing/dl_test_snippets.h"
#include "flutter/display_list/testing/dl_test_surface_provider.h"
#include "flutter/display_list/utils/dl_comparable.h"
#include "flutter/fml/file.h"
#include "flutter/fml/math.h"
#include "flutter/testing/display_list_testing.h"
#include "flutter/testing/testing.h"
#ifdef IMPELLER_SUPPORTS_RENDERING
#include "flutter/impeller/typographer/backends/skia/text_frame_skia.h"
#endif  // IMPELLER_SUPPORTS_RENDERING

#include "third_party/skia/include/core/SkBBHFactory.h"
#include "third_party/skia/include/core/SkColorFilter.h"
#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkFontMgr.h"
#include "third_party/skia/include/core/SkPictureRecorder.h"
#include "third_party/skia/include/core/SkStream.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/core/SkTypeface.h"
#include "third_party/skia/include/effects/SkDashPathEffect.h"
#include "third_party/skia/include/effects/SkGradientShader.h"
#include "third_party/skia/include/effects/SkImageFilters.h"
#include "third_party/skia/include/encode/SkPngEncoder.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/GrRecordingContext.h"
#include "third_party/skia/include/gpu/ganesh/GrTypes.h"
#include "txt/platform.h"

namespace flutter {
namespace testing {

using ClipOp = DlCanvas::ClipOp;
using PointMode = DlCanvas::PointMode;

constexpr int kTestWidth = 200;
constexpr int kTestHeight = 200;
constexpr int kRenderWidth = 100;
constexpr int kRenderHeight = 100;
constexpr int kRenderHalfWidth = 50;
constexpr int kRenderHalfHeight = 50;
constexpr int kRenderLeft = (kTestWidth - kRenderWidth) / 2;
constexpr int kRenderTop = (kTestHeight - kRenderHeight) / 2;
constexpr int kRenderRight = kRenderLeft + kRenderWidth;
constexpr int kRenderBottom = kRenderTop + kRenderHeight;
constexpr int kRenderCenterX = (kRenderLeft + kRenderRight) / 2;
constexpr int kRenderCenterY = (kRenderTop + kRenderBottom) / 2;
constexpr SkScalar kRenderRadius = std::min(kRenderWidth, kRenderHeight) / 2.0;
constexpr SkScalar kRenderCornerRadius = kRenderRadius / 5.0;

constexpr SkPoint kTestCenter = SkPoint::Make(kTestWidth / 2, kTestHeight / 2);
constexpr SkRect kTestBounds2 = SkRect::MakeWH(kTestWidth, kTestHeight);
constexpr SkRect kRenderBounds =
    SkRect::MakeLTRB(kRenderLeft, kRenderTop, kRenderRight, kRenderBottom);

// The tests try 3 miter limit values, 0.0, 4.0 (the default), and 10.0
// These values will allow us to construct a diamond that spans the
// width or height of the render box and still show the miter for 4.0
// and 10.0.
// These values were discovered by drawing a diamond path in Skia fiddle
// and then playing with the cross-axis size until the miter was about
// as large as it could get before it got cut off.

// The X offsets which will be used for tall vertical diamonds are
// expressed in terms of the rendering height to obtain the proper angle
constexpr SkScalar kMiterExtremeDiamondOffsetX = kRenderHeight * 0.04;
constexpr SkScalar kMiter10DiamondOffsetX = kRenderHeight * 0.051;
constexpr SkScalar kMiter4DiamondOffsetX = kRenderHeight * 0.14;

// The Y offsets which will be used for long horizontal diamonds are
// expressed in terms of the rendering width to obtain the proper angle
constexpr SkScalar kMiterExtremeDiamondOffsetY = kRenderWidth * 0.04;
constexpr SkScalar kMiter10DiamondOffsetY = kRenderWidth * 0.051;
constexpr SkScalar kMiter4DiamondOffsetY = kRenderWidth * 0.14;

// Render 3 vertical and horizontal diamonds each
// designed to break at the tested miter limits
// 0.0, 4.0 and 10.0
// Center is biased by 0.5 to include more pixel centers in the
// thin miters
constexpr SkScalar kXOffset0 = kRenderCenterX + 0.5;
constexpr SkScalar kXOffsetL1 = kXOffset0 - kMiter4DiamondOffsetX;
constexpr SkScalar kXOffsetL2 = kXOffsetL1 - kMiter10DiamondOffsetX;
constexpr SkScalar kXOffsetL3 = kXOffsetL2 - kMiter10DiamondOffsetX;
constexpr SkScalar kXOffsetR1 = kXOffset0 + kMiter4DiamondOffsetX;
constexpr SkScalar kXOffsetR2 = kXOffsetR1 + kMiterExtremeDiamondOffsetX;
constexpr SkScalar kXOffsetR3 = kXOffsetR2 + kMiterExtremeDiamondOffsetX;
constexpr SkPoint kVerticalMiterDiamondPoints[] = {
    // Vertical diamonds:
    //  M10   M4  Mextreme
    //   /\   /|\   /\       top of RenderBounds
    //  /  \ / | \ /  \              to
    // <----X--+--X---->         RenderCenter
    //  \  / \ | / \  /              to
    //   \/   \|/   \/      bottom of RenderBounds
    // clang-format off
    SkPoint::Make(kXOffsetL3, kRenderCenterY),
    SkPoint::Make(kXOffsetL2, kRenderTop),
    SkPoint::Make(kXOffsetL1, kRenderCenterY),
    SkPoint::Make(kXOffset0,  kRenderTop),
    SkPoint::Make(kXOffsetR1, kRenderCenterY),
    SkPoint::Make(kXOffsetR2, kRenderTop),
    SkPoint::Make(kXOffsetR3, kRenderCenterY),
    SkPoint::Make(kXOffsetR2, kRenderBottom),
    SkPoint::Make(kXOffsetR1, kRenderCenterY),
    SkPoint::Make(kXOffset0,  kRenderBottom),
    SkPoint::Make(kXOffsetL1, kRenderCenterY),
    SkPoint::Make(kXOffsetL2, kRenderBottom),
    SkPoint::Make(kXOffsetL3, kRenderCenterY),
    // clang-format on
};
const int kVerticalMiterDiamondPointCount =
    sizeof(kVerticalMiterDiamondPoints) /
    sizeof(kVerticalMiterDiamondPoints[0]);

constexpr SkScalar kYOffset0 = kRenderCenterY + 0.5;
constexpr SkScalar kYOffsetU1 = kXOffset0 - kMiter4DiamondOffsetY;
constexpr SkScalar kYOffsetU2 = kYOffsetU1 - kMiter10DiamondOffsetY;
constexpr SkScalar kYOffsetU3 = kYOffsetU2 - kMiter10DiamondOffsetY;
constexpr SkScalar kYOffsetD1 = kXOffset0 + kMiter4DiamondOffsetY;
constexpr SkScalar kYOffsetD2 = kYOffsetD1 + kMiterExtremeDiamondOffsetY;
constexpr SkScalar kYOffsetD3 = kYOffsetD2 + kMiterExtremeDiamondOffsetY;
const SkPoint kHorizontalMiterDiamondPoints[] = {
    // Horizontal diamonds
    // Same configuration as Vertical diamonds above but
    // rotated 90 degrees
    // clang-format off
    SkPoint::Make(kRenderCenterX, kYOffsetU3),
    SkPoint::Make(kRenderLeft,    kYOffsetU2),
    SkPoint::Make(kRenderCenterX, kYOffsetU1),
    SkPoint::Make(kRenderLeft,    kYOffset0),
    SkPoint::Make(kRenderCenterX, kYOffsetD1),
    SkPoint::Make(kRenderLeft,    kYOffsetD2),
    SkPoint::Make(kRenderCenterX, kYOffsetD3),
    SkPoint::Make(kRenderRight,   kYOffsetD2),
    SkPoint::Make(kRenderCenterX, kYOffsetD1),
    SkPoint::Make(kRenderRight,   kYOffset0),
    SkPoint::Make(kRenderCenterX, kYOffsetU1),
    SkPoint::Make(kRenderRight,   kYOffsetU2),
    SkPoint::Make(kRenderCenterX, kYOffsetU3),
    // clang-format on
};
const int kHorizontalMiterDiamondPointCount =
    (sizeof(kHorizontalMiterDiamondPoints) /
     sizeof(kHorizontalMiterDiamondPoints[0]));

namespace {
constexpr uint8_t toC(DlScalar fComp) {
  return round(fComp * 255);
}

constexpr uint32_t PremultipliedArgb(const DlColor& color) {
  if (color.isOpaque()) {
    return color.argb();
  }
  DlScalar f = color.getAlphaF();
  return (color.argb() & 0xFF000000) |      //
         toC(color.getRedF() * f) << 16 |   //
         toC(color.getGreenF() * f) << 8 |  //
         toC(color.getBlueF() * f);
}
}  // namespace

class SkImageSampling {
 public:
  static constexpr SkSamplingOptions kNearestNeighbor =
      SkSamplingOptions(SkFilterMode::kNearest);
  static constexpr SkSamplingOptions kLinear =
      SkSamplingOptions(SkFilterMode::kLinear);
  static constexpr SkSamplingOptions kMipmapLinear =
      SkSamplingOptions(SkFilterMode::kLinear, SkMipmapMode::kLinear);
  static constexpr SkSamplingOptions kCubic =
      SkSamplingOptions(SkCubicResampler{1 / 3.0f, 1 / 3.0f});
};

static void DrawCheckerboard(DlCanvas* canvas) {
  DlPaint p0, p1;
  p0.setDrawStyle(DlDrawStyle::kFill);
  p0.setColor(DlColor(0xff00fe00));  // off-green
  p1.setDrawStyle(DlDrawStyle::kFill);
  p1.setColor(DlColor::kBlue());
  // Some pixels need some transparency for DstIn testing
  p1.setAlpha(128);
  int cbdim = 5;
  int width = canvas->GetBaseLayerSize().width();
  int height = canvas->GetBaseLayerSize().height();
  for (int y = 0; y < width; y += cbdim) {
    for (int x = 0; x < height; x += cbdim) {
      DlPaint& cellp = ((x + y) & 1) == 0 ? p0 : p1;
      canvas->DrawRect(SkRect::MakeXYWH(x, y, cbdim, cbdim), cellp);
    }
  }
}

static void DrawCheckerboard(SkCanvas* canvas) {
  DlSkCanvasAdapter dl_canvas(canvas);
  DrawCheckerboard(&dl_canvas);
}

static std::shared_ptr<DlImageColorSource> MakeColorSource(
    const sk_sp<DlImage>& image) {
  return std::make_shared<DlImageColorSource>(image,                //
                                              DlTileMode::kRepeat,  //
                                              DlTileMode::kRepeat,  //
                                              DlImageSampling::kLinear);
}

static sk_sp<SkShader> MakeColorSource(const sk_sp<SkImage>& image) {
  return image->makeShader(SkTileMode::kRepeat,  //
                           SkTileMode::kRepeat,  //
                           SkImageSampling::kLinear);
}

// Used to show "INFO" warnings about tests that are omitted on certain
// backends, but only once for the entire test run to avoid warning spam
class OncePerBackendWarning {
 public:
  explicit OncePerBackendWarning(const std::string& warning)
      : warning_(warning) {}

  void warn(const std::string& name) {
    if (warnings_sent_.find(name) == warnings_sent_.end()) {
      warnings_sent_.insert(name);
      FML_LOG(INFO) << warning_ << " on " << name;
    }
  }

 private:
  std::string warning_;
  std::set<std::string> warnings_sent_;
};

// A class to specify how much tolerance to allow in bounds estimates.
// For some attributes, the machinery must make some conservative
// assumptions as to the extent of the bounds, but some of our test
// parameters do not produce bounds that expand by the full conservative
// estimates. This class provides a number of tweaks to apply to the
// pixel bounds to account for the conservative factors.
//
// An instance is passed along through the methods and if any test adds
// a paint attribute or other modifier that will cause a more conservative
// estimate for bounds, it can modify the factors here to account for it.
// Ideally, all tests will be executed with geometry that will trigger
// the conservative cases anyway and all attributes will be combined with
// other attributes that make their output more predictable, but in those
// cases where a given test sequence cannot really provide attributes to
// demonstrate the worst case scenario, they can modify these factors to
// avoid false bounds overflow notifications.
class BoundsTolerance {
 public:
  BoundsTolerance() = default;
  BoundsTolerance(const BoundsTolerance&) = default;

  BoundsTolerance addBoundsPadding(SkScalar bounds_pad_x,
                                   SkScalar bounds_pad_y) const {
    BoundsTolerance copy = BoundsTolerance(*this);
    copy.bounds_pad_.offset(bounds_pad_x, bounds_pad_y);
    return copy;
  }

  BoundsTolerance mulScale(SkScalar scale_x, SkScalar scale_y) const {
    BoundsTolerance copy = BoundsTolerance(*this);
    copy.scale_.fX *= scale_x;
    copy.scale_.fY *= scale_y;
    return copy;
  }

  BoundsTolerance addAbsolutePadding(SkScalar absolute_pad_x,
                                     SkScalar absolute_pad_y) const {
    BoundsTolerance copy = BoundsTolerance(*this);
    copy.absolute_pad_.offset(absolute_pad_x, absolute_pad_y);
    return copy;
  }

  BoundsTolerance addPostClipPadding(SkScalar absolute_pad_x,
                                     SkScalar absolute_pad_y) const {
    BoundsTolerance copy = BoundsTolerance(*this);
    copy.clip_pad_.offset(absolute_pad_x, absolute_pad_y);
    return copy;
  }

  BoundsTolerance addDiscreteOffset(SkScalar discrete_offset) const {
    BoundsTolerance copy = BoundsTolerance(*this);
    copy.discrete_offset_ += discrete_offset;
    return copy;
  }

  BoundsTolerance clip(SkRect clip) const {
    BoundsTolerance copy = BoundsTolerance(*this);
    if (!copy.clip_.intersect(clip)) {
      copy.clip_.setEmpty();
    }
    return copy;
  }

  static SkRect Scale(const SkRect& rect, const SkPoint& scales) {
    SkScalar outset_x = rect.width() * (scales.fX - 1);
    SkScalar outset_y = rect.height() * (scales.fY - 1);
    return rect.makeOutset(outset_x, outset_y);
  }

  bool overflows(SkIRect pix_bounds,
                 int worst_bounds_pad_x,
                 int worst_bounds_pad_y) const {
    SkRect allowed = SkRect::Make(pix_bounds);
    allowed.outset(bounds_pad_.fX, bounds_pad_.fY);
    allowed = Scale(allowed, scale_);
    allowed.outset(absolute_pad_.fX, absolute_pad_.fY);
    if (!allowed.intersect(clip_)) {
      allowed.setEmpty();
    }
    allowed.outset(clip_pad_.fX, clip_pad_.fY);
    SkIRect rounded = allowed.roundOut();
    int pad_left = std::max(0, pix_bounds.fLeft - rounded.fLeft);
    int pad_top = std::max(0, pix_bounds.fTop - rounded.fTop);
    int pad_right = std::max(0, pix_bounds.fRight - rounded.fRight);
    int pad_bottom = std::max(0, pix_bounds.fBottom - rounded.fBottom);
    int allowed_pad_x = std::max(pad_left, pad_right);
    int allowed_pad_y = std::max(pad_top, pad_bottom);
    if (worst_bounds_pad_x > allowed_pad_x ||
        worst_bounds_pad_y > allowed_pad_y) {
      FML_LOG(ERROR) << "acceptable bounds padding: "  //
                     << allowed_pad_x << ", " << allowed_pad_y;
    }
    return (worst_bounds_pad_x > allowed_pad_x ||
            worst_bounds_pad_y > allowed_pad_y);
  }

  SkScalar discrete_offset() const { return discrete_offset_; }

  bool operator==(BoundsTolerance const& other) const {
    return bounds_pad_ == other.bounds_pad_ && scale_ == other.scale_ &&
           absolute_pad_ == other.absolute_pad_ && clip_ == other.clip_ &&
           clip_pad_ == other.clip_pad_ &&
           discrete_offset_ == other.discrete_offset_;
  }

 private:
  SkPoint bounds_pad_ = {0, 0};
  SkPoint scale_ = {1, 1};
  SkPoint absolute_pad_ = {0, 0};
  SkRect clip_ = {-1E9, -1E9, 1E9, 1E9};
  SkPoint clip_pad_ = {0, 0};

  SkScalar discrete_offset_ = 0;
};

template <typename C, typename P, typename I>
struct RenderContext {
  C canvas;
  P paint;
  I image;
};
using SkSetupContext = RenderContext<SkCanvas*, SkPaint&, sk_sp<SkImage>>;
using DlSetupContext = RenderContext<DlCanvas*, DlPaint&, sk_sp<DlImage>>;
using SkRenderContext =
    RenderContext<SkCanvas*, const SkPaint&, sk_sp<SkImage>>;
using DlRenderContext =
    RenderContext<DlCanvas*, const DlPaint&, sk_sp<DlImage>>;

using SkSetup = const std::function<void(const SkSetupContext&)>;
using SkRenderer = const std::function<void(const SkRenderContext&)>;
using DlSetup = const std::function<void(const DlSetupContext&)>;
using DlRenderer = const std::function<void(const DlRenderContext&)>;
static const SkSetup kEmptySkSetup = [](const SkSetupContext&) {};
static const SkRenderer kEmptySkRenderer = [](const SkRenderContext&) {};
static const DlSetup kEmptyDlSetup = [](const DlSetupContext&) {};
static const DlRenderer kEmptyDlRenderer = [](const DlRenderContext&) {};

using PixelFormat = DlSurfaceProvider::PixelFormat;
using BackendType = DlSurfaceProvider::BackendType;

class RenderResult {
 public:
  virtual ~RenderResult() = default;

  virtual sk_sp<SkImage> image() const = 0;
  virtual int width() const = 0;
  virtual int height() const = 0;
  virtual const uint32_t* addr32(int x, int y) const = 0;
  virtual void write(const std::string& path) const = 0;
};

class SkRenderResult final : public RenderResult {
 public:
  explicit SkRenderResult(const sk_sp<SkSurface>& surface,
                          bool take_snapshot = false) {
    SkImageInfo info = surface->imageInfo();
    info = SkImageInfo::MakeN32Premul(info.dimensions());
    addr_ = malloc(info.computeMinByteSize() * info.height());
    pixmap_.reset(info, addr_, info.minRowBytes());
    surface->readPixels(pixmap_, 0, 0);
    if (take_snapshot) {
      image_ = surface->makeImageSnapshot();
    }
  }
  ~SkRenderResult() override { free(addr_); }

  sk_sp<SkImage> image() const override { return image_; }
  int width() const override { return pixmap_.width(); }
  int height() const override { return pixmap_.height(); }
  const uint32_t* addr32(int x, int y) const override {
    return pixmap_.addr32(x, y);
  }
  void write(const std::string& path) const {
    auto stream = SkFILEWStream(path.c_str());
    SkPngEncoder::Options options;
    SkPngEncoder::Encode(&stream, pixmap_, options);
    stream.flush();
  }

 private:
  sk_sp<SkImage> image_;
  SkPixmap pixmap_;
  void* addr_ = nullptr;
};

class ImpellerRenderResult final : public RenderResult {
 public:
  explicit ImpellerRenderResult(sk_sp<DlPixelData> screenshot,
                                SkRect render_bounds)
      : screenshot_(std::move(screenshot)), render_bounds_(render_bounds) {}
  ~ImpellerRenderResult() override = default;

  sk_sp<SkImage> image() const override { return nullptr; };
  int width() const override { return screenshot_->width(); };
  int height() const override { return screenshot_->height(); }
  const uint32_t* addr32(int x, int y) const override {
    return screenshot_->addr32(x, y);
  }
  void write(const std::string& path) const override {
    screenshot_->write(path);
  }
  const SkRect& render_bounds() const { return render_bounds_; }

 private:
  const sk_sp<DlPixelData> screenshot_;
  SkRect render_bounds_;
};

struct RenderJobInfo {
  int width = kTestWidth;
  int height = kTestHeight;
  DlColor bg = DlColor::kTransparent();
  SkScalar scale = SK_Scalar1;
  SkScalar opacity = SK_Scalar1;
};

struct JobRenderer {
  virtual void Render(SkCanvas* canvas, const RenderJobInfo& info) = 0;
  virtual bool targets_impeller() const { return false; }
};

struct MatrixClipJobRenderer : public JobRenderer {
 public:
  const SkMatrix& setup_matrix() const {
    FML_CHECK(is_setup_);
    return setup_matrix_;
  }

  const SkIRect& setup_clip_bounds() const {
    FML_CHECK(is_setup_);
    return setup_clip_bounds_;
  }

 protected:
  bool is_setup_ = false;
  SkMatrix setup_matrix_;
  SkIRect setup_clip_bounds_;
};

struct SkJobRenderer : public MatrixClipJobRenderer {
  explicit SkJobRenderer(const SkSetup& sk_setup,
                         const SkRenderer& sk_render,
                         const SkRenderer& sk_restore,
                         const sk_sp<SkImage>& sk_image)
      : sk_setup_(sk_setup),
        sk_render_(sk_render),
        sk_restore_(sk_restore),
        sk_image_(sk_image) {}

  void Render(SkCanvas* canvas, const RenderJobInfo& info) override {
    FML_DCHECK(info.opacity == SK_Scalar1);
    SkPaint paint;
    sk_setup_({canvas, paint, sk_image_});
    setup_paint_ = paint;
    setup_matrix_ = canvas->getTotalMatrix();
    setup_clip_bounds_ = canvas->getDeviceClipBounds();
    is_setup_ = true;
    sk_render_({canvas, paint, sk_image_});
    sk_restore_({canvas, paint, sk_image_});
  }

  sk_sp<SkPicture> MakePicture(const RenderJobInfo& info) {
    SkPictureRecorder recorder;
    SkRTreeFactory rtree_factory;
    SkCanvas* cv = recorder.beginRecording(kTestBounds2, &rtree_factory);
    Render(cv, info);
    return recorder.finishRecordingAsPicture();
  }

  const SkPaint& setup_paint() const {
    FML_CHECK(is_setup_);
    return setup_paint_;
  }

 private:
  const SkSetup sk_setup_;
  const SkRenderer sk_render_;
  const SkRenderer sk_restore_;
  sk_sp<SkImage> sk_image_;
  SkPaint setup_paint_;
};

struct DlJobRenderer : public MatrixClipJobRenderer {
  explicit DlJobRenderer(const DlSetup& dl_setup,
                         const DlRenderer& dl_render,
                         const DlRenderer& dl_restore,
                         const sk_sp<DlImage>& dl_image)
      : dl_setup_(dl_setup),
        dl_render_(dl_render),
        dl_restore_(dl_restore),
        dl_image_(dl_image) {}

  void Render(SkCanvas* sk_canvas, const RenderJobInfo& info) override {
    DlSkCanvasAdapter canvas(sk_canvas);
    Render(&canvas, info);
  }

  void Render(DlCanvas* canvas, const RenderJobInfo& info) {
    FML_DCHECK(info.opacity == SK_Scalar1);
    DlPaint paint;
    dl_setup_({canvas, paint, dl_image_});
    setup_paint_ = paint;
    setup_matrix_ = canvas->GetTransform();
    setup_clip_bounds_ = canvas->GetDestinationClipBounds().roundOut();
    is_setup_ = true;
    dl_render_({canvas, paint, dl_image_});
    dl_restore_({canvas, paint, dl_image_});
  }

  sk_sp<DisplayList> MakeDisplayList(const RenderJobInfo& info) {
    DisplayListBuilder builder(kTestBounds2);
    Render(&builder, info);
    return builder.Build();
  }

  const DlPaint& setup_paint() const {
    FML_CHECK(is_setup_);
    return setup_paint_;
  }

  bool targets_impeller() const override {
    return dl_image_->impeller_texture() != nullptr;
  }

 private:
  const DlSetup dl_setup_;
  const DlRenderer dl_render_;
  const DlRenderer dl_restore_;
  const sk_sp<DlImage> dl_image_;
  DlPaint setup_paint_;
};

struct SkPictureJobRenderer : public JobRenderer {
  explicit SkPictureJobRenderer(sk_sp<SkPicture> picture)
      : picture_(std::move(picture)) {}

  void Render(SkCanvas* canvas, const RenderJobInfo& info) {
    FML_DCHECK(info.opacity == SK_Scalar1);
    picture_->playback(canvas);
  }

 private:
  sk_sp<SkPicture> picture_;
};

struct DisplayListJobRenderer : public JobRenderer {
  explicit DisplayListJobRenderer(sk_sp<DisplayList> display_list)
      : display_list_(std::move(display_list)) {}

  void Render(SkCanvas* canvas, const RenderJobInfo& info) {
    DlSkCanvasAdapter(canvas).DrawDisplayList(display_list_, info.opacity);
  }

 private:
  sk_sp<DisplayList> display_list_;
};

class RenderEnvironment {
 public:
  static bool EnableImpeller;

  RenderEnvironment(const DlSurfaceProvider* provider, PixelFormat format)
      : provider_(provider), format_(format) {
    if (provider->supports(format)) {
      surface_1x_ =
          provider->MakeOffscreenSurface(kTestWidth, kTestHeight, format);
      surface_2x_ = provider->MakeOffscreenSurface(kTestWidth * 2,
                                                   kTestHeight * 2, format);
    }
  }

  static RenderEnvironment Make565(const DlSurfaceProvider* provider) {
    return RenderEnvironment(provider, PixelFormat::k565PixelFormat);
  }

  static RenderEnvironment MakeN32(const DlSurfaceProvider* provider) {
    return RenderEnvironment(provider, PixelFormat::kN32PremulPixelFormat);
  }

  void init_ref(SkSetup& sk_setup,
                SkRenderer& sk_renderer,
                DlSetup& dl_setup,
                DlRenderer& dl_renderer,
                DlRenderer& imp_renderer,
                DlColor bg = DlColor::kTransparent()) {
    SkJobRenderer sk_job(sk_setup, sk_renderer, kEmptySkRenderer, kTestSkImage);
    RenderJobInfo info = {
        .bg = bg,
    };
    ref_sk_result_ = getResult(info, sk_job);
    DlJobRenderer dl_job(dl_setup, dl_renderer, kEmptyDlRenderer, kTestDlImage);
    ref_dl_result_ = getResult(info, dl_job);
    ref_dl_paint_ = dl_job.setup_paint();
    ref_matrix_ = dl_job.setup_matrix();
    ref_clip_bounds_ = dl_job.setup_clip_bounds();
    ASSERT_EQ(sk_job.setup_matrix(), ref_matrix_);
    ASSERT_EQ(sk_job.setup_clip_bounds(), ref_clip_bounds_);
    if (supports_impeller()) {
      test_impeller_image_ = makeTestImpellerImage(provider_);
      DlJobRenderer imp_job(dl_setup, imp_renderer, kEmptyDlRenderer,
                            test_impeller_image_);
      ref_impeller_result_ = getImpellerResult(info, imp_job);
    }
  }

  std::unique_ptr<RenderResult> getResult(const RenderJobInfo& info,
                                          JobRenderer& renderer) const {
    auto surface = getSurface(info.width, info.height);
    FML_DCHECK(surface != nullptr);
    auto canvas = surface->getCanvas();
    canvas->clear(ToSk(info.bg));

    int restore_count = canvas->save();
    canvas->scale(info.scale, info.scale);
    renderer.Render(canvas, info);
    canvas->restoreToCount(restore_count);

    if (GrDirectContext* dContext =
            GrAsDirectContext(surface->recordingContext())) {
      dContext->flushAndSubmit(surface.get(), GrSyncCpu::kYes);
    }
    return std::make_unique<SkRenderResult>(surface);
  }

  std::unique_ptr<RenderResult> getResult(sk_sp<DisplayList> dl) const {
    DisplayListJobRenderer job(std::move(dl));
    RenderJobInfo info = {};
    return getResult(info, job);
  }

  std::unique_ptr<ImpellerRenderResult> getImpellerResult(
      const RenderJobInfo& info,
      DlJobRenderer& renderer) const {
    FML_DCHECK(info.scale == SK_Scalar1);

    DisplayListBuilder builder;
    builder.Clear(info.bg);
    auto render_dl = renderer.MakeDisplayList(info);
    builder.DrawDisplayList(render_dl);
    auto dl = builder.Build();
    auto snap = provider_->ImpellerSnapshot(dl, kTestWidth, kTestHeight);
    return std::make_unique<ImpellerRenderResult>(std::move(snap),
                                                  render_dl->bounds());
  }

  const DlSurfaceProvider* provider() const { return provider_; }
  bool valid() const { return provider_->supports(format_); }
  const std::string backend_name() const { return provider_->backend_name(); }
  bool supports_impeller() const {
    return EnableImpeller && provider_->supports_impeller();
  }

  PixelFormat format() const { return format_; }
  const DlPaint& ref_dl_paint() const { return ref_dl_paint_; }
  const SkMatrix& ref_matrix() const { return ref_matrix_; }
  const SkIRect& ref_clip_bounds() const { return ref_clip_bounds_; }
  const RenderResult* ref_sk_result() const { return ref_sk_result_.get(); }
  const RenderResult* ref_dl_result() const { return ref_dl_result_.get(); }
  const ImpellerRenderResult* ref_impeller_result() const {
    return ref_impeller_result_.get();
  }

  const sk_sp<SkImage> sk_image() const { return kTestSkImage; }
  const sk_sp<DlImage> dl_image() const { return kTestDlImage; }
  const sk_sp<DlImage> impeller_image() const { return test_impeller_image_; }

 private:
  sk_sp<SkSurface> getSurface(int width, int height) const {
    FML_DCHECK(valid());
    FML_DCHECK(surface_1x_ != nullptr);
    FML_DCHECK(surface_2x_ != nullptr);
    if (width == kTestWidth && height == kTestHeight) {
      return surface_1x_->sk_surface();
    }
    if (width == kTestWidth * 2 && height == kTestHeight * 2) {
      return surface_2x_->sk_surface();
    }
    FML_LOG(ERROR) << "Test surface size (" << width << " x " << height
                   << ") not supported.";
    FML_DCHECK(false);
    return nullptr;
  }

  const DlSurfaceProvider* provider_;
  const PixelFormat format_;
  std::shared_ptr<DlSurfaceInstance> surface_1x_;
  std::shared_ptr<DlSurfaceInstance> surface_2x_;

  DlPaint ref_dl_paint_;
  SkMatrix ref_matrix_;
  SkIRect ref_clip_bounds_;
  std::unique_ptr<RenderResult> ref_sk_result_;
  std::unique_ptr<RenderResult> ref_dl_result_;
  std::unique_ptr<ImpellerRenderResult> ref_impeller_result_;
  sk_sp<DlImage> test_impeller_image_;

  static const sk_sp<SkImage> kTestSkImage;
  static const sk_sp<DlImage> kTestDlImage;
  static const sk_sp<SkImage> makeTestSkImage() {
    sk_sp<SkSurface> surface = SkSurfaces::Raster(
        SkImageInfo::MakeN32Premul(kRenderWidth, kRenderHeight));
    DrawCheckerboard(surface->getCanvas());
    return surface->makeImageSnapshot();
  }
  static const sk_sp<DlImage> makeTestImpellerImage(
      const DlSurfaceProvider* provider) {
    FML_DCHECK(provider->supports_impeller());
    DisplayListBuilder builder(SkRect::MakeWH(kRenderWidth, kRenderHeight));
    DrawCheckerboard(&builder);
    return provider->MakeImpellerImage(builder.Build(),  //
                                       kRenderWidth, kRenderHeight);
  }
};

const sk_sp<SkImage> RenderEnvironment::kTestSkImage = makeTestSkImage();
const sk_sp<DlImage> RenderEnvironment::kTestDlImage =
    DlImage::Make(kTestSkImage);

class CaseParameters {
 public:
  explicit CaseParameters(std::string info)
      : CaseParameters(std::move(info), kEmptySkSetup, kEmptyDlSetup) {}

  CaseParameters(std::string info, SkSetup& sk_setup, DlSetup& dl_setup)
      : CaseParameters(std::move(info),
                       sk_setup,
                       dl_setup,
                       kEmptySkRenderer,
                       kEmptyDlRenderer,
                       DlColor(SK_ColorTRANSPARENT),
                       false,
                       false,
                       false) {}

  CaseParameters(std::string info,
                 SkSetup& sk_setup,
                 DlSetup& dl_setup,
                 SkRenderer& sk_restore,
                 DlRenderer& dl_restore,
                 DlColor bg,
                 bool has_diff_clip,
                 bool has_mutating_save_layer,
                 bool fuzzy_compare_components)
      : info_(std::move(info)),
        bg_(bg),
        sk_setup_(sk_setup),
        dl_setup_(dl_setup),
        sk_restore_(sk_restore),
        dl_restore_(dl_restore),
        has_diff_clip_(has_diff_clip),
        has_mutating_save_layer_(has_mutating_save_layer),
        fuzzy_compare_components_(fuzzy_compare_components) {}

  CaseParameters with_restore(SkRenderer& sk_restore,
                              DlRenderer& dl_restore,
                              bool mutating_layer,
                              bool fuzzy_compare_components = false) {
    return CaseParameters(info_, sk_setup_, dl_setup_, sk_restore, dl_restore,
                          bg_, has_diff_clip_, mutating_layer,
                          fuzzy_compare_components);
  }

  CaseParameters with_bg(DlColor bg) {
    return CaseParameters(info_, sk_setup_, dl_setup_, sk_restore_, dl_restore_,
                          bg, has_diff_clip_, has_mutating_save_layer_,
                          fuzzy_compare_components_);
  }

  CaseParameters with_diff_clip() {
    return CaseParameters(info_, sk_setup_, dl_setup_, sk_restore_, dl_restore_,
                          bg_, true, has_mutating_save_layer_,
                          fuzzy_compare_components_);
  }

  std::string info() const { return info_; }
  DlColor bg() const { return bg_; }
  bool has_diff_clip() const { return has_diff_clip_; }
  bool has_mutating_save_layer() const { return has_mutating_save_layer_; }
  bool fuzzy_compare_components() const { return fuzzy_compare_components_; }

  SkSetup sk_setup() const { return sk_setup_; }
  DlSetup dl_setup() const { return dl_setup_; }
  SkRenderer sk_restore() const { return sk_restore_; }
  DlRenderer dl_restore() const { return dl_restore_; }

 private:
  const std::string info_;
  const DlColor bg_;
  const SkSetup sk_setup_;
  const DlSetup dl_setup_;
  const SkRenderer sk_restore_;
  const DlRenderer dl_restore_;
  const bool has_diff_clip_;
  const bool has_mutating_save_layer_;
  const bool fuzzy_compare_components_;
};

class TestParameters {
 public:
  TestParameters(const SkRenderer& sk_renderer,
                 const DlRenderer& dl_renderer,
                 const DisplayListAttributeFlags& flags)
      : TestParameters(sk_renderer, dl_renderer, dl_renderer, flags) {}

  TestParameters(const SkRenderer& sk_renderer,
                 const DlRenderer& dl_renderer,
                 const DlRenderer& imp_renderer,
                 const DisplayListAttributeFlags& flags)
      : sk_renderer_(sk_renderer),
        dl_renderer_(dl_renderer),
        imp_renderer_(imp_renderer),
        flags_(flags) {}

  bool uses_paint() const { return !flags_.ignores_paint(); }
  bool uses_gradient() const { return flags_.applies_shader(); }

  bool impeller_compatible(const DlPaint& paint) const {
    if (is_draw_text_blob()) {
      // Non-color text is rendered as paths
      if (paint.getColorSourcePtr()) {
        return false;
      }
      // Non-filled text (stroke or stroke and fill) is rendered as paths
      if (paint.getDrawStyle() != DlDrawStyle::kFill) {
        return false;
      }
    }
    return true;
  }

  bool should_match(const RenderEnvironment& env,
                    const CaseParameters& caseP,
                    const DlPaint& attr,
                    const MatrixClipJobRenderer& renderer) const {
    if (caseP.has_mutating_save_layer()) {
      return false;
    }
    if (env.ref_clip_bounds() != renderer.setup_clip_bounds() ||
        caseP.has_diff_clip()) {
      return false;
    }
    if (env.ref_matrix() != renderer.setup_matrix() && !flags_.is_flood()) {
      return false;
    }
    if (flags_.ignores_paint()) {
      return true;
    }
    const DlPaint& ref_attr = env.ref_dl_paint();
    if (flags_.applies_anti_alias() &&  //
        ref_attr.isAntiAlias() != attr.isAntiAlias()) {
      if (renderer.targets_impeller()) {
        // Impeller only does MSAA, ignoring the AA attribute
        // https://github.com/flutter/flutter/issues/104721
      } else {
        return false;
      }
    }
    if (flags_.applies_color() &&  //
        ref_attr.getColor() != attr.getColor()) {
      return false;
    }
    if (flags_.applies_blend() &&  //
        ref_attr.getBlendMode() != attr.getBlendMode()) {
      return false;
    }
    if (flags_.applies_color_filter() &&  //
        (ref_attr.isInvertColors() != attr.isInvertColors() ||
         NotEquals(ref_attr.getColorFilter(), attr.getColorFilter()))) {
      return false;
    }
    if (flags_.applies_mask_filter() &&  //
        NotEquals(ref_attr.getMaskFilter(), attr.getMaskFilter())) {
      return false;
    }
    if (flags_.applies_image_filter() &&  //
        ref_attr.getImageFilter() != attr.getImageFilter()) {
      return false;
    }
    if (flags_.applies_shader() &&  //
        NotEquals(ref_attr.getColorSource(), attr.getColorSource())) {
      return false;
    }

    bool is_stroked = flags_.is_stroked(attr.getDrawStyle());
    if (flags_.is_stroked(ref_attr.getDrawStyle()) != is_stroked) {
      return false;
    }
    if (!is_stroked) {
      return true;
    }
    if (ref_attr.getStrokeWidth() != attr.getStrokeWidth()) {
      return false;
    }
    DisplayListSpecialGeometryFlags geo_flags =
        flags_.GeometryFlags(is_stroked);
    if (geo_flags.may_have_end_caps() &&  //
        getCap(ref_attr, geo_flags) != getCap(attr, geo_flags)) {
      return false;
    }
    if (geo_flags.may_have_joins()) {
      if (ref_attr.getStrokeJoin() != attr.getStrokeJoin()) {
        return false;
      }
      if (ref_attr.getStrokeJoin() == DlStrokeJoin::kMiter) {
        SkScalar ref_miter = ref_attr.getStrokeMiter();
        SkScalar test_miter = attr.getStrokeMiter();
        // miter limit < 1.4 affects right angles
        if (geo_flags.may_have_acute_joins() ||  //
            ref_miter < 1.4 || test_miter < 1.4) {
          if (ref_miter != test_miter) {
            return false;
          }
        }
      }
    }
    return true;
  }

  DlStrokeCap getCap(const DlPaint& attr,
                     DisplayListSpecialGeometryFlags geo_flags) const {
    DlStrokeCap cap = attr.getStrokeCap();
    if (geo_flags.butt_cap_becomes_square() && cap == DlStrokeCap::kButt) {
      return DlStrokeCap::kSquare;
    }
    return cap;
  }

  const BoundsTolerance adjust(const BoundsTolerance& tolerance,
                               const DlPaint& paint,
                               const SkMatrix& matrix) const {
    if (is_draw_text_blob() && tolerance.discrete_offset() > 0) {
      // drawTextBlob needs just a little more leeway when using a
      // discrete path effect.
      return tolerance.addBoundsPadding(2, 2);
    }
    if (is_draw_line()) {
      return lineAdjust(tolerance, paint, matrix);
    }
    if (is_draw_arc_center()) {
      if (paint.getDrawStyle() != DlDrawStyle::kFill &&
          paint.getStrokeJoin() == DlStrokeJoin::kMiter) {
        // the miter join at the center of an arc does not really affect
        // its bounds in any of our test cases, but the bounds code needs
        // to take it into account for the cases where it might, so we
        // relax our tolerance to reflect the miter bounds padding.
        SkScalar miter_pad =
            paint.getStrokeMiter() * paint.getStrokeWidth() * 0.5f;
        return tolerance.addBoundsPadding(miter_pad, miter_pad);
      }
    }
    return tolerance;
  }

  const BoundsTolerance lineAdjust(const BoundsTolerance& tolerance,
                                   const DlPaint& paint,
                                   const SkMatrix& matrix) const {
    SkScalar adjust = 0.0;
    SkScalar half_width = paint.getStrokeWidth() * 0.5f;
    if (tolerance.discrete_offset() > 0) {
      // When a discrete path effect is added, the bounds calculations must
      // allow for miters in any direction, but a horizontal line will not
      // have miters in the horizontal direction, similarly for vertical
      // lines, and diagonal lines will have miters off at a "45 degree"
      // angle that don't expand the bounds much at all.
      // Also, the discrete offset will not move any points parallel with
      // the line, so provide tolerance for both miters and offset.
      adjust =
          half_width * paint.getStrokeMiter() + tolerance.discrete_offset();
    }

    DisplayListSpecialGeometryFlags geo_flags = flags_.GeometryFlags(true);
    if (paint.getStrokeCap() == DlStrokeCap::kButt &&
        !geo_flags.butt_cap_becomes_square()) {
      adjust = std::max(adjust, half_width);
    }
    if (adjust == 0) {
      return tolerance;
    }
    SkScalar h_tolerance;
    SkScalar v_tolerance;
    if (is_horizontal_line()) {
      FML_DCHECK(!is_vertical_line());
      h_tolerance = adjust;
      v_tolerance = 0;
    } else if (is_vertical_line()) {
      h_tolerance = 0;
      v_tolerance = adjust;
    } else {
      // The perpendicular miters just do not impact the bounds of
      // diagonal lines at all as they are aimed in the wrong direction
      // to matter. So allow tolerance in both axes.
      h_tolerance = v_tolerance = adjust;
    }
    BoundsTolerance new_tolerance =
        tolerance.addBoundsPadding(h_tolerance, v_tolerance);
    return new_tolerance;
  }

  const SkRenderer& sk_renderer() const { return sk_renderer_; }
  const DlRenderer& dl_renderer() const { return dl_renderer_; }
  const DlRenderer& imp_renderer() const { return imp_renderer_; }

  // Tests that call drawTextBlob with an sk_ref paint attribute will cause
  // those attributes to be stored in an internal Skia cache so we need
  // to expect that the |sk_ref.unique()| call will fail in those cases.
  // See: (TBD(flar) - file Skia bug)
  bool is_draw_text_blob() const { return is_draw_text_blob_; }
  bool is_draw_display_list() const { return is_draw_display_list_; }
  bool is_draw_line() const { return is_draw_line_; }
  bool is_draw_arc_center() const { return is_draw_arc_center_; }
  bool is_draw_path() const { return is_draw_path_; }
  bool is_horizontal_line() const { return is_horizontal_line_; }
  bool is_vertical_line() const { return is_vertical_line_; }
  bool ignores_dashes() const { return ignores_dashes_; }

  TestParameters& set_draw_text_blob() {
    is_draw_text_blob_ = true;
    return *this;
  }
  TestParameters& set_draw_display_list() {
    is_draw_display_list_ = true;
    return *this;
  }
  TestParameters& set_draw_line() {
    is_draw_line_ = true;
    return *this;
  }
  TestParameters& set_draw_arc_center() {
    is_draw_arc_center_ = true;
    return *this;
  }
  TestParameters& set_draw_path() {
    is_draw_path_ = true;
    return *this;
  }
  TestParameters& set_ignores_dashes() {
    ignores_dashes_ = true;
    return *this;
  }
  TestParameters& set_horizontal_line() {
    is_horizontal_line_ = true;
    return *this;
  }
  TestParameters& set_vertical_line() {
    is_vertical_line_ = true;
    return *this;
  }

 private:
  const SkRenderer sk_renderer_;
  const DlRenderer dl_renderer_;
  const DlRenderer imp_renderer_;
  const DisplayListAttributeFlags flags_;

  bool is_draw_text_blob_ = false;
  bool is_draw_display_list_ = false;
  bool is_draw_line_ = false;
  bool is_draw_arc_center_ = false;
  bool is_draw_path_ = false;
  bool ignores_dashes_ = false;
  bool is_horizontal_line_ = false;
  bool is_vertical_line_ = false;
};

class CanvasCompareTester {
 public:
  static std::vector<BackendType> TestBackends;
  static std::string ImpellerFailureImageDirectory;
  static bool SaveImpellerFailureImages;
  static std::vector<std::string> ImpellerFailureImages;
  static bool ImpellerSupported;

  static std::unique_ptr<DlSurfaceProvider> GetProvider(BackendType type) {
    auto provider = DlSurfaceProvider::Create(type);
    if (provider == nullptr) {
      FML_LOG(ERROR) << "provider " << DlSurfaceProvider::BackendName(type)
                     << " not supported (ignoring)";
      return nullptr;
    }
    provider->InitializeSurface(kTestWidth, kTestHeight,
                                PixelFormat::kN32PremulPixelFormat);
    return provider;
  }

  static void ClearProviders() { TestBackends.clear(); }

  static bool AddProvider(BackendType type) {
    auto provider = GetProvider(type);
    if (!provider) {
      return false;
    }
    if (provider->supports_impeller()) {
      ImpellerSupported = true;
    }
    TestBackends.push_back(type);
    return true;
  }

  static BoundsTolerance DefaultTolerance;

  static void RenderAll(const TestParameters& params,
                        const BoundsTolerance& tolerance = DefaultTolerance) {
    for (auto& back_end : TestBackends) {
      auto provider = GetProvider(back_end);
      RenderEnvironment env = RenderEnvironment::MakeN32(provider.get());
      env.init_ref(kEmptySkSetup, params.sk_renderer(),  //
                   kEmptyDlSetup, params.dl_renderer(), params.imp_renderer());
      quickCompareToReference(env, "default");
      if (env.supports_impeller()) {
        auto impeller_result = env.ref_impeller_result();
        if (!checkPixels(impeller_result, impeller_result->render_bounds(),
                         "Impeller reference")) {
          std::string test_name =
              ::testing::UnitTest::GetInstance()->current_test_info()->name();
          save_to_png(impeller_result, test_name + " (Impeller reference)",
                      "base rendering was blank or out of bounds");
        }
      } else {
        static OncePerBackendWarning warnings("No Impeller output tests");
        warnings.warn(env.backend_name());
      }

      RenderWithTransforms(params, env, tolerance);
      RenderWithClips(params, env, tolerance);
      RenderWithSaveRestore(params, env, tolerance);
      // Only test attributes if the canvas version uses the paint object
      if (params.uses_paint()) {
        RenderWithAttributes(params, env, tolerance);
      }
    }
  }

  static void RenderWithSaveRestore(const TestParameters& testP,
                                    const RenderEnvironment& env,
                                    const BoundsTolerance& tolerance) {
    SkRect clip =
        SkRect::MakeXYWH(kRenderCenterX - 1, kRenderCenterY - 1, 2, 2);
    SkRect rect = SkRect::MakeXYWH(kRenderCenterX, kRenderCenterY, 10, 10);
    DlColor alpha_layer_color = DlColor::kCyan().withAlpha(0x7f);
    SkRenderer sk_safe_restore = [=](const SkRenderContext& ctx) {
      // Draw another primitive to disable peephole optimizations
      ctx.canvas->drawRect(kRenderBounds.makeOffset(500, 500), SkPaint());
      ctx.canvas->restore();
    };
    DlRenderer dl_safe_restore = [=](const DlRenderContext& ctx) {
      // Draw another primitive to disable peephole optimizations
      // As the rendering op rejection in the DisplayList Builder
      // gets smarter and smarter, this operation has had to get
      // sneakier and sneakier about specifying an operation that
      // won't practically show up in the output, but technically
      // can't be culled.
      ctx.canvas->DrawRect(
          SkRect::MakeXYWH(kRenderCenterX, kRenderCenterY, 0.0001, 0.0001),
          DlPaint());
      ctx.canvas->Restore();
    };
    SkRenderer sk_opt_restore = [=](const SkRenderContext& ctx) {
      // Just a simple restore to allow peephole optimizations to occur
      ctx.canvas->restore();
    };
    DlRenderer dl_opt_restore = [=](const DlRenderContext& ctx) {
      // Just a simple restore to allow peephole optimizations to occur
      ctx.canvas->Restore();
    };
    SkRect layer_bounds = kRenderBounds.makeInset(15, 15);
    RenderWith(testP, env, tolerance,
               CaseParameters(
                   "With prior save/clip/restore",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->save();
                     ctx.canvas->clipRect(clip, SkClipOp::kIntersect, false);
                     SkPaint p2;
                     ctx.canvas->drawRect(rect, p2);
                     p2.setBlendMode(SkBlendMode::kClear);
                     ctx.canvas->drawRect(rect, p2);
                     ctx.canvas->restore();
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->Save();
                     ctx.canvas->ClipRect(clip, ClipOp::kIntersect, false);
                     DlPaint p2;
                     ctx.canvas->DrawRect(rect, p2);
                     p2.setBlendMode(DlBlendMode::kClear);
                     ctx.canvas->DrawRect(rect, p2);
                     ctx.canvas->Restore();
                   }));
    RenderWith(testP, env, tolerance,
               CaseParameters(
                   "saveLayer no paint, no bounds",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->saveLayer(nullptr, nullptr);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->SaveLayer(nullptr, nullptr);
                   })
                   .with_restore(sk_safe_restore, dl_safe_restore, false));
    RenderWith(testP, env, tolerance,
               CaseParameters(
                   "saveLayer no paint, with bounds",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->saveLayer(layer_bounds, nullptr);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->SaveLayer(&layer_bounds, nullptr);
                   })
                   .with_restore(sk_safe_restore, dl_safe_restore, true));
    RenderWith(testP, env, tolerance,
               CaseParameters(
                   "saveLayer with alpha, no bounds",
                   [=](const SkSetupContext& ctx) {
                     SkPaint save_p;
                     save_p.setColor(ToSk(alpha_layer_color));
                     ctx.canvas->saveLayer(nullptr, &save_p);
                   },
                   [=](const DlSetupContext& ctx) {
                     DlPaint save_p;
                     save_p.setColor(alpha_layer_color);
                     ctx.canvas->SaveLayer(nullptr, &save_p);
                   })
                   .with_restore(sk_safe_restore, dl_safe_restore, true));
    RenderWith(testP, env, tolerance,
               CaseParameters(
                   "saveLayer with peephole alpha, no bounds",
                   [=](const SkSetupContext& ctx) {
                     SkPaint save_p;
                     save_p.setColor(ToSk(alpha_layer_color));
                     ctx.canvas->saveLayer(nullptr, &save_p);
                   },
                   [=](const DlSetupContext& ctx) {
                     DlPaint save_p;
                     save_p.setColor(alpha_layer_color);
                     ctx.canvas->SaveLayer(nullptr, &save_p);
                   })
                   .with_restore(sk_opt_restore, dl_opt_restore, true, true));
    RenderWith(testP, env, tolerance,
               CaseParameters(
                   "saveLayer with alpha and bounds",
                   [=](const SkSetupContext& ctx) {
                     SkPaint save_p;
                     save_p.setColor(ToSk(alpha_layer_color));
                     ctx.canvas->saveLayer(layer_bounds, &save_p);
                   },
                   [=](const DlSetupContext& ctx) {
                     DlPaint save_p;
                     save_p.setColor(alpha_layer_color);
                     ctx.canvas->SaveLayer(&layer_bounds, &save_p);
                   })
                   .with_restore(sk_safe_restore, dl_safe_restore, true));
    {
      // Being able to see a backdrop blur requires a non-default background
      // so we create a new environment for these tests that has a checkerboard
      // background that can be blurred by the backdrop filter. We also want
      // to avoid the rendered primitive from obscuring the blurred background
      // so we set an alpha value which works for all primitives except for
      // drawColor which can override the alpha with its color, but it now uses
      // a non-opaque color to avoid that problem.
      RenderEnvironment backdrop_env =
          RenderEnvironment::MakeN32(env.provider());
      SkSetup sk_backdrop_setup = [=](const SkSetupContext& ctx) {
        SkPaint setup_p;
        setup_p.setShader(MakeColorSource(ctx.image));
        ctx.canvas->drawPaint(setup_p);
      };
      DlSetup dl_backdrop_setup = [=](const DlSetupContext& ctx) {
        DlPaint setup_p;
        setup_p.setColorSource(MakeColorSource(ctx.image));
        ctx.canvas->DrawPaint(setup_p);
      };
      SkSetup sk_content_setup = [=](const SkSetupContext& ctx) {
        ctx.paint.setAlpha(ctx.paint.getAlpha() / 2);
      };
      DlSetup dl_content_setup = [=](const DlSetupContext& ctx) {
        ctx.paint.setAlpha(ctx.paint.getAlpha() / 2);
      };
      backdrop_env.init_ref(sk_backdrop_setup, testP.sk_renderer(),
                            dl_backdrop_setup, testP.dl_renderer(),
                            testP.imp_renderer());
      quickCompareToReference(backdrop_env, "backdrop");

      DlBlurImageFilter dl_backdrop(5, 5, DlTileMode::kDecal);
      auto sk_backdrop =
          SkImageFilters::Blur(5, 5, SkTileMode::kDecal, nullptr);
      RenderWith(testP, backdrop_env, tolerance,
                 CaseParameters(
                     "saveLayer with backdrop",
                     [=](const SkSetupContext& ctx) {
                       sk_backdrop_setup(ctx);
                       ctx.canvas->saveLayer(SkCanvas::SaveLayerRec(
                           nullptr, nullptr, sk_backdrop.get(),
                           SkTileMode::kDecal, nullptr, 0));
                       sk_content_setup(ctx);
                     },
                     [=](const DlSetupContext& ctx) {
                       dl_backdrop_setup(ctx);
                       ctx.canvas->SaveLayer(nullptr, nullptr, &dl_backdrop);
                       dl_content_setup(ctx);
                     })
                     .with_restore(sk_safe_restore, dl_safe_restore, true));
      RenderWith(testP, backdrop_env, tolerance,
                 CaseParameters(
                     "saveLayer with bounds and backdrop",
                     [=](const SkSetupContext& ctx) {
                       sk_backdrop_setup(ctx);
                       ctx.canvas->saveLayer(SkCanvas::SaveLayerRec(
                           &layer_bounds, nullptr, sk_backdrop.get(),
                           SkTileMode::kDecal, nullptr, 0));
                       sk_content_setup(ctx);
                     },
                     [=](const DlSetupContext& ctx) {
                       dl_backdrop_setup(ctx);
                       ctx.canvas->SaveLayer(&layer_bounds, nullptr,
                                             &dl_backdrop);
                       dl_content_setup(ctx);
                     })
                     .with_restore(sk_safe_restore, dl_safe_restore, true));
      RenderWith(testP, backdrop_env, tolerance,
                 CaseParameters(
                     "clipped saveLayer with backdrop",
                     [=](const SkSetupContext& ctx) {
                       sk_backdrop_setup(ctx);
                       ctx.canvas->clipRect(layer_bounds);
                       ctx.canvas->saveLayer(SkCanvas::SaveLayerRec(
                           nullptr, nullptr, sk_backdrop.get(),
                           SkTileMode::kDecal, nullptr, 0));
                       sk_content_setup(ctx);
                     },
                     [=](const DlSetupContext& ctx) {
                       dl_backdrop_setup(ctx);
                       ctx.canvas->ClipRect(layer_bounds);
                       ctx.canvas->SaveLayer(nullptr, nullptr, &dl_backdrop);
                       dl_content_setup(ctx);
                     })
                     .with_restore(sk_safe_restore, dl_safe_restore, true));
    }

    {
      // clang-format off
      constexpr float rotate_alpha_color_matrix[20] = {
          0, 1, 0,  0 , 0,
          0, 0, 1,  0 , 0,
          1, 0, 0,  0 , 0,
          0, 0, 0, 0.5, 0,
      };
      // clang-format on
      auto dl_alpha_rotate_filter =
          DlColorFilter::MakeMatrix(rotate_alpha_color_matrix);
      auto sk_alpha_rotate_filter =
          SkColorFilters::Matrix(rotate_alpha_color_matrix);
      {
        RenderWith(testP, env, tolerance,
                   CaseParameters(
                       "saveLayer ColorFilter, no bounds",
                       [=](const SkSetupContext& ctx) {
                         SkPaint save_p;
                         save_p.setColorFilter(sk_alpha_rotate_filter);
                         ctx.canvas->saveLayer(nullptr, &save_p);
                         ctx.paint.setStrokeWidth(5.0);
                       },
                       [=](const DlSetupContext& ctx) {
                         DlPaint save_p;
                         save_p.setColorFilter(dl_alpha_rotate_filter);
                         ctx.canvas->SaveLayer(nullptr, &save_p);
                         ctx.paint.setStrokeWidth(5.0);
                       })
                       .with_restore(sk_safe_restore, dl_safe_restore, true));
      }
      {
        RenderWith(testP, env, tolerance,
                   CaseParameters(
                       "saveLayer ColorFilter and bounds",
                       [=](const SkSetupContext& ctx) {
                         SkPaint save_p;
                         save_p.setColorFilter(sk_alpha_rotate_filter);
                         ctx.canvas->saveLayer(kRenderBounds, &save_p);
                         ctx.paint.setStrokeWidth(5.0);
                       },
                       [=](const DlSetupContext& ctx) {
                         DlPaint save_p;
                         save_p.setColorFilter(dl_alpha_rotate_filter);
                         ctx.canvas->SaveLayer(&kRenderBounds, &save_p);
                         ctx.paint.setStrokeWidth(5.0);
                       })
                       .with_restore(sk_safe_restore, dl_safe_restore, true));
      }
    }

    {
      // clang-format off
      constexpr float color_matrix[20] = {
          0.5, 0, 0, 0, 0.5,
          0, 0.5, 0, 0, 0.5,
          0, 0, 0.5, 0, 0.5,
          0, 0, 0, 1, 0,
      };
      // clang-format on
      auto dl_color_filter = DlColorFilter::MakeMatrix(color_matrix);
      auto dl_cf_image_filter = DlImageFilter::MakeColorFilter(dl_color_filter);
      auto sk_cf_image_filter = SkImageFilters::ColorFilter(
          SkColorFilters::Matrix(color_matrix), nullptr);
      {
        RenderWith(testP, env, tolerance,
                   CaseParameters(
                       "saveLayer ImageFilter, no bounds",
                       [=](const SkSetupContext& ctx) {
                         SkPaint save_p;
                         save_p.setImageFilter(sk_cf_image_filter);
                         ctx.canvas->saveLayer(nullptr, &save_p);
                         ctx.paint.setStrokeWidth(5.0);
                       },
                       [=](const DlSetupContext& ctx) {
                         DlPaint save_p;
                         save_p.setImageFilter(dl_cf_image_filter);
                         ctx.canvas->SaveLayer(nullptr, &save_p);
                         ctx.paint.setStrokeWidth(5.0);
                       })
                       .with_restore(sk_safe_restore, dl_safe_restore, true));
      }
      {
        RenderWith(testP, env, tolerance,
                   CaseParameters(
                       "saveLayer ImageFilter and bounds",
                       [=](const SkSetupContext& ctx) {
                         SkPaint save_p;
                         save_p.setImageFilter(sk_cf_image_filter);
                         ctx.canvas->saveLayer(kRenderBounds, &save_p);
                         ctx.paint.setStrokeWidth(5.0);
                       },
                       [=](const DlSetupContext& ctx) {
                         DlPaint save_p;
                         save_p.setImageFilter(dl_cf_image_filter);
                         ctx.canvas->SaveLayer(&kRenderBounds, &save_p);
                         ctx.paint.setStrokeWidth(5.0);
                       })
                       .with_restore(sk_safe_restore, dl_safe_restore, true));
      }
    }
  }

  static void RenderWithAttributes(const TestParameters& testP,
                                   const RenderEnvironment& env,
                                   const BoundsTolerance& tolerance) {
    RenderWith(testP, env, tolerance, CaseParameters("Defaults Test"));

    {
      // CPU renderer with default line width of 0 does not show antialiasing
      // for stroked primitives, so we make a new reference with a non-trivial
      // stroke width to demonstrate the differences
      RenderEnvironment aa_env = RenderEnvironment::MakeN32(env.provider());
      // Tweak the bounds tolerance for the displacement of 1/10 of a pixel
      const BoundsTolerance aa_tolerance = tolerance.addBoundsPadding(1, 1);
      auto sk_aa_setup = [=](SkSetupContext ctx, bool is_aa) {
        ctx.canvas->translate(0.1, 0.1);
        ctx.paint.setAntiAlias(is_aa);
        ctx.paint.setStrokeWidth(5.0);
      };
      auto dl_aa_setup = [=](DlSetupContext ctx, bool is_aa) {
        ctx.canvas->Translate(0.1, 0.1);
        ctx.paint.setAntiAlias(is_aa);
        ctx.paint.setStrokeWidth(5.0);
      };
      aa_env.init_ref(
          [=](const SkSetupContext& ctx) { sk_aa_setup(ctx, false); },
          testP.sk_renderer(),
          [=](const DlSetupContext& ctx) { dl_aa_setup(ctx, false); },
          testP.dl_renderer(), testP.imp_renderer());
      quickCompareToReference(aa_env, "AntiAlias");
      RenderWith(
          testP, aa_env, aa_tolerance,
          CaseParameters(
              "AntiAlias == True",
              [=](const SkSetupContext& ctx) { sk_aa_setup(ctx, true); },
              [=](const DlSetupContext& ctx) { dl_aa_setup(ctx, true); }));
      RenderWith(
          testP, aa_env, aa_tolerance,
          CaseParameters(
              "AntiAlias == False",
              [=](const SkSetupContext& ctx) { sk_aa_setup(ctx, false); },
              [=](const DlSetupContext& ctx) { dl_aa_setup(ctx, false); }));
    }

    RenderWith(  //
        testP, env, tolerance,
        CaseParameters(
            "Color == Blue",
            [=](const SkSetupContext& ctx) {
              ctx.paint.setColor(SK_ColorBLUE);
            },
            [=](const DlSetupContext& ctx) {
              ctx.paint.setColor(DlColor::kBlue());
            }));
    RenderWith(  //
        testP, env, tolerance,
        CaseParameters(
            "Color == Green",
            [=](const SkSetupContext& ctx) {
              ctx.paint.setColor(SK_ColorGREEN);
            },
            [=](const DlSetupContext& ctx) {
              ctx.paint.setColor(DlColor::kGreen());
            }));

    RenderWithStrokes(testP, env, tolerance);

    {
      // half opaque cyan
      DlColor blendable_color = DlColor::kCyan().withAlpha(0x7f);
      DlColor bg = DlColor::kWhite();

      RenderWith(testP, env, tolerance,
                 CaseParameters(
                     "Blend == SrcIn",
                     [=](const SkSetupContext& ctx) {
                       ctx.paint.setBlendMode(SkBlendMode::kSrcIn);
                       ctx.paint.setColor(blendable_color.argb());
                     },
                     [=](const DlSetupContext& ctx) {
                       ctx.paint.setBlendMode(DlBlendMode::kSrcIn);
                       ctx.paint.setColor(blendable_color);
                     })
                     .with_bg(bg));
      RenderWith(testP, env, tolerance,
                 CaseParameters(
                     "Blend == DstIn",
                     [=](const SkSetupContext& ctx) {
                       ctx.paint.setBlendMode(SkBlendMode::kDstIn);
                       ctx.paint.setColor(blendable_color.argb());
                     },
                     [=](const DlSetupContext& ctx) {
                       ctx.paint.setBlendMode(DlBlendMode::kDstIn);
                       ctx.paint.setColor(blendable_color);
                     })
                     .with_bg(bg));
    }

    {
      // Being able to see a blur requires some non-default attributes,
      // like a non-trivial stroke width and a shader rather than a color
      // (for drawPaint) so we create a new environment for these tests.
      RenderEnvironment blur_env = RenderEnvironment::MakeN32(env.provider());
      SkSetup sk_blur_setup = [=](const SkSetupContext& ctx) {
        ctx.paint.setShader(MakeColorSource(ctx.image));
        ctx.paint.setStrokeWidth(5.0);
      };
      DlSetup dl_blur_setup = [=](const DlSetupContext& ctx) {
        ctx.paint.setColorSource(MakeColorSource(ctx.image));
        ctx.paint.setStrokeWidth(5.0);
      };
      blur_env.init_ref(sk_blur_setup, testP.sk_renderer(),  //
                        dl_blur_setup, testP.dl_renderer(),
                        testP.imp_renderer());
      quickCompareToReference(blur_env, "blur");
      DlBlurImageFilter dl_filter_decal_5(5.0, 5.0, DlTileMode::kDecal);
      auto sk_filter_decal_5 =
          SkImageFilters::Blur(5.0, 5.0, SkTileMode::kDecal, nullptr);
      BoundsTolerance blur_5_tolerance = tolerance.addBoundsPadding(4, 4);
      {
        RenderWith(testP, blur_env, blur_5_tolerance,
                   CaseParameters(
                       "ImageFilter == Decal Blur 5",
                       [=](const SkSetupContext& ctx) {
                         sk_blur_setup(ctx);
                         ctx.paint.setImageFilter(sk_filter_decal_5);
                       },
                       [=](const DlSetupContext& ctx) {
                         dl_blur_setup(ctx);
                         ctx.paint.setImageFilter(&dl_filter_decal_5);
                       }));
      }
      DlBlurImageFilter dl_filter_clamp_5(5.0, 5.0, DlTileMode::kClamp);
      auto sk_filter_clamp_5 =
          SkImageFilters::Blur(5.0, 5.0, SkTileMode::kClamp, nullptr);
      {
        RenderWith(testP, blur_env, blur_5_tolerance,
                   CaseParameters(
                       "ImageFilter == Clamp Blur 5",
                       [=](const SkSetupContext& ctx) {
                         sk_blur_setup(ctx);
                         ctx.paint.setImageFilter(sk_filter_clamp_5);
                       },
                       [=](const DlSetupContext& ctx) {
                         dl_blur_setup(ctx);
                         ctx.paint.setImageFilter(&dl_filter_clamp_5);
                       }));
      }
    }

    {
      // Being able to see a dilate requires some non-default attributes,
      // like a non-trivial stroke width and a shader rather than a color
      // (for drawPaint) so we create a new environment for these tests.
      RenderEnvironment dilate_env = RenderEnvironment::MakeN32(env.provider());
      SkSetup sk_dilate_setup = [=](const SkSetupContext& ctx) {
        ctx.paint.setShader(MakeColorSource(ctx.image));
        ctx.paint.setStrokeWidth(5.0);
      };
      DlSetup dl_dilate_setup = [=](const DlSetupContext& ctx) {
        ctx.paint.setColorSource(MakeColorSource(ctx.image));
        ctx.paint.setStrokeWidth(5.0);
      };
      dilate_env.init_ref(sk_dilate_setup, testP.sk_renderer(),  //
                          dl_dilate_setup, testP.dl_renderer(),
                          testP.imp_renderer());
      quickCompareToReference(dilate_env, "dilate");
      DlDilateImageFilter dl_dilate_filter_5(5.0, 5.0);
      auto sk_dilate_filter_5 = SkImageFilters::Dilate(5.0, 5.0, nullptr);
      RenderWith(testP, dilate_env, tolerance,
                 CaseParameters(
                     "ImageFilter == Dilate 5",
                     [=](const SkSetupContext& ctx) {
                       sk_dilate_setup(ctx);
                       ctx.paint.setImageFilter(sk_dilate_filter_5);
                     },
                     [=](const DlSetupContext& ctx) {
                       dl_dilate_setup(ctx);
                       ctx.paint.setImageFilter(&dl_dilate_filter_5);
                     }));
    }

    {
      // Being able to see an erode requires some non-default attributes,
      // like a non-trivial stroke width and a shader rather than a color
      // (for drawPaint) so we create a new environment for these tests.
      RenderEnvironment erode_env = RenderEnvironment::MakeN32(env.provider());
      SkSetup sk_erode_setup = [=](const SkSetupContext& ctx) {
        ctx.paint.setShader(MakeColorSource(ctx.image));
        ctx.paint.setStrokeWidth(6.0);
      };
      DlSetup dl_erode_setup = [=](const DlSetupContext& ctx) {
        ctx.paint.setColorSource(MakeColorSource(ctx.image));
        ctx.paint.setStrokeWidth(6.0);
      };
      erode_env.init_ref(sk_erode_setup, testP.sk_renderer(),  //
                         dl_erode_setup, testP.dl_renderer(),
                         testP.imp_renderer());
      quickCompareToReference(erode_env, "erode");
      // do not erode too much, because some tests assert there are enough
      // pixels that are changed.
      DlErodeImageFilter dl_erode_filter_1(1.0, 1.0);
      auto sk_erode_filter_1 = SkImageFilters::Erode(1.0, 1.0, nullptr);
      RenderWith(testP, erode_env, tolerance,
                 CaseParameters(
                     "ImageFilter == Erode 1",
                     [=](const SkSetupContext& ctx) {
                       sk_erode_setup(ctx);
                       ctx.paint.setImageFilter(sk_erode_filter_1);
                     },
                     [=](const DlSetupContext& ctx) {
                       dl_erode_setup(ctx);
                       ctx.paint.setImageFilter(&dl_erode_filter_1);
                     }));
    }

    {
      // clang-format off
      constexpr float rotate_color_matrix[20] = {
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          1, 0, 0, 0, 0,
          0, 0, 0, 1, 0,
      };
      constexpr float invert_color_matrix[20] = {
        -1.0,    0,    0, 1.0,   0,
           0, -1.0,    0, 1.0,   0,
           0,    0, -1.0, 1.0,   0,
         1.0,  1.0,  1.0, 1.0,   0,
      };
      // clang-format on
      auto dl_color_filter = DlColorFilter::MakeMatrix(rotate_color_matrix);
      auto sk_color_filter = SkColorFilters::Matrix(rotate_color_matrix);
      {
        DlColor bg = DlColor::kWhite();
        RenderWith(testP, env, tolerance,
                   CaseParameters(
                       "ColorFilter == RotateRGB",
                       [=](const SkSetupContext& ctx) {
                         ctx.paint.setColor(SK_ColorYELLOW);
                         ctx.paint.setColorFilter(sk_color_filter);
                       },
                       [=](const DlSetupContext& ctx) {
                         ctx.paint.setColor(DlColor::kYellow());
                         ctx.paint.setColorFilter(dl_color_filter);
                       })
                       .with_bg(bg));
      }
      {
        DlColor bg = DlColor::kWhite();
        RenderWith(testP, env, tolerance,
                   CaseParameters(
                       "ColorFilter == Invert",
                       [=](const SkSetupContext& ctx) {
                         ctx.paint.setColor(SK_ColorYELLOW);
                         ctx.paint.setColorFilter(
                             SkColorFilters::Matrix(invert_color_matrix));
                       },
                       [=](const DlSetupContext& ctx) {
                         ctx.paint.setColor(DlColor::kYellow());
                         ctx.paint.setInvertColors(true);
                       })
                       .with_bg(bg));
      }
    }

    {
      const DlBlurMaskFilter dl_mask_filter(DlBlurStyle::kNormal, 5.0);
      auto sk_mask_filter = SkMaskFilter::MakeBlur(kNormal_SkBlurStyle, 5.0);
      BoundsTolerance blur_5_tolerance = tolerance.addBoundsPadding(4, 4);
      {
        // Stroked primitives need some non-trivial stroke size to be blurred
        RenderWith(testP, env, blur_5_tolerance,
                   CaseParameters(
                       "MaskFilter == Blur 5",
                       [=](const SkSetupContext& ctx) {
                         ctx.paint.setStrokeWidth(5.0);
                         ctx.paint.setMaskFilter(sk_mask_filter);
                       },
                       [=](const DlSetupContext& ctx) {
                         ctx.paint.setStrokeWidth(5.0);
                         ctx.paint.setMaskFilter(&dl_mask_filter);
                       }));
      }
    }

    {
      SkPoint sk_end_points[] = {
          SkPoint::Make(kRenderBounds.fLeft, kRenderBounds.fTop),
          SkPoint::Make(kRenderBounds.fRight, kRenderBounds.fBottom),
      };
      DlPoint dl_end_points[] = {
          DlPoint(kRenderBounds.fLeft, kRenderBounds.fTop),
          DlPoint(kRenderBounds.fRight, kRenderBounds.fBottom),
      };
      DlColor dl_colors[] = {
          DlColor::kGreen(),
          DlColor::kYellow().withAlpha(0x7f),
          DlColor::kBlue(),
      };
      SkColor sk_colors[] = {
          SK_ColorGREEN,
          SkColorSetA(SK_ColorYELLOW, 0x7f),
          SK_ColorBLUE,
      };
      float stops[] = {
          0.0,
          0.5,
          1.0,
      };
      auto dl_gradient =
          DlColorSource::MakeLinear(dl_end_points[0], dl_end_points[1], 3,
                                    dl_colors, stops, DlTileMode::kMirror);
      auto sk_gradient = SkGradientShader::MakeLinear(
          sk_end_points, sk_colors, stops, 3, SkTileMode::kMirror, 0, nullptr);
      {
        RenderWith(testP, env, tolerance,
                   CaseParameters(
                       "LinearGradient GYB",
                       [=](const SkSetupContext& ctx) {
                         ctx.paint.setShader(sk_gradient);
                         ctx.paint.setDither(testP.uses_gradient());
                       },
                       [=](const DlSetupContext& ctx) {
                         ctx.paint.setColorSource(dl_gradient);
                       }));
      }
    }
  }

  static void RenderWithStrokes(const TestParameters& testP,
                                const RenderEnvironment& env,
                                const BoundsTolerance& tolerance_in) {
    // The test cases were generated with geometry that will try to fill
    // out the various miter limits used for testing, but they can be off
    // by a couple of pixels so we will relax bounds testing for strokes by
    // a couple of pixels.
    BoundsTolerance tolerance = tolerance_in.addBoundsPadding(2, 2);
    RenderWith(testP, env, tolerance,
               CaseParameters(
                   "Fill",
                   [=](const SkSetupContext& ctx) {
                     ctx.paint.setStyle(SkPaint::kFill_Style);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.paint.setDrawStyle(DlDrawStyle::kFill);
                   }));
    // Skia on HW produces a strong miter consistent with width=1.0
    // for any width less than a pixel, but the bounds computations of
    // both DL and SkPicture do not account for this. We will get
    // OOB pixel errors for the highly mitered drawPath geometry if
    // we don't set stroke width to 1.0 for that test on HW.
    // See https://bugs.chromium.org/p/skia/issues/detail?id=14046
    bool no_hairlines =
        testP.is_draw_path() &&
        env.provider()->backend_type() != BackendType::kSoftwareBackend;
    RenderWith(testP, env, tolerance,
               CaseParameters(
                   "Stroke + defaults",
                   [=](const SkSetupContext& ctx) {
                     if (no_hairlines) {
                       ctx.paint.setStrokeWidth(1.0);
                     }
                     ctx.paint.setStyle(SkPaint::kStroke_Style);
                   },
                   [=](const DlSetupContext& ctx) {
                     if (no_hairlines) {
                       ctx.paint.setStrokeWidth(1.0);
                     }
                     ctx.paint.setDrawStyle(DlDrawStyle::kStroke);
                   }));

    RenderWith(testP, env, tolerance,
               CaseParameters(
                   "Fill + unnecessary StrokeWidth 10",
                   [=](const SkSetupContext& ctx) {
                     ctx.paint.setStyle(SkPaint::kFill_Style);
                     ctx.paint.setStrokeWidth(10.0);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.paint.setDrawStyle(DlDrawStyle::kFill);
                     ctx.paint.setStrokeWidth(10.0);
                   }));

    RenderEnvironment stroke_base_env =
        RenderEnvironment::MakeN32(env.provider());
    SkSetup sk_stroke_setup = [=](const SkSetupContext& ctx) {
      ctx.paint.setStyle(SkPaint::kStroke_Style);
      ctx.paint.setStrokeWidth(5.0);
    };
    DlSetup dl_stroke_setup = [=](const DlSetupContext& ctx) {
      ctx.paint.setDrawStyle(DlDrawStyle::kStroke);
      ctx.paint.setStrokeWidth(5.0);
    };
    stroke_base_env.init_ref(sk_stroke_setup, testP.sk_renderer(),
                             dl_stroke_setup, testP.dl_renderer(),
                             testP.imp_renderer());
    quickCompareToReference(stroke_base_env, "stroke");

    RenderWith(testP, stroke_base_env, tolerance,
               CaseParameters(
                   "Stroke Width 10",
                   [=](const SkSetupContext& ctx) {
                     ctx.paint.setStyle(SkPaint::kStroke_Style);
                     ctx.paint.setStrokeWidth(10.0);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.paint.setDrawStyle(DlDrawStyle::kStroke);
                     ctx.paint.setStrokeWidth(10.0);
                   }));
    RenderWith(testP, stroke_base_env, tolerance,
               CaseParameters(
                   "Stroke Width 5",
                   [=](const SkSetupContext& ctx) {
                     ctx.paint.setStyle(SkPaint::kStroke_Style);
                     ctx.paint.setStrokeWidth(5.0);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.paint.setDrawStyle(DlDrawStyle::kStroke);
                     ctx.paint.setStrokeWidth(5.0);
                   }));

    RenderWith(testP, stroke_base_env, tolerance,
               CaseParameters(
                   "Stroke Width 5, Square Cap",
                   [=](const SkSetupContext& ctx) {
                     ctx.paint.setStyle(SkPaint::kStroke_Style);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeCap(SkPaint::kSquare_Cap);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.paint.setDrawStyle(DlDrawStyle::kStroke);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeCap(DlStrokeCap::kSquare);
                   }));
    RenderWith(testP, stroke_base_env, tolerance,
               CaseParameters(
                   "Stroke Width 5, Round Cap",
                   [=](const SkSetupContext& ctx) {
                     ctx.paint.setStyle(SkPaint::kStroke_Style);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeCap(SkPaint::kRound_Cap);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.paint.setDrawStyle(DlDrawStyle::kStroke);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeCap(DlStrokeCap::kRound);
                   }));

    RenderWith(testP, stroke_base_env, tolerance,
               CaseParameters(
                   "Stroke Width 5, Bevel Join",
                   [=](const SkSetupContext& ctx) {
                     ctx.paint.setStyle(SkPaint::kStroke_Style);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeJoin(SkPaint::kBevel_Join);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.paint.setDrawStyle(DlDrawStyle::kStroke);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeJoin(DlStrokeJoin::kBevel);
                   }));
    RenderWith(testP, stroke_base_env, tolerance,
               CaseParameters(
                   "Stroke Width 5, Round Join",
                   [=](const SkSetupContext& ctx) {
                     ctx.paint.setStyle(SkPaint::kStroke_Style);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeJoin(SkPaint::kRound_Join);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.paint.setDrawStyle(DlDrawStyle::kStroke);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeJoin(DlStrokeJoin::kRound);
                   }));

    RenderWith(testP, stroke_base_env, tolerance,
               CaseParameters(
                   "Stroke Width 5, Miter 10",
                   [=](const SkSetupContext& ctx) {
                     ctx.paint.setStyle(SkPaint::kStroke_Style);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeMiter(10.0);
                     ctx.paint.setStrokeJoin(SkPaint::kMiter_Join);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.paint.setDrawStyle(DlDrawStyle::kStroke);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeMiter(10.0);
                     ctx.paint.setStrokeJoin(DlStrokeJoin::kMiter);
                   }));

    RenderWith(testP, stroke_base_env, tolerance,
               CaseParameters(
                   "Stroke Width 5, Miter 0",
                   [=](const SkSetupContext& ctx) {
                     ctx.paint.setStyle(SkPaint::kStroke_Style);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeMiter(0.0);
                     ctx.paint.setStrokeJoin(SkPaint::kMiter_Join);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.paint.setDrawStyle(DlDrawStyle::kStroke);
                     ctx.paint.setStrokeWidth(5.0);
                     ctx.paint.setStrokeMiter(0.0);
                     ctx.paint.setStrokeJoin(DlStrokeJoin::kMiter);
                   }));
  }

  static void RenderWithTransforms(const TestParameters& testP,
                                   const RenderEnvironment& env,
                                   const BoundsTolerance& tolerance) {
    // If the rendering method does not fill the corners of the original
    // bounds, then the estimate under rotation or skewing will be off
    // so we scale the padding by about 5% to compensate.
    BoundsTolerance skewed_tolerance = tolerance.mulScale(1.05, 1.05);
    RenderWith(  //
        testP, env, tolerance,
        CaseParameters(
            "Translate 5, 10",  //
            [=](const SkSetupContext& ctx) { ctx.canvas->translate(5, 10); },
            [=](const DlSetupContext& ctx) { ctx.canvas->Translate(5, 10); }));
    RenderWith(  //
        testP, env, tolerance,
        CaseParameters(
            "Scale +5%",  //
            [=](const SkSetupContext& ctx) { ctx.canvas->scale(1.05, 1.05); },
            [=](const DlSetupContext& ctx) { ctx.canvas->Scale(1.05, 1.05); }));
    RenderWith(  //
        testP, env, skewed_tolerance,
        CaseParameters(
            "Rotate 5 degrees",  //
            [=](const SkSetupContext& ctx) { ctx.canvas->rotate(5); },
            [=](const DlSetupContext& ctx) { ctx.canvas->Rotate(5); }));
    RenderWith(  //
        testP, env, skewed_tolerance,
        CaseParameters(
            "Skew 5%",  //
            [=](const SkSetupContext& ctx) { ctx.canvas->skew(0.05, 0.05); },
            [=](const DlSetupContext& ctx) { ctx.canvas->Skew(0.05, 0.05); }));
    {
      // This rather odd transform can cause slight differences in
      // computing in-bounds samples depending on which base rendering
      // routine Skia uses. Making sure our matrix values are powers
      // of 2 reduces, but does not eliminate, these slight differences
      // in calculation when we are comparing rendering with an alpha
      // to rendering opaque colors in the group opacity tests, for
      // example.
      SkScalar tweak = 1.0 / 16.0;
      SkMatrix tx = SkMatrix::MakeAll(1.0 + tweak, tweak, 5,   //
                                      tweak, 1.0 + tweak, 10,  //
                                      0, 0, 1);
      RenderWith(  //
          testP, env, skewed_tolerance,
          CaseParameters(
              "Transform 2D Affine",
              [=](const SkSetupContext& ctx) { ctx.canvas->concat(tx); },
              [=](const DlSetupContext& ctx) { ctx.canvas->Transform(tx); }));
    }
    {
      SkM44 m44 = SkM44(1, 0, 0, kRenderCenterX,  //
                        0, 1, 0, kRenderCenterY,  //
                        0, 0, 1, 0,               //
                        0, 0, .001, 1);
      m44.preConcat(
          SkM44::Rotate({1, 0, 0}, math::kPi / 60));  // 3 degrees around X
      m44.preConcat(
          SkM44::Rotate({0, 1, 0}, math::kPi / 45));  // 4 degrees around Y
      m44.preTranslate(-kRenderCenterX, -kRenderCenterY);
      RenderWith(  //
          testP, env, skewed_tolerance,
          CaseParameters(
              "Transform Full Perspective",
              [=](const SkSetupContext& ctx) { ctx.canvas->concat(m44); },
              [=](const DlSetupContext& ctx) { ctx.canvas->Transform(m44); }));
    }
  }

  static void RenderWithClips(const TestParameters& testP,
                              const RenderEnvironment& env,
                              const BoundsTolerance& diff_tolerance) {
    // We used to use an inset of 15.5 pixels here, but since Skia's rounding
    // behavior at the center of pixels does not match between HW and SW, we
    // ended up with some clips including different pixels between the two
    // destinations and this interacted poorly with the carefully chosen
    // geometry in some of the tests which was designed to have just the
    // right features fully filling the clips based on the SW rounding. By
    // moving to a 15.4 inset, the edge of the clip is never on the "rounding
    // edge" of a pixel.
    SkRect r_clip = kRenderBounds.makeInset(15.4, 15.4);
    BoundsTolerance intersect_tolerance = diff_tolerance.clip(r_clip);
    intersect_tolerance = intersect_tolerance.addPostClipPadding(1, 1);
    RenderWith(testP, env, intersect_tolerance,
               CaseParameters(
                   "Hard ClipRect inset by 15.4",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->clipRect(r_clip, SkClipOp::kIntersect, false);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->ClipRect(r_clip, ClipOp::kIntersect, false);
                   }));
    RenderWith(testP, env, intersect_tolerance,
               CaseParameters(
                   "AntiAlias ClipRect inset by 15.4",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->clipRect(r_clip, SkClipOp::kIntersect, true);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->ClipRect(r_clip, ClipOp::kIntersect, true);
                   }));
    RenderWith(testP, env, diff_tolerance,
               CaseParameters(
                   "Hard ClipRect Diff, inset by 15.4",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->clipRect(r_clip, SkClipOp::kDifference, false);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->ClipRect(r_clip, ClipOp::kDifference, false);
                   })
                   .with_diff_clip());
    // Skia lacks clipOval and requires us to make an oval SkRRect
    SkRRect rr_oval_clip = SkRRect::MakeOval(r_clip);
    RenderWith(testP, env, intersect_tolerance,
               CaseParameters(
                   "Hard ClipOval",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->clipRRect(rr_oval_clip, SkClipOp::kIntersect,
                                           false);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->ClipOval(r_clip, ClipOp::kIntersect, false);
                   }));
    RenderWith(testP, env, intersect_tolerance,
               CaseParameters(
                   "AntiAlias ClipOval",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->clipRRect(rr_oval_clip, SkClipOp::kIntersect,
                                           true);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->ClipOval(r_clip, ClipOp::kIntersect, true);
                   }));
    RenderWith(testP, env, diff_tolerance,
               CaseParameters(
                   "Hard ClipOval Diff",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->clipRRect(rr_oval_clip, SkClipOp::kDifference,
                                           false);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->ClipOval(r_clip, ClipOp::kDifference, false);
                   })
                   .with_diff_clip());
    // This test RR clip used to use very small radii, but due to
    // optimizations in the HW rrect rasterization, this caused small
    // bulges in the corners of the RRect which were interpreted as
    // "clip overruns" by the clip OOB pixel testing code. Using less
    // abusively small radii fixes the problem.
    SkRRect rr_clip = SkRRect::MakeRectXY(r_clip, 9, 9);
    RenderWith(testP, env, intersect_tolerance,
               CaseParameters(
                   "Hard ClipRRect with radius of 15.4",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->clipRRect(rr_clip, SkClipOp::kIntersect,
                                           false);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->ClipRRect(rr_clip, ClipOp::kIntersect, false);
                   }));
    RenderWith(testP, env, intersect_tolerance,
               CaseParameters(
                   "AntiAlias ClipRRect with radius of 15.4",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->clipRRect(rr_clip, SkClipOp::kIntersect, true);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->ClipRRect(rr_clip, ClipOp::kIntersect, true);
                   }));
    RenderWith(testP, env, diff_tolerance,
               CaseParameters(
                   "Hard ClipRRect Diff, with radius of 15.4",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->clipRRect(rr_clip, SkClipOp::kDifference,
                                           false);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->ClipRRect(rr_clip, ClipOp::kDifference, false);
                   })
                   .with_diff_clip());
    SkPath path_clip = SkPath();
    path_clip.setFillType(SkPathFillType::kEvenOdd);
    path_clip.addRect(r_clip);
    path_clip.addCircle(kRenderCenterX, kRenderCenterY, 1.0);
    RenderWith(testP, env, intersect_tolerance,
               CaseParameters(
                   "Hard ClipPath inset by 15.4",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->clipPath(path_clip, SkClipOp::kIntersect,
                                          false);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->ClipPath(path_clip, ClipOp::kIntersect, false);
                   }));
    RenderWith(testP, env, intersect_tolerance,
               CaseParameters(
                   "AntiAlias ClipPath inset by 15.4",
                   [=](const SkSetupContext& ctx) {
                     ctx.canvas->clipPath(path_clip, SkClipOp::kIntersect,
                                          true);
                   },
                   [=](const DlSetupContext& ctx) {
                     ctx.canvas->ClipPath(path_clip, ClipOp::kIntersect, true);
                   }));
    RenderWith(
        testP, env, diff_tolerance,
        CaseParameters(
            "Hard ClipPath Diff, inset by 15.4",
            [=](const SkSetupContext& ctx) {
              ctx.canvas->clipPath(path_clip, SkClipOp::kDifference, false);
            },
            [=](const DlSetupContext& ctx) {
              ctx.canvas->ClipPath(path_clip, ClipOp::kDifference, false);
            })
            .with_diff_clip());
  }

  enum class DirectoryStatus {
    kExisted,
    kCreated,
    kFailed,
  };

  static DirectoryStatus CheckDir(const std::string& dir) {
    auto ret =
        fml::OpenDirectory(dir.c_str(), false, fml::FilePermission::kRead);
    if (ret.is_valid()) {
      return DirectoryStatus::kExisted;
    }
    ret =
        fml::OpenDirectory(dir.c_str(), true, fml::FilePermission::kReadWrite);
    if (ret.is_valid()) {
      return DirectoryStatus::kCreated;
    }
    FML_LOG(ERROR) << "Could not create directory (" << dir
                   << ") for impeller failure images" << ", ret = " << ret.get()
                   << ", errno = " << errno;
    return DirectoryStatus::kFailed;
  }

  static void SetupImpellerFailureImageDirectory() {
    std::string base_dir = "./impeller_failure_images";
    if (CheckDir(base_dir) == DirectoryStatus::kFailed) {
      return;
    }
    for (int i = 0; i < 10000; i++) {
      std::string sub_dir = std::to_string(i);
      while (sub_dir.length() < 4) {
        sub_dir = "0" + sub_dir;
      }
      std::string try_dir = base_dir + "/" + sub_dir;
      switch (CheckDir(try_dir)) {
        case DirectoryStatus::kExisted:
          break;
        case DirectoryStatus::kCreated:
          ImpellerFailureImageDirectory = try_dir;
          return;
        case DirectoryStatus::kFailed:
          return;
      }
    }
    FML_LOG(ERROR) << "Too many output directories for Impeller failure images";
  }

  static void save_to_png(const RenderResult* result,
                          const std::string& op_desc,
                          const std::string& reason) {
    if (!SaveImpellerFailureImages) {
      return;
    }
    if (ImpellerFailureImageDirectory.length() == 0) {
      SetupImpellerFailureImageDirectory();
      if (ImpellerFailureImageDirectory.length() == 0) {
        SaveImpellerFailureImages = false;
        return;
      }
    }

    std::string filename = ImpellerFailureImageDirectory + "/";
    for (const char& ch : op_desc) {
      filename += (ch == ':' || ch == ' ') ? '_' : ch;
    }
    filename = filename + ".png";
    result->write(filename);
    ImpellerFailureImages.push_back(filename);
    FML_LOG(ERROR) << reason << ": " << filename;
  }

  static void RenderWith(const TestParameters& testP,
                         const RenderEnvironment& env,
                         const BoundsTolerance& tolerance_in,
                         const CaseParameters& caseP) {
    std::string test_name =
        ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::string info =
        env.backend_name() + ": " + test_name + " (" + caseP.info() + ")";
    const DlColor bg = caseP.bg();
    RenderJobInfo base_info = {
        .bg = bg,
    };

    // sk_result is a direct rendering via SkCanvas to SkSurface
    // DisplayList mechanisms are not involved in this operation
    SkJobRenderer sk_job(caseP.sk_setup(),     //
                         testP.sk_renderer(),  //
                         caseP.sk_restore(),   //
                         env.sk_image());
    auto sk_result = env.getResult(base_info, sk_job);

    DlJobRenderer dl_job(caseP.dl_setup(),     //
                         testP.dl_renderer(),  //
                         caseP.dl_restore(),   //
                         env.dl_image());
    auto dl_result = env.getResult(base_info, dl_job);

    EXPECT_EQ(sk_job.setup_matrix(), dl_job.setup_matrix());
    EXPECT_EQ(sk_job.setup_clip_bounds(), dl_job.setup_clip_bounds());
    ASSERT_EQ(sk_result->width(), kTestWidth) << info;
    ASSERT_EQ(sk_result->height(), kTestHeight) << info;
    ASSERT_EQ(dl_result->width(), kTestWidth) << info;
    ASSERT_EQ(dl_result->height(), kTestHeight) << info;

    const BoundsTolerance tolerance =
        testP.adjust(tolerance_in, dl_job.setup_paint(), dl_job.setup_matrix());
    const sk_sp<SkPicture> sk_picture = sk_job.MakePicture(base_info);
    const sk_sp<DisplayList> display_list = dl_job.MakeDisplayList(base_info);

    SkRect sk_bounds = sk_picture->cullRect();
    checkPixels(sk_result.get(), sk_bounds, info + " (Skia reference)", bg);

    if (testP.should_match(env, caseP, dl_job.setup_paint(), dl_job)) {
      quickCompareToReference(env.ref_sk_result(), sk_result.get(), true,
                              info + " (attribute should not have effect)");
    } else {
      quickCompareToReference(env.ref_sk_result(), sk_result.get(), false,
                              info + " (attribute should affect rendering)");
    }

    // If either the reference setup or the test setup contain attributes
    // that Impeller doesn't support, we skip the Impeller testing. This
    // is mostly stroked or patterned text which is vectored through drawPath
    // for Impeller.
    if (env.supports_impeller() &&
        testP.impeller_compatible(dl_job.setup_paint()) &&
        testP.impeller_compatible(env.ref_dl_paint())) {
      DlJobRenderer imp_job(caseP.dl_setup(),      //
                            testP.imp_renderer(),  //
                            caseP.dl_restore(),    //
                            env.impeller_image());
      auto imp_result = env.getImpellerResult(base_info, imp_job);
      std::string imp_info = info + " (Impeller)";
      bool success = checkPixels(imp_result.get(), imp_result->render_bounds(),
                                 imp_info, bg);
      if (testP.should_match(env, caseP, imp_job.setup_paint(), imp_job)) {
        success = success &&                //
                  quickCompareToReference(  //
                      env.ref_impeller_result(), imp_result.get(), true,
                      imp_info + " (attribute should not have effect)");
      } else {
        success = success &&                //
                  quickCompareToReference(  //
                      env.ref_impeller_result(), imp_result.get(), false,
                      imp_info + " (attribute should affect rendering)");
      }
      if (SaveImpellerFailureImages && !success) {
        FML_LOG(ERROR) << "Impeller issue encountered for: "
                       << *imp_job.MakeDisplayList(base_info);
        save_to_png(imp_result.get(), info + " (Impeller Result)",
                    "output saved in");
        save_to_png(env.ref_impeller_result(), info + " (Impeller Reference)",
                    "compare to reference without attributes");
        save_to_png(sk_result.get(), info + " (Skia Result)",
                    "and to Skia reference with attributes");
        save_to_png(env.ref_sk_result(), info + " (Skia Reference)",
                    "and to Skia reference without attributes");
      }
    }

    quickCompareToReference(sk_result.get(), dl_result.get(), true,
                            info + " (DlCanvas output matches SkCanvas)");

    {
      SkRect dl_bounds = display_list->bounds();
      if (!sk_bounds.roundOut().contains(dl_bounds)) {
        FML_LOG(ERROR) << "For " << info;
        FML_LOG(ERROR) << "sk ref: "  //
                       << sk_bounds.fLeft << ", " << sk_bounds.fTop << " => "
                       << sk_bounds.fRight << ", " << sk_bounds.fBottom;
        FML_LOG(ERROR) << "dl: "  //
                       << dl_bounds.fLeft << ", " << dl_bounds.fTop << " => "
                       << dl_bounds.fRight << ", " << dl_bounds.fBottom;
        if (!dl_bounds.contains(sk_bounds)) {
          FML_LOG(ERROR) << "DisplayList bounds are too small!";
        }
        if (!dl_bounds.isEmpty() &&
            !sk_bounds.roundOut().contains(dl_bounds.roundOut())) {
          FML_LOG(ERROR) << "###### DisplayList bounds larger than reference!";
        }
      }

      // This EXPECT sometimes triggers, but when it triggers and I examine
      // the ref_bounds, they are always unnecessarily large and since the
      // pixel OOB tests in the compare method do not trigger, we will trust
      // the DL bounds.
      // EXPECT_TRUE(dl_bounds.contains(ref_bounds)) << info;

      // When we are drawing a DisplayList, the display_list built above
      // will contain just a single drawDisplayList call plus the case
      // attribute. The sk_picture will, however, contain a list of all
      // of the embedded calls in the display list and so the op counts
      // will not be equal between the two.
      if (!testP.is_draw_display_list()) {
        EXPECT_EQ(static_cast<int>(display_list->op_count()),
                  sk_picture->approximateOpCount())
            << info;
        EXPECT_EQ(static_cast<int>(display_list->op_count()),
                  sk_picture->approximateOpCount())
            << info;
      }

      DisplayListJobRenderer dl_builder_job(display_list);
      auto dl_builder_result = env.getResult(base_info, dl_builder_job);
      if (caseP.fuzzy_compare_components()) {
        compareToReference(
            dl_builder_result.get(), dl_result.get(),
            info + " (DlCanvas DL output close to Builder Dl output)",
            &dl_bounds, &tolerance, bg, true);
      } else {
        quickCompareToReference(
            dl_builder_result.get(), dl_result.get(), true,
            info + " (DlCanvas DL output matches Builder Dl output)");
      }

      compareToReference(dl_result.get(), sk_result.get(),
                         info + " (DisplayList built directly -> surface)",
                         &dl_bounds, &tolerance, bg,
                         caseP.fuzzy_compare_components());

      if (display_list->can_apply_group_opacity()) {
        checkGroupOpacity(env, display_list, dl_result.get(),
                          info + " with Group Opacity", bg);
      }
    }

    {
      // This sequence uses an SkPicture generated previously from the SkCanvas
      // calls and a DisplayList generated previously from the DlCanvas calls
      // and renders both back under a transform (scale(2x)) to see if their
      // rendering is affected differently by a change of matrix between
      // recording time and rendering time.
      const int test_width_2 = kTestWidth * 2;
      const int test_height_2 = kTestHeight * 2;
      const SkScalar test_scale = 2.0;

      SkPictureJobRenderer sk_job_x2(sk_picture);
      RenderJobInfo info_2x = {
          .width = test_width_2,
          .height = test_height_2,
          .bg = bg,
          .scale = test_scale,
      };
      auto ref_x2_result = env.getResult(info_2x, sk_job_x2);
      ASSERT_EQ(ref_x2_result->width(), test_width_2) << info;
      ASSERT_EQ(ref_x2_result->height(), test_height_2) << info;

      DisplayListJobRenderer dl_job_x2(display_list);
      auto test_x2_result = env.getResult(info_2x, dl_job_x2);
      compareToReference(test_x2_result.get(), ref_x2_result.get(),
                         info + " (Both rendered scaled 2x)", nullptr, nullptr,
                         bg, caseP.fuzzy_compare_components(),  //
                         test_width_2, test_height_2, false);
    }
  }

  static bool fuzzyCompare(uint32_t pixel_a, uint32_t pixel_b, int fudge) {
    for (int i = 0; i < 32; i += 8) {
      int comp_a = (pixel_a >> i) & 0xff;
      int comp_b = (pixel_b >> i) & 0xff;
      if (std::abs(comp_a - comp_b) > fudge) {
        return false;
      }
    }
    return true;
  }

  static int groupOpacityFudgeFactor(const RenderEnvironment& env) {
    if (env.format() == PixelFormat::k565PixelFormat) {
      return 9;
    }
    if (env.provider()->backend_type() == BackendType::kOpenGlBackend) {
      // OpenGL gets a little fuzzy at times. Still, "within 5" (aka +/-4)
      // for byte samples is not bad, though the other backends give +/-1
      return 5;
    }
    return 2;
  }
  static void checkGroupOpacity(const RenderEnvironment& env,
                                const sk_sp<DisplayList>& display_list,
                                const RenderResult* ref_result,
                                const std::string& info,
                                DlColor bg) {
    SkScalar opacity = 128.0 / 255.0;

    DisplayListJobRenderer opacity_job(display_list);
    RenderJobInfo opacity_info = {
        .bg = bg,
        .opacity = opacity,
    };
    auto group_opacity_result = env.getResult(opacity_info, opacity_job);

    ASSERT_EQ(group_opacity_result->width(), kTestWidth) << info;
    ASSERT_EQ(group_opacity_result->height(), kTestHeight) << info;

    ASSERT_EQ(ref_result->width(), kTestWidth) << info;
    ASSERT_EQ(ref_result->height(), kTestHeight) << info;

    int pixels_touched = 0;
    int pixels_different = 0;
    int max_diff = 0;
    // We need to allow some slight differences per component due to the
    // fact that rearranging discrete calculations can compound round off
    // errors. Off-by-2 is enough for 8 bit components, but for the 565
    // tests we allow at least 9 which is the maximum distance between
    // samples when converted to 8 bits. (You might think it would be a
    // max step of 8 converting 5 bits to 8 bits, but it is really
    // converting 31 steps to 255 steps with an average step size of
    // 8.23 - 24 of the steps are by 8, but 7 of them are by 9.)
    int fudge = groupOpacityFudgeFactor(env);
    for (int y = 0; y < kTestHeight; y++) {
      const uint32_t* ref_row = ref_result->addr32(0, y);
      const uint32_t* test_row = group_opacity_result->addr32(0, y);
      for (int x = 0; x < kTestWidth; x++) {
        uint32_t ref_pixel = ref_row[x];
        uint32_t test_pixel = test_row[x];
        if (ref_pixel != bg.argb() || test_pixel != bg.argb()) {
          pixels_touched++;
          for (int i = 0; i < 32; i += 8) {
            int ref_comp = (ref_pixel >> i) & 0xff;
            int bg_comp = (bg.argb() >> i) & 0xff;
            SkScalar faded_comp = bg_comp + (ref_comp - bg_comp) * opacity;
            int test_comp = (test_pixel >> i) & 0xff;
            if (std::abs(faded_comp - test_comp) > fudge) {
              int diff = std::abs(faded_comp - test_comp);
              if (max_diff < diff) {
                max_diff = diff;
              }
              pixels_different++;
              break;
            }
          }
        }
      }
    }
    ASSERT_GT(pixels_touched, 20) << info;
    if (pixels_different > 1) {
      FML_LOG(ERROR) << "max diff == " << max_diff << " for " << info;
    }
    ASSERT_LE(pixels_different, 1) << info;
  }

  static bool checkPixels(const RenderResult* ref_result,
                          const SkRect ref_bounds,
                          const std::string& info,
                          const DlColor bg = DlColor::kTransparent()) {
    uint32_t untouched = PremultipliedArgb(bg);
    int pixels_touched = 0;
    int pixels_oob = 0;
    SkIRect i_bounds = ref_bounds.roundOut();
    EXPECT_EQ(ref_result->width(), kTestWidth) << info;
    EXPECT_EQ(ref_result->height(), kTestWidth) << info;
    for (int y = 0; y < kTestHeight; y++) {
      const uint32_t* ref_row = ref_result->addr32(0, y);
      for (int x = 0; x < kTestWidth; x++) {
        if (ref_row[x] != untouched) {
          pixels_touched++;
          if (!i_bounds.contains(x, y)) {
            pixels_oob++;
          }
        }
      }
    }
    EXPECT_EQ(pixels_oob, 0) << info;
    EXPECT_GT(pixels_touched, 0) << info;
    return pixels_oob == 0 && pixels_touched > 0;
  }

  static int countModifiedTransparentPixels(const RenderResult* ref_result,
                                            const RenderResult* test_result) {
    int count = 0;
    for (int y = 0; y < kTestHeight; y++) {
      const uint32_t* ref_row = ref_result->addr32(0, y);
      const uint32_t* test_row = test_result->addr32(0, y);
      for (int x = 0; x < kTestWidth; x++) {
        if (ref_row[x] != test_row[x]) {
          if (ref_row[x] == 0) {
            count++;
          }
        }
      }
    }
    return count;
  }

  static void quickCompareToReference(const RenderEnvironment& env,
                                      const std::string& info) {
    quickCompareToReference(env.ref_sk_result(), env.ref_dl_result(), true,
                            info + " reference rendering");
  }

  static bool quickCompareToReference(const RenderResult* ref_result,
                                      const RenderResult* test_result,
                                      bool should_match,
                                      const std::string& info) {
    int w = test_result->width();
    int h = test_result->height();
    EXPECT_EQ(w, ref_result->width()) << info;
    EXPECT_EQ(h, ref_result->height()) << info;
    int pixels_different = 0;
    for (int y = 0; y < h; y++) {
      const uint32_t* ref_row = ref_result->addr32(0, y);
      const uint32_t* test_row = test_result->addr32(0, y);
      for (int x = 0; x < w; x++) {
        if (ref_row[x] != test_row[x]) {
          if (should_match && pixels_different < 5) {
            FML_LOG(ERROR) << std::hex << ref_row[x] << " != " << test_row[x];
          }
          pixels_different++;
        }
      }
    }
    if (should_match) {
      EXPECT_EQ(pixels_different, 0) << info;
      return pixels_different == 0;
    } else {
      EXPECT_NE(pixels_different, 0) << info;
      return pixels_different != 0;
    }
  }

  static void compareToReference(const RenderResult* test_result,
                                 const RenderResult* ref_result,
                                 const std::string& info,
                                 const SkRect* bounds,
                                 const BoundsTolerance* tolerance,
                                 const DlColor bg,
                                 bool fuzzyCompares = false,
                                 int width = kTestWidth,
                                 int height = kTestHeight,
                                 bool printMismatches = false) {
    uint32_t untouched = PremultipliedArgb(bg);
    ASSERT_EQ(test_result->width(), width) << info;
    ASSERT_EQ(test_result->height(), height) << info;
    SkIRect i_bounds =
        bounds ? bounds->roundOut() : SkIRect::MakeWH(width, height);

    int pixels_different = 0;
    int pixels_oob = 0;
    int min_x = width;
    int min_y = height;
    int max_x = 0;
    int max_y = 0;
    for (int y = 0; y < height; y++) {
      const uint32_t* ref_row = ref_result->addr32(0, y);
      const uint32_t* test_row = test_result->addr32(0, y);
      for (int x = 0; x < width; x++) {
        if (bounds && test_row[x] != untouched) {
          if (min_x > x) {
            min_x = x;
          }
          if (min_y > y) {
            min_y = y;
          }
          if (max_x <= x) {
            max_x = x + 1;
          }
          if (max_y <= y) {
            max_y = y + 1;
          }
          if (!i_bounds.contains(x, y)) {
            pixels_oob++;
          }
        }
        bool match = fuzzyCompares ? fuzzyCompare(test_row[x], ref_row[x], 1)
                                   : test_row[x] == ref_row[x];
        if (!match) {
          if (printMismatches && pixels_different < 5) {
            FML_LOG(ERROR) << "pix[" << x << ", " << y
                           << "] mismatch: " << std::hex << test_row[x]
                           << "(test) != (ref)" << ref_row[x] << std::dec;
          }
          pixels_different++;
        }
      }
    }
    if (pixels_oob > 0) {
      FML_LOG(ERROR) << "pix bounds["  //
                     << min_x << ", " << min_y << " => " << max_x << ", "
                     << max_y << "]";
      FML_LOG(ERROR) << "dl_bounds["                               //
                     << bounds->fLeft << ", " << bounds->fTop      //
                     << " => "                                     //
                     << bounds->fRight << ", " << bounds->fBottom  //
                     << "]";
    } else if (bounds) {
      showBoundsOverflow(info, i_bounds, tolerance, min_x, min_y, max_x, max_y);
    }
    ASSERT_EQ(pixels_oob, 0) << info;
    ASSERT_EQ(pixels_different, 0) << info;
  }

  static void showBoundsOverflow(const std::string& info,
                                 SkIRect& bounds,
                                 const BoundsTolerance* tolerance,
                                 int pixLeft,
                                 int pixTop,
                                 int pixRight,
                                 int pixBottom) {
    int pad_left = std::max(0, pixLeft - bounds.fLeft);
    int pad_top = std::max(0, pixTop - bounds.fTop);
    int pad_right = std::max(0, bounds.fRight - pixRight);
    int pad_bottom = std::max(0, bounds.fBottom - pixBottom);
    SkIRect pix_bounds =
        SkIRect::MakeLTRB(pixLeft, pixTop, pixRight, pixBottom);
    SkISize pix_size = pix_bounds.size();
    int pix_width = pix_size.width();
    int pix_height = pix_size.height();
    int worst_pad_x = std::max(pad_left, pad_right);
    int worst_pad_y = std::max(pad_top, pad_bottom);
    if (tolerance->overflows(pix_bounds, worst_pad_x, worst_pad_y)) {
      FML_LOG(ERROR) << "Computed bounds for " << info;
      FML_LOG(ERROR) << "pix bounds["                        //
                     << pixLeft << ", " << pixTop << " => "  //
                     << pixRight << ", " << pixBottom        //
                     << "]";
      FML_LOG(ERROR) << "dl_bounds["                             //
                     << bounds.fLeft << ", " << bounds.fTop      //
                     << " => "                                   //
                     << bounds.fRight << ", " << bounds.fBottom  //
                     << "]";
      FML_LOG(ERROR) << "Bounds overly conservative by up to "     //
                     << worst_pad_x << ", " << worst_pad_y         //
                     << " (" << (worst_pad_x * 100.0 / pix_width)  //
                     << "%, " << (worst_pad_y * 100.0 / pix_height) << "%)";
      int pix_area = pix_size.area();
      int dl_area = bounds.width() * bounds.height();
      FML_LOG(ERROR) << "Total overflow area: " << (dl_area - pix_area)  //
                     << " (+" << (dl_area * 100.0 / pix_area - 100.0)    //
                     << "% larger)";
      FML_LOG(ERROR);
    }
  }

  static sk_sp<SkTextBlob> MakeTextBlob(const std::string& string,
                                        SkScalar font_height) {
    SkFont font = CreateTestFontOfSize(font_height);
    sk_sp<SkTypeface> face = font.refTypeface();
    FML_CHECK(face);
    FML_CHECK(face->countGlyphs() > 0) << "No glyphs in font";
    return SkTextBlob::MakeFromText(string.c_str(), string.size(), font,
                                    SkTextEncoding::kUTF8);
  }
};

std::vector<BackendType> CanvasCompareTester::TestBackends;
std::string CanvasCompareTester::ImpellerFailureImageDirectory = "";
bool CanvasCompareTester::SaveImpellerFailureImages = false;
std::vector<std::string> CanvasCompareTester::ImpellerFailureImages;
bool CanvasCompareTester::ImpellerSupported = false;
bool RenderEnvironment::EnableImpeller = true;

BoundsTolerance CanvasCompareTester::DefaultTolerance =
    BoundsTolerance().addAbsolutePadding(1, 1);

// Eventually this bare bones testing::Test fixture will subsume the
// CanvasCompareTester and the TestParameters could then become just
// configuration calls made upon the fixture.
template <typename BaseT>
class DisplayListRenderingTestBase : public BaseT,
                                     protected DisplayListOpFlags {
 public:
  DisplayListRenderingTestBase() = default;

  static bool StartsWith(std::string str, std::string prefix) {
    if (prefix.length() > str.length()) {
      return false;
    }
    for (size_t i = 0; i < prefix.length(); i++) {
      if (str[i] != prefix[i]) {
        return false;
      }
    }
    return true;
  }

  static void SetUpTestSuite() {
    bool do_software = true;
    bool do_opengl = false;
    bool do_metal = false;
    std::vector<std::string> args = ::testing::internal::GetArgvs();
    for (auto p_arg = std::next(args.begin()); p_arg != args.end(); p_arg++) {
      std::string arg = *p_arg;
      bool enable = true;
      if (arg == "--save-impeller-failures") {
        CanvasCompareTester::SaveImpellerFailureImages = true;
        continue;
      }
      if (StartsWith(arg, "--no")) {
        enable = false;
        arg = "-" + arg.substr(4);
      } else if (StartsWith(arg, "--disable")) {
        enable = false;
        arg = "--en" + arg.substr(5);
      }
      if (arg == "--enable-software") {
        do_software = enable;
      } else if (arg == "--enable-opengl" || arg == "--enable-gl") {
        do_opengl = enable;
      } else if (arg == "--enable-metal") {
        do_metal = enable;
      } else if (arg == "--enable-impeller") {
        RenderEnvironment::EnableImpeller = enable;
      }
    }
    // Multiple test suites use this test base. Make sure that they don't
    // double-register the supported providers.
    CanvasCompareTester::ClearProviders();
    if (do_software) {
      CanvasCompareTester::AddProvider(BackendType::kSoftwareBackend);
    }
    if (do_opengl) {
      CanvasCompareTester::AddProvider(BackendType::kOpenGlBackend);
    }
    if (do_metal) {
      CanvasCompareTester::AddProvider(BackendType::kMetalBackend);
    }
    std::string providers = "";
    for (auto& back_end : CanvasCompareTester::TestBackends) {
      providers += " " + DlSurfaceProvider::BackendName(back_end);
    }
    std::string libraries = " Skia";
    if (CanvasCompareTester::ImpellerSupported &&
        RenderEnvironment::EnableImpeller) {
      libraries += " Impeller";
    }
    FML_LOG(INFO) << "Running tests on [" << providers  //
                  << " ], and [" << libraries << " ]";
  }

  static void TearDownTestSuite() {
    if (CanvasCompareTester::ImpellerFailureImages.size() > 0) {
      FML_LOG(INFO);
      FML_LOG(INFO) << CanvasCompareTester::ImpellerFailureImages.size()
                    << " images saved in "
                    << CanvasCompareTester::ImpellerFailureImageDirectory;
      for (const auto& filename : CanvasCompareTester::ImpellerFailureImages) {
        FML_LOG(INFO) << "  " << filename;
      }
      FML_LOG(INFO);
    }
  }

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(DisplayListRenderingTestBase);
};
using DisplayListRendering = DisplayListRenderingTestBase<::testing::Test>;

TEST_F(DisplayListRendering, DrawPaint) {
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            ctx.canvas->drawPaint(ctx.paint);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawPaint(ctx.paint);
          },
          kDrawPaintFlags));
}

TEST_F(DisplayListRendering, DrawOpaqueColor) {
  // We use a non-opaque color to avoid obliterating any backdrop filter output
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            // DrawColor is not tested against attributes because it is supposed
            // to ignore them. So, if the paint has an alpha, it is because we
            // are doing a saveLayer+backdrop test and we need to not flood over
            // the backdrop output with a solid color. So, we perform an alpha
            // drawColor for that case only.
            SkColor color = SkColorSetA(SK_ColorMAGENTA, ctx.paint.getAlpha());
            ctx.canvas->drawColor(color);
          },
          [=](const DlRenderContext& ctx) {
            // DrawColor is not tested against attributes because it is supposed
            // to ignore them. So, if the paint has an alpha, it is because we
            // are doing a saveLayer+backdrop test and we need to not flood over
            // the backdrop output with a solid color. So, we transfer the alpha
            // from the paint for that case only.
            ctx.canvas->DrawColor(
                DlColor::kMagenta().withAlpha(ctx.paint.getAlpha()));
          },
          kDrawColorFlags));
}

TEST_F(DisplayListRendering, DrawAlphaColor) {
  // We use a non-opaque color to avoid obliterating any backdrop filter output
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawColor(0x7FFF00FF);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawColor(DlColor(0x7FFF00FF));
          },
          kDrawColorFlags));
}

TEST_F(DisplayListRendering, DrawDiagonalLines) {
  SkPoint p1 = SkPoint::Make(kRenderLeft, kRenderTop);
  SkPoint p2 = SkPoint::Make(kRenderRight, kRenderBottom);
  SkPoint p3 = SkPoint::Make(kRenderLeft, kRenderBottom);
  SkPoint p4 = SkPoint::Make(kRenderRight, kRenderTop);
  // Adding some edge to edge diagonals that run through the points about
  // 16 units in from the center of that edge.
  // Adding some edge center to edge center diagonals to better fill
  // out the RRect Clip so bounds checking sees less empty bounds space.
  SkPoint p5 = SkPoint::Make(kRenderCenterX, kRenderTop + 15);
  SkPoint p6 = SkPoint::Make(kRenderRight - 15, kRenderCenterY);
  SkPoint p7 = SkPoint::Make(kRenderCenterX, kRenderBottom - 15);
  SkPoint p8 = SkPoint::Make(kRenderLeft + 15, kRenderCenterY);

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            // Skia requires kStroke style on horizontal and vertical
            // lines to get the bounds correct.
            // See https://bugs.chromium.org/p/skia/issues/detail?id=12446
            SkPaint p = ctx.paint;
            p.setStyle(SkPaint::kStroke_Style);
            ctx.canvas->drawLine(p1, p2, p);
            ctx.canvas->drawLine(p3, p4, p);
            ctx.canvas->drawLine(p5, p6, p);
            ctx.canvas->drawLine(p7, p8, p);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawLine(p1, p2, ctx.paint);
            ctx.canvas->DrawLine(p3, p4, ctx.paint);
            ctx.canvas->DrawLine(p5, p6, ctx.paint);
            ctx.canvas->DrawLine(p7, p8, ctx.paint);
          },
          kDrawLineFlags)
          .set_draw_line());
}

TEST_F(DisplayListRendering, DrawHorizontalLines) {
  SkPoint p1 = SkPoint::Make(kRenderLeft, kRenderTop + 16);
  SkPoint p2 = SkPoint::Make(kRenderRight, kRenderTop + 16);
  SkPoint p3 = SkPoint::Make(kRenderLeft, kRenderCenterY);
  SkPoint p4 = SkPoint::Make(kRenderRight, kRenderCenterY);
  SkPoint p5 = SkPoint::Make(kRenderLeft, kRenderBottom - 16);
  SkPoint p6 = SkPoint::Make(kRenderRight, kRenderBottom - 16);

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            // Skia requires kStroke style on horizontal and vertical
            // lines to get the bounds correct.
            // See https://bugs.chromium.org/p/skia/issues/detail?id=12446
            SkPaint p = ctx.paint;
            p.setStyle(SkPaint::kStroke_Style);
            ctx.canvas->drawLine(p1, p2, p);
            ctx.canvas->drawLine(p3, p4, p);
            ctx.canvas->drawLine(p5, p6, p);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawLine(p1, p2, ctx.paint);
            ctx.canvas->DrawLine(p3, p4, ctx.paint);
            ctx.canvas->DrawLine(p5, p6, ctx.paint);
          },
          kDrawHVLineFlags)
          .set_draw_line()
          .set_horizontal_line());
}

TEST_F(DisplayListRendering, DrawVerticalLines) {
  SkPoint p1 = SkPoint::Make(kRenderLeft + 16, kRenderTop);
  SkPoint p2 = SkPoint::Make(kRenderLeft + 16, kRenderBottom);
  SkPoint p3 = SkPoint::Make(kRenderCenterX, kRenderTop);
  SkPoint p4 = SkPoint::Make(kRenderCenterX, kRenderBottom);
  SkPoint p5 = SkPoint::Make(kRenderRight - 16, kRenderTop);
  SkPoint p6 = SkPoint::Make(kRenderRight - 16, kRenderBottom);

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            // Skia requires kStroke style on horizontal and vertical
            // lines to get the bounds correct.
            // See https://bugs.chromium.org/p/skia/issues/detail?id=12446
            SkPaint p = ctx.paint;
            p.setStyle(SkPaint::kStroke_Style);
            ctx.canvas->drawLine(p1, p2, p);
            ctx.canvas->drawLine(p3, p4, p);
            ctx.canvas->drawLine(p5, p6, p);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawLine(p1, p2, ctx.paint);
            ctx.canvas->DrawLine(p3, p4, ctx.paint);
            ctx.canvas->DrawLine(p5, p6, ctx.paint);
          },
          kDrawHVLineFlags)
          .set_draw_line()
          .set_vertical_line());
}

TEST_F(DisplayListRendering, DrawDiagonalDashedLines) {
  SkPoint p1 = SkPoint::Make(kRenderLeft, kRenderTop);
  SkPoint p2 = SkPoint::Make(kRenderRight, kRenderBottom);
  SkPoint p3 = SkPoint::Make(kRenderLeft, kRenderBottom);
  SkPoint p4 = SkPoint::Make(kRenderRight, kRenderTop);
  // Adding some edge to edge diagonals that run through the points about
  // 16 units in from the center of that edge.
  // Adding some edge center to edge center diagonals to better fill
  // out the RRect Clip so bounds checking sees less empty bounds space.
  SkPoint p5 = SkPoint::Make(kRenderCenterX, kRenderTop + 15);
  SkPoint p6 = SkPoint::Make(kRenderRight - 15, kRenderCenterY);
  SkPoint p7 = SkPoint::Make(kRenderCenterX, kRenderBottom - 15);
  SkPoint p8 = SkPoint::Make(kRenderLeft + 15, kRenderCenterY);

  // Full diagonals are 100x100 which are 140 in length
  // Dashing them with 25 on, 5 off means that the last
  // dash goes from 120 to 145 which means both ends of the
  // diagonals will be in an "on" dash for maximum bounds

  // Edge to edge diagonals are 50x50 which are 70 in length
  // Dashing them with 25 on, 5 off means that the last
  // dash goes from 60 to 85 which means both ends of the
  // edge diagonals will be in a dash segment

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            // Skia requires kStroke style on horizontal and vertical
            // lines to get the bounds correct.
            // See https://bugs.chromium.org/p/skia/issues/detail?id=12446
            SkPaint p = ctx.paint;
            p.setStyle(SkPaint::kStroke_Style);
            SkScalar intervals[2] = {25.0f, 5.0f};
            p.setPathEffect(SkDashPathEffect::Make(intervals, 2, 0.0f));
            ctx.canvas->drawLine(p1, p2, p);
            ctx.canvas->drawLine(p3, p4, p);
            ctx.canvas->drawLine(p5, p6, p);
            ctx.canvas->drawLine(p7, p8, p);
          },
          [=](const DlRenderContext& ctx) {                           //
            ctx.canvas->DrawDashedLine(ToDlPoint(p1), ToDlPoint(p2),  //
                                       25.0f, 5.0f, ctx.paint);
            ctx.canvas->DrawDashedLine(ToDlPoint(p3), ToDlPoint(p4),  //
                                       25.0f, 5.0f, ctx.paint);
            ctx.canvas->DrawDashedLine(ToDlPoint(p5), ToDlPoint(p6),  //
                                       25.0f, 5.0f, ctx.paint);
            ctx.canvas->DrawDashedLine(ToDlPoint(p7), ToDlPoint(p8),  //
                                       25.0f, 5.0f, ctx.paint);
          },
          kDrawLineFlags)
          .set_draw_line());
}

TEST_F(DisplayListRendering, DrawRect) {
  // Bounds are offset by 0.5 pixels to induce AA
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            ctx.canvas->drawRect(kRenderBounds.makeOffset(0.5, 0.5), ctx.paint);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawRect(kRenderBounds.makeOffset(0.5, 0.5), ctx.paint);
          },
          kDrawRectFlags));
}

TEST_F(DisplayListRendering, DrawOval) {
  SkRect rect = kRenderBounds.makeInset(0, 10);

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            ctx.canvas->drawOval(rect, ctx.paint);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawOval(rect, ctx.paint);
          },
          kDrawOvalFlags));
}

TEST_F(DisplayListRendering, DrawCircle) {
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            ctx.canvas->drawCircle(kTestCenter, kRenderRadius, ctx.paint);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawCircle(kTestCenter, kRenderRadius, ctx.paint);
          },
          kDrawCircleFlags));
}

TEST_F(DisplayListRendering, DrawRRect) {
  SkRRect rrect = SkRRect::MakeRectXY(kRenderBounds, kRenderCornerRadius,
                                      kRenderCornerRadius);
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            ctx.canvas->drawRRect(rrect, ctx.paint);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawRRect(rrect, ctx.paint);
          },
          kDrawRRectFlags));
}

TEST_F(DisplayListRendering, DrawDRRect) {
  SkRRect outer = SkRRect::MakeRectXY(kRenderBounds, kRenderCornerRadius,
                                      kRenderCornerRadius);
  SkRect inner_bounds = kRenderBounds.makeInset(30.0, 30.0);
  SkRRect inner = SkRRect::MakeRectXY(inner_bounds, kRenderCornerRadius,
                                      kRenderCornerRadius);
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            ctx.canvas->drawDRRect(outer, inner, ctx.paint);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawDRRect(outer, inner, ctx.paint);
          },
          kDrawDRRectFlags));
}

TEST_F(DisplayListRendering, DrawPath) {
  SkPath path;

  // unclosed lines to show some caps
  path.moveTo(kRenderLeft + 15, kRenderTop + 15);
  path.lineTo(kRenderRight - 15, kRenderBottom - 15);
  path.moveTo(kRenderLeft + 15, kRenderBottom - 15);
  path.lineTo(kRenderRight - 15, kRenderTop + 15);

  path.addRect(kRenderBounds);

  // miter diamonds horizontally and vertically to show miters
  path.moveTo(kVerticalMiterDiamondPoints[0]);
  for (int i = 1; i < kVerticalMiterDiamondPointCount; i++) {
    path.lineTo(kVerticalMiterDiamondPoints[i]);
  }
  path.close();
  path.moveTo(kHorizontalMiterDiamondPoints[0]);
  for (int i = 1; i < kHorizontalMiterDiamondPointCount; i++) {
    path.lineTo(kHorizontalMiterDiamondPoints[i]);
  }
  path.close();

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            ctx.canvas->drawPath(path, ctx.paint);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawPath(path, ctx.paint);
          },
          kDrawPathFlags)
          .set_draw_path());
}

TEST_F(DisplayListRendering, DrawArc) {
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            ctx.canvas->drawArc(kRenderBounds, 60, 330, false, ctx.paint);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawArc(kRenderBounds, 60, 330, false, ctx.paint);
          },
          kDrawArcNoCenterFlags));
}

TEST_F(DisplayListRendering, DrawArcCenter) {
  // Center arcs that inscribe nearly a whole circle except for a small
  // arc extent gap have 2 angles that may appear or disappear at the
  // various miter limits tested (0, 4, and 10).
  // The center angle here is 12 degrees which shows a miter
  // at limit=10, but not 0 or 4.
  // The arcs at the corners where it turns in towards the
  // center show miters at 4 and 10, but not 0.
  // Limit == 0, neither corner does a miter
  // Limit == 4, only the edge "turn-in" corners miter
  // Limit == 10, edge and center corners all miter
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            ctx.canvas->drawArc(kRenderBounds, 60, 360 - 12, true, ctx.paint);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawArc(kRenderBounds, 60, 360 - 12, true, ctx.paint);
          },
          kDrawArcWithCenterFlags)
          .set_draw_arc_center());
}

TEST_F(DisplayListRendering, DrawPointsAsPoints) {
  // The +/- 16 points are designed to fall just inside the clips
  // that are tested against so we avoid lots of undrawn pixels
  // in the accumulated bounds.
  const SkScalar x0 = kRenderLeft;
  const SkScalar x1 = kRenderLeft + 16;
  const SkScalar x2 = (kRenderLeft + kRenderCenterX) * 0.5;
  const SkScalar x3 = kRenderCenterX + 0.1;
  const SkScalar x4 = (kRenderRight + kRenderCenterX) * 0.5;
  const SkScalar x5 = kRenderRight - 16;
  const SkScalar x6 = kRenderRight - 1;

  const SkScalar y0 = kRenderTop;
  const SkScalar y1 = kRenderTop + 16;
  const SkScalar y2 = (kRenderTop + kRenderCenterY) * 0.5;
  const SkScalar y3 = kRenderCenterY + 0.1;
  const SkScalar y4 = (kRenderBottom + kRenderCenterY) * 0.5;
  const SkScalar y5 = kRenderBottom - 16;
  const SkScalar y6 = kRenderBottom - 1;

  // clang-format off
  const SkPoint points[] = {
      {x0, y0}, {x1, y0}, {x2, y0}, {x3, y0}, {x4, y0}, {x5, y0}, {x6, y0},
      {x0, y1}, {x1, y1}, {x2, y1}, {x3, y1}, {x4, y1}, {x5, y1}, {x6, y1},
      {x0, y2}, {x1, y2}, {x2, y2}, {x3, y2}, {x4, y2}, {x5, y2}, {x6, y2},
      {x0, y3}, {x1, y3}, {x2, y3}, {x3, y3}, {x4, y3}, {x5, y3}, {x6, y3},
      {x0, y4}, {x1, y4}, {x2, y4}, {x3, y4}, {x4, y4}, {x5, y4}, {x6, y4},
      {x0, y5}, {x1, y5}, {x2, y5}, {x3, y5}, {x4, y5}, {x5, y5}, {x6, y5},
      {x0, y6}, {x1, y6}, {x2, y6}, {x3, y6}, {x4, y6}, {x5, y6}, {x6, y6},
  };
  // clang-format on
  const int count = sizeof(points) / sizeof(points[0]);

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            // Skia requires kStroke style on horizontal and vertical
            // lines to get the bounds correct.
            // See https://bugs.chromium.org/p/skia/issues/detail?id=12446
            SkPaint p = ctx.paint;
            p.setStyle(SkPaint::kStroke_Style);
            auto mode = SkCanvas::kPoints_PointMode;
            ctx.canvas->drawPoints(mode, count, points, p);
          },
          [=](const DlRenderContext& ctx) {
            auto mode = PointMode::kPoints;
            ctx.canvas->DrawPoints(mode, count, points, ctx.paint);
          },
          kDrawPointsAsPointsFlags)
          .set_draw_line()
          .set_ignores_dashes());
}

TEST_F(DisplayListRendering, DrawPointsAsLines) {
  const SkScalar x0 = kRenderLeft + 1;
  const SkScalar x1 = kRenderLeft + 16;
  const SkScalar x2 = kRenderRight - 16;
  const SkScalar x3 = kRenderRight - 1;

  const SkScalar y0 = kRenderTop;
  const SkScalar y1 = kRenderTop + 16;
  const SkScalar y2 = kRenderBottom - 16;
  const SkScalar y3 = kRenderBottom - 1;

  // clang-format off
  const SkPoint points[] = {
      // Outer box
      {x0, y0}, {x3, y0},
      {x3, y0}, {x3, y3},
      {x3, y3}, {x0, y3},
      {x0, y3}, {x0, y0},

      // Diagonals
      {x0, y0}, {x3, y3}, {x3, y0}, {x0, y3},

      // Inner box
      {x1, y1}, {x2, y1},
      {x2, y1}, {x2, y2},
      {x2, y2}, {x1, y2},
      {x1, y2}, {x1, y1},
  };
  // clang-format on

  const int count = sizeof(points) / sizeof(points[0]);
  ASSERT_TRUE((count & 1) == 0);
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            // Skia requires kStroke style on horizontal and vertical
            // lines to get the bounds correct.
            // See https://bugs.chromium.org/p/skia/issues/detail?id=12446
            SkPaint p = ctx.paint;
            p.setStyle(SkPaint::kStroke_Style);
            auto mode = SkCanvas::kLines_PointMode;
            ctx.canvas->drawPoints(mode, count, points, p);
          },
          [=](const DlRenderContext& ctx) {
            auto mode = PointMode::kLines;
            ctx.canvas->DrawPoints(mode, count, points, ctx.paint);
          },
          kDrawPointsAsLinesFlags));
}

TEST_F(DisplayListRendering, DrawPointsAsPolygon) {
  const SkPoint points1[] = {
      // RenderBounds box with a diamond
      SkPoint::Make(kRenderLeft, kRenderTop),
      SkPoint::Make(kRenderRight, kRenderTop),
      SkPoint::Make(kRenderRight, kRenderBottom),
      SkPoint::Make(kRenderLeft, kRenderBottom),
      SkPoint::Make(kRenderLeft, kRenderTop),

      SkPoint::Make(kRenderCenterX, kRenderTop + 15),
      SkPoint::Make(kRenderRight - 15, kRenderCenterY),
      SkPoint::Make(kRenderCenterX, kRenderBottom - 15),
      SkPoint::Make(kRenderLeft + 15, kRenderCenterY),
      SkPoint::Make(kRenderCenterX, kRenderTop + 15),
  };
  const int count1 = sizeof(points1) / sizeof(points1[0]);

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            // Skia requires kStroke style on horizontal and vertical
            // lines to get the bounds correct.
            // See https://bugs.chromium.org/p/skia/issues/detail?id=12446
            SkPaint p = ctx.paint;
            p.setStyle(SkPaint::kStroke_Style);
            auto mode = SkCanvas::kPolygon_PointMode;
            ctx.canvas->drawPoints(mode, count1, points1, p);
          },
          [=](const DlRenderContext& ctx) {
            auto mode = PointMode::kPolygon;
            ctx.canvas->DrawPoints(mode, count1, points1, ctx.paint);
          },
          kDrawPointsAsPolygonFlags));
}

TEST_F(DisplayListRendering, DrawVerticesWithColors) {
  // Cover as many sides of the box with only 6 vertices:
  // +----------+
  // |xxxxxxxxxx|
  // |    xxxxxx|
  // |       xxx|
  // |xxx       |
  // |xxxxxx    |
  // |xxxxxxxxxx|
  // +----------|
  const SkPoint pts[6] = {
      // Upper-Right corner, full top, half right coverage
      SkPoint::Make(kRenderLeft, kRenderTop),
      SkPoint::Make(kRenderRight, kRenderTop),
      SkPoint::Make(kRenderRight, kRenderCenterY),
      // Lower-Left corner, full bottom, half left coverage
      SkPoint::Make(kRenderLeft, kRenderBottom),
      SkPoint::Make(kRenderLeft, kRenderCenterY),
      SkPoint::Make(kRenderRight, kRenderBottom),
  };
  const DlColor dl_colors[6] = {
      DlColor::kRed(),  DlColor::kBlue(),   DlColor::kGreen(),
      DlColor::kCyan(), DlColor::kYellow(), DlColor::kMagenta(),
  };
  const SkColor sk_colors[6] = {
      SK_ColorRED,  SK_ColorBLUE,   SK_ColorGREEN,
      SK_ColorCYAN, SK_ColorYELLOW, SK_ColorMAGENTA,
  };
  const std::shared_ptr<DlVertices> dl_vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 6, ToDlPoints(pts), nullptr, dl_colors);
  const auto sk_vertices =
      SkVertices::MakeCopy(SkVertices::VertexMode::kTriangles_VertexMode, 6,
                           pts, nullptr, sk_colors);

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawVertices(sk_vertices, SkBlendMode::kSrcOver,
                                     ctx.paint);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawVertices(dl_vertices, DlBlendMode::kSrcOver,
                                     ctx.paint);
          },
          kDrawVerticesFlags));
}

TEST_F(DisplayListRendering, DrawVerticesWithImage) {
  // Cover as many sides of the box with only 6 vertices:
  // +----------+
  // |xxxxxxxxxx|
  // |    xxxxxx|
  // |       xxx|
  // |xxx       |
  // |xxxxxx    |
  // |xxxxxxxxxx|
  // +----------|
  const SkPoint pts[6] = {
      // Upper-Right corner, full top, half right coverage
      SkPoint::Make(kRenderLeft, kRenderTop),
      SkPoint::Make(kRenderRight, kRenderTop),
      SkPoint::Make(kRenderRight, kRenderCenterY),
      // Lower-Left corner, full bottom, half left coverage
      SkPoint::Make(kRenderLeft, kRenderBottom),
      SkPoint::Make(kRenderLeft, kRenderCenterY),
      SkPoint::Make(kRenderRight, kRenderBottom),
  };
  const SkPoint tex[6] = {
      SkPoint::Make(kRenderWidth / 2.0, 0),
      SkPoint::Make(0, kRenderHeight),
      SkPoint::Make(kRenderWidth, kRenderHeight),
      SkPoint::Make(kRenderWidth / 2, kRenderHeight),
      SkPoint::Make(0, 0),
      SkPoint::Make(kRenderWidth, 0),
  };
  const std::shared_ptr<DlVertices> dl_vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 6, ToDlPoints(pts), ToDlPoints(tex), nullptr);
  const auto sk_vertices = SkVertices::MakeCopy(
      SkVertices::VertexMode::kTriangles_VertexMode, 6, pts, tex, nullptr);

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            SkPaint v_paint = ctx.paint;
            if (v_paint.getShader() == nullptr) {
              v_paint.setShader(MakeColorSource(ctx.image));
            }
            ctx.canvas->drawVertices(sk_vertices, SkBlendMode::kSrcOver,
                                     v_paint);
          },
          [=](const DlRenderContext& ctx) {  //
            DlPaint v_paint = ctx.paint;
            if (v_paint.getColorSource() == nullptr) {
              v_paint.setColorSource(MakeColorSource(ctx.image));
            }
            ctx.canvas->DrawVertices(dl_vertices, DlBlendMode::kSrcOver,
                                     v_paint);
          },
          kDrawVerticesFlags));
}

TEST_F(DisplayListRendering, DrawImageNearest) {
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawImage(ctx.image, kRenderLeft, kRenderTop,
                                  SkImageSampling::kNearestNeighbor,
                                  &ctx.paint);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawImage(ctx.image,  //
                                  SkPoint::Make(kRenderLeft, kRenderTop),
                                  DlImageSampling::kNearestNeighbor,
                                  &ctx.paint);
          },
          kDrawImageWithPaintFlags));
}

TEST_F(DisplayListRendering, DrawImageNearestNoPaint) {
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawImage(ctx.image, kRenderLeft, kRenderTop,
                                  SkImageSampling::kNearestNeighbor, nullptr);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawImage(ctx.image,
                                  SkPoint::Make(kRenderLeft, kRenderTop),
                                  DlImageSampling::kNearestNeighbor, nullptr);
          },
          kDrawImageFlags));
}

TEST_F(DisplayListRendering, DrawImageLinear) {
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawImage(ctx.image, kRenderLeft, kRenderTop,
                                  SkImageSampling::kLinear, &ctx.paint);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawImage(ctx.image,
                                  SkPoint::Make(kRenderLeft, kRenderTop),
                                  DlImageSampling::kLinear, &ctx.paint);
          },
          kDrawImageWithPaintFlags));
}

TEST_F(DisplayListRendering, DrawImageRectNearest) {
  SkRect src = SkRect::MakeIWH(kRenderWidth, kRenderHeight).makeInset(5, 5);
  SkRect dst = kRenderBounds.makeInset(10.5, 10.5);
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawImageRect(
                ctx.image, src, dst, SkImageSampling::kNearestNeighbor,
                &ctx.paint, SkCanvas::kFast_SrcRectConstraint);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawImageRect(
                ctx.image, src, dst, DlImageSampling::kNearestNeighbor,
                &ctx.paint, DlCanvas::SrcRectConstraint::kFast);
          },
          kDrawImageRectWithPaintFlags));
}

TEST_F(DisplayListRendering, DrawImageRectNearestNoPaint) {
  SkRect src = SkRect::MakeIWH(kRenderWidth, kRenderHeight).makeInset(5, 5);
  SkRect dst = kRenderBounds.makeInset(10.5, 10.5);
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawImageRect(
                ctx.image, src, dst, SkImageSampling::kNearestNeighbor,  //
                nullptr, SkCanvas::kFast_SrcRectConstraint);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawImageRect(
                ctx.image, src, dst, DlImageSampling::kNearestNeighbor,  //
                nullptr, DlCanvas::SrcRectConstraint::kFast);
          },
          kDrawImageRectFlags));
}

TEST_F(DisplayListRendering, DrawImageRectLinear) {
  SkRect src = SkRect::MakeIWH(kRenderWidth, kRenderHeight).makeInset(5, 5);
  SkRect dst = kRenderBounds.makeInset(10.5, 10.5);
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawImageRect(
                ctx.image, src, dst, SkImageSampling::kLinear,  //
                &ctx.paint, SkCanvas::kFast_SrcRectConstraint);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawImageRect(
                ctx.image, src, dst, DlImageSampling::kLinear,  //
                &ctx.paint, DlCanvas::SrcRectConstraint::kFast);
          },
          kDrawImageRectWithPaintFlags));
}

TEST_F(DisplayListRendering, DrawImageNineNearest) {
  SkIRect src = SkIRect::MakeWH(kRenderWidth, kRenderHeight).makeInset(25, 25);
  SkRect dst = kRenderBounds.makeInset(10.5, 10.5);
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawImageNine(ctx.image.get(), src, dst,
                                      SkFilterMode::kNearest, &ctx.paint);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawImageNine(ctx.image, src, dst,
                                      DlFilterMode::kNearest, &ctx.paint);
          },
          kDrawImageNineWithPaintFlags));
}

TEST_F(DisplayListRendering, DrawImageNineNearestNoPaint) {
  SkIRect src = SkIRect::MakeWH(kRenderWidth, kRenderHeight).makeInset(25, 25);
  SkRect dst = kRenderBounds.makeInset(10.5, 10.5);
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawImageNine(ctx.image.get(), src, dst,
                                      SkFilterMode::kNearest, nullptr);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawImageNine(ctx.image, src, dst,
                                      DlFilterMode::kNearest, nullptr);
          },
          kDrawImageNineFlags));
}

TEST_F(DisplayListRendering, DrawImageNineLinear) {
  SkIRect src = SkIRect::MakeWH(kRenderWidth, kRenderHeight).makeInset(25, 25);
  SkRect dst = kRenderBounds.makeInset(10.5, 10.5);
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawImageNine(ctx.image.get(), src, dst,
                                      SkFilterMode::kLinear, &ctx.paint);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawImageNine(ctx.image, src, dst,
                                      DlFilterMode::kLinear, &ctx.paint);
          },
          kDrawImageNineWithPaintFlags));
}

TEST_F(DisplayListRendering, DrawAtlasNearest) {
  const SkRSXform xform[] = {
      // clang-format off
      { 1.2f,  0.0f, kRenderLeft,  kRenderTop},
      { 0.0f,  1.2f, kRenderRight, kRenderTop},
      {-1.2f,  0.0f, kRenderRight, kRenderBottom},
      { 0.0f, -1.2f, kRenderLeft,  kRenderBottom},
      // clang-format on
  };
  const SkRect tex[] = {
      // clang-format off
      {0,                0,                 kRenderHalfWidth, kRenderHalfHeight},
      {kRenderHalfWidth, 0,                 kRenderWidth,     kRenderHalfHeight},
      {kRenderHalfWidth, kRenderHalfHeight, kRenderWidth,     kRenderHeight},
      {0,                kRenderHalfHeight, kRenderHalfWidth, kRenderHeight},
      // clang-format on
  };
  const SkColor sk_colors[] = {
      SK_ColorBLUE,
      SK_ColorGREEN,
      SK_ColorYELLOW,
      SK_ColorMAGENTA,
  };
  const DlColor dl_colors[] = {
      DlColor::kBlue(),
      DlColor::kGreen(),
      DlColor::kYellow(),
      DlColor::kMagenta(),
  };
  const DlImageSampling dl_sampling = DlImageSampling::kNearestNeighbor;
  const SkSamplingOptions sk_sampling = SkImageSampling::kNearestNeighbor;
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawAtlas(ctx.image.get(), xform, tex, sk_colors, 4,
                                  SkBlendMode::kSrcOver, sk_sampling, nullptr,
                                  &ctx.paint);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawAtlas(ctx.image, xform, tex, dl_colors, 4,
                                  DlBlendMode::kSrcOver, dl_sampling, nullptr,
                                  &ctx.paint);
          },
          kDrawAtlasWithPaintFlags));
}

TEST_F(DisplayListRendering, DrawAtlasNearestNoPaint) {
  const SkRSXform xform[] = {
      // clang-format off
      { 1.2f,  0.0f, kRenderLeft,  kRenderTop},
      { 0.0f,  1.2f, kRenderRight, kRenderTop},
      {-1.2f,  0.0f, kRenderRight, kRenderBottom},
      { 0.0f, -1.2f, kRenderLeft,  kRenderBottom},
      // clang-format on
  };
  const SkRect tex[] = {
      // clang-format off
      {0,                0,                 kRenderHalfWidth, kRenderHalfHeight},
      {kRenderHalfWidth, 0,                 kRenderWidth,     kRenderHalfHeight},
      {kRenderHalfWidth, kRenderHalfHeight, kRenderWidth,     kRenderHeight},
      {0,                kRenderHalfHeight, kRenderHalfWidth, kRenderHeight},
      // clang-format on
  };
  const SkColor sk_colors[] = {
      SK_ColorBLUE,
      SK_ColorGREEN,
      SK_ColorYELLOW,
      SK_ColorMAGENTA,
  };
  const DlColor dl_colors[] = {
      DlColor::kBlue(),
      DlColor::kGreen(),
      DlColor::kYellow(),
      DlColor::kMagenta(),
  };
  const DlImageSampling dl_sampling = DlImageSampling::kNearestNeighbor;
  const SkSamplingOptions sk_sampling = SkImageSampling::kNearestNeighbor;
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawAtlas(ctx.image.get(), xform, tex, sk_colors, 4,
                                  SkBlendMode::kSrcOver, sk_sampling,  //
                                  nullptr, nullptr);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawAtlas(ctx.image, xform, tex, dl_colors, 4,
                                  DlBlendMode::kSrcOver, dl_sampling,  //
                                  nullptr, nullptr);
          },
          kDrawAtlasFlags));
}

TEST_F(DisplayListRendering, DrawAtlasLinear) {
  const SkRSXform xform[] = {
      // clang-format off
      { 1.2f,  0.0f, kRenderLeft,  kRenderTop},
      { 0.0f,  1.2f, kRenderRight, kRenderTop},
      {-1.2f,  0.0f, kRenderRight, kRenderBottom},
      { 0.0f, -1.2f, kRenderLeft,  kRenderBottom},
      // clang-format on
  };
  const SkRect tex[] = {
      // clang-format off
      {0,                0,                 kRenderHalfWidth, kRenderHalfHeight},
      {kRenderHalfWidth, 0,                 kRenderWidth,     kRenderHalfHeight},
      {kRenderHalfWidth, kRenderHalfHeight, kRenderWidth,     kRenderHeight},
      {0,                kRenderHalfHeight, kRenderHalfWidth, kRenderHeight},
      // clang-format on
  };
  const SkColor sk_colors[] = {
      SK_ColorBLUE,
      SK_ColorGREEN,
      SK_ColorYELLOW,
      SK_ColorMAGENTA,
  };
  const DlColor dl_colors[] = {
      DlColor::kBlue(),
      DlColor::kGreen(),
      DlColor::kYellow(),
      DlColor::kMagenta(),
  };
  const DlImageSampling dl_sampling = DlImageSampling::kLinear;
  const SkSamplingOptions sk_sampling = SkImageSampling::kLinear;
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            ctx.canvas->drawAtlas(ctx.image.get(), xform, tex, sk_colors, 2,
                                  SkBlendMode::kSrcOver, sk_sampling,  //
                                  nullptr, &ctx.paint);
          },
          [=](const DlRenderContext& ctx) {
            ctx.canvas->DrawAtlas(ctx.image, xform, tex, dl_colors, 2,
                                  DlBlendMode::kSrcOver, dl_sampling,  //
                                  nullptr, &ctx.paint);
          },
          kDrawAtlasWithPaintFlags));
}

sk_sp<DisplayList> makeTestDisplayList() {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setDrawStyle(DlDrawStyle::kFill);
  paint.setColor(DlColor(SK_ColorRED));
  builder.DrawRect(
      SkRect{kRenderLeft, kRenderTop, kRenderCenterX, kRenderCenterY}, paint);
  paint.setColor(DlColor(SK_ColorBLUE));
  builder.DrawRect(
      SkRect{kRenderCenterX, kRenderTop, kRenderRight, kRenderCenterY}, paint);
  paint.setColor(DlColor(SK_ColorGREEN));
  builder.DrawRect(
      SkRect{kRenderLeft, kRenderCenterY, kRenderCenterX, kRenderBottom},
      paint);
  paint.setColor(DlColor(SK_ColorYELLOW));
  builder.DrawRect(
      SkRect{kRenderCenterX, kRenderCenterY, kRenderRight, kRenderBottom},
      paint);
  return builder.Build();
}

TEST_F(DisplayListRendering, DrawDisplayList) {
  sk_sp<DisplayList> display_list = makeTestDisplayList();
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            DlSkCanvasAdapter(ctx.canvas).DrawDisplayList(display_list);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawDisplayList(display_list);
          },
          kDrawDisplayListFlags)
          .set_draw_display_list());
}

TEST_F(DisplayListRendering, DrawTextBlob) {
  // TODO(https://github.com/flutter/flutter/issues/82202): Remove once the
  // performance overlay can use Fuchsia's font manager instead of the empty
  // default.
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "Rendering comparisons require a valid default font manager";
#else
  sk_sp<SkTextBlob> blob =
      CanvasCompareTester::MakeTextBlob("Testing", kRenderHeight * 0.33f);
#ifdef IMPELLER_SUPPORTS_RENDERING
  auto frame = impeller::MakeTextFrameFromTextBlobSkia(blob);
#endif  // IMPELLER_SUPPORTS_RENDERING
  SkScalar render_y_1_3 = kRenderTop + kRenderHeight * 0.3;
  SkScalar render_y_2_3 = kRenderTop + kRenderHeight * 0.6;
  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {
            auto paint = ctx.paint;
            ctx.canvas->drawTextBlob(blob, kRenderLeft, render_y_1_3, paint);
            ctx.canvas->drawTextBlob(blob, kRenderLeft, render_y_2_3, paint);
            ctx.canvas->drawTextBlob(blob, kRenderLeft, kRenderBottom, paint);
          },
          [=](const DlRenderContext& ctx) {
            auto paint = ctx.paint;
            ctx.canvas->DrawTextBlob(blob, kRenderLeft, render_y_1_3, paint);
            ctx.canvas->DrawTextBlob(blob, kRenderLeft, render_y_2_3, paint);
            ctx.canvas->DrawTextBlob(blob, kRenderLeft, kRenderBottom, paint);
          },
#ifdef IMPELLER_SUPPORTS_RENDERING
          [=](const DlRenderContext& ctx) {
            auto paint = ctx.paint;
            ctx.canvas->DrawTextFrame(frame, kRenderLeft, render_y_1_3, paint);
            ctx.canvas->DrawTextFrame(frame, kRenderLeft, render_y_2_3, paint);
            ctx.canvas->DrawTextFrame(frame, kRenderLeft, kRenderBottom, paint);
          },
#endif  // IMPELLER_SUPPORTS_RENDERING
          kDrawTextBlobFlags)
          .set_draw_text_blob(),
      // From examining the bounds differential for the "Default" case, the
      // SkTextBlob adds a padding of ~32 on the left, ~30 on the right,
      // ~12 on top and ~8 on the bottom, so we add 33h & 13v allowed
      // padding to the tolerance
      CanvasCompareTester::DefaultTolerance.addBoundsPadding(33, 13));
  EXPECT_TRUE(blob->unique());
#endif  // OS_FUCHSIA
}

TEST_F(DisplayListRendering, DrawShadow) {
  SkPath path;
  path.addRoundRect(
      {
          kRenderLeft + 10,
          kRenderTop,
          kRenderRight - 10,
          kRenderBottom - 20,
      },
      kRenderCornerRadius, kRenderCornerRadius);
  const DlColor color = DlColor::kDarkGrey();
  const SkScalar elevation = 7;

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            DlSkCanvasDispatcher::DrawShadow(ctx.canvas, path, color, elevation,
                                             false, 1.0);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawShadow(path, color, elevation, false, 1.0);
          },
          kDrawShadowFlags),
      CanvasCompareTester::DefaultTolerance.addBoundsPadding(3, 3));
}

TEST_F(DisplayListRendering, DrawShadowTransparentOccluder) {
  SkPath path;
  path.addRoundRect(
      {
          kRenderLeft + 10,
          kRenderTop,
          kRenderRight - 10,
          kRenderBottom - 20,
      },
      kRenderCornerRadius, kRenderCornerRadius);
  const DlColor color = DlColor::kDarkGrey();
  const SkScalar elevation = 7;

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            DlSkCanvasDispatcher::DrawShadow(ctx.canvas, path, color, elevation,
                                             true, 1.0);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawShadow(path, color, elevation, true, 1.0);
          },
          kDrawShadowFlags),
      CanvasCompareTester::DefaultTolerance.addBoundsPadding(3, 3));
}

TEST_F(DisplayListRendering, DrawShadowDpr) {
  SkPath path;
  path.addRoundRect(
      {
          kRenderLeft + 10,
          kRenderTop,
          kRenderRight - 10,
          kRenderBottom - 20,
      },
      kRenderCornerRadius, kRenderCornerRadius);
  const DlColor color = DlColor::kDarkGrey();
  const SkScalar elevation = 7;

  CanvasCompareTester::RenderAll(  //
      TestParameters(
          [=](const SkRenderContext& ctx) {  //
            DlSkCanvasDispatcher::DrawShadow(ctx.canvas, path, color, elevation,
                                             false, 1.5);
          },
          [=](const DlRenderContext& ctx) {  //
            ctx.canvas->DrawShadow(path, color, elevation, false, 1.5);
          },
          kDrawShadowFlags),
      CanvasCompareTester::DefaultTolerance.addBoundsPadding(3, 3));
}

TEST_F(DisplayListRendering, SaveLayerClippedContentStillFilters) {
  // draw rect is just outside of render bounds on the right
  const SkRect draw_rect = SkRect::MakeLTRB(  //
      kRenderRight + 1,                       //
      kRenderTop,                             //
      kTestBounds2.fRight,                    //
      kRenderBottom                           //
  );
  TestParameters test_params(
      [=](const SkRenderContext& ctx) {
        auto layer_filter =
            SkImageFilters::Blur(10.0f, 10.0f, SkTileMode::kDecal, nullptr);
        SkPaint layer_paint;
        layer_paint.setImageFilter(layer_filter);
        ctx.canvas->save();
        ctx.canvas->clipRect(kRenderBounds, SkClipOp::kIntersect, false);
        ctx.canvas->saveLayer(&kTestBounds2, &layer_paint);
        ctx.canvas->drawRect(draw_rect, ctx.paint);
        ctx.canvas->restore();
        ctx.canvas->restore();
      },
      [=](const DlRenderContext& ctx) {
        auto layer_filter =
            DlImageFilter::MakeBlur(10.0f, 10.0f, DlTileMode::kDecal);
        DlPaint layer_paint;
        layer_paint.setImageFilter(layer_filter);
        ctx.canvas->Save();
        ctx.canvas->ClipRect(kRenderBounds, ClipOp::kIntersect, false);
        ctx.canvas->SaveLayer(&kTestBounds2, &layer_paint);
        ctx.canvas->DrawRect(draw_rect, ctx.paint);
        ctx.canvas->Restore();
        ctx.canvas->Restore();
      },
      kSaveLayerWithPaintFlags);
  CaseParameters case_params("Filtered SaveLayer with clipped content");
  BoundsTolerance tolerance = BoundsTolerance().addAbsolutePadding(6.0f, 6.0f);

  for (auto& back_end : CanvasCompareTester::TestBackends) {
    auto provider = CanvasCompareTester::GetProvider(back_end);
    RenderEnvironment env = RenderEnvironment::MakeN32(provider.get());
    env.init_ref(kEmptySkSetup, test_params.sk_renderer(),  //
                 kEmptyDlSetup, test_params.dl_renderer(),
                 test_params.imp_renderer());
    CanvasCompareTester::quickCompareToReference(env, "default");
    CanvasCompareTester::RenderWith(test_params, env, tolerance, case_params);
  }
}

TEST_F(DisplayListRendering, SaveLayerConsolidation) {
  float commutable_color_matrix[]{
      // clang-format off
      0, 1, 0, 0, 0,
      0, 0, 1, 0, 0,
      1, 0, 0, 0, 0,
      0, 0, 0, 1, 0,
      // clang-format on
  };
  float non_commutable_color_matrix[]{
      // clang-format off
      0, 1, 0, .1, 0,
      0, 0, 1, .1, 0,
      1, 0, 0, .1, 0,
      0, 0, 0, .7, 0,
      // clang-format on
  };
  DlMatrix contract_matrix;
  contract_matrix.Translate({kRenderCenterX, kRenderCenterY});
  contract_matrix.Scale({0.9f, 0.9f});
  contract_matrix.Translate({kRenderCenterX, kRenderCenterY});

  std::vector<SkScalar> opacities = {
      0,
      0.5f,
      SK_Scalar1,
  };
  std::vector<std::shared_ptr<const DlColorFilter>> color_filters = {
      DlColorFilter::MakeBlend(DlColor::kCyan(), DlBlendMode::kSrcATop),
      DlColorFilter::MakeMatrix(commutable_color_matrix),
      DlColorFilter::MakeMatrix(non_commutable_color_matrix),
      DlColorFilter::MakeSrgbToLinearGamma(),
      DlColorFilter::MakeLinearToSrgbGamma(),
  };
  std::vector<std::shared_ptr<DlImageFilter>> image_filters = {
      DlImageFilter::MakeBlur(5.0f, 5.0f, DlTileMode::kDecal),
      DlImageFilter::MakeDilate(5.0f, 5.0f),
      DlImageFilter::MakeErode(5.0f, 5.0f),
      DlImageFilter::MakeMatrix(contract_matrix, DlImageSampling::kLinear),
  };

  auto render_content = [](DisplayListBuilder& builder) {
    builder.DrawRect(
        SkRect{kRenderLeft, kRenderTop, kRenderCenterX, kRenderCenterY},
        DlPaint(DlColor::kYellow()));
    builder.DrawRect(
        SkRect{kRenderCenterX, kRenderTop, kRenderRight, kRenderCenterY},
        DlPaint(DlColor::kRed()));
    builder.DrawRect(
        SkRect{kRenderLeft, kRenderCenterY, kRenderCenterX, kRenderBottom},
        DlPaint(DlColor::kBlue()));
    builder.DrawRect(
        SkRect{kRenderCenterX, kRenderCenterY, kRenderRight, kRenderBottom},
        DlPaint(DlColor::kRed().modulateOpacity(0.5f)));
  };

  auto test_attributes_env =
      [render_content](DlPaint& paint1, DlPaint& paint2,
                       const DlPaint& paint_both, bool same, bool rev_same,
                       const std::string& desc1, const std::string& desc2,
                       const RenderEnvironment* env) {
        DisplayListBuilder nested_builder;
        nested_builder.SaveLayer(&kTestBounds2, &paint1);
        nested_builder.SaveLayer(&kTestBounds2, &paint2);
        render_content(nested_builder);
        auto nested_results = env->getResult(nested_builder.Build());

        DisplayListBuilder reverse_builder;
        reverse_builder.SaveLayer(&kTestBounds2, &paint2);
        reverse_builder.SaveLayer(&kTestBounds2, &paint1);
        render_content(reverse_builder);
        auto reverse_results = env->getResult(reverse_builder.Build());

        DisplayListBuilder combined_builder;
        combined_builder.SaveLayer(&kTestBounds2, &paint_both);
        render_content(combined_builder);
        auto combined_results = env->getResult(combined_builder.Build());

        // Set this boolean to true to test if combinations that are marked
        // as incompatible actually are compatible despite our predictions.
        // Some of the combinations that we treat as incompatible actually
        // are compatible with swapping the order of the operations, but
        // it would take a bit of new infrastructure to really identify
        // those combinations. The only hard constraint to test here is
        // when we claim that they are compatible and they aren't.
        const bool always = false;

        // In some circumstances, Skia can combine image filter evaluations
        // and elide a renderpass. In this case rounding and precision of inputs
        // to color filters may cause the output to differ by 1.
        if (always || same) {
          CanvasCompareTester::compareToReference(
              nested_results.get(), combined_results.get(),
              "nested " + desc1 + " then " + desc2, /*bounds=*/nullptr,
              /*tolerance=*/nullptr, DlColor::kTransparent(),
              /*fuzzyCompares=*/true, combined_results->width(),
              combined_results->height(), /*printMismatches=*/true);
        }
        if (always || rev_same) {
          CanvasCompareTester::compareToReference(
              reverse_results.get(), combined_results.get(),
              "nested " + desc2 + " then " + desc1, /*bounds=*/nullptr,
              /*tolerance=*/nullptr, DlColor::kTransparent(),
              /*fuzzyCompares=*/true, combined_results->width(),
              combined_results->height(), /*printMismatches=*/true);
        }
      };

  auto test_attributes = [test_attributes_env](DlPaint& paint1, DlPaint& paint2,
                                               const DlPaint& paint_both,
                                               bool same, bool rev_same,
                                               const std::string& desc1,
                                               const std::string& desc2) {
    for (auto& back_end : CanvasCompareTester::TestBackends) {
      auto provider = CanvasCompareTester::GetProvider(back_end);
      auto env = std::make_unique<RenderEnvironment>(
          provider.get(), PixelFormat::kN32PremulPixelFormat);
      test_attributes_env(paint1, paint2, paint_both,  //
                          same, rev_same, desc1, desc2, env.get());
    }
  };

  // CF then Opacity should always work.
  // The reverse sometimes works.
  for (size_t cfi = 0; cfi < color_filters.size(); cfi++) {
    auto color_filter = color_filters[cfi];
    std::string cf_desc = "color filter #" + std::to_string(cfi + 1);
    DlPaint nested_paint1 = DlPaint().setColorFilter(color_filter);

    for (size_t oi = 0; oi < opacities.size(); oi++) {
      SkScalar opacity = opacities[oi];
      std::string op_desc = "opacity " + std::to_string(opacity);
      DlPaint nested_paint2 = DlPaint().setOpacity(opacity);

      DlPaint combined_paint = nested_paint1;
      combined_paint.setOpacity(opacity);

      bool op_then_cf_works = opacity <= 0.0 || opacity >= 1.0 ||
                              color_filter->can_commute_with_opacity();

      test_attributes(nested_paint1, nested_paint2, combined_paint, true,
                      op_then_cf_works, cf_desc, op_desc);
    }
  }

  // Opacity then IF should always work.
  // The reverse can also work for some values of opacity.
  // The reverse should also theoretically work for some IFs, but we
  // get some rounding errors that are more than just trivial.
  for (size_t oi = 0; oi < opacities.size(); oi++) {
    SkScalar opacity = opacities[oi];
    std::string op_desc = "opacity " + std::to_string(opacity);
    DlPaint nested_paint1 = DlPaint().setOpacity(opacity);

    for (size_t ifi = 0; ifi < image_filters.size(); ifi++) {
      auto image_filter = image_filters[ifi];
      std::string if_desc = "image filter #" + std::to_string(ifi + 1);
      DlPaint nested_paint2 = DlPaint().setImageFilter(image_filter);

      DlPaint combined_paint = nested_paint1;
      combined_paint.setImageFilter(image_filter);

      bool if_then_op_works = opacity <= 0.0 || opacity >= 1.0;
      test_attributes(nested_paint1, nested_paint2, combined_paint, true,
                      if_then_op_works, op_desc, if_desc);
    }
  }

  // CF then IF should always work.
  // The reverse might work, but we lack the infrastructure to check it.
  for (size_t cfi = 0; cfi < color_filters.size(); cfi++) {
    auto color_filter = color_filters[cfi];
    std::string cf_desc = "color filter #" + std::to_string(cfi + 1);
    DlPaint nested_paint1 = DlPaint().setColorFilter(color_filter);

    for (size_t ifi = 0; ifi < image_filters.size(); ifi++) {
      auto image_filter = image_filters[ifi];
      std::string if_desc = "image filter #" + std::to_string(ifi + 1);
      DlPaint nested_paint2 = DlPaint().setImageFilter(image_filter);

      DlPaint combined_paint = nested_paint1;
      combined_paint.setImageFilter(image_filter);

      test_attributes(nested_paint1, nested_paint2, combined_paint, true, false,
                      cf_desc, if_desc);
    }
  }
}

TEST_F(DisplayListRendering, MatrixColorFilterModifyTransparencyCheck) {
  auto test_matrix = [](int element, SkScalar value) {
    // clang-format off
    float matrix[] = {
        1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
    };
    // clang-format on
    std::string desc =
        "matrix[" + std::to_string(element) + "] = " + std::to_string(value);
    float original_value = matrix[element];
    matrix[element] = value;
    // Here we instantiate a DlMatrixColorFilter directly so that it is
    // not affected by the "NOP" detection in the factory. We sould not
    // need to do this if we tested by just rendering the filter color
    // over the source color with the filter blend mode instead of
    // rendering via a ColorFilter, but this test is more "black box".
    DlMatrixColorFilter filter(matrix);
    auto dl_filter = DlColorFilter::MakeMatrix(matrix);
    bool is_identity = (dl_filter == nullptr || original_value == value);

    DlPaint paint(DlColor(0x7f7f7f7f));
    DlPaint filter_save_paint = DlPaint().setColorFilter(&filter);

    DisplayListBuilder builder1;
    builder1.Translate(kTestCenter.fX, kTestCenter.fY);
    builder1.Rotate(45);
    builder1.Translate(-kTestCenter.fX, -kTestCenter.fY);
    builder1.DrawRect(kRenderBounds, paint);
    auto display_list1 = builder1.Build();

    DisplayListBuilder builder2;
    builder2.Translate(kTestCenter.fX, kTestCenter.fY);
    builder2.Rotate(45);
    builder2.Translate(-kTestCenter.fX, -kTestCenter.fY);
    builder2.SaveLayer(&kTestBounds2, &filter_save_paint);
    builder2.DrawRect(kRenderBounds, paint);
    builder2.Restore();
    auto display_list2 = builder2.Build();

    for (auto& back_end : CanvasCompareTester::TestBackends) {
      auto provider = CanvasCompareTester::GetProvider(back_end);
      auto env = std::make_unique<RenderEnvironment>(
          provider.get(), PixelFormat::kN32PremulPixelFormat);
      auto results1 = env->getResult(display_list1);
      auto results2 = env->getResult(display_list2);
      CanvasCompareTester::quickCompareToReference(
          results1.get(), results2.get(), is_identity,
          desc + " filter affects rendering");
      int modified_transparent_pixels =
          CanvasCompareTester::countModifiedTransparentPixels(results1.get(),
                                                              results2.get());
      EXPECT_EQ(filter.modifies_transparent_black(),
                modified_transparent_pixels != 0)
          << desc;
    }
  };

  // Tests identity (matrix[0] already == 1 in an identity filter)
  test_matrix(0, 1);
  // test_matrix(19, 1);
  for (int i = 0; i < 20; i++) {
    test_matrix(i, -0.25);
    test_matrix(i, 0);
    test_matrix(i, 0.25);
    test_matrix(i, 1);
    test_matrix(i, 1.25);
    test_matrix(i, SK_ScalarNaN);
    test_matrix(i, SK_ScalarInfinity);
    test_matrix(i, -SK_ScalarInfinity);
  }
}

TEST_F(DisplayListRendering, MatrixColorFilterOpacityCommuteCheck) {
  auto test_matrix = [](int element, SkScalar value) {
    // clang-format off
    float matrix[] = {
        1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
    };
    // clang-format on
    std::string desc =
        "matrix[" + std::to_string(element) + "] = " + std::to_string(value);
    matrix[element] = value;
    auto filter = DlColorFilter::MakeMatrix(matrix);
    EXPECT_EQ(std::isfinite(value), filter != nullptr);

    DlPaint paint(DlColor(0x80808080));
    DlPaint opacity_save_paint = DlPaint().setOpacity(0.5);
    DlPaint filter_save_paint = DlPaint().setColorFilter(filter);

    DisplayListBuilder builder1;
    builder1.SaveLayer(&kTestBounds2, &opacity_save_paint);
    builder1.SaveLayer(&kTestBounds2, &filter_save_paint);
    // builder1.DrawRect(kRenderBounds.makeOffset(20, 20), DlPaint());
    builder1.DrawRect(kRenderBounds, paint);
    builder1.Restore();
    builder1.Restore();
    auto display_list1 = builder1.Build();

    DisplayListBuilder builder2;
    builder2.SaveLayer(&kTestBounds2, &filter_save_paint);
    builder2.SaveLayer(&kTestBounds2, &opacity_save_paint);
    // builder1.DrawRect(kRenderBounds.makeOffset(20, 20), DlPaint());
    builder2.DrawRect(kRenderBounds, paint);
    builder2.Restore();
    builder2.Restore();
    auto display_list2 = builder2.Build();

    for (auto& back_end : CanvasCompareTester::TestBackends) {
      auto provider = CanvasCompareTester::GetProvider(back_end);
      auto env = std::make_unique<RenderEnvironment>(
          provider.get(), PixelFormat::kN32PremulPixelFormat);
      auto results1 = env->getResult(display_list1);
      auto results2 = env->getResult(display_list2);
      if (!filter || filter->can_commute_with_opacity()) {
        CanvasCompareTester::compareToReference(
            results2.get(), results1.get(), desc, nullptr, nullptr,
            DlColor::kTransparent(), true, kTestWidth, kTestHeight, true);
      } else {
        CanvasCompareTester::quickCompareToReference(
            results1.get(), results2.get(), false, desc);
      }
    }
  };

  // Tests identity (matrix[0] already == 1 in an identity filter)
  test_matrix(0, 1);
  // test_matrix(19, 1);
  for (int i = 0; i < 20; i++) {
    test_matrix(i, -0.25);
    test_matrix(i, 0);
    test_matrix(i, 0.25);
    test_matrix(i, 1);
    test_matrix(i, 1.1);
    test_matrix(i, SK_ScalarNaN);
    test_matrix(i, SK_ScalarInfinity);
    test_matrix(i, -SK_ScalarInfinity);
  }
}

#define FOR_EACH_BLEND_MODE_ENUM(FUNC) \
  FUNC(kClear)                         \
  FUNC(kSrc)                           \
  FUNC(kDst)                           \
  FUNC(kSrcOver)                       \
  FUNC(kDstOver)                       \
  FUNC(kSrcIn)                         \
  FUNC(kDstIn)                         \
  FUNC(kSrcOut)                        \
  FUNC(kDstOut)                        \
  FUNC(kSrcATop)                       \
  FUNC(kDstATop)                       \
  FUNC(kXor)                           \
  FUNC(kPlus)                          \
  FUNC(kModulate)                      \
  FUNC(kScreen)                        \
  FUNC(kOverlay)                       \
  FUNC(kDarken)                        \
  FUNC(kLighten)                       \
  FUNC(kColorDodge)                    \
  FUNC(kColorBurn)                     \
  FUNC(kHardLight)                     \
  FUNC(kSoftLight)                     \
  FUNC(kDifference)                    \
  FUNC(kExclusion)                     \
  FUNC(kMultiply)                      \
  FUNC(kHue)                           \
  FUNC(kSaturation)                    \
  FUNC(kColor)                         \
  FUNC(kLuminosity)

// This function serves both to enhance error output below and to double
// check that the macro supplies all modes (otherwise it won't compile)
static std::string BlendModeToString(DlBlendMode mode) {
  switch (mode) {
#define MODE_CASE(m)   \
  case DlBlendMode::m: \
    return #m;
    FOR_EACH_BLEND_MODE_ENUM(MODE_CASE)
#undef MODE_CASE
  }
}

TEST_F(DisplayListRendering, BlendColorFilterModifyTransparencyCheck) {
  auto test_mode_color = [](DlBlendMode mode, DlColor color) {
    std::stringstream desc_str;
    std::string mode_string = BlendModeToString(mode);
    desc_str << "blend[" << mode_string << ", " << color << "]";
    std::string desc = desc_str.str();
    DlBlendColorFilter filter(color, mode);
    if (filter.modifies_transparent_black()) {
      ASSERT_NE(DlColorFilter::MakeBlend(color, mode), nullptr) << desc;
    }

    DlPaint paint(DlColor(0x7f7f7f7f));
    DlPaint filter_save_paint = DlPaint().setColorFilter(&filter);

    DisplayListBuilder builder1;
    builder1.Translate(kTestCenter.fX, kTestCenter.fY);
    builder1.Rotate(45);
    builder1.Translate(-kTestCenter.fX, -kTestCenter.fY);
    builder1.DrawRect(kRenderBounds, paint);
    auto display_list1 = builder1.Build();

    DisplayListBuilder builder2;
    builder2.Translate(kTestCenter.fX, kTestCenter.fY);
    builder2.Rotate(45);
    builder2.Translate(-kTestCenter.fX, -kTestCenter.fY);
    builder2.SaveLayer(&kTestBounds2, &filter_save_paint);
    builder2.DrawRect(kRenderBounds, paint);
    builder2.Restore();
    auto display_list2 = builder2.Build();

    for (auto& back_end : CanvasCompareTester::TestBackends) {
      auto provider = CanvasCompareTester::GetProvider(back_end);
      auto env = std::make_unique<RenderEnvironment>(
          provider.get(), PixelFormat::kN32PremulPixelFormat);
      auto results1 = env->getResult(display_list1);
      auto results2 = env->getResult(display_list2);
      int modified_transparent_pixels =
          CanvasCompareTester::countModifiedTransparentPixels(results1.get(),
                                                              results2.get());
      EXPECT_EQ(filter.modifies_transparent_black(),
                modified_transparent_pixels != 0)
          << desc;
    }
  };

  auto test_mode = [&test_mode_color](DlBlendMode mode) {
    test_mode_color(mode, DlColor::kTransparent());
    test_mode_color(mode, DlColor::kWhite());
    test_mode_color(mode, DlColor::kWhite().modulateOpacity(0.5));
    test_mode_color(mode, DlColor::kBlack());
    test_mode_color(mode, DlColor::kBlack().modulateOpacity(0.5));
  };

#define TEST_MODE(V) test_mode(DlBlendMode::V);
  FOR_EACH_BLEND_MODE_ENUM(TEST_MODE)
#undef TEST_MODE
}

TEST_F(DisplayListRendering, BlendColorFilterOpacityCommuteCheck) {
  auto test_mode_color = [](DlBlendMode mode, DlColor color) {
    std::stringstream desc_str;
    std::string mode_string = BlendModeToString(mode);
    desc_str << "blend[" << mode_string << ", " << color << "]";
    std::string desc = desc_str.str();
    DlBlendColorFilter filter(color, mode);
    if (filter.can_commute_with_opacity()) {
      // If it can commute with opacity, then it might also be a NOP,
      // so we won't necessarily get a non-null return from |::Make()|
    } else {
      ASSERT_NE(DlColorFilter::MakeBlend(color, mode), nullptr) << desc;
    }

    DlPaint paint(DlColor(0x80808080));
    DlPaint opacity_save_paint = DlPaint().setOpacity(0.5);
    DlPaint filter_save_paint = DlPaint().setColorFilter(&filter);

    DisplayListBuilder builder1;
    builder1.SaveLayer(&kTestBounds2, &opacity_save_paint);
    builder1.SaveLayer(&kTestBounds2, &filter_save_paint);
    // builder1.DrawRect(kRenderBounds.makeOffset(20, 20), DlPaint());
    builder1.DrawRect(kRenderBounds, paint);
    builder1.Restore();
    builder1.Restore();
    auto display_list1 = builder1.Build();

    DisplayListBuilder builder2;
    builder2.SaveLayer(&kTestBounds2, &filter_save_paint);
    builder2.SaveLayer(&kTestBounds2, &opacity_save_paint);
    // builder1.DrawRect(kRenderBounds.makeOffset(20, 20), DlPaint());
    builder2.DrawRect(kRenderBounds, paint);
    builder2.Restore();
    builder2.Restore();
    auto display_list2 = builder2.Build();

    for (auto& back_end : CanvasCompareTester::TestBackends) {
      auto provider = CanvasCompareTester::GetProvider(back_end);
      auto env = std::make_unique<RenderEnvironment>(
          provider.get(), PixelFormat::kN32PremulPixelFormat);
      auto results1 = env->getResult(display_list1);
      auto results2 = env->getResult(display_list2);
      if (filter.can_commute_with_opacity()) {
        CanvasCompareTester::compareToReference(
            results2.get(), results1.get(), desc, nullptr, nullptr,
            DlColor::kTransparent(), true, kTestWidth, kTestHeight, true);
      } else {
        CanvasCompareTester::quickCompareToReference(
            results1.get(), results2.get(), false, desc);
      }
    }
  };

  auto test_mode = [&test_mode_color](DlBlendMode mode) {
    test_mode_color(mode, DlColor::kTransparent());
    test_mode_color(mode, DlColor::kWhite());
    test_mode_color(mode, DlColor::kWhite().modulateOpacity(0.5));
    test_mode_color(mode, DlColor::kBlack());
    test_mode_color(mode, DlColor::kBlack().modulateOpacity(0.5));
  };

#define TEST_MODE(V) test_mode(DlBlendMode::V);
  FOR_EACH_BLEND_MODE_ENUM(TEST_MODE)
#undef TEST_MODE
}

class DisplayListNopTest : public DisplayListRendering {
  // The following code uses the acronym MTB for "modifies_transparent_black"

 protected:
  DisplayListNopTest() : DisplayListRendering() {
    test_src_colors = {
        DlColor::kBlack().withAlpha(0),     // transparent black
        DlColor::kBlack().withAlpha(0x7f),  // half transparent black
        DlColor::kWhite().withAlpha(0x7f),  // half transparent white
        DlColor::kBlack(),                  // opaque black
        DlColor::kWhite(),                  // opaque white
        DlColor::kRed(),                    // opaque red
        DlColor::kGreen(),                  // opaque green
        DlColor::kBlue(),                   // opaque blue
        DlColor::kDarkGrey(),               // dark grey
        DlColor::kLightGrey(),              // light grey
    };

    // We test against a color cube of 3x3x3 colors [55,aa,ff]
    // plus transparency as the first color/pixel
    test_dst_colors.push_back(DlColor::kTransparent());
    const int step = 0x55;
    static_assert(step * 3 == 255);
    for (int a = step; a < 256; a += step) {
      for (int r = step; r < 256; r += step) {
        for (int g = step; g < 256; g += step) {
          for (int b = step; b < 256; b += step) {
            test_dst_colors.push_back(DlColor(a << 24 | r << 16 | g << 8 | b));
          }
        }
      }
    }

    static constexpr float color_filter_matrix_nomtb[] = {
        0.0001, 0.0001, 0.0001, 0.9997, 0.0,  //
        0.0001, 0.0001, 0.0001, 0.9997, 0.0,  //
        0.0001, 0.0001, 0.0001, 0.9997, 0.0,  //
        0.0001, 0.0001, 0.0001, 0.9997, 0.0,  //
    };
    static constexpr float color_filter_matrix_mtb[] = {
        0.0001, 0.0001, 0.0001, 0.9997, 0.0,  //
        0.0001, 0.0001, 0.0001, 0.9997, 0.0,  //
        0.0001, 0.0001, 0.0001, 0.9997, 0.0,  //
        0.0001, 0.0001, 0.0001, 0.9997, 0.1,  //
    };
    color_filter_nomtb = DlColorFilter::MakeMatrix(color_filter_matrix_nomtb);
    color_filter_mtb = DlColorFilter::MakeMatrix(color_filter_matrix_mtb);
    EXPECT_FALSE(color_filter_nomtb->modifies_transparent_black());
    EXPECT_TRUE(color_filter_mtb->modifies_transparent_black());

    test_data =
        get_output(test_dst_colors.size(), 1, true, [this](SkCanvas* canvas) {
          int x = 0;
          for (DlColor color : test_dst_colors) {
            SkPaint paint;
            paint.setColor(ToSk(color));
            paint.setBlendMode(SkBlendMode::kSrc);
            canvas->drawRect(SkRect::MakeXYWH(x, 0, 1, 1), paint);
            x++;
          }
        });

    // For image-on-image tests, the src and dest images will have repeated
    // rows/columns that have every color, but laid out at right angles to
    // each other so we see an interaction with every test color against
    // every other test color.
    int data_count = test_data->image()->width();
    test_image_dst_data = get_output(
        data_count, data_count, true, [this, data_count](SkCanvas* canvas) {
          ASSERT_EQ(test_data->width(), data_count);
          ASSERT_EQ(test_data->height(), 1);
          for (int y = 0; y < data_count; y++) {
            canvas->drawImage(test_data->image().get(), 0, y);
          }
        });
    test_image_src_data = get_output(
        data_count, data_count, true, [this, data_count](SkCanvas* canvas) {
          ASSERT_EQ(test_data->width(), data_count);
          ASSERT_EQ(test_data->height(), 1);
          canvas->translate(data_count, 0);
          canvas->rotate(90);
          for (int y = 0; y < data_count; y++) {
            canvas->drawImage(test_data->image().get(), 0, y);
          }
        });
    // Double check that the pixel data is laid out in orthogonal stripes
    for (int y = 0; y < data_count; y++) {
      for (int x = 0; x < data_count; x++) {
        EXPECT_EQ(*test_image_dst_data->addr32(x, y), *test_data->addr32(x, 0));
        EXPECT_EQ(*test_image_src_data->addr32(x, y), *test_data->addr32(y, 0));
      }
    }
  }

  // These flags are 0 by default until they encounter a counter-example
  // result and get set.
  static constexpr int kWasNotNop = 0x1;  // Some tested pixel was modified
  static constexpr int kWasMTB = 0x2;     // A transparent pixel was modified

  std::vector<DlColor> test_src_colors;
  std::vector<DlColor> test_dst_colors;

  std::shared_ptr<const DlColorFilter> color_filter_nomtb;
  std::shared_ptr<const DlColorFilter> color_filter_mtb;

  // A 1-row image containing every color in test_dst_colors
  std::unique_ptr<RenderResult> test_data;

  // A square image containing test_data duplicated in each row
  std::unique_ptr<RenderResult> test_image_dst_data;

  // A square image containing test_data duplicated in each column
  std::unique_ptr<RenderResult> test_image_src_data;

  std::unique_ptr<RenderResult> get_output(
      int w,
      int h,
      bool snapshot,
      const std::function<void(SkCanvas*)>& renderer) {
    auto surface = SkSurfaces::Raster(SkImageInfo::MakeN32Premul(w, h));
    SkCanvas* canvas = surface->getCanvas();
    renderer(canvas);
    return std::make_unique<SkRenderResult>(surface, snapshot);
  }

  int check_color_result(DlColor dst_color,
                         DlColor result_color,
                         const sk_sp<DisplayList>& dl,
                         const std::string& desc) {
    int ret = 0;
    bool is_error = false;
    if (dst_color.isTransparent() && !result_color.isTransparent()) {
      ret |= kWasMTB;
      is_error = !dl->modifies_transparent_black();
    }
    if (result_color != dst_color) {
      ret |= kWasNotNop;
      is_error = (dl->op_count() == 0u);
    }
    if (is_error) {
      FML_LOG(ERROR) << std::hex << dst_color << " filters to " << result_color
                     << desc;
    }
    return ret;
  }

  int check_image_result(const std::unique_ptr<RenderResult>& dst_data,
                         const std::unique_ptr<RenderResult>& result_data,
                         const sk_sp<DisplayList>& dl,
                         const std::string& desc) {
    EXPECT_EQ(dst_data->width(), result_data->width());
    EXPECT_EQ(dst_data->height(), result_data->height());
    int all_flags = 0;
    for (int y = 0; y < dst_data->height(); y++) {
      const uint32_t* dst_pixels = dst_data->addr32(0, y);
      const uint32_t* result_pixels = result_data->addr32(0, y);
      for (int x = 0; x < dst_data->width(); x++) {
        all_flags |= check_color_result(DlColor(dst_pixels[x]),
                                        DlColor(result_pixels[x]), dl, desc);
      }
    }
    return all_flags;
  }

  void report_results(int all_flags,
                      const sk_sp<DisplayList>& dl,
                      const std::string& desc) {
    if (!dl->modifies_transparent_black()) {
      EXPECT_TRUE((all_flags & kWasMTB) == 0);
    } else if ((all_flags & kWasMTB) == 0) {
      FML_LOG(INFO) << "combination does not affect transparency: " << desc;
    }
    if (dl->op_count() == 0u) {
      EXPECT_TRUE((all_flags & kWasNotNop) == 0);
    } else if ((all_flags & kWasNotNop) == 0) {
      FML_LOG(INFO) << "combination could be classified as a nop: " << desc;
    }
  };

  void test_mode_color_via_filter(DlBlendMode mode, DlColor color) {
    std::stringstream desc_stream;
    desc_stream << " using SkColorFilter::filterColor() with: ";
    desc_stream << BlendModeToString(mode);
    desc_stream << "/" << color;
    std::string desc = desc_stream.str();
    DisplayListBuilder builder(DlRect::MakeWH(100.0f, 100.0f));
    DlPaint paint = DlPaint(color).setBlendMode(mode);
    builder.DrawRect(SkRect{0.0f, 0.0f, 10.0f, 10.0f}, paint);
    auto dl = builder.Build();
    if (dl->modifies_transparent_black()) {
      ASSERT_TRUE(dl->op_count() != 0u);
    }

    auto sk_mode = static_cast<SkBlendMode>(mode);
    auto sk_color_filter = SkColorFilters::Blend(ToSk(color), sk_mode);
    auto srgb = SkColorSpace::MakeSRGB();
    int all_flags = 0;
    if (sk_color_filter) {
      for (DlColor dst_color : test_dst_colors) {
        SkColor4f dst_color_f = SkColor4f::FromColor(ToSk(dst_color));
        DlColor result = DlColor(
            sk_color_filter->filterColor4f(dst_color_f, srgb.get(), srgb.get())
                .toSkColor());
        all_flags |= check_color_result(dst_color, result, dl, desc);
      }
      if ((all_flags & kWasMTB) != 0) {
        EXPECT_FALSE(sk_color_filter->isAlphaUnchanged());
      }
    }
    report_results(all_flags, dl, desc);
  };

  void test_mode_color_via_rendering(DlBlendMode mode, DlColor color) {
    std::stringstream desc_stream;
    desc_stream << " rendering with: ";
    desc_stream << BlendModeToString(mode);
    desc_stream << "/" << color;
    std::string desc = desc_stream.str();
    auto test_image = test_data->image();
    SkRect test_bounds =
        SkRect::MakeWH(test_image->width(), test_image->height());
    DisplayListBuilder builder(test_bounds);
    DlPaint dl_paint = DlPaint(color).setBlendMode(mode);
    builder.DrawRect(test_bounds, dl_paint);
    auto dl = builder.Build();
    bool dl_is_elided = dl->op_count() == 0u;
    bool dl_affects_transparent_pixels = dl->modifies_transparent_black();
    ASSERT_TRUE(!dl_is_elided || !dl_affects_transparent_pixels);

    auto sk_mode = static_cast<SkBlendMode>(mode);
    SkPaint sk_paint;
    sk_paint.setBlendMode(sk_mode);
    sk_paint.setColor(ToSk(color));
    for (auto& back_end : CanvasCompareTester::TestBackends) {
      auto provider = CanvasCompareTester::GetProvider(back_end);
      auto result_surface = provider->MakeOffscreenSurface(
          test_image->width(), test_image->height(),
          DlSurfaceProvider::kN32PremulPixelFormat);
      SkCanvas* result_canvas = result_surface->sk_surface()->getCanvas();
      result_canvas->clear(SK_ColorTRANSPARENT);
      result_canvas->drawImage(test_image.get(), 0, 0);
      result_canvas->drawRect(test_bounds, sk_paint);
      if (GrDirectContext* direct_context = GrAsDirectContext(
              result_surface->sk_surface()->recordingContext())) {
        direct_context->flushAndSubmit();
        direct_context->flushAndSubmit(result_surface->sk_surface().get(),
                                       GrSyncCpu::kYes);
      }
      const std::unique_ptr<RenderResult> result_pixels =
          std::make_unique<SkRenderResult>(result_surface->sk_surface());

      int all_flags = check_image_result(test_data, result_pixels, dl, desc);
      report_results(all_flags, dl, desc);
    }
  };

  void test_attributes_image(DlBlendMode mode,
                             DlColor color,
                             const DlColorFilter* color_filter,
                             DlImageFilter* image_filter) {
    // if (true) { return; }
    std::stringstream desc_stream;
    desc_stream << " rendering with: ";
    desc_stream << BlendModeToString(mode);
    desc_stream << "/" << color;
    std::string cf_mtb = color_filter
                             ? color_filter->modifies_transparent_black()
                                   ? "modifies transparency"
                                   : "preserves transparency"
                             : "no filter";
    desc_stream << ", CF: " << cf_mtb;
    std::string if_mtb = image_filter
                             ? image_filter->modifies_transparent_black()
                                   ? "modifies transparency"
                                   : "preserves transparency"
                             : "no filter";
    desc_stream << ", IF: " << if_mtb;
    std::string desc = desc_stream.str();

    DisplayListBuilder builder(DlRect::MakeWH(100.0f, 100.0f));
    DlPaint paint = DlPaint(color)                     //
                        .setBlendMode(mode)            //
                        .setColorFilter(color_filter)  //
                        .setImageFilter(image_filter);
    builder.DrawImage(DlImage::Make(test_image_src_data->image()),
                      SkPoint{0, 0}, DlImageSampling::kNearestNeighbor, &paint);
    auto dl = builder.Build();

    int w = test_image_src_data->width();
    int h = test_image_src_data->height();
    auto sk_mode = static_cast<SkBlendMode>(mode);
    SkPaint sk_paint;
    sk_paint.setBlendMode(sk_mode);
    sk_paint.setColor(ToSk(color));
    sk_paint.setColorFilter(ToSk(color_filter));
    sk_paint.setImageFilter(ToSk(image_filter));
    for (auto& back_end : CanvasCompareTester::TestBackends) {
      auto provider = CanvasCompareTester::GetProvider(back_end);
      auto result_surface = provider->MakeOffscreenSurface(
          w, h, DlSurfaceProvider::kN32PremulPixelFormat);
      SkCanvas* result_canvas = result_surface->sk_surface()->getCanvas();
      result_canvas->clear(SK_ColorTRANSPARENT);
      result_canvas->drawImage(test_image_dst_data->image(), 0, 0);
      result_canvas->drawImage(test_image_src_data->image(), 0, 0,
                               SkSamplingOptions(), &sk_paint);
      if (GrDirectContext* direct_context = GrAsDirectContext(
              result_surface->sk_surface()->recordingContext())) {
        direct_context->flushAndSubmit();
        direct_context->flushAndSubmit(result_surface->sk_surface().get(),
                                       GrSyncCpu::kYes);
      }
      std::unique_ptr<RenderResult> result_pixels =
          std::make_unique<SkRenderResult>(result_surface->sk_surface());

      int all_flags =
          check_image_result(test_image_dst_data, result_pixels, dl, desc);
      report_results(all_flags, dl, desc);
    }
  };
};

TEST_F(DisplayListNopTest, BlendModeAndColorViaColorFilter) {
  auto test_mode_filter = [this](DlBlendMode mode) {
    for (DlColor color : test_src_colors) {
      test_mode_color_via_filter(mode, color);
    }
  };

#define TEST_MODE(V) test_mode_filter(DlBlendMode::V);
  FOR_EACH_BLEND_MODE_ENUM(TEST_MODE)
#undef TEST_MODE
}

TEST_F(DisplayListNopTest, BlendModeAndColorByRendering) {
  auto test_mode_render = [this](DlBlendMode mode) {
    // First check rendering a variety of colors onto image
    for (DlColor color : test_src_colors) {
      test_mode_color_via_rendering(mode, color);
    }
  };

#define TEST_MODE(V) test_mode_render(DlBlendMode::V);
  FOR_EACH_BLEND_MODE_ENUM(TEST_MODE)
#undef TEST_MODE
}

TEST_F(DisplayListNopTest, BlendModeAndColorAndFiltersByRendering) {
  auto test_mode_render = [this](DlBlendMode mode) {
    auto image_filter_nomtb = DlColorFilterImageFilter(color_filter_nomtb);
    auto image_filter_mtb = DlColorFilterImageFilter(color_filter_mtb);
    for (DlColor color : test_src_colors) {
      test_attributes_image(mode, color, nullptr, nullptr);
      test_attributes_image(mode, color, color_filter_nomtb.get(), nullptr);
      test_attributes_image(mode, color, color_filter_mtb.get(), nullptr);
      test_attributes_image(mode, color, nullptr, &image_filter_nomtb);
      test_attributes_image(mode, color, nullptr, &image_filter_mtb);
    }
  };

#define TEST_MODE(V) test_mode_render(DlBlendMode::V);
  FOR_EACH_BLEND_MODE_ENUM(TEST_MODE)
#undef TEST_MODE
}

#undef FOR_EACH_BLEND_MODE_ENUM

}  // namespace testing
}  // namespace flutter
