// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_ASSERTIONS_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_ASSERTIONS_H_

#include <sstream>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/embedder_engine.h"
#include "flutter/testing/assertions.h"
#include "gtest/gtest.h"
#include "third_party/skia/include/core/SkPoint.h"
#include "third_party/skia/include/core/SkSize.h"

// NOLINTBEGIN(google-objc-function-naming)

//------------------------------------------------------------------------------
// Equality
//------------------------------------------------------------------------------
inline bool operator==(const FlutterPoint& a, const FlutterPoint& b) {
  return flutter::testing::NumberNear(a.x, b.x) &&
         flutter::testing::NumberNear(a.y, b.y);
}

inline bool operator==(const FlutterRect& a, const FlutterRect& b) {
  return flutter::testing::NumberNear(a.left, b.left) &&
         flutter::testing::NumberNear(a.top, b.top) &&
         flutter::testing::NumberNear(a.right, b.right) &&
         flutter::testing::NumberNear(a.bottom, b.bottom);
}

inline bool operator==(const FlutterSize& a, const FlutterSize& b) {
  return flutter::testing::NumberNear(a.width, b.width) &&
         flutter::testing::NumberNear(a.height, b.height);
}

inline bool operator==(const FlutterRoundedRect& a,
                       const FlutterRoundedRect& b) {
  return a.rect == b.rect &&
         a.upper_left_corner_radius == b.upper_left_corner_radius &&
         a.upper_right_corner_radius == b.upper_right_corner_radius &&
         a.lower_right_corner_radius == b.lower_right_corner_radius &&
         a.lower_left_corner_radius == b.lower_left_corner_radius;
}

inline bool operator==(const FlutterTransformation& a,
                       const FlutterTransformation& b) {
  return a.scaleX == b.scaleX && a.skewX == b.skewX && a.transX == b.transX &&
         a.skewY == b.skewY && a.scaleY == b.scaleY && a.transY == b.transY &&
         a.pers0 == b.pers0 && a.pers1 == b.pers1 && a.pers2 == b.pers2;
}

inline bool operator==(const FlutterOpenGLTexture& a,
                       const FlutterOpenGLTexture& b) {
  return a.target == b.target && a.name == b.name && a.format == b.format &&
         a.user_data == b.user_data &&
         a.destruction_callback == b.destruction_callback;
}

inline bool operator==(const FlutterOpenGLFramebuffer& a,
                       const FlutterOpenGLFramebuffer& b) {
  return a.target == b.target && a.name == b.name &&
         a.user_data == b.user_data &&
         a.destruction_callback == b.destruction_callback;
}

inline bool operator==(const FlutterOpenGLSurface& a,
                       const FlutterOpenGLSurface& b) {
  return a.make_current_callback == b.make_current_callback &&
         a.user_data == b.user_data && a.format == b.format &&
         a.destruction_callback == b.destruction_callback;
}

inline bool operator==(const FlutterMetalTexture& a,
                       const FlutterMetalTexture& b) {
  return a.texture_id == b.texture_id && a.texture == b.texture;
}

inline bool operator==(const FlutterVulkanImage& a,
                       const FlutterVulkanImage& b) {
  return a.image == b.image && a.format == b.format;
}

inline bool operator==(const FlutterVulkanBackingStore& a,
                       const FlutterVulkanBackingStore& b) {
  return a.image == b.image;
}

inline bool operator==(const FlutterMetalBackingStore& a,
                       const FlutterMetalBackingStore& b) {
  return a.texture == b.texture;
}

inline bool operator==(const FlutterOpenGLBackingStore& a,
                       const FlutterOpenGLBackingStore& b) {
  if (!(a.type == b.type)) {
    return false;
  }

  switch (a.type) {
    case kFlutterOpenGLTargetTypeTexture:
      return a.texture == b.texture;
    case kFlutterOpenGLTargetTypeFramebuffer:
      return a.framebuffer == b.framebuffer;
    case kFlutterOpenGLTargetTypeSurface:
      return a.surface == b.surface;
  }

  return false;
}

inline bool operator==(const FlutterSoftwareBackingStore& a,
                       const FlutterSoftwareBackingStore& b) {
  return a.allocation == b.allocation && a.row_bytes == b.row_bytes &&
         a.height == b.height && a.user_data == b.user_data &&
         a.destruction_callback == b.destruction_callback;
}

inline bool operator==(const FlutterSoftwareBackingStore2& a,
                       const FlutterSoftwareBackingStore2& b) {
  return a.allocation == b.allocation && a.row_bytes == b.row_bytes &&
         a.height == b.height && a.user_data == b.user_data &&
         a.destruction_callback == b.destruction_callback &&
         a.pixel_format == b.pixel_format;
}

inline bool operator==(const FlutterRegion& a, const FlutterRegion& b) {
  if (a.struct_size != b.struct_size || a.rects_count != b.rects_count) {
    return false;
  }
  for (size_t i = 0; i < a.rects_count; i++) {
    if (!(a.rects[i] == b.rects[i])) {
      return false;
    }
  }
  return true;
}

inline bool operator==(const FlutterBackingStorePresentInfo& a,
                       const FlutterBackingStorePresentInfo& b) {
  return a.struct_size == b.struct_size && *a.paint_region == *b.paint_region;
}

inline bool operator==(const FlutterBackingStore& a,
                       const FlutterBackingStore& b) {
  if (!(a.struct_size == b.struct_size && a.user_data == b.user_data &&
        a.type == b.type && a.did_update == b.did_update)) {
    return false;
  }

  switch (a.type) {
    case kFlutterBackingStoreTypeOpenGL:
      return a.open_gl == b.open_gl;
    case kFlutterBackingStoreTypeSoftware:
      return a.software == b.software;
    case kFlutterBackingStoreTypeMetal:
      return a.metal == b.metal;
    case kFlutterBackingStoreTypeVulkan:
      return a.vulkan == b.vulkan;
    case kFlutterBackingStoreTypeSoftware2:
      return a.software2 == b.software2;
  }

  return false;
}

inline bool operator==(const FlutterPlatformViewMutation& a,
                       const FlutterPlatformViewMutation& b) {
  if (a.type != b.type) {
    return false;
  }

  switch (a.type) {
    case kFlutterPlatformViewMutationTypeOpacity:
      return flutter::testing::NumberNear(a.opacity, b.opacity);
    case kFlutterPlatformViewMutationTypeClipRect:
      return a.clip_rect == b.clip_rect;
    case kFlutterPlatformViewMutationTypeClipRoundedRect:
      return a.clip_rounded_rect == b.clip_rounded_rect;
    case kFlutterPlatformViewMutationTypeTransformation:
      return a.transformation == b.transformation;
  }

  return false;
}

inline bool operator==(const FlutterPlatformView& a,
                       const FlutterPlatformView& b) {
  if (!(a.struct_size == b.struct_size && a.identifier == b.identifier &&
        a.mutations_count == b.mutations_count)) {
    return false;
  }

  for (size_t i = 0; i < a.mutations_count; ++i) {
    if (!(*a.mutations[i] == *b.mutations[i])) {
      return false;
    }
  }

  return true;
}

inline bool operator==(const FlutterLayer& a, const FlutterLayer& b) {
  if (!(a.struct_size == b.struct_size && a.type == b.type &&
        a.offset == b.offset && a.size == b.size)) {
    return false;
  }

  switch (a.type) {
    case kFlutterLayerContentTypeBackingStore:
      return *a.backing_store == *b.backing_store &&
             *a.backing_store_present_info == *b.backing_store_present_info;
    case kFlutterLayerContentTypePlatformView:
      return *a.platform_view == *b.platform_view;
  }

  return false;
}

//------------------------------------------------------------------------------
// Printing
//------------------------------------------------------------------------------

inline std::ostream& operator<<(std::ostream& out, const FlutterPoint& point) {
  return out << "(" << point.x << ", " << point.y << ")";
}

inline std::ostream& operator<<(std::ostream& out, const FlutterRect& r) {
  return out << "LTRB (" << r.left << ", " << r.top << ", " << r.right << ", "
             << r.bottom << ")";
}

inline std::ostream& operator<<(std::ostream& out, const FlutterSize& size) {
  return out << "(" << size.width << ", " << size.height << ")";
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterRoundedRect& r) {
  out << "Rect: " << r.rect << ", ";
  out << "Upper Left Corner Radius: " << r.upper_left_corner_radius << ", ";
  out << "Upper Right Corner Radius: " << r.upper_right_corner_radius << ", ";
  out << "Lower Right Corner Radius: " << r.lower_right_corner_radius << ", ";
  out << "Lower Left Corner Radius: " << r.lower_left_corner_radius;
  return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterTransformation& t) {
  out << "Scale X: " << t.scaleX << ", ";
  out << "Skew X: " << t.skewX << ", ";
  out << "Trans X: " << t.transX << ", ";
  out << "Skew Y: " << t.skewY << ", ";
  out << "Scale Y: " << t.scaleY << ", ";
  out << "Trans Y: " << t.transY << ", ";
  out << "Pers 0: " << t.pers0 << ", ";
  out << "Pers 1: " << t.pers1 << ", ";
  out << "Pers 2: " << t.pers2;
  return out;
}

inline std::string FlutterLayerContentTypeToString(
    FlutterLayerContentType type) {
  switch (type) {
    case kFlutterLayerContentTypeBackingStore:
      return "kFlutterLayerContentTypeBackingStore";
    case kFlutterLayerContentTypePlatformView:
      return "kFlutterLayerContentTypePlatformView";
  }
  return "Unknown";
}

inline std::string FlutterBackingStoreTypeToString(
    FlutterBackingStoreType type) {
  switch (type) {
    case kFlutterBackingStoreTypeOpenGL:
      return "kFlutterBackingStoreTypeOpenGL";
    case kFlutterBackingStoreTypeSoftware:
      return "kFlutterBackingStoreTypeSoftware";
    case kFlutterBackingStoreTypeMetal:
      return "kFlutterBackingStoreTypeMetal";
    case kFlutterBackingStoreTypeVulkan:
      return "kFlutterBackingStoreTypeVulkan";
    case kFlutterBackingStoreTypeSoftware2:
      return "kFlutterBackingStoreTypeSoftware2";
  }
  return "Unknown";
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterOpenGLTexture& item) {
  return out << "(FlutterOpenGLTexture) Target: 0x" << std::hex << item.target
             << std::dec << " Name: " << item.name << " Format: " << item.format
             << " User Data: " << item.user_data << " Destruction Callback: "
             << reinterpret_cast<void*>(item.destruction_callback);
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterOpenGLFramebuffer& item) {
  return out << "(FlutterOpenGLFramebuffer) Target: 0x" << std::hex
             << item.target << std::dec << " Name: " << item.name
             << " User Data: " << item.user_data << " Destruction Callback: "
             << reinterpret_cast<void*>(item.destruction_callback);
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterOpenGLSurface& item) {
  return out << "(FlutterOpenGLSurface) Make Current Callback: "
             << reinterpret_cast<void*>(item.make_current_callback)
             << " User Data: " << item.user_data << "Format: " << item.format
             << " Destruction Callback: "
             << reinterpret_cast<void*>(item.destruction_callback);
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterMetalTexture& item) {
  return out << "(FlutterMetalTexture) Texture ID: " << std::hex
             << item.texture_id << std::dec << " Handle: 0x" << std::hex
             << item.texture;
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterVulkanImage& item) {
  return out << "(FlutterVulkanTexture) Image Handle: " << std::hex
             << item.image << std::dec << " Format: " << item.format;
}

inline std::string FlutterPlatformViewMutationTypeToString(
    FlutterPlatformViewMutationType type) {
  switch (type) {
    case kFlutterPlatformViewMutationTypeOpacity:
      return "kFlutterPlatformViewMutationTypeOpacity";
    case kFlutterPlatformViewMutationTypeClipRect:
      return "kFlutterPlatformViewMutationTypeClipRect";
    case kFlutterPlatformViewMutationTypeClipRoundedRect:
      return "kFlutterPlatformViewMutationTypeClipRoundedRect";
    case kFlutterPlatformViewMutationTypeTransformation:
      return "kFlutterPlatformViewMutationTypeTransformation";
  }
  return "Unknown";
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterPlatformViewMutation& m) {
  out << "(FlutterPlatformViewMutation) Type: "
      << FlutterPlatformViewMutationTypeToString(m.type) << " ";
  switch (m.type) {
    case kFlutterPlatformViewMutationTypeOpacity:
      out << "Opacity: " << m.opacity;
    case kFlutterPlatformViewMutationTypeClipRect:
      out << "Clip Rect: " << m.clip_rect;
    case kFlutterPlatformViewMutationTypeClipRoundedRect:
      out << "Clip Rounded Rect: " << m.clip_rounded_rect;
    case kFlutterPlatformViewMutationTypeTransformation:
      out << "Transformation: " << m.transformation;
  }
  return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterPlatformView& platform_view) {
  out << "["
      << "(FlutterPlatformView) Struct Size: " << platform_view.struct_size
      << " Identifier: " << platform_view.identifier
      << " Mutations Count: " << platform_view.mutations_count;

  if (platform_view.mutations_count > 0) {
    out << std::endl;
    for (size_t i = 0; i < platform_view.mutations_count; i++) {
      out << "Mutation " << i << ": " << *platform_view.mutations[i]
          << std::endl;
    }
  }

  out << "]";

  return out;
}

inline std::string FlutterOpenGLTargetTypeToString(
    FlutterOpenGLTargetType type) {
  switch (type) {
    case kFlutterOpenGLTargetTypeTexture:
      return "kFlutterOpenGLTargetTypeTexture";
    case kFlutterOpenGLTargetTypeFramebuffer:
      return "kFlutterOpenGLTargetTypeFramebuffer";
    case kFlutterOpenGLTargetTypeSurface:
      return "kFlutterOpenGLTargetTypeSurface";
  }
  return "Unknown";
}

inline std::string FlutterSoftwarePixelFormatToString(
    FlutterSoftwarePixelFormat pixfmt) {
  switch (pixfmt) {
    case kFlutterSoftwarePixelFormatGray8:
      return "kFlutterSoftwarePixelFormatGray8";
    case kFlutterSoftwarePixelFormatRGB565:
      return "kFlutterSoftwarePixelFormatRGB565";
    case kFlutterSoftwarePixelFormatRGBA4444:
      return "kFlutterSoftwarePixelFormatRGBA4444";
    case kFlutterSoftwarePixelFormatRGBA8888:
      return "kFlutterSoftwarePixelFormatRGBA8888";
    case kFlutterSoftwarePixelFormatRGBX8888:
      return "kFlutterSoftwarePixelFormatRGBX8888";
    case kFlutterSoftwarePixelFormatBGRA8888:
      return "kFlutterSoftwarePixelFormatBGRA8888";
    case kFlutterSoftwarePixelFormatNative32:
      return "kFlutterSoftwarePixelFormatNative32";
    default:
      FML_LOG(ERROR) << "Invalid software rendering pixel format";
  }
  return "Unknown";
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterOpenGLBackingStore& item) {
  out << "(FlutterOpenGLBackingStore) Type: "
      << FlutterOpenGLTargetTypeToString(item.type) << " ";
  switch (item.type) {
    case kFlutterOpenGLTargetTypeTexture:
      out << item.texture;
      break;
    case kFlutterOpenGLTargetTypeFramebuffer:
      out << item.framebuffer;
      break;
    case kFlutterOpenGLTargetTypeSurface:
      out << item.surface;
      break;
  }
  return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterSoftwareBackingStore& item) {
  return out << "(FlutterSoftwareBackingStore) Allocation: " << item.allocation
             << " Row Bytes: " << item.row_bytes << " Height: " << item.height
             << " User Data: " << item.user_data << " Destruction Callback: "
             << reinterpret_cast<void*>(item.destruction_callback);
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterMetalBackingStore& item) {
  return out << "(FlutterMetalBackingStore) Texture: " << item.texture;
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterVulkanBackingStore& item) {
  return out << "(FlutterVulkanBackingStore) Image: " << item.image;
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterSoftwareBackingStore2& item) {
  return out << "(FlutterSoftwareBackingStore2) Allocation: " << item.allocation
             << " Row Bytes: " << item.row_bytes << " Height: " << item.height
             << " User Data: " << item.user_data << " Destruction Callback: "
             << reinterpret_cast<void*>(item.destruction_callback)
             << " Pixel Format: "
             << FlutterSoftwarePixelFormatToString(item.pixel_format);
}

inline std::ostream& operator<<(std::ostream& out,
                                const FlutterBackingStore& backing_store) {
  out << "(FlutterBackingStore) Struct size: " << backing_store.struct_size
      << " User Data: " << backing_store.user_data
      << " Type: " << FlutterBackingStoreTypeToString(backing_store.type)
      << " ";

  switch (backing_store.type) {
    case kFlutterBackingStoreTypeOpenGL:
      out << backing_store.open_gl;
      break;

    case kFlutterBackingStoreTypeSoftware:
      out << backing_store.software;
      break;

    case kFlutterBackingStoreTypeMetal:
      out << backing_store.metal;
      break;

    case kFlutterBackingStoreTypeVulkan:
      out << backing_store.vulkan;
      break;

    case kFlutterBackingStoreTypeSoftware2:
      out << backing_store.software2;
      break;
  }

  return out;
}

inline std::ostream& operator<<(std::ostream& out, const FlutterLayer& layer) {
  out << "(Flutter Layer) Struct size: " << layer.struct_size
      << " Type: " << FlutterLayerContentTypeToString(layer.type);

  switch (layer.type) {
    case kFlutterLayerContentTypeBackingStore:
      out << *layer.backing_store;
      break;
    case kFlutterLayerContentTypePlatformView:
      out << *layer.platform_view;
      break;
  }

  return out << " Offset: " << layer.offset << " Size: " << layer.size;
}

//------------------------------------------------------------------------------
// Factories and Casts
//------------------------------------------------------------------------------

inline FlutterPoint FlutterPointMake(double x, double y) {
  FlutterPoint point = {};
  point.x = x;
  point.y = y;
  return point;
}

inline FlutterSize FlutterSizeMake(double width, double height) {
  FlutterSize size = {};
  size.width = width;
  size.height = height;
  return size;
}

inline FlutterSize FlutterSizeMake(const SkVector& vector) {
  FlutterSize size = {};
  size.width = vector.x();
  size.height = vector.y();
  return size;
}

inline FlutterTransformation FlutterTransformationMake(const SkMatrix& matrix) {
  FlutterTransformation transformation = {};
  transformation.scaleX = matrix[SkMatrix::kMScaleX];
  transformation.skewX = matrix[SkMatrix::kMSkewX];
  transformation.transX = matrix[SkMatrix::kMTransX];
  transformation.skewY = matrix[SkMatrix::kMSkewY];
  transformation.scaleY = matrix[SkMatrix::kMScaleY];
  transformation.transY = matrix[SkMatrix::kMTransY];
  transformation.pers0 = matrix[SkMatrix::kMPersp0];
  transformation.pers1 = matrix[SkMatrix::kMPersp1];
  transformation.pers2 = matrix[SkMatrix::kMPersp2];
  return transformation;
}

inline SkMatrix SkMatrixMake(const FlutterTransformation& xformation) {
  return SkMatrix::MakeAll(xformation.scaleX,  //
                           xformation.skewX,   //
                           xformation.transX,  //
                           xformation.skewY,   //
                           xformation.scaleY,  //
                           xformation.transY,  //
                           xformation.pers0,   //
                           xformation.pers1,   //
                           xformation.pers2    //
  );
}

inline flutter::EmbedderEngine* ToEmbedderEngine(const FlutterEngine& engine) {
  return reinterpret_cast<flutter::EmbedderEngine*>(engine);
}

inline FlutterRect FlutterRectMake(const SkRect& rect) {
  FlutterRect r = {};
  r.left = rect.left();
  r.top = rect.top();
  r.right = rect.right();
  r.bottom = rect.bottom();
  return r;
}

inline FlutterRect FlutterRectMakeLTRB(double l, double t, double r, double b) {
  FlutterRect rect = {};
  rect.left = l;
  rect.top = t;
  rect.right = r;
  rect.bottom = b;
  return rect;
}

inline SkRect SkRectMake(const FlutterRect& rect) {
  return SkRect::MakeLTRB(rect.left, rect.top, rect.right, rect.bottom);
}

inline FlutterRoundedRect FlutterRoundedRectMake(const SkRRect& rect) {
  FlutterRoundedRect r = {};
  r.rect = FlutterRectMake(rect.rect());
  r.upper_left_corner_radius =
      FlutterSizeMake(rect.radii(SkRRect::Corner::kUpperLeft_Corner));
  r.upper_right_corner_radius =
      FlutterSizeMake(rect.radii(SkRRect::Corner::kUpperRight_Corner));
  r.lower_right_corner_radius =
      FlutterSizeMake(rect.radii(SkRRect::Corner::kLowerRight_Corner));
  r.lower_left_corner_radius =
      FlutterSizeMake(rect.radii(SkRRect::Corner::kLowerLeft_Corner));
  return r;
}

// NOLINTEND(google-objc-function-naming)

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_ASSERTIONS_H_
