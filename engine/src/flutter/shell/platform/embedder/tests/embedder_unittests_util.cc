// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include <limits>
#include <utility>

#include "flutter/shell/platform/embedder/tests/embedder_test_backingstore_producer.h"
#include "flutter/shell/platform/embedder/tests/embedder_unittests_util.h"

#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/encode/SkPngEncoder.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"

namespace flutter {
namespace testing {

sk_sp<SkSurface> CreateRenderSurface(const FlutterLayer& layer,
                                     GrDirectContext* context) {
  const auto image_info =
      SkImageInfo::MakeN32Premul(layer.size.width, layer.size.height);
  auto surface = context ? SkSurfaces::RenderTarget(
                               context,                   // context
                               skgpu::Budgeted::kNo,      // budgeted
                               image_info,                // image info
                               1,                         // sample count
                               kTopLeft_GrSurfaceOrigin,  // surface origin
                               nullptr,                   // surface properties
                               false                      // mipmaps

                               )
                         : SkSurfaces::Raster(image_info);
  FML_CHECK(surface != nullptr);
  return surface;
}

// Normalizes the color-space, color-type and alpha-type for comparison.
static sk_sp<SkData> NormalizeImage(const sk_sp<SkImage>& image) {
  // To avoid clipping, convert to a very wide gamut, and a high bit depth.
  sk_sp<SkColorSpace> norm_colorspace = SkColorSpace::MakeRGB(
      SkNamedTransferFn::kRec2020, SkNamedGamut::kRec2020);
  SkImageInfo norm_image_info =
      SkImageInfo::Make(image->width(), image->height(),
                        SkColorType::kR16G16B16A16_unorm_SkColorType,
                        SkAlphaType::kUnpremul_SkAlphaType, norm_colorspace);
  size_t row_bytes = norm_image_info.minRowBytes();
  size_t size = norm_image_info.computeByteSize(row_bytes);
  sk_sp<SkData> data = SkData::MakeUninitialized(size);
  if (!data) {
    FML_CHECK(false) << "Unable to allocate data.";
  }

  bool success = image->readPixels(norm_image_info, data->writable_data(),
                                   row_bytes, 0, 0);
  if (!success) {
    FML_CHECK(false) << "Unable to read pixels.";
  }

  return data;
}

bool RasterImagesAreSame(const sk_sp<SkImage>& a, const sk_sp<SkImage>& b) {
  if (!a || !b) {
    return false;
  }

  FML_CHECK(!a->isTextureBacked());
  FML_CHECK(!b->isTextureBacked());

  sk_sp<SkData> normalized_a = NormalizeImage(a);
  sk_sp<SkData> normalized_b = NormalizeImage(b);

  return normalized_a->equals(normalized_b.get());
}

std::string FixtureNameForBackend(EmbedderTestContextType backend,
                                  const std::string& name) {
  switch (backend) {
    case EmbedderTestContextType::kVulkanContext:
      return "vk_" + name;
    default:
      return name;
  }
}

EmbedderTestBackingStoreProducer::RenderTargetType GetRenderTargetFromBackend(
    EmbedderTestContextType backend,
    bool opengl_framebuffer) {
  switch (backend) {
    case EmbedderTestContextType::kVulkanContext:
      return EmbedderTestBackingStoreProducer::RenderTargetType::kVulkanImage;
    case EmbedderTestContextType::kOpenGLContext:
      if (opengl_framebuffer) {
        return EmbedderTestBackingStoreProducer::RenderTargetType::
            kOpenGLFramebuffer;
      }
      return EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture;
    case EmbedderTestContextType::kMetalContext:
      return EmbedderTestBackingStoreProducer::RenderTargetType::kMetalTexture;
    case EmbedderTestContextType::kSoftwareContext:
      return EmbedderTestBackingStoreProducer::RenderTargetType::
          kSoftwareBuffer;
  }
}

void ConfigureBackingStore(FlutterBackingStore& backing_store,
                           EmbedderTestContextType backend,
                           bool opengl_framebuffer) {
  switch (backend) {
    case EmbedderTestContextType::kVulkanContext:
      backing_store.type = kFlutterBackingStoreTypeVulkan;
      break;
    case EmbedderTestContextType::kOpenGLContext:
      if (opengl_framebuffer) {
        backing_store.type = kFlutterBackingStoreTypeOpenGL;
        backing_store.open_gl.type = kFlutterOpenGLTargetTypeFramebuffer;
      } else {
        backing_store.type = kFlutterBackingStoreTypeOpenGL;
        backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;
      }
      break;
    case EmbedderTestContextType::kMetalContext:
      backing_store.type = kFlutterBackingStoreTypeMetal;
      break;
    case EmbedderTestContextType::kSoftwareContext:
      backing_store.type = kFlutterBackingStoreTypeSoftware;
      break;
  }
}

bool WriteImageToDisk(const fml::UniqueFD& directory,
                      const std::string& name,
                      const sk_sp<SkImage>& image) {
  if (!image) {
    return false;
  }

  auto data = SkPngEncoder::Encode(nullptr, image.get(), {});

  if (!data) {
    return false;
  }

  fml::NonOwnedMapping mapping(static_cast<const uint8_t*>(data->data()),
                               data->size());
  return WriteAtomically(directory, name.c_str(), mapping);
}

bool ImageMatchesFixture(const std::string& fixture_file_name,
                         const sk_sp<SkImage>& scene_image) {
  fml::FileMapping fixture_image_mapping(OpenFixture(fixture_file_name));

  FML_CHECK(fixture_image_mapping.GetSize() != 0u)
      << "Could not find fixture: " << fixture_file_name;

  auto encoded_image = SkData::MakeWithoutCopy(
      fixture_image_mapping.GetMapping(), fixture_image_mapping.GetSize());
  auto fixture_image =
      SkImages::DeferredFromEncodedData(std::move(encoded_image))
          ->makeRasterImage();

  FML_CHECK(fixture_image) << "Could not create image from fixture: "
                           << fixture_file_name;

  FML_CHECK(scene_image) << "Invalid scene image.";

  auto scene_image_subset = scene_image->makeSubset(
      nullptr,
      SkIRect::MakeWH(fixture_image->width(), fixture_image->height()));

  FML_CHECK(scene_image_subset)
      << "Could not create image subset for fixture comparison: "
      << scene_image_subset;

  const auto images_are_same =
      RasterImagesAreSame(scene_image_subset, fixture_image);

  // If the images are not the same, this predicate is going to indicate test
  // failure. Dump both the actual image and the expectation to disk to the
  // test author can figure out what went wrong.
  if (!images_are_same) {
    const auto fixtures_path = GetFixturesPath();

    const auto actual_file_name = "actual_" + fixture_file_name;
    const auto expect_file_name = "expectation_" + fixture_file_name;

    auto fixtures_fd = OpenFixturesDirectory();

    FML_CHECK(
        WriteImageToDisk(fixtures_fd, actual_file_name, scene_image_subset))
        << "Could not write file to disk: " << actual_file_name;

    FML_CHECK(WriteImageToDisk(fixtures_fd, expect_file_name, fixture_image))
        << "Could not write file to disk: " << expect_file_name;

    FML_LOG(ERROR) << "Image did not match expectation." << std::endl
                   << "Expected:"
                   << fml::paths::JoinPaths({fixtures_path, expect_file_name})
                   << std::endl
                   << "Got:"
                   << fml::paths::JoinPaths({fixtures_path, actual_file_name})
                   << std::endl;
  }
  return images_are_same;
}

bool ImageMatchesFixture(const std::string& fixture_file_name,
                         std::future<sk_sp<SkImage>>& scene_image) {
  return ImageMatchesFixture(fixture_file_name, scene_image.get());
}

bool SurfacePixelDataMatchesBytes(SkSurface* surface,
                                  const std::vector<uint8_t>& bytes) {
  SkPixmap pixmap;
  auto ok = surface->peekPixels(&pixmap);
  if (!ok) {
    return false;
  }

  auto matches = (pixmap.rowBytes() == bytes.size()) &&
                 (memcmp(bytes.data(), pixmap.addr(), bytes.size()) == 0);

  if (!matches) {
    FML_LOG(ERROR) << "SkImage pixel data didn't match bytes.";

    {
      const uint8_t* addr = static_cast<const uint8_t*>(pixmap.addr());
      std::stringstream stream;
      for (size_t i = 0; i < pixmap.computeByteSize(); ++i) {
        stream << "0x" << std::setfill('0') << std::setw(2) << std::uppercase
               << std::hex << static_cast<int>(addr[i]);
        if (i != pixmap.computeByteSize() - 1) {
          stream << ", ";
        }
      }
      FML_LOG(ERROR) << "  Actual:   " << stream.str();
    }
    {
      std::stringstream stream;
      for (auto b = bytes.begin(); b != bytes.end(); ++b) {
        stream << "0x" << std::setfill('0') << std::setw(2) << std::uppercase
               << std::hex << static_cast<int>(*b);
        if (b != bytes.end() - 1) {
          stream << ", ";
        }
      }
      FML_LOG(ERROR) << "  Expected: " << stream.str();
    }
  }

  return matches;
}

bool SurfacePixelDataMatchesBytes(std::future<SkSurface*>& surface_future,
                                  const std::vector<uint8_t>& bytes) {
  return SurfacePixelDataMatchesBytes(surface_future.get(), bytes);
}

void FilterMutationsByType(
    const FlutterPlatformViewMutation** mutations,
    size_t count,
    FlutterPlatformViewMutationType type,
    const std::function<void(const FlutterPlatformViewMutation& mutation)>&
        handler) {
  if (mutations == nullptr) {
    return;
  }

  for (size_t i = 0; i < count; ++i) {
    const FlutterPlatformViewMutation* mutation = mutations[i];
    if (mutation->type != type) {
      continue;
    }

    handler(*mutation);
  }
}

void FilterMutationsByType(
    const FlutterPlatformView* view,
    FlutterPlatformViewMutationType type,
    const std::function<void(const FlutterPlatformViewMutation& mutation)>&
        handler) {
  return FilterMutationsByType(view->mutations, view->mutations_count, type,
                               handler);
}

SkMatrix GetTotalMutationTransformationMatrix(
    const FlutterPlatformViewMutation** mutations,
    size_t count) {
  SkMatrix collected;

  FilterMutationsByType(
      mutations, count, kFlutterPlatformViewMutationTypeTransformation,
      [&](const auto& mutation) {
        collected.preConcat(SkMatrixMake(mutation.transformation));
      });

  return collected;
}

SkMatrix GetTotalMutationTransformationMatrix(const FlutterPlatformView* view) {
  return GetTotalMutationTransformationMatrix(view->mutations,
                                              view->mutations_count);
}

}  // namespace testing
}  // namespace flutter
