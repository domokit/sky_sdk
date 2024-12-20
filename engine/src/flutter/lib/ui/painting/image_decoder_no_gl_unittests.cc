// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/image_decoder_no_gl_unittests.h"
#include <memory>

#include "flutter/fml/endianness.h"
#include "impeller/renderer/capabilities.h"
#include "include/core/SkColorType.h"

namespace flutter {
namespace testing {

// Tests are disabled for fuchsia.
#if defined(OS_FUCHSIA)
#pragma GCC diagnostic ignored "-Wunreachable-code"
#endif

namespace {

bool IsPngWithPLTE(const uint8_t* bytes, size_t size) {
  constexpr std::string_view kPngMagic = "\x89PNG\x0d\x0a\x1a\x0a";
  constexpr std::string_view kPngPlte = "PLTE";
  constexpr uint32_t kLengthBytes = 4;
  constexpr uint32_t kTypeBytes = 4;
  constexpr uint32_t kCrcBytes = 4;

  if (size < kPngMagic.size()) {
    return false;
  }

  if (memcmp(bytes, kPngMagic.data(), kPngMagic.size()) != 0) {
    return false;
  }

  const uint8_t* end = bytes + size;
  const uint8_t* loc = bytes + kPngMagic.size();
  while (loc + kLengthBytes + kTypeBytes <= end) {
    uint32_t chunk_length =
        fml::BigEndianToArch(*reinterpret_cast<const uint32_t*>(loc));

    if (memcmp(loc + kLengthBytes, kPngPlte.data(), kPngPlte.size()) == 0) {
      return true;
    }

    loc += kLengthBytes + kTypeBytes + chunk_length + kCrcBytes;
  }

  return false;
}

}  // namespace

float HalfToFloat(uint16_t half) {
  switch (half) {
    case 0x7c00:
      return std::numeric_limits<float>::infinity();
    case 0xfc00:
      return -std::numeric_limits<float>::infinity();
  }
  bool negative = half >> 15;
  uint16_t exponent = (half >> 10) & 0x1f;
  uint16_t fraction = half & 0x3ff;
  float fExponent = exponent - 15.0f;
  float fFraction = static_cast<float>(fraction) / 1024.f;
  float pow_value = powf(2.0f, fExponent);
  return (negative ? -1.f : 1.f) * pow_value * (1.0f + fFraction);
}

float DecodeBGR10(uint32_t x) {
  const float max = 1.25098f;
  const float min = -0.752941f;
  const float intercept = min;
  const float slope = (max - min) / 1024.0f;
  return (x * slope) + intercept;
}

TEST(ImageDecoderNoGLTest, ImpellerWideGamutDisplayP3) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "Fuchsia can't load the test fixtures.";
#endif
  auto data = flutter::testing::OpenFixtureAsSkData("DisplayP3Logo.png");
  auto image = SkImages::DeferredFromEncodedData(data);
  std::shared_ptr<impeller::Capabilities> capabilities =
      impeller::CapabilitiesBuilder()
          .SetSupportsTextureToTextureBlits(true)
          .Build();
  ASSERT_TRUE(image != nullptr);
  ASSERT_EQ(SkISize::Make(100, 100), image->dimensions());

  ImageGeneratorRegistry registry;
  std::shared_ptr<ImageGenerator> generator =
      registry.CreateCompatibleGenerator(data);
  ASSERT_TRUE(generator);

  auto descriptor = fml::MakeRefCounted<ImageDescriptor>(std::move(data),
                                                         std::move(generator));

  ASSERT_FALSE(
      IsPngWithPLTE(descriptor->data()->bytes(), descriptor->data()->size()));

#if IMPELLER_SUPPORTS_RENDERING
  std::shared_ptr<impeller::Allocator> allocator =
      std::make_shared<impeller::TestImpellerAllocator>();
  std::optional<DecompressResult> wide_result =
      ImageDecoderImpeller::DecompressTexture(
          descriptor.get(), SkISize::Make(100, 100), {100, 100},
          /*supports_wide_gamut=*/true, capabilities, allocator);
  ASSERT_TRUE(wide_result.has_value());
  ASSERT_EQ(wide_result->image_info.colorType(), kRGBA_F16_SkColorType);
  ASSERT_TRUE(wide_result->image_info.colorSpace()->isSRGB());

  const SkPixmap& wide_pixmap = wide_result->sk_bitmap->pixmap();
  const uint16_t* half_ptr = static_cast<const uint16_t*>(wide_pixmap.addr());
  bool found_deep_red = false;
  for (int i = 0; i < wide_pixmap.width() * wide_pixmap.height(); ++i) {
    float red = HalfToFloat(*half_ptr++);
    float green = HalfToFloat(*half_ptr++);
    float blue = HalfToFloat(*half_ptr++);
    half_ptr++;  // alpha
    if (fabsf(red - 1.0931f) < 0.01f && fabsf(green - -0.2268f) < 0.01f &&
        fabsf(blue - -0.1501f) < 0.01f) {
      found_deep_red = true;
      break;
    }
  }

  ASSERT_TRUE(found_deep_red);
  std::optional<DecompressResult> narrow_result =
      ImageDecoderImpeller::DecompressTexture(
          descriptor.get(), SkISize::Make(100, 100), {100, 100},
          /*supports_wide_gamut=*/false, capabilities, allocator);

  ASSERT_TRUE(narrow_result.has_value());
  ASSERT_EQ(narrow_result->image_info.colorType(), kRGBA_8888_SkColorType);
#endif  // IMPELLER_SUPPORTS_RENDERING
}

TEST(ImageDecoderNoGLTest, ImpellerWideGamutIndexedPng) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "Fuchsia can't load the test fixtures.";
#endif
  auto data = flutter::testing::OpenFixtureAsSkData("WideGamutIndexed.png");
  auto image = SkImages::DeferredFromEncodedData(data);
  std::shared_ptr<impeller::Capabilities> capabilities =
      impeller::CapabilitiesBuilder()
          .SetSupportsTextureToTextureBlits(true)
          .Build();
  ASSERT_TRUE(image != nullptr);
  ASSERT_EQ(SkISize::Make(100, 100), image->dimensions());

  ImageGeneratorRegistry registry;
  std::shared_ptr<ImageGenerator> generator =
      registry.CreateCompatibleGenerator(data);
  ASSERT_TRUE(generator);

  auto descriptor = fml::MakeRefCounted<ImageDescriptor>(std::move(data),
                                                         std::move(generator));

  ASSERT_TRUE(
      IsPngWithPLTE(descriptor->data()->bytes(), descriptor->data()->size()));

#if IMPELLER_SUPPORTS_RENDERING
  std::shared_ptr<impeller::Allocator> allocator =
      std::make_shared<impeller::TestImpellerAllocator>();
  std::optional<DecompressResult> wide_result =
      ImageDecoderImpeller::DecompressTexture(
          descriptor.get(), SkISize::Make(100, 100), {100, 100},
          /*supports_wide_gamut=*/true, capabilities, allocator);
  ASSERT_EQ(wide_result->image_info.colorType(), kBGR_101010x_XR_SkColorType);
  ASSERT_TRUE(wide_result->image_info.colorSpace()->isSRGB());

  const SkPixmap& wide_pixmap = wide_result->sk_bitmap->pixmap();
  const uint32_t* pixel_ptr = static_cast<const uint32_t*>(wide_pixmap.addr());
  bool found_deep_red = false;
  for (int i = 0; i < wide_pixmap.width() * wide_pixmap.height(); ++i) {
    uint32_t pixel = *pixel_ptr++;
    float blue = DecodeBGR10((pixel >> 0) & 0x3ff);
    float green = DecodeBGR10((pixel >> 10) & 0x3ff);
    float red = DecodeBGR10((pixel >> 20) & 0x3ff);
    if (fabsf(red - 1.0931f) < 0.01f && fabsf(green - -0.2268f) < 0.01f &&
        fabsf(blue - -0.1501f) < 0.01f) {
      found_deep_red = true;
      break;
    }
  }

  ASSERT_TRUE(found_deep_red);
  std::optional<DecompressResult> narrow_result =
      ImageDecoderImpeller::DecompressTexture(
          descriptor.get(), SkISize::Make(100, 100), {100, 100},
          /*supports_wide_gamut=*/false, capabilities, allocator);

  ASSERT_TRUE(narrow_result.has_value());
  ASSERT_EQ(narrow_result->image_info.colorType(), kRGBA_8888_SkColorType);
#endif  // IMPELLER_SUPPORTS_RENDERING
}

TEST(ImageDecoderNoGLTest, ImpellerUnmultipliedAlphaPng) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "Fuchsia can't load the test fixtures.";
#endif
  auto data = flutter::testing::OpenFixtureAsSkData("unmultiplied_alpha.png");
  auto image = SkImages::DeferredFromEncodedData(data);
  std::shared_ptr<impeller::Capabilities> capabilities =
      impeller::CapabilitiesBuilder()
          .SetSupportsTextureToTextureBlits(true)
          .Build();
  ASSERT_TRUE(image != nullptr);
  ASSERT_EQ(SkISize::Make(11, 11), image->dimensions());

  ImageGeneratorRegistry registry;
  std::shared_ptr<ImageGenerator> generator =
      registry.CreateCompatibleGenerator(data);
  ASSERT_TRUE(generator);

  auto descriptor = fml::MakeRefCounted<ImageDescriptor>(std::move(data),
                                                         std::move(generator));

#if IMPELLER_SUPPORTS_RENDERING
  std::shared_ptr<impeller::Allocator> allocator =
      std::make_shared<impeller::TestImpellerAllocator>();
  std::optional<DecompressResult> result =
      ImageDecoderImpeller::DecompressTexture(
          descriptor.get(), SkISize::Make(11, 11), {11, 11},
          /*supports_wide_gamut=*/true, capabilities, allocator);
  ASSERT_EQ(result->image_info.colorType(), kRGBA_8888_SkColorType);

  const SkPixmap& pixmap = result->sk_bitmap->pixmap();
  const uint32_t* pixel_ptr = static_cast<const uint32_t*>(pixmap.addr());
  // Test the upper left pixel is premultiplied and not solid red.
  ASSERT_EQ(*pixel_ptr, (uint32_t)0x1000001);
  // Test a pixel in the green box is still green.
  ASSERT_EQ(*(pixel_ptr + 11 * 4 + 4), (uint32_t)0xFF00FF00);

#endif  // IMPELLER_SUPPORTS_RENDERING
}

}  // namespace testing
}  // namespace flutter
