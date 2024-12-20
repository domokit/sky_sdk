// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/text/paragraph_builder.h"

#include <cstring>

#include "flutter/common/settings.h"
#include "flutter/common/task_runners.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/task_runner.h"
#include "flutter/lib/ui/text/font_collection.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "flutter/lib/ui/window/platform_configuration.h"
#include "flutter/third_party/txt/src/txt/font_style.h"
#include "flutter/third_party/txt/src/txt/font_weight.h"
#include "flutter/third_party/txt/src/txt/paragraph_style.h"
#include "flutter/third_party/txt/src/txt/text_baseline.h"
#include "flutter/third_party/txt/src/txt/text_decoration.h"
#include "flutter/third_party/txt/src/txt/text_style.h"
#include "third_party/icu/source/common/unicode/ustring.h"
#include "third_party/skia/include/core/SkColor.h"
#include "third_party/tonic/converter/dart_converter.h"
#include "third_party/tonic/dart_args.h"
#include "third_party/tonic/dart_binding_macros.h"
#include "third_party/tonic/dart_library_natives.h"
#include "third_party/tonic/typed_data/dart_byte_data.h"

namespace flutter {
namespace {

const double kTextHeightNone = 0.0;

// TextStyle

const int kTSLeadingDistributionIndex = 0;
const int kTSColorIndex = 1;
const int kTSTextDecorationIndex = 2;
const int kTSTextDecorationColorIndex = 3;
const int kTSTextDecorationStyleIndex = 4;
const int kTSFontWeightIndex = 5;
const int kTSFontStyleIndex = 6;
const int kTSTextBaselineIndex = 7;
const int kTSTextDecorationThicknessIndex = 8;
const int kTSFontFamilyIndex = 9;
const int kTSFontSizeIndex = 10;
const int kTSLetterSpacingIndex = 11;
const int kTSWordSpacingIndex = 12;
const int kTSHeightIndex = 13;
const int kTSLocaleIndex = 14;
const int kTSBackgroundIndex = 15;
const int kTSForegroundIndex = 16;
const int kTSTextShadowsIndex = 17;
const int kTSFontFeaturesIndex = 18;
const int kTSFontVariationsIndex = 19;

const int kTSLeadingDistributionMask = 1 << kTSLeadingDistributionIndex;
const int kTSColorMask = 1 << kTSColorIndex;
const int kTSTextDecorationMask = 1 << kTSTextDecorationIndex;
const int kTSTextDecorationColorMask = 1 << kTSTextDecorationColorIndex;
const int kTSTextDecorationStyleMask = 1 << kTSTextDecorationStyleIndex;
const int kTSTextDecorationThicknessMask = 1 << kTSTextDecorationThicknessIndex;
const int kTSFontWeightMask = 1 << kTSFontWeightIndex;
const int kTSFontStyleMask = 1 << kTSFontStyleIndex;
const int kTSTextBaselineMask = 1 << kTSTextBaselineIndex;
const int kTSFontFamilyMask = 1 << kTSFontFamilyIndex;
const int kTSFontSizeMask = 1 << kTSFontSizeIndex;
const int kTSLetterSpacingMask = 1 << kTSLetterSpacingIndex;
const int kTSWordSpacingMask = 1 << kTSWordSpacingIndex;
const int kTSHeightMask = 1 << kTSHeightIndex;
const int kTSLocaleMask = 1 << kTSLocaleIndex;
const int kTSBackgroundMask = 1 << kTSBackgroundIndex;
const int kTSForegroundMask = 1 << kTSForegroundIndex;
const int kTSTextShadowsMask = 1 << kTSTextShadowsIndex;
const int kTSFontFeaturesMask = 1 << kTSFontFeaturesIndex;
const int kTSFontVariationsMask = 1 << kTSFontVariationsIndex;

// ParagraphStyle

const int kPSTextAlignIndex = 1;
const int kPSTextDirectionIndex = 2;
const int kPSFontWeightIndex = 3;
const int kPSFontStyleIndex = 4;
const int kPSMaxLinesIndex = 5;
const int kPSTextHeightBehaviorIndex = 6;
const int kPSFontFamilyIndex = 7;
const int kPSFontSizeIndex = 8;
const int kPSHeightIndex = 9;
const int kPSStrutStyleIndex = 10;
const int kPSEllipsisIndex = 11;
const int kPSLocaleIndex = 12;

const int kPSTextAlignMask = 1 << kPSTextAlignIndex;
const int kPSTextDirectionMask = 1 << kPSTextDirectionIndex;
const int kPSFontWeightMask = 1 << kPSFontWeightIndex;
const int kPSFontStyleMask = 1 << kPSFontStyleIndex;
const int kPSMaxLinesMask = 1 << kPSMaxLinesIndex;
const int kPSFontFamilyMask = 1 << kPSFontFamilyIndex;
const int kPSFontSizeMask = 1 << kPSFontSizeIndex;
const int kPSHeightMask = 1 << kPSHeightIndex;
const int kPSTextHeightBehaviorMask = 1 << kPSTextHeightBehaviorIndex;
const int kPSStrutStyleMask = 1 << kPSStrutStyleIndex;
const int kPSEllipsisMask = 1 << kPSEllipsisIndex;
const int kPSLocaleMask = 1 << kPSLocaleIndex;

// TextShadows decoding

constexpr uint32_t kColorDefault = 0xFF000000;
constexpr uint32_t kBytesPerShadow = 16;
constexpr uint32_t kShadowPropertiesCount = 4;
constexpr uint32_t kColorOffset = 0;
constexpr uint32_t kXOffset = 1;
constexpr uint32_t kYOffset = 2;
constexpr uint32_t kBlurOffset = 3;

// FontFeature decoding
constexpr uint32_t kBytesPerFontFeature = 8;
constexpr uint32_t kFontFeatureTagLength = 4;

// FontVariation decoding
constexpr uint32_t kBytesPerFontVariation = 8;
constexpr uint32_t kFontVariationTagLength = 4;

// Strut decoding
const int kSFontWeightIndex = 0;
const int kSFontStyleIndex = 1;
const int kSFontFamilyIndex = 2;
const int kSLeadingDistributionIndex = 3;
const int kSFontSizeIndex = 4;
const int kSHeightIndex = 5;
const int kSLeadingIndex = 6;
const int kSForceStrutHeightIndex = 7;

const int kSFontWeightMask = 1 << kSFontWeightIndex;
const int kSFontStyleMask = 1 << kSFontStyleIndex;
const int kSFontFamilyMask = 1 << kSFontFamilyIndex;
const int kSLeadingDistributionMask = 1 << kSLeadingDistributionIndex;
const int kSFontSizeMask = 1 << kSFontSizeIndex;
const int kSHeightMask = 1 << kSHeightIndex;
const int kSLeadingMask = 1 << kSLeadingIndex;
const int kSForceStrutHeightMask = 1 << kSForceStrutHeightIndex;

}  // namespace

IMPLEMENT_WRAPPERTYPEINFO(ui, ParagraphBuilder);

void ParagraphBuilder::Create(Dart_Handle wrapper,
                              Dart_Handle encoded_handle,
                              Dart_Handle strutData,
                              const std::string& fontFamily,
                              const std::vector<std::string>& strutFontFamilies,
                              double fontSize,
                              double height,
                              const std::u16string& ellipsis,
                              const std::string& locale) {
  UIDartState::ThrowIfUIOperationsProhibited();
  auto res = fml::MakeRefCounted<ParagraphBuilder>(
      encoded_handle, strutData, fontFamily, strutFontFamilies, fontSize,
      height, ellipsis, locale);
  res->AssociateWithDartWrapper(wrapper);
}

// returns true if there is a font family defined. Font family is the only
// parameter passed directly.
void decodeStrut(Dart_Handle strut_data,
                 const std::vector<std::string>& strut_font_families,
                 txt::ParagraphStyle& paragraph_style) {  // NOLINT
  if (strut_data == Dart_Null()) {
    return;
  }

  tonic::DartByteData byte_data(strut_data);
  if (byte_data.length_in_bytes() == 0) {
    return;
  }
  paragraph_style.strut_enabled = true;

  const uint8_t* uint8_data = static_cast<const uint8_t*>(byte_data.data());
  uint8_t mask = uint8_data[0];

  // Data is stored in order of increasing size, eg, 8 bit ints will be before
  // any 32 bit ints. In addition, the order of decoding is the same order
  // as it is encoded, and the order is used to maintain consistency.
  size_t byte_count = 1;
  if (mask & kSFontWeightMask) {
    paragraph_style.strut_font_weight =
        static_cast<txt::FontWeight>(uint8_data[byte_count++]);
  }
  if (mask & kSFontStyleMask) {
    paragraph_style.strut_font_style =
        static_cast<txt::FontStyle>(uint8_data[byte_count++]);
  }

  paragraph_style.strut_half_leading = mask & kSLeadingDistributionMask;

  std::vector<float> float_data;
  float_data.resize((byte_data.length_in_bytes() - byte_count) / 4);
  memcpy(float_data.data(),
         static_cast<const char*>(byte_data.data()) + byte_count,
         byte_data.length_in_bytes() - byte_count);
  size_t float_count = 0;
  if (mask & kSFontSizeMask) {
    paragraph_style.strut_font_size = float_data[float_count++];
  }
  if (mask & kSHeightMask) {
    paragraph_style.strut_height = float_data[float_count++];
    paragraph_style.strut_has_height_override = true;
  }
  if (mask & kSLeadingMask) {
    paragraph_style.strut_leading = float_data[float_count++];
  }

  // The boolean is stored as the last bit in the bitmask, as null
  // and false have the same behavior.
  paragraph_style.force_strut_height = mask & kSForceStrutHeightMask;

  if (mask & kSFontFamilyMask) {
    paragraph_style.strut_font_families = strut_font_families;
  } else {
    // Provide an empty font name so that the platform default font will be
    // used.
    paragraph_style.strut_font_families.push_back("");
  }
}

ParagraphBuilder::ParagraphBuilder(
    Dart_Handle encoded_data,
    Dart_Handle strutData,
    const std::string& fontFamily,
    const std::vector<std::string>& strutFontFamilies,
    double fontSize,
    double height,
    const std::u16string& ellipsis,
    const std::string& locale) {
  int32_t mask = 0;
  txt::ParagraphStyle style;
  {
    tonic::Int32List encoded(encoded_data);

    mask = encoded[0];

    if (mask & kPSTextAlignMask) {
      style.text_align =
          static_cast<txt::TextAlign>(encoded[kPSTextAlignIndex]);
    }

    if (mask & kPSTextDirectionMask) {
      style.text_direction =
          static_cast<txt::TextDirection>(encoded[kPSTextDirectionIndex]);
    }

    if (mask & kPSFontWeightMask) {
      style.font_weight =
          static_cast<txt::FontWeight>(encoded[kPSFontWeightIndex]);
    }

    if (mask & kPSFontStyleMask) {
      style.font_style =
          static_cast<txt::FontStyle>(encoded[kPSFontStyleIndex]);
    }

    if (mask & kPSFontFamilyMask) {
      style.font_family = fontFamily;
    }

    if (mask & kPSFontSizeMask) {
      style.font_size = fontSize;
    }

    if (mask & kPSHeightMask) {
      style.height = height;
      style.has_height_override = true;
    }

    if (mask & kPSTextHeightBehaviorMask) {
      style.text_height_behavior = encoded[kPSTextHeightBehaviorIndex];
    }

    if (mask & kPSMaxLinesMask) {
      style.max_lines = encoded[kPSMaxLinesIndex];
    }
  }

  if (mask & kPSStrutStyleMask) {
    decodeStrut(strutData, strutFontFamilies, style);
  }

  if (mask & kPSEllipsisMask) {
    style.ellipsis = ellipsis;
  }

  if (mask & kPSLocaleMask) {
    style.locale = locale;
  }

  FontCollection& font_collection = UIDartState::Current()
                                        ->platform_configuration()
                                        ->client()
                                        ->GetFontCollection();

  auto impeller_enabled = UIDartState::Current()->IsImpellerEnabled();
  m_paragraph_builder_ = txt::ParagraphBuilder::CreateSkiaBuilder(
      style, font_collection.GetFontCollection(), impeller_enabled);
}

ParagraphBuilder::~ParagraphBuilder() = default;

void decodeTextShadows(
    Dart_Handle shadows_data,
    std::vector<txt::TextShadow>& decoded_shadows) {  // NOLINT
  decoded_shadows.clear();

  tonic::DartByteData byte_data(shadows_data);
  FML_CHECK(byte_data.length_in_bytes() % kBytesPerShadow == 0);

  const uint32_t* uint_data = static_cast<const uint32_t*>(byte_data.data());
  const float* float_data = static_cast<const float*>(byte_data.data());

  size_t shadow_count = byte_data.length_in_bytes() / kBytesPerShadow;
  size_t shadow_count_offset = 0;
  for (size_t shadow_index = 0; shadow_index < shadow_count; ++shadow_index) {
    shadow_count_offset = shadow_index * kShadowPropertiesCount;
    SkColor color =
        uint_data[shadow_count_offset + kColorOffset] ^ kColorDefault;
    decoded_shadows.emplace_back(
        color,
        SkPoint::Make(float_data[shadow_count_offset + kXOffset],
                      float_data[shadow_count_offset + kYOffset]),
        float_data[shadow_count_offset + kBlurOffset]);
  }
}

void decodeFontFeatures(Dart_Handle font_features_data,
                        txt::FontFeatures& font_features) {  // NOLINT
  tonic::DartByteData byte_data(font_features_data);
  FML_CHECK(byte_data.length_in_bytes() % kBytesPerFontFeature == 0);

  size_t feature_count = byte_data.length_in_bytes() / kBytesPerFontFeature;
  for (size_t feature_index = 0; feature_index < feature_count;
       ++feature_index) {
    size_t feature_offset = feature_index * kBytesPerFontFeature;
    const char* feature_bytes =
        static_cast<const char*>(byte_data.data()) + feature_offset;
    std::string tag(feature_bytes, kFontFeatureTagLength);
    int32_t value = *(reinterpret_cast<const int32_t*>(feature_bytes +
                                                       kFontFeatureTagLength));
    font_features.SetFeature(tag, value);
  }
}

void decodeFontVariations(Dart_Handle font_variations_data,
                          txt::FontVariations& font_variations) {  // NOLINT
  tonic::DartByteData byte_data(font_variations_data);
  FML_CHECK(byte_data.length_in_bytes() % kBytesPerFontVariation == 0);

  size_t variation_count = byte_data.length_in_bytes() / kBytesPerFontVariation;
  for (size_t variation_index = 0; variation_index < variation_count;
       ++variation_index) {
    size_t variation_offset = variation_index * kBytesPerFontVariation;
    const char* variation_bytes =
        static_cast<const char*>(byte_data.data()) + variation_offset;
    std::string tag(variation_bytes, kFontVariationTagLength);
    float value = *(reinterpret_cast<const float*>(variation_bytes +
                                                   kFontVariationTagLength));
    font_variations.SetAxisValue(tag, value);
  }
}

void ParagraphBuilder::pushStyle(const tonic::Int32List& encoded,
                                 const std::vector<std::string>& fontFamilies,
                                 double fontSize,
                                 double letterSpacing,
                                 double wordSpacing,
                                 double height,
                                 double decorationThickness,
                                 const std::string& locale,
                                 Dart_Handle background_objects,
                                 Dart_Handle background_data,
                                 Dart_Handle foreground_objects,
                                 Dart_Handle foreground_data,
                                 Dart_Handle shadows_data,
                                 Dart_Handle font_features_data,
                                 Dart_Handle font_variations_data) {
  FML_DCHECK(encoded.num_elements() == 9);

  int32_t mask = encoded[0];

  // Set to use the properties of the previous style if the property is not
  // explicitly given.
  txt::TextStyle style = m_paragraph_builder_->PeekStyle();

  style.half_leading = mask & kTSLeadingDistributionMask;
  // Only change the style property from the previous value if a new explicitly
  // set value is available
  if (mask & kTSColorMask) {
    style.color = encoded[kTSColorIndex];
  }

  if (mask & kTSTextDecorationMask) {
    style.decoration =
        static_cast<txt::TextDecoration>(encoded[kTSTextDecorationIndex]);
  }

  if (mask & kTSTextDecorationColorMask) {
    style.decoration_color = encoded[kTSTextDecorationColorIndex];
  }

  if (mask & kTSTextDecorationStyleMask) {
    style.decoration_style = static_cast<txt::TextDecorationStyle>(
        encoded[kTSTextDecorationStyleIndex]);
  }

  if (mask & kTSTextDecorationThicknessMask) {
    style.decoration_thickness_multiplier = decorationThickness;
  }

  if (mask & kTSTextBaselineMask) {
    // TODO(abarth): Implement TextBaseline. The CSS version of this
    // property wasn't wired up either.
  }

  if (mask & (kTSFontWeightMask | kTSFontStyleMask | kTSFontSizeMask |
              kTSLetterSpacingMask | kTSWordSpacingMask)) {
    if (mask & kTSFontWeightMask) {
      style.font_weight =
          static_cast<txt::FontWeight>(encoded[kTSFontWeightIndex]);
    }

    if (mask & kTSFontStyleMask) {
      style.font_style =
          static_cast<txt::FontStyle>(encoded[kTSFontStyleIndex]);
    }

    if (mask & kTSFontSizeMask) {
      style.font_size = fontSize;
    }

    if (mask & kTSLetterSpacingMask) {
      style.letter_spacing = letterSpacing;
    }

    if (mask & kTSWordSpacingMask) {
      style.word_spacing = wordSpacing;
    }
  }

  if (mask & kTSHeightMask) {
    style.height = height;
    style.has_height_override = style.height != kTextHeightNone;
  }

  if (mask & kTSLocaleMask) {
    style.locale = locale;
  }

  if (mask & kTSBackgroundMask) {
    Paint background(background_objects, background_data);
    if (background.isNotNull()) {
      DlPaint dl_paint;
      background.toDlPaint(dl_paint, DlTileMode::kDecal);
      style.background = dl_paint;
    }
  }

  if (mask & kTSForegroundMask) {
    Paint foreground(foreground_objects, foreground_data);
    if (foreground.isNotNull()) {
      DlPaint dl_paint;
      foreground.toDlPaint(dl_paint, DlTileMode::kDecal);
      style.foreground = dl_paint;
    }
  }

  if (mask & kTSTextShadowsMask) {
    decodeTextShadows(shadows_data, style.text_shadows);
  }

  if (mask & kTSFontFamilyMask) {
    // The child style's font families override the parent's font families.
    // If the child's fonts are not available, then the font collection will
    // use the system fallback fonts (not the parent's fonts).
    style.font_families = fontFamilies;
  }

  if (mask & kTSFontFeaturesMask) {
    decodeFontFeatures(font_features_data, style.font_features);
  }

  if (mask & kTSFontVariationsMask) {
    decodeFontVariations(font_variations_data, style.font_variations);
  }

  m_paragraph_builder_->PushStyle(style);
}

void ParagraphBuilder::pop() {
  m_paragraph_builder_->Pop();
}

Dart_Handle ParagraphBuilder::addText(const std::u16string& text) {
  if (text.empty()) {
    return Dart_Null();
  }

  // Use ICU to validate the UTF-16 input.  Calling u_strToUTF8 with a null
  // output buffer will return U_BUFFER_OVERFLOW_ERROR if the input is well
  // formed.
  const UChar* text_ptr = reinterpret_cast<const UChar*>(text.data());
  UErrorCode error_code = U_ZERO_ERROR;
  u_strToUTF8(nullptr, 0, nullptr, text_ptr, text.size(), &error_code);
  if (error_code != U_BUFFER_OVERFLOW_ERROR) {
    return tonic::ToDart("string is not well-formed UTF-16");
  }

  m_paragraph_builder_->AddText(text);

  return Dart_Null();
}

void ParagraphBuilder::addPlaceholder(double width,
                                      double height,
                                      unsigned alignment,
                                      double baseline_offset,
                                      unsigned baseline) {
  txt::PlaceholderRun placeholder_run(
      width, height, static_cast<txt::PlaceholderAlignment>(alignment),
      static_cast<txt::TextBaseline>(baseline), baseline_offset);

  m_paragraph_builder_->AddPlaceholder(placeholder_run);
}

void ParagraphBuilder::build(Dart_Handle paragraph_handle) {
  Paragraph::Create(paragraph_handle, m_paragraph_builder_->Build());
  m_paragraph_builder_.reset();
  ClearDartWrapper();
}

}  // namespace flutter
