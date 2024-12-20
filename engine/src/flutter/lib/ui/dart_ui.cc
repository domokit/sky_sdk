// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/dart_ui.h"

#include <mutex>
#include <string_view>

#include "flutter/common/constants.h"
#include "flutter/common/settings.h"
#include "flutter/fml/build_config.h"
#include "flutter/lib/ui/compositing/scene.h"
#include "flutter/lib/ui/compositing/scene_builder.h"
#include "flutter/lib/ui/dart_runtime_hooks.h"
#include "flutter/lib/ui/isolate_name_server/isolate_name_server_natives.h"
#include "flutter/lib/ui/painting/canvas.h"
#include "flutter/lib/ui/painting/codec.h"
#include "flutter/lib/ui/painting/color_filter.h"
#include "flutter/lib/ui/painting/engine_layer.h"
#include "flutter/lib/ui/painting/fragment_program.h"
#include "flutter/lib/ui/painting/fragment_shader.h"
#include "flutter/lib/ui/painting/gradient.h"
#include "flutter/lib/ui/painting/image.h"
#include "flutter/lib/ui/painting/image_descriptor.h"
#include "flutter/lib/ui/painting/image_filter.h"
#include "flutter/lib/ui/painting/image_shader.h"
#include "flutter/lib/ui/painting/immutable_buffer.h"
#include "flutter/lib/ui/painting/path.h"
#include "flutter/lib/ui/painting/path_measure.h"
#include "flutter/lib/ui/painting/picture.h"
#include "flutter/lib/ui/painting/picture_recorder.h"
#include "flutter/lib/ui/painting/vertices.h"
#include "flutter/lib/ui/semantics/semantics_update.h"
#include "flutter/lib/ui/semantics/semantics_update_builder.h"
#include "flutter/lib/ui/semantics/string_attribute.h"
#include "flutter/lib/ui/text/font_collection.h"
#include "flutter/lib/ui/text/paragraph.h"
#include "flutter/lib/ui/text/paragraph_builder.h"
#include "flutter/lib/ui/window/platform_configuration.h"
#include "flutter/lib/ui/window/platform_isolate.h"
#include "third_party/tonic/converter/dart_converter.h"
#include "third_party/tonic/dart_args.h"
#include "third_party/tonic/logging/dart_error.h"

using tonic::ToDart;

namespace flutter {

typedef CanvasImage Image;
typedef CanvasPathMeasure PathMeasure;
typedef CanvasGradient Gradient;
typedef CanvasPath Path;

// List of native static functions used as @Native functions.
// Items are tuples of ('function_name', 'parameter_count'), where:
//   'function_name' is the fully qualified name of the native function.
//   'parameter_count' is the number of parameters the function has.
//
// These are used to:
// - Instantiate FfiDispatcher templates to automatically create FFI Native
//   bindings.
//   If the name does not match a native function, the template will fail to
//   instatiate, resulting in a compile time error.
// - Resolve the native function pointer associated with an @Native function.
//   If there is a mismatch between name or parameter count an @Native is
//   trying to resolve, an exception will be thrown.
#define FFI_FUNCTION_LIST(V)                                       \
  /* Constructors */                                               \
  V(Canvas::Create)                                                \
  V(ColorFilter::Create)                                           \
  V(FragmentProgram::Create)                                       \
  V(ReusableFragmentShader::Create)                                \
  V(Gradient::Create)                                              \
  V(ImageFilter::Create)                                           \
  V(ImageShader::Create)                                           \
  V(ParagraphBuilder::Create)                                      \
  V(PathMeasure::Create)                                           \
  V(Path::Create)                                                  \
  V(PictureRecorder::Create)                                       \
  V(SceneBuilder::Create)                                          \
  V(SemanticsUpdateBuilder::Create)                                \
  /* Other */                                                      \
  V(FontCollection::LoadFontFromList)                              \
  V(ImageDescriptor::initEncoded)                                  \
  V(ImmutableBuffer::init)                                         \
  V(ImmutableBuffer::initFromAsset)                                \
  V(ImmutableBuffer::initFromFile)                                 \
  V(ImageDescriptor::initRaw)                                      \
  V(IsolateNameServerNatives::LookupPortByName)                    \
  V(IsolateNameServerNatives::RegisterPortWithName)                \
  V(IsolateNameServerNatives::RemovePortNameMapping)               \
  V(NativeStringAttribute::initLocaleStringAttribute)              \
  V(NativeStringAttribute::initSpellOutStringAttribute)            \
  V(PlatformConfigurationNativeApi::DefaultRouteName)              \
  V(PlatformConfigurationNativeApi::ScheduleFrame)                 \
  V(PlatformConfigurationNativeApi::EndWarmUpFrame)                \
  V(PlatformConfigurationNativeApi::Render)                        \
  V(PlatformConfigurationNativeApi::UpdateSemantics)               \
  V(PlatformConfigurationNativeApi::SetNeedsReportTimings)         \
  V(PlatformConfigurationNativeApi::SetIsolateDebugName)           \
  V(PlatformConfigurationNativeApi::RequestDartPerformanceMode)    \
  V(PlatformConfigurationNativeApi::GetPersistentIsolateData)      \
  V(PlatformConfigurationNativeApi::ComputePlatformResolvedLocale) \
  V(PlatformConfigurationNativeApi::SendPlatformMessage)           \
  V(PlatformConfigurationNativeApi::RespondToPlatformMessage)      \
  V(PlatformConfigurationNativeApi::GetRootIsolateToken)           \
  V(PlatformConfigurationNativeApi::RegisterBackgroundIsolate)     \
  V(PlatformConfigurationNativeApi::SendPortPlatformMessage)       \
  V(PlatformConfigurationNativeApi::SendChannelUpdate)             \
  V(PlatformConfigurationNativeApi::GetScaledFontSize)             \
  V(PlatformIsolateNativeApi::IsRunningOnPlatformThread)           \
  V(PlatformIsolateNativeApi::Spawn)                               \
  V(DartRuntimeHooks::Logger_PrintDebugString)                     \
  V(DartRuntimeHooks::Logger_PrintString)                          \
  V(DartRuntimeHooks::ScheduleMicrotask)                           \
  V(DartRuntimeHooks::GetCallbackHandle)                           \
  V(DartRuntimeHooks::GetCallbackFromHandle)                       \
  V(DartPluginRegistrant_EnsureInitialized)                        \
  V(Vertices::init)

// List of native instance methods used as @Native functions.
// Items are tuples of ('class_name', 'method_name', 'parameter_count'), where:
//   'class_name' is the name of the class containing the method.
//   'method_name' is the name of the method.
//   'parameter_count' is the number of parameters the method has including the
//                     implicit `this` parameter.
//
// These are used to:
// - Instantiate FfiDispatcher templates to automatically create FFI Native
//   bindings.
//   If the name does not match a native function, the template will fail to
//   instatiate, resulting in a compile time error.
// - Resolve the native function pointer associated with an @Native function.
//   If there is a mismatch between names or parameter count an @Native is
//   trying to resolve, an exception will be thrown.
#define FFI_METHOD_LIST(V)                       \
  V(Canvas, clipPath)                            \
  V(Canvas, clipRect)                            \
  V(Canvas, clipRRect)                           \
  V(Canvas, drawArc)                             \
  V(Canvas, drawAtlas)                           \
  V(Canvas, drawCircle)                          \
  V(Canvas, drawColor)                           \
  V(Canvas, drawDRRect)                          \
  V(Canvas, drawImage)                           \
  V(Canvas, drawImageNine)                       \
  V(Canvas, drawImageRect)                       \
  V(Canvas, drawLine)                            \
  V(Canvas, drawOval)                            \
  V(Canvas, drawPaint)                           \
  V(Canvas, drawPath)                            \
  V(Canvas, drawPicture)                         \
  V(Canvas, drawPoints)                          \
  V(Canvas, drawRRect)                           \
  V(Canvas, drawRect)                            \
  V(Canvas, drawShadow)                          \
  V(Canvas, drawVertices)                        \
  V(Canvas, getDestinationClipBounds)            \
  V(Canvas, getLocalClipBounds)                  \
  V(Canvas, getSaveCount)                        \
  V(Canvas, getTransform)                        \
  V(Canvas, restore)                             \
  V(Canvas, restoreToCount)                      \
  V(Canvas, rotate)                              \
  V(Canvas, save)                                \
  V(Canvas, saveLayer)                           \
  V(Canvas, saveLayerWithoutBounds)              \
  V(Canvas, scale)                               \
  V(Canvas, skew)                                \
  V(Canvas, transform)                           \
  V(Canvas, translate)                           \
  V(Codec, dispose)                              \
  V(Codec, frameCount)                           \
  V(Codec, getNextFrame)                         \
  V(Codec, repetitionCount)                      \
  V(ColorFilter, initLinearToSrgbGamma)          \
  V(ColorFilter, initMatrix)                     \
  V(ColorFilter, initMode)                       \
  V(ColorFilter, initSrgbToLinearGamma)          \
  V(EngineLayer, dispose)                        \
  V(FragmentProgram, initFromAsset)              \
  V(ReusableFragmentShader, Dispose)             \
  V(ReusableFragmentShader, SetImageSampler)     \
  V(ReusableFragmentShader, ValidateSamplers)    \
  V(ReusableFragmentShader, ValidateImageFilter) \
  V(Gradient, initLinear)                        \
  V(Gradient, initRadial)                        \
  V(Gradient, initSweep)                         \
  V(Gradient, initTwoPointConical)               \
  V(Image, dispose)                              \
  V(Image, width)                                \
  V(Image, height)                               \
  V(Image, toByteData)                           \
  V(Image, colorSpace)                           \
  V(ImageDescriptor, bytesPerPixel)              \
  V(ImageDescriptor, dispose)                    \
  V(ImageDescriptor, height)                     \
  V(ImageDescriptor, instantiateCodec)           \
  V(ImageDescriptor, width)                      \
  V(ImageFilter, initBlur)                       \
  V(ImageFilter, initDilate)                     \
  V(ImageFilter, initErode)                      \
  V(ImageFilter, initColorFilter)                \
  V(ImageFilter, initComposeFilter)              \
  V(ImageFilter, initShader)                     \
  V(ImageFilter, initMatrix)                     \
  V(ImageShader, dispose)                        \
  V(ImageShader, initWithImage)                  \
  V(ImmutableBuffer, dispose)                    \
  V(ImmutableBuffer, length)                     \
  V(ParagraphBuilder, addPlaceholder)            \
  V(ParagraphBuilder, addText)                   \
  V(ParagraphBuilder, build)                     \
  V(ParagraphBuilder, pop)                       \
  V(ParagraphBuilder, pushStyle)                 \
  V(Paragraph, alphabeticBaseline)               \
  V(Paragraph, computeLineMetrics)               \
  V(Paragraph, didExceedMaxLines)                \
  V(Paragraph, dispose)                          \
  V(Paragraph, getClosestGlyphInfo)              \
  V(Paragraph, getGlyphInfoAt)                   \
  V(Paragraph, getLineBoundary)                  \
  V(Paragraph, getLineMetricsAt)                 \
  V(Paragraph, getLineNumberAt)                  \
  V(Paragraph, getNumberOfLines)                 \
  V(Paragraph, getPositionForOffset)             \
  V(Paragraph, getRectsForPlaceholders)          \
  V(Paragraph, getRectsForRange)                 \
  V(Paragraph, getWordBoundary)                  \
  V(Paragraph, height)                           \
  V(Paragraph, ideographicBaseline)              \
  V(Paragraph, layout)                           \
  V(Paragraph, longestLine)                      \
  V(Paragraph, maxIntrinsicWidth)                \
  V(Paragraph, minIntrinsicWidth)                \
  V(Paragraph, paint)                            \
  V(Paragraph, width)                            \
  V(PathMeasure, setPath)                        \
  V(PathMeasure, getLength)                      \
  V(PathMeasure, getPosTan)                      \
  V(PathMeasure, getSegment)                     \
  V(PathMeasure, isClosed)                       \
  V(PathMeasure, nextContour)                    \
  V(Path, addArc)                                \
  V(Path, addOval)                               \
  V(Path, addPath)                               \
  V(Path, addPathWithMatrix)                     \
  V(Path, addPolygon)                            \
  V(Path, addRRect)                              \
  V(Path, addRect)                               \
  V(Path, arcTo)                                 \
  V(Path, arcToPoint)                            \
  V(Path, clone)                                 \
  V(Path, close)                                 \
  V(Path, conicTo)                               \
  V(Path, contains)                              \
  V(Path, cubicTo)                               \
  V(Path, extendWithPath)                        \
  V(Path, extendWithPathAndMatrix)               \
  V(Path, getBounds)                             \
  V(Path, getFillType)                           \
  V(Path, lineTo)                                \
  V(Path, moveTo)                                \
  V(Path, op)                                    \
  V(Path, quadraticBezierTo)                     \
  V(Path, relativeArcToPoint)                    \
  V(Path, relativeConicTo)                       \
  V(Path, relativeCubicTo)                       \
  V(Path, relativeLineTo)                        \
  V(Path, relativeMoveTo)                        \
  V(Path, relativeQuadraticBezierTo)             \
  V(Path, reset)                                 \
  V(Path, setFillType)                           \
  V(Path, shift)                                 \
  V(Path, transform)                             \
  V(PictureRecorder, endRecording)               \
  V(Picture, GetAllocationSize)                  \
  V(Picture, dispose)                            \
  V(Picture, toImage)                            \
  V(Picture, toImageSync)                        \
  V(SceneBuilder, addPerformanceOverlay)         \
  V(SceneBuilder, addPicture)                    \
  V(SceneBuilder, addPlatformView)               \
  V(SceneBuilder, addRetained)                   \
  V(SceneBuilder, addTexture)                    \
  V(SceneBuilder, build)                         \
  V(SceneBuilder, pop)                           \
  V(SceneBuilder, pushBackdropFilter)            \
  V(SceneBuilder, pushClipPath)                  \
  V(SceneBuilder, pushClipRRect)                 \
  V(SceneBuilder, pushClipRect)                  \
  V(SceneBuilder, pushColorFilter)               \
  V(SceneBuilder, pushImageFilter)               \
  V(SceneBuilder, pushOffset)                    \
  V(SceneBuilder, pushOpacity)                   \
  V(SceneBuilder, pushShaderMask)                \
  V(SceneBuilder, pushTransformHandle)           \
  V(Scene, dispose)                              \
  V(Scene, toImage)                              \
  V(Scene, toImageSync)                          \
  V(SemanticsUpdateBuilder, build)               \
  V(SemanticsUpdateBuilder, updateCustomAction)  \
  V(SemanticsUpdateBuilder, updateNode)          \
  V(SemanticsUpdate, dispose)                    \
  V(Vertices, dispose)

#define FFI_FUNCTION_INSERT(FUNCTION)           \
  g_function_dispatchers.insert(std::make_pair( \
      std::string_view(#FUNCTION),              \
      reinterpret_cast<void*>(                  \
          tonic::FfiDispatcher<void, decltype(&FUNCTION), &FUNCTION>::Call)));

#define FFI_METHOD_INSERT(CLASS, METHOD)                                       \
  g_function_dispatchers.insert(                                               \
      std::make_pair(std::string_view(#CLASS "::" #METHOD),                    \
                     reinterpret_cast<void*>(                                  \
                         tonic::FfiDispatcher<CLASS, decltype(&CLASS::METHOD), \
                                              &CLASS::METHOD>::Call)));

namespace {

std::once_flag g_dispatchers_init_flag;
std::unordered_map<std::string_view, void*> g_function_dispatchers;

void* ResolveFfiNativeFunction(const char* name, uintptr_t args) {
  auto it = g_function_dispatchers.find(name);
  return (it != g_function_dispatchers.end()) ? it->second : nullptr;
}

void InitDispatcherMap() {
  FFI_FUNCTION_LIST(FFI_FUNCTION_INSERT)
  FFI_METHOD_LIST(FFI_METHOD_INSERT)
}

}  // anonymous namespace

void DartUI::InitForIsolate(const Settings& settings) {
  std::call_once(g_dispatchers_init_flag, InitDispatcherMap);

  auto dart_ui = Dart_LookupLibrary(ToDart("dart:ui"));
  if (Dart_IsError(dart_ui)) {
    Dart_PropagateError(dart_ui);
  }

  // Set up FFI Native resolver for dart:ui.
  Dart_Handle result =
      Dart_SetFfiNativeResolver(dart_ui, ResolveFfiNativeFunction);
  if (Dart_IsError(result)) {
    Dart_PropagateError(result);
  }

  if (settings.enable_impeller) {
    result = Dart_SetField(dart_ui, ToDart("_impellerEnabled"), Dart_True());
    if (Dart_IsError(result)) {
      Dart_PropagateError(result);
    }
  }

  if (settings.enable_platform_isolates) {
    result =
        Dart_SetField(dart_ui, ToDart("_platformIsolatesEnabled"), Dart_True());
    if (Dart_IsError(result)) {
      Dart_PropagateError(result);
    }
  }

  result = Dart_SetField(dart_ui, ToDart("_implicitViewId"),
                         Dart_NewInteger(kFlutterImplicitViewId));
  if (Dart_IsError(result)) {
    Dart_PropagateError(result);
  }
}

}  // namespace flutter
