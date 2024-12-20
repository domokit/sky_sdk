// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_IMPELLER_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_IMPELLER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

///-----------------------------------------------------------------------------
///-----------------------------------------------------------------------------
/// -------  ___                      _ _                _    ____ ___  --------
/// ------- |_ _|_ __ ___  _ __   ___| | | ___ _ __     / \  |  _ \_ _| --------
/// -------  | || '_ ` _ \| '_ \ / _ \ | |/ _ \ '__|   / _ \ | |_) | |  --------
/// -------  | || | | | | | |_) |  __/ | |  __/ |     / ___ \|  __/| |  --------
/// ------- |___|_| |_| |_| .__/ \___|_|_|\___|_|    /_/   \_\_|  |___| --------
/// -------               |_|                                           --------
///-----------------------------------------------------------------------------
///-----------------------------------------------------------------------------
///
/// This file describes a high-level, single-header, dependency-free, 2D
/// graphics API.
///
/// The API fundamentals that include details about the object model, reference
/// counting, and null-safety are described in the README.
///
#if defined(__cplusplus)
#define IMPELLER_EXTERN_C extern "C"
#define IMPELLER_EXTERN_C_BEGIN IMPELLER_EXTERN_C {
#define IMPELLER_EXTERN_C_END }
#else  // defined(__cplusplus)
#define IMPELLER_EXTERN_C
#define IMPELLER_EXTERN_C_BEGIN
#define IMPELLER_EXTERN_C_END
#endif  // defined(__cplusplus)

#ifdef _WIN32
#define IMPELLER_EXPORT_DECORATION __declspec(dllexport)
#else
#define IMPELLER_EXPORT_DECORATION __attribute__((visibility("default")))
#endif

#ifndef IMPELLER_NO_EXPORT
#define IMPELLER_EXPORT IMPELLER_EXPORT_DECORATION
#else  // IMPELLER_NO_EXPORT
#define IMPELLER_EXPORT
#endif  // IMPELLER_NO_EXPORT

#ifdef __clang__
#define IMPELLER_NULLABLE _Nullable
#define IMPELLER_NONNULL _Nonnull
#else  // __clang__
#define IMPELLER_NULLABLE
#define IMPELLER_NONNULL
#endif  // __clang__

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 202000L)
#define IMPELLER_NODISCARD [[nodiscard]]
#else  // defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 202000L)
#define IMPELLER_NODISCARD
#endif  // defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 202000L)

IMPELLER_EXTERN_C_BEGIN

//------------------------------------------------------------------------------
/// @brief      Pack a version in a uint32_t.
///
/// @param[in]  variant  The version variant.
/// @param[in]  major    The major version.
/// @param[in]  minor    The minor version.
/// @param[in]  patch    The patch version.
///
/// @return     The packed version number.
///
#define IMPELLER_MAKE_VERSION(variant, major, minor, patch)        \
  ((((uint32_t)(variant)) << 29U) | (((uint32_t)(major)) << 22U) | \
   (((uint32_t)(minor)) << 12U) | ((uint32_t)(patch)))

#define IMPELLER_VERSION_VARIANT 1
#define IMPELLER_VERSION_MAJOR 1
#define IMPELLER_VERSION_MINOR 2
#define IMPELLER_VERSION_PATCH 0

//------------------------------------------------------------------------------
/// The current Impeller API version.
///
/// This version must be passed to APIs that create top-level objects like
/// graphics contexts. Construction of the context may fail if the API version
/// expected by the caller is not supported by the library.
///
/// The version currently supported by the library is returned by a call to
/// `ImpellerGetVersion`
///
/// Since there are no API stability guarantees today, passing a version that is
/// different to the one returned by `ImpellerGetVersion` will always fail.
///
/// @see `ImpellerGetVersion`
///
#define IMPELLER_VERSION                                                  \
  IMPELLER_MAKE_VERSION(IMPELLER_VERSION_VARIANT, IMPELLER_VERSION_MAJOR, \
                        IMPELLER_VERSION_MINOR, IMPELLER_VERSION_PATCH)

//------------------------------------------------------------------------------
/// @param[in]  version  The packed version.
///
/// @return     The version variant.
///
#define IMPELLER_VERSION_GET_VARIANT(version) ((uint32_t)(version) >> 29U)

//------------------------------------------------------------------------------
/// @param[in]  version  The packed version.
///
/// @return     The major version.
///
#define IMPELLER_VERSION_GET_MAJOR(version) \
  (((uint32_t)(version) >> 22U) & 0x7FU)

//------------------------------------------------------------------------------
/// @param[in]  version  The packed version.
///
/// @return     The minor version.
///
#define IMPELLER_VERSION_GET_MINOR(version) \
  (((uint32_t)(version) >> 12U) & 0x3FFU)

//------------------------------------------------------------------------------
/// @param[in]  version  The packed version.
///
/// @return     The patch version.
///
#define IMPELLER_VERSION_GET_PATCH(version) ((uint32_t)(version) & 0xFFFU)

//------------------------------------------------------------------------------
// Handles
//------------------------------------------------------------------------------

#define IMPELLER_INTERNAL_HANDLE_NAME(handle) handle##_
#define IMPELLER_DEFINE_HANDLE(handle) \
  typedef struct IMPELLER_INTERNAL_HANDLE_NAME(handle) * handle;

//------------------------------------------------------------------------------
/// An Impeller graphics context. Contexts are platform and client-rendering-API
/// specific.
///
/// Contexts are thread-safe objects that are expensive to create. Most
/// applications will only ever create a single context during their lifetimes.
/// Once setup, Impeller is ready to render frames as performantly as possible.
///
/// During setup, context create the underlying graphics pipelines, allocators,
/// worker threads, etc...
///
/// The general guidance is to create as few contexts as possible (typically
/// just one) and share them as much as possible.
///
IMPELLER_DEFINE_HANDLE(ImpellerContext);

//------------------------------------------------------------------------------
/// Display lists represent encoded rendering intent. These objects are
/// immutable, reusable, thread-safe, and context-agnostic.
///
/// While it is perfectly fine to create new display lists per frame, there may
/// be opportunities for optimization when display lists are reused multiple
/// times.
///
IMPELLER_DEFINE_HANDLE(ImpellerDisplayList);

//------------------------------------------------------------------------------
/// Display list builders allow for the incremental creation of display lists.
///
/// Display list builders are context-agnostic.
///
IMPELLER_DEFINE_HANDLE(ImpellerDisplayListBuilder);

//------------------------------------------------------------------------------
/// Paints control the behavior of draw calls encoded in a display list.
///
/// Like display lists, paints are context-agnostic.
///
IMPELLER_DEFINE_HANDLE(ImpellerPaint);

//------------------------------------------------------------------------------
/// Color filters are functions that take two colors and mix them to produce a
/// single color. This color is then merged with the destination during
/// blending.
///
IMPELLER_DEFINE_HANDLE(ImpellerColorFilter);

//------------------------------------------------------------------------------
/// Color sources are functions that generate colors for each texture element
/// covered by a draw call. The colors for each element can be generated using a
/// mathematical function (to produce gradients for example) or sampled from a
/// texture.
///
IMPELLER_DEFINE_HANDLE(ImpellerColorSource);

//------------------------------------------------------------------------------
/// Image filters are functions that are applied regions of a texture to produce
/// a single color. Contrast this with color filters that operate independently
/// on a per-pixel basis. The generated color is then merged with the
/// destination during blending.
///
IMPELLER_DEFINE_HANDLE(ImpellerImageFilter);

//------------------------------------------------------------------------------
/// Mask filters are functions that are applied over a shape after it has been
/// drawn but before it has been blended into the final image.
///
IMPELLER_DEFINE_HANDLE(ImpellerMaskFilter);

//------------------------------------------------------------------------------
/// Typography contexts allow for the layout and rendering of text.
///
/// These are typically expensive to create and applications will only ever need
/// to create a single one of these during their lifetimes.
///
/// Unlike graphics context, typograhy contexts are not thread-safe. These must
/// be created, used, and collected on a single thread.
///
IMPELLER_DEFINE_HANDLE(ImpellerTypographyContext);

//------------------------------------------------------------------------------
/// An immutable, fully laid out paragraph.
///
IMPELLER_DEFINE_HANDLE(ImpellerParagraph);

//------------------------------------------------------------------------------
/// Paragraph builders allow for the creation of fully laid out paragraphs
/// (which themselves are immutable).
///
/// To build a paragraph, users push/pop paragraph styles onto a stack then add
/// UTF-8 encoded text. The properties on the top of paragraph style stack when
/// the text is added are used to layout and shape that subset of the paragraph.
///
/// @see      `ImpellerParagraphStyle`
///
IMPELLER_DEFINE_HANDLE(ImpellerParagraphBuilder);

//------------------------------------------------------------------------------
/// Specified when building a paragraph, paragraph styles are managed in a stack
/// with specify text properties to apply to text that is added to the paragraph
/// builder.
///
IMPELLER_DEFINE_HANDLE(ImpellerParagraphStyle);

//------------------------------------------------------------------------------
/// Represents a two-dimensional path that is immutable and graphics context
/// agnostic.
///
/// Paths in Impeller consist of linear, cubic Bézier curve, and quadratic
/// Bézier curve segments. All other shapes are approximations using these
/// building blocks.
///
/// Paths are created using path builder that allow for the configuration of the
/// path segments, how they are filled, and/or stroked.
///
IMPELLER_DEFINE_HANDLE(ImpellerPath);

//------------------------------------------------------------------------------
/// Path builders allow for the incremental building up of paths.
///
IMPELLER_DEFINE_HANDLE(ImpellerPathBuilder);

//------------------------------------------------------------------------------
/// A surface represents a render target for Impeller to direct the rendering
/// intent specified the form of display lists to.
///
/// Render targets are how Impeller API users perform Window System Integration
/// (WSI). Users wrap swapchain images as surfaces and draw display lists onto
/// these surfaces to present content.
///
/// Creating surfaces is typically platform and client-rendering-API specific.
///
IMPELLER_DEFINE_HANDLE(ImpellerSurface);

//------------------------------------------------------------------------------
/// A reference to a texture whose data is resident on the GPU. These can be
/// referenced in draw calls and paints.
///
/// Creating textures is extremely expensive. Creating a single one can
/// typically comfortably blow the frame budget of an application. Textures
/// should be created on background threads.
///
/// @warning    While textures themselves are thread safe, some context types
///             (like OpenGL) may need extra configuration to be able to operate
///             from multiple threads.
///
IMPELLER_DEFINE_HANDLE(ImpellerTexture);

//------------------------------------------------------------------------------
// Signatures
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// A callback invoked by Impeller that passes a user supplied baton back to the
/// user. Impeller does not interpret the baton in any way. The way the baton is
/// specified and the thread on which the callback is invoked depends on how the
/// user supplies the callback to Impeller.
///
typedef void (*ImpellerCallback)(void* IMPELLER_NULLABLE user_data);

//------------------------------------------------------------------------------
/// A callback used by Impeller to allow the user to resolve function pointers.
/// A user supplied baton that is uninterpreted by Impeller is passed back to
/// the user in the callback. How the baton is specified to Impeller and the
/// thread on which the callback is invoked depends on how the callback is
/// specified to Impeller.
///
typedef void* IMPELLER_NULLABLE (*ImpellerProcAddressCallback)(
    const char* IMPELLER_NONNULL proc_name,
    void* IMPELLER_NULLABLE user_data);

//------------------------------------------------------------------------------
// Enumerations
// -----------------------------------------------------------------------------
typedef enum ImpellerFillType {
  kImpellerFillTypeNonZero,
  kImpellerFillTypeOdd,
} ImpellerFillType;

typedef enum ImpellerClipOperation {
  kImpellerClipOperationDifference,
  kImpellerClipOperationIntersect,
} ImpellerClipOperation;

typedef enum ImpellerBlendMode {
  kImpellerBlendModeClear,
  kImpellerBlendModeSource,
  kImpellerBlendModeDestination,
  kImpellerBlendModeSourceOver,
  kImpellerBlendModeDestinationOver,
  kImpellerBlendModeSourceIn,
  kImpellerBlendModeDestinationIn,
  kImpellerBlendModeSourceOut,
  kImpellerBlendModeDestinationOut,
  kImpellerBlendModeSourceATop,
  kImpellerBlendModeDestinationATop,
  kImpellerBlendModeXor,
  kImpellerBlendModePlus,
  kImpellerBlendModeModulate,
  kImpellerBlendModeScreen,
  kImpellerBlendModeOverlay,
  kImpellerBlendModeDarken,
  kImpellerBlendModeLighten,
  kImpellerBlendModeColorDodge,
  kImpellerBlendModeColorBurn,
  kImpellerBlendModeHardLight,
  kImpellerBlendModeSoftLight,
  kImpellerBlendModeDifference,
  kImpellerBlendModeExclusion,
  kImpellerBlendModeMultiply,
  kImpellerBlendModeHue,
  kImpellerBlendModeSaturation,
  kImpellerBlendModeColor,
  kImpellerBlendModeLuminosity,
} ImpellerBlendMode;

typedef enum ImpellerDrawStyle {
  kImpellerDrawStyleFill,
  kImpellerDrawStyleStroke,
  kImpellerDrawStyleStrokeAndFill,
} ImpellerDrawStyle;

typedef enum ImpellerStrokeCap {
  kImpellerStrokeCapButt,
  kImpellerStrokeCapRound,
  kImpellerStrokeCapSquare,
} ImpellerStrokeCap;

typedef enum ImpellerStrokeJoin {
  kImpellerStrokeJoinMiter,
  kImpellerStrokeJoinRound,
  kImpellerStrokeJoinBevel,
} ImpellerStrokeJoin;

typedef enum ImpellerPixelFormat {
  kImpellerPixelFormatRGBA8888,
} ImpellerPixelFormat;

typedef enum ImpellerTextureSampling {
  kImpellerTextureSamplingNearestNeighbor,
  kImpellerTextureSamplingLinear,
} ImpellerTextureSampling;

typedef enum ImpellerTileMode {
  kImpellerTileModeClamp,
  kImpellerTileModeRepeat,
  kImpellerTileModeMirror,
  kImpellerTileModeDecal,
} ImpellerTileMode;

typedef enum ImpellerBlurStyle {
  kImpellerBlurStyleNormal,
  kImpellerBlurStyleSolid,
  kImpellerBlurStyleOuter,
  kImpellerBlurStyleInner,
} ImpellerBlurStyle;

typedef enum ImpellerColorSpace {
  kImpellerColorSpaceSRGB,
  kImpellerColorSpaceExtendedSRGB,
  kImpellerColorSpaceDisplayP3,
} ImpellerColorSpace;

typedef enum ImpellerFontWeight {
  kImpellerFontWeight100,  // Thin
  kImpellerFontWeight200,  // Extra-Light
  kImpellerFontWeight300,  // Light
  kImpellerFontWeight400,  // Normal/Regular
  kImpellerFontWeight500,  // Medium
  kImpellerFontWeight600,  // Semi-bold
  kImpellerFontWeight700,  // Bold
  kImpellerFontWeight800,  // Extra-Bold
  kImpellerFontWeight900,  // Black
} ImpellerFontWeight;

typedef enum ImpellerFontStyle {
  kImpellerFontStyleNormal,
  kImpellerFontStyleItalic,
} ImpellerFontStyle;

typedef enum ImpellerTextAlignment {
  kImpellerTextAlignmentLeft,
  kImpellerTextAlignmentRight,
  kImpellerTextAlignmentCenter,
  kImpellerTextAlignmentJustify,
  kImpellerTextAlignmentStart,
  kImpellerTextAlignmentEnd,
} ImpellerTextAlignment;

typedef enum ImpellerTextDirection {
  kImpellerTextDirectionRTL,
  kImpellerTextDirectionLTR,
} ImpellerTextDirection;

//------------------------------------------------------------------------------
// Non-opaque structs
// -----------------------------------------------------------------------------
typedef struct ImpellerRect {
  float x;
  float y;
  float width;
  float height;
} ImpellerRect;

typedef struct ImpellerPoint {
  float x;
  float y;
} ImpellerPoint;

typedef struct ImpellerSize {
  float width;
  float height;
} ImpellerSize;

typedef struct ImpellerISize {
  int64_t width;
  int64_t height;
} ImpellerISize;

//------------------------------------------------------------------------------
/// A 4x4 transformation matrix using column-major storage.
///
/// ```
/// | m[0] m[4] m[8]  m[12] |
/// | m[1] m[5] m[9]  m[13] |
/// | m[2] m[6] m[10] m[14] |
/// | m[3] m[7] m[11] m[15] |
/// ```
///
typedef struct ImpellerMatrix {
  float m[16];
} ImpellerMatrix;

//------------------------------------------------------------------------------
/// A 4x5 matrix using row-major storage used for transforming color values.
///
/// To transform color values, a 5x5 matrix is constructed with the 5th row
/// being identity. Then the following transformation is performed:
///
/// ```
/// | R' |   | m[0]  m[1]  m[2]  m[3]  m[4]  |   | R |
/// | G' |   | m[5]  m[6]  m[7]  m[8]  m[9]  |   | G |
/// | B' | = | m[10] m[11] m[12] m[13] m[14] | * | B |
/// | A' |   | m[15] m[16] m[17] m[18] m[19] |   | A |
/// | 1  |   | 0     0     0     0     1     |   | 1 |
/// ```
///
/// The translation column (m[4], m[9], m[14], m[19]) must be specified in
/// non-normalized 8-bit unsigned integer space (0 to 255). Values outside this
/// range will produce undefined results.
///
/// The identity transformation is thus:
///
/// ```
/// 1, 0, 0, 0, 0,
/// 0, 1, 0, 0, 0,
/// 0, 0, 1, 0, 0,
/// 0, 0, 0, 1, 0,
/// ```
///
/// Some examples:
///
/// To invert all colors:
///
/// ```
/// -1,  0,  0, 0, 255,
///  0, -1,  0, 0, 255,
///  0,  0, -1, 0, 255,
///  0,  0,  0, 1,   0,
/// ```
///
/// To apply a sepia filter:
///
/// ```
/// 0.393, 0.769, 0.189, 0, 0,
/// 0.349, 0.686, 0.168, 0, 0,
/// 0.272, 0.534, 0.131, 0, 0,
/// 0,     0,     0,     1, 0,
/// ```
///
/// To apply a grayscale conversion filter:
///
/// ```
///  0.2126, 0.7152, 0.0722, 0, 0,
///  0.2126, 0.7152, 0.0722, 0, 0,
///  0.2126, 0.7152, 0.0722, 0, 0,
///  0,      0,      0,      1, 0,
/// ```
///
/// @see      ImpellerColorFilter
///
typedef struct ImpellerColorMatrix {
  float m[20];
} ImpellerColorMatrix;

typedef struct ImpellerRoundingRadii {
  ImpellerPoint top_left;
  ImpellerPoint bottom_left;
  ImpellerPoint top_right;
  ImpellerPoint bottom_right;
} ImpellerRoundingRadii;

typedef struct ImpellerColor {
  float red;
  float green;
  float blue;
  float alpha;
  ImpellerColorSpace color_space;
} ImpellerColor;

typedef struct ImpellerTextureDescriptor {
  ImpellerPixelFormat pixel_format;
  ImpellerISize size;
  uint32_t mip_count;
} ImpellerTextureDescriptor;

typedef struct ImpellerMapping {
  const uint8_t* IMPELLER_NONNULL data;
  uint64_t length;
  ImpellerCallback IMPELLER_NULLABLE on_release;
} ImpellerMapping;

//------------------------------------------------------------------------------
// Version
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Get the version of Impeller standalone API. This is the API that
///             will be accepted for validity checks when provided to the
///             context creation methods.
///
///             The current version of the API  is denoted by the
///             `IMPELLER_VERSION` macro. This version must be passed to APIs
///             that create top-level objects like graphics contexts.
///             Construction of the context may fail if the API version expected
///             by the caller is not supported by the library.
///
///             Since there are no API stability guarantees today, passing a
///             version that is different to the one returned by
///             `ImpellerGetVersion` will always fail.
///
/// @see        `ImpellerContextCreateOpenGLESNew`
///
/// @return     The version of the standalone API.
///
IMPELLER_EXPORT
uint32_t ImpellerGetVersion();

//------------------------------------------------------------------------------
// Context
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Create an OpenGL(ES) Impeller context.
///
/// @warning    Unlike other context types, the OpenGL ES context can only be
///             created, used, and collected on the calling thread. This
///             restriction may be lifted in the future once reactor workers are
///             exposed in the API. No other context types have threading
///             restrictions. Till reactor workers can be used, using the
///             context on a background thread will cause a stall of OpenGL
///             operations.
///
/// @param[in]  version      The version of the Impeller
///                          standalone API. See `ImpellerGetVersion`. If the
///                          specified here is not compatible with the version
///                          of the library, context creation will fail and NULL
///                          context returned from this call.
/// @param[in]  gl_proc_address_callback
///                          The gl proc address callback. For instance,
///                          `eglGetProcAddress`.
/// @param[in]  gl_proc_address_callback_user_data
///                          The gl proc address callback user data baton. This
///                          pointer is not interpreted by Impeller and will be
///                          returned as user data in the proc address callback.
///                          user data.
///
/// @return     The context or NULL if one cannot be created.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerContext IMPELLER_NULLABLE
ImpellerContextCreateOpenGLESNew(
    uint32_t version,
    ImpellerProcAddressCallback IMPELLER_NONNULL gl_proc_address_callback,
    void* IMPELLER_NULLABLE gl_proc_address_callback_user_data);

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  context  The context.
///
IMPELLER_EXPORT
void ImpellerContextRetain(ImpellerContext IMPELLER_NULLABLE context);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  context  The context.
///
IMPELLER_EXPORT
void ImpellerContextRelease(ImpellerContext IMPELLER_NULLABLE context);

//------------------------------------------------------------------------------
// Surface
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Create a new surface by wrapping an existing framebuffer object.
///             The framebuffer must be complete as determined by
///             `glCheckFramebufferStatus`. The framebuffer is still owned by
///             the caller and it must be collected once the surface is
///             collected.
///
/// @param[in]  context  The context.
/// @param[in]  fbo      The framebuffer object handle.
/// @param[in]  format   The format of the framebuffer.
/// @param[in]  size     The size of the framebuffer is texels.
///
/// @return     The surface if once can be created, NULL otherwise.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerSurface IMPELLER_NULLABLE
ImpellerSurfaceCreateWrappedFBONew(ImpellerContext IMPELLER_NULLABLE context,
                                   uint64_t fbo,
                                   ImpellerPixelFormat format,
                                   const ImpellerISize* IMPELLER_NULLABLE size);

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  surface  The surface.
///
IMPELLER_EXPORT
void ImpellerSurfaceRetain(ImpellerSurface IMPELLER_NULLABLE surface);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  surface  The surface.
///
IMPELLER_EXPORT
void ImpellerSurfaceRelease(ImpellerSurface IMPELLER_NULLABLE surface);

//------------------------------------------------------------------------------
/// @brief      Draw a display list onto the surface. The same display list can
///             be drawn multiple times to different surfaces.
///
/// @warning    In the OpenGL backend, Impeller will not make an effort to
///             preserve the OpenGL state that is current in the context.
///             Embedders that perform additional OpenGL operations in the
///             context should expect the reset state after control transitions
///             back to them. Key state to watch out for would be the viewports,
///             stencil rects, test toggles, resource (texture, framebuffer,
///             buffer) bindings, etc...
///
/// @param[in]  surface       The surface to draw the display list to.
/// @param[in]  display_list  The display list to draw onto the surface.
///
/// @return     If the display list could be drawn onto the surface.
///
IMPELLER_EXPORT
bool ImpellerSurfaceDrawDisplayList(
    ImpellerSurface IMPELLER_NULLABLE surface,
    ImpellerDisplayList IMPELLER_NONNULL display_list);

//------------------------------------------------------------------------------
// Path
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  path  The path.
///
IMPELLER_EXPORT
void ImpellerPathRetain(ImpellerPath IMPELLER_NULLABLE path);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  path  The path.
///
IMPELLER_EXPORT
void ImpellerPathRelease(ImpellerPath IMPELLER_NULLABLE path);

//------------------------------------------------------------------------------
// Path Builder
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Create a new path builder. Paths themselves are immutable.
///             A builder builds these immutable paths.
///
/// @return     The path builder.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerPathBuilder IMPELLER_NULLABLE
ImpellerPathBuilderNew();

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  builder  The builder.
///
IMPELLER_EXPORT
void ImpellerPathBuilderRetain(ImpellerPathBuilder IMPELLER_NULLABLE builder);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  builder  The builder.
///
IMPELLER_EXPORT
void ImpellerPathBuilderRelease(ImpellerPathBuilder IMPELLER_NULLABLE builder);

//------------------------------------------------------------------------------
/// @brief      Move the cursor to the specified location.
///
/// @param[in]  builder   The builder.
/// @param[in]  location  The location.
///
IMPELLER_EXPORT
void ImpellerPathBuilderMoveTo(ImpellerPathBuilder IMPELLER_NONNULL builder,
                               const ImpellerPoint* IMPELLER_NONNULL location);

//------------------------------------------------------------------------------
/// @brief      Add a line segment from the current cursor location to the given
///             location. The cursor location is updated to be at the endpoint.
///
/// @param[in]  builder   The builder.
/// @param[in]  location  The location.
///
IMPELLER_EXPORT
void ImpellerPathBuilderLineTo(ImpellerPathBuilder IMPELLER_NONNULL builder,
                               const ImpellerPoint* IMPELLER_NONNULL location);

//------------------------------------------------------------------------------
/// @brief      Add a quadratic curve from whose start point is the cursor to
///             the specified end point using the a single control point.
///
///             The new location of the cursor after this call is the end point.
///
/// @param[in]  builder        The builder.
/// @param[in]  control_point  The control point.
/// @param[in]  end_point      The end point.
///
IMPELLER_EXPORT
void ImpellerPathBuilderQuadraticCurveTo(
    ImpellerPathBuilder IMPELLER_NONNULL builder,
    const ImpellerPoint* IMPELLER_NONNULL control_point,
    const ImpellerPoint* IMPELLER_NONNULL end_point);

//------------------------------------------------------------------------------
/// @brief      Add a cubic curve whose start point is current cursor location
///             to the specified end point using the two specified control
///             points.
///
///             The new location of the cursor after this call is the end point
///             supplied.
///
/// @param[in]  builder          The builder
/// @param[in]  control_point_1  The control point 1
/// @param[in]  control_point_2  The control point 2
/// @param[in]  end_point        The end point
///
IMPELLER_EXPORT
void ImpellerPathBuilderCubicCurveTo(
    ImpellerPathBuilder IMPELLER_NONNULL builder,
    const ImpellerPoint* IMPELLER_NONNULL control_point_1,
    const ImpellerPoint* IMPELLER_NONNULL control_point_2,
    const ImpellerPoint* IMPELLER_NONNULL end_point);

//------------------------------------------------------------------------------
/// @brief      Adds a rectangle to the path.
///
/// @param[in]  builder  The builder.
/// @param[in]  rect     The rectangle.
///
IMPELLER_EXPORT
void ImpellerPathBuilderAddRect(ImpellerPathBuilder IMPELLER_NONNULL builder,
                                const ImpellerRect* IMPELLER_NONNULL rect);

//------------------------------------------------------------------------------
/// @brief      Add an arc to the path.
///
/// @param[in]  builder              The builder.
/// @param[in]  oval_bounds          The oval bounds.
/// @param[in]  start_angle_degrees  The start angle in degrees.
/// @param[in]  end_angle_degrees    The end angle in degrees.
///
IMPELLER_EXPORT
void ImpellerPathBuilderAddArc(ImpellerPathBuilder IMPELLER_NONNULL builder,
                               const ImpellerRect* IMPELLER_NONNULL oval_bounds,
                               float start_angle_degrees,
                               float end_angle_degrees);

//------------------------------------------------------------------------------
/// @brief      Add an oval to the path.
///
/// @param[in]  builder      The builder.
/// @param[in]  oval_bounds  The oval bounds.
///
IMPELLER_EXPORT
void ImpellerPathBuilderAddOval(
    ImpellerPathBuilder IMPELLER_NONNULL builder,
    const ImpellerRect* IMPELLER_NONNULL oval_bounds);

//------------------------------------------------------------------------------
/// @brief      Add a rounded rect with potentially non-uniform radii to the
///             path.
///
/// @param[in]  builder         The builder.
/// @param[in]  rect            The rectangle.
/// @param[in]  rounding_radii  The rounding radii.
///
IMPELLER_EXPORT
void ImpellerPathBuilderAddRoundedRect(
    ImpellerPathBuilder IMPELLER_NONNULL builder,
    const ImpellerRect* IMPELLER_NONNULL rect,
    const ImpellerRoundingRadii* IMPELLER_NONNULL rounding_radii);

//------------------------------------------------------------------------------
/// @brief      Close the path.
///
/// @param[in]  builder  The builder.
///
IMPELLER_EXPORT
void ImpellerPathBuilderClose(ImpellerPathBuilder IMPELLER_NONNULL builder);

//------------------------------------------------------------------------------
/// @brief      Create a new path by copying the existing built-up path. The
///             existing path can continue being added to.
///
/// @param[in]  builder  The builder.
/// @param[in]  fill     The fill.
///
/// @return     The impeller path.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerPath IMPELLER_NULLABLE
ImpellerPathBuilderCopyPathNew(ImpellerPathBuilder IMPELLER_NONNULL builder,
                               ImpellerFillType fill);

//------------------------------------------------------------------------------
/// @brief      Create a new path using the existing built-up path. The existing
///             path builder now contains an empty path.
///
/// @param[in]  builder  The builder.
/// @param[in]  fill     The fill.
///
/// @return     The impeller path.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerPath IMPELLER_NULLABLE
ImpellerPathBuilderTakePathNew(ImpellerPathBuilder IMPELLER_NONNULL builder,
                               ImpellerFillType fill);

//------------------------------------------------------------------------------
// Paint
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Create a new paint with default values.
///
/// @return     The impeller paint.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerPaint IMPELLER_NULLABLE
ImpellerPaintNew();

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  paint  The paint.
///
IMPELLER_EXPORT
void ImpellerPaintRetain(ImpellerPaint IMPELLER_NULLABLE paint);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  paint  The paint.
///
IMPELLER_EXPORT
void ImpellerPaintRelease(ImpellerPaint IMPELLER_NULLABLE paint);

//------------------------------------------------------------------------------
/// @brief      Set the paint color.
///
/// @param[in]  paint  The paint.
/// @param[in]  color  The color.
///
IMPELLER_EXPORT
void ImpellerPaintSetColor(ImpellerPaint IMPELLER_NONNULL paint,
                           const ImpellerColor* IMPELLER_NONNULL color);

//------------------------------------------------------------------------------
/// @brief      Set the paint blend mode. The blend mode controls how the new
///             paints contents are mixed with the values already drawn using
///             previous draw calls.
///
/// @param[in]  paint  The paint.
/// @param[in]  mode   The mode.
///
IMPELLER_EXPORT
void ImpellerPaintSetBlendMode(ImpellerPaint IMPELLER_NONNULL paint,
                               ImpellerBlendMode mode);

//------------------------------------------------------------------------------
/// @brief      Set the paint draw style. The style controls if the closed
///             shapes are filled and/or stroked.
///
/// @param[in]  paint  The paint.
/// @param[in]  style  The style.
///
IMPELLER_EXPORT
void ImpellerPaintSetDrawStyle(ImpellerPaint IMPELLER_NONNULL paint,
                               ImpellerDrawStyle style);

//------------------------------------------------------------------------------
/// @brief      Sets how strokes rendered using this paint are capped.
///
/// @param[in]  paint  The paint.
/// @param[in]  cap    The stroke cap style.
///
IMPELLER_EXPORT
void ImpellerPaintSetStrokeCap(ImpellerPaint IMPELLER_NONNULL paint,
                               ImpellerStrokeCap cap);

//------------------------------------------------------------------------------
/// @brief      Sets how strokes rendered using this paint are joined.
///
/// @param[in]  paint  The paint.
/// @param[in]  join   The join.
///
IMPELLER_EXPORT
void ImpellerPaintSetStrokeJoin(ImpellerPaint IMPELLER_NONNULL paint,
                                ImpellerStrokeJoin join);

//------------------------------------------------------------------------------
/// @brief      Set the width of the strokes rendered using this paint.
///
/// @param[in]  paint  The paint.
/// @param[in]  width  The width.
///
IMPELLER_EXPORT
void ImpellerPaintSetStrokeWidth(ImpellerPaint IMPELLER_NONNULL paint,
                                 float width);

//------------------------------------------------------------------------------
/// @brief      Set the miter limit of the strokes rendered using this paint.
///
/// @param[in]  paint  The paint.
/// @param[in]  miter  The miter limit.
///
IMPELLER_EXPORT
void ImpellerPaintSetStrokeMiter(ImpellerPaint IMPELLER_NONNULL paint,
                                 float miter);

//------------------------------------------------------------------------------
/// @brief      Set the color filter of the paint.
///
///             Color filters are functions that take two colors and mix them to
///             produce a single color. This color is then usually merged with
///             the destination during blending.
///
/// @param[in]  paint         The paint.
/// @param[in]  color_filter  The color filter.
///
IMPELLER_EXPORT
void ImpellerPaintSetColorFilter(
    ImpellerPaint IMPELLER_NONNULL paint,
    ImpellerColorFilter IMPELLER_NONNULL color_filter);

//------------------------------------------------------------------------------
/// @brief      Set the color source of the paint.
///
///             Color sources are functions that generate colors for each
///             texture element covered by a draw call.
///
/// @param[in]  paint         The paint.
/// @param[in]  color_source  The color source.
///
IMPELLER_EXPORT
void ImpellerPaintSetColorSource(
    ImpellerPaint IMPELLER_NONNULL paint,
    ImpellerColorSource IMPELLER_NONNULL color_source);

//------------------------------------------------------------------------------
/// @brief      Set the image filter of a paint.
///
///             Image filters are functions that are applied to regions of a
///             texture to produce a single color.
///
/// @param[in]  paint         The paint.
/// @param[in]  image_filter  The image filter.
///
IMPELLER_EXPORT
void ImpellerPaintSetImageFilter(
    ImpellerPaint IMPELLER_NONNULL paint,
    ImpellerImageFilter IMPELLER_NONNULL image_filter);

//------------------------------------------------------------------------------
/// @brief      Set the mask filter of a paint.
///
/// @param[in]  paint        The paint.
/// @param[in]  mask_filter  The mask filter.
///
IMPELLER_EXPORT
void ImpellerPaintSetMaskFilter(
    ImpellerPaint IMPELLER_NONNULL paint,
    ImpellerMaskFilter IMPELLER_NONNULL mask_filter);

//------------------------------------------------------------------------------
// Texture
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Create a texture with decompressed bytes.
///
///             Impeller will do its best to perform the transfer of this data
///             to GPU memory with a minimal number of copies. Towards this
///             end, it may need to send this data to a different thread for
///             preparation and transfer. To facilitate this transfer, it is
///             recommended that the content mapping have a release callback
///             attach to it. When there is a release callback, Impeller assumes
///             that collection of the data can be deferred till texture upload
///             is done and can happen on a background thread. When there is no
///             release callback, Impeller may try to perform an eager copy of
///             the data if it needs to perform data preparation and transfer on
///             a background thread.
///
///             Whether an extra data copy actually occurs will always depend on
///             the rendering backend in use. But it is best practice to provide
///             a release callback and be resilient to the data being released
///             in a deferred manner on a background thread.
///
/// @warning    Do **not** supply compressed image data directly (PNG, JPEG,
///             etc...). This function only works with tightly packed
///             decompressed data.
///
/// @param[in]  context                        The context.
/// @param[in]  descriptor                     The texture descriptor.
/// @param[in]  contents                       The contents.
/// @param[in]  contents_on_release_user_data  The baton passes to the contents
///                                            release callback if one exists.
///
/// @return     The texture if one can be created using the provided data, NULL
///             otherwise.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerTexture IMPELLER_NULLABLE
ImpellerTextureCreateWithContentsNew(
    ImpellerContext IMPELLER_NONNULL context,
    const ImpellerTextureDescriptor* IMPELLER_NONNULL descriptor,
    const ImpellerMapping* IMPELLER_NONNULL contents,
    void* IMPELLER_NULLABLE contents_on_release_user_data);

//------------------------------------------------------------------------------
/// @brief      Create a texture with an externally created OpenGL texture
///             handle.
///
///             Ownership of the handle is transferred over to Impeller after a
///             successful call to this method. Impeller is responsible for
///             calling glDeleteTextures on this handle. Do **not** collect this
///             handle yourself as this will lead to a double-free.
///
///             The handle must be created in the same context as the one used
///             by Impeller. If a different context is used, that context must
///             be in the same sharegroup as Impellers OpenGL context and all
///             synchronization of texture contents must already be complete.
///
///             If the context is not an OpenGL context, this call will always
///             fail.
///
/// @param[in]  context     The context
/// @param[in]  descriptor  The descriptor
/// @param[in]  handle      The handle
///
/// @return     The texture if one could be created by adopting the supplied
///             texture handle, NULL otherwise.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerTexture IMPELLER_NULLABLE
ImpellerTextureCreateWithOpenGLTextureHandleNew(
    ImpellerContext IMPELLER_NONNULL context,
    const ImpellerTextureDescriptor* IMPELLER_NONNULL descriptor,
    uint64_t handle  // transfer-in ownership
);

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  texture  The texture.
///
IMPELLER_EXPORT
void ImpellerTextureRetain(ImpellerTexture IMPELLER_NULLABLE texture);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  texture  The texture.
///
IMPELLER_EXPORT
void ImpellerTextureRelease(ImpellerTexture IMPELLER_NULLABLE texture);

//------------------------------------------------------------------------------
/// @brief      Get the OpenGL handle associated with this texture. If this is
///             not an OpenGL texture, this method will always return 0.
///
///             OpenGL handles are lazily created, this method will return
///             GL_NONE is no OpenGL handle is available. To ensure that this
///             call eagerly creates an OpenGL texture, call this on a thread
///             where Impeller knows there is an OpenGL context available.
///
/// @param[in]  texture  The texture.
///
/// @return     The OpenGL handle if one is available, GL_NONE otherwise.
///
IMPELLER_EXPORT
uint64_t ImpellerTextureGetOpenGLHandle(
    ImpellerTexture IMPELLER_NONNULL texture);

//------------------------------------------------------------------------------
// Color Sources
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  color_source  The color source.
///

IMPELLER_EXPORT
void ImpellerColorSourceRetain(
    ImpellerColorSource IMPELLER_NULLABLE color_source);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  color_source  The color source.
///
IMPELLER_EXPORT
void ImpellerColorSourceRelease(
    ImpellerColorSource IMPELLER_NULLABLE color_source);

//------------------------------------------------------------------------------
/// @brief      Create a color source that forms a linear gradient.
///
/// @param[in]  start_point     The start point.
/// @param[in]  end_point       The end point.
/// @param[in]  stop_count      The stop count.
/// @param[in]  colors          The colors.
/// @param[in]  stops           The stops.
/// @param[in]  tile_mode       The tile mode.
/// @param[in]  transformation  The transformation.
///
/// @return     The color source.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerColorSource IMPELLER_NULLABLE
ImpellerColorSourceCreateLinearGradientNew(
    const ImpellerPoint* IMPELLER_NONNULL start_point,
    const ImpellerPoint* IMPELLER_NONNULL end_point,
    uint32_t stop_count,
    const ImpellerColor* IMPELLER_NONNULL colors,
    const float* IMPELLER_NONNULL stops,
    ImpellerTileMode tile_mode,
    const ImpellerMatrix* IMPELLER_NULLABLE transformation);

//------------------------------------------------------------------------------
/// @brief      Create a color source that forms a radial gradient.
///
/// @param[in]  center          The center.
/// @param[in]  radius          The radius.
/// @param[in]  stop_count      The stop count.
/// @param[in]  colors          The colors.
/// @param[in]  stops           The stops.
/// @param[in]  tile_mode       The tile mode.
/// @param[in]  transformation  The transformation.
///
/// @return     The color source.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerColorSource IMPELLER_NULLABLE
ImpellerColorSourceCreateRadialGradientNew(
    const ImpellerPoint* IMPELLER_NONNULL center,
    float radius,
    uint32_t stop_count,
    const ImpellerColor* IMPELLER_NONNULL colors,
    const float* IMPELLER_NONNULL stops,
    ImpellerTileMode tile_mode,
    const ImpellerMatrix* IMPELLER_NULLABLE transformation);

//------------------------------------------------------------------------------
/// @brief      Create a color source that forms a conical gradient.
///
/// @param[in]  start_center    The start center.
/// @param[in]  start_radius    The start radius.
/// @param[in]  end_center      The end center.
/// @param[in]  end_radius      The end radius.
/// @param[in]  stop_count      The stop count.
/// @param[in]  colors          The colors.
/// @param[in]  stops           The stops.
/// @param[in]  tile_mode       The tile mode.
/// @param[in]  transformation  The transformation.
///
/// @return     The color source.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerColorSource IMPELLER_NULLABLE
ImpellerColorSourceCreateConicalGradientNew(
    const ImpellerPoint* IMPELLER_NONNULL start_center,
    float start_radius,
    const ImpellerPoint* IMPELLER_NONNULL end_center,
    float end_radius,
    uint32_t stop_count,
    const ImpellerColor* IMPELLER_NONNULL colors,
    const float* IMPELLER_NONNULL stops,
    ImpellerTileMode tile_mode,
    const ImpellerMatrix* IMPELLER_NULLABLE transformation);

//------------------------------------------------------------------------------
/// @brief      Create a color source that forms a sweep gradient.
///
/// @param[in]  center          The center.
/// @param[in]  start           The start.
/// @param[in]  end             The end.
/// @param[in]  stop_count      The stop count.
/// @param[in]  colors          The colors.
/// @param[in]  stops           The stops.
/// @param[in]  tile_mode       The tile mode.
/// @param[in]  transformation  The transformation.
///
/// @return     The color source.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerColorSource IMPELLER_NULLABLE
ImpellerColorSourceCreateSweepGradientNew(
    const ImpellerPoint* IMPELLER_NONNULL center,
    float start,
    float end,
    uint32_t stop_count,
    const ImpellerColor* IMPELLER_NONNULL colors,
    const float* IMPELLER_NONNULL stops,
    ImpellerTileMode tile_mode,
    const ImpellerMatrix* IMPELLER_NULLABLE transformation);

//------------------------------------------------------------------------------
/// @brief      Create a color source that samples from an image.
///
/// @param[in]  image                 The image.
/// @param[in]  horizontal_tile_mode  The horizontal tile mode.
/// @param[in]  vertical_tile_mode    The vertical tile mode.
/// @param[in]  sampling              The sampling.
/// @param[in]  transformation        The transformation.
///
/// @return     The color source.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerColorSource IMPELLER_NULLABLE
ImpellerColorSourceCreateImageNew(
    ImpellerTexture IMPELLER_NONNULL image,
    ImpellerTileMode horizontal_tile_mode,
    ImpellerTileMode vertical_tile_mode,
    ImpellerTextureSampling sampling,
    const ImpellerMatrix* IMPELLER_NULLABLE transformation);

//------------------------------------------------------------------------------
// Color Filters
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  color_filter  The color filter.
///
IMPELLER_EXPORT
void ImpellerColorFilterRetain(
    ImpellerColorFilter IMPELLER_NULLABLE color_filter);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  color_filter  The color filter.
///
IMPELLER_EXPORT
void ImpellerColorFilterRelease(
    ImpellerColorFilter IMPELLER_NULLABLE color_filter);

//------------------------------------------------------------------------------
/// @brief      Create a color filter that performs blending of pixel values
///             independently.
///
/// @param[in]  color       The color.
/// @param[in]  blend_mode  The blend mode.
///
/// @return     The color filter.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerColorFilter IMPELLER_NULLABLE
ImpellerColorFilterCreateBlendNew(const ImpellerColor* IMPELLER_NONNULL color,
                                  ImpellerBlendMode blend_mode);

//------------------------------------------------------------------------------
/// @brief      Create a color filter that transforms pixel color values
///             independently.
///
/// @param[in]  color_matrix  The color matrix.
///
/// @return     The color filter.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerColorFilter IMPELLER_NULLABLE
ImpellerColorFilterCreateColorMatrixNew(
    const ImpellerColorMatrix* IMPELLER_NONNULL color_matrix);

//------------------------------------------------------------------------------
// Mask Filters
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  mask_filter  The mask filter.
///
IMPELLER_EXPORT
void ImpellerMaskFilterRetain(ImpellerMaskFilter IMPELLER_NULLABLE mask_filter);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  mask_filter  The mask filter.
///
IMPELLER_EXPORT
void ImpellerMaskFilterRelease(
    ImpellerMaskFilter IMPELLER_NULLABLE mask_filter);

//------------------------------------------------------------------------------
/// @brief      Create a mask filter that blurs contents in the masked shape.
///
/// @param[in]  style  The style.
/// @param[in]  sigma  The sigma.
///
/// @return     The mask filter.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerMaskFilter IMPELLER_NULLABLE
ImpellerMaskFilterCreateBlurNew(ImpellerBlurStyle style, float sigma);

//------------------------------------------------------------------------------
// Image Filters
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  image_filter  The image filter.
///
IMPELLER_EXPORT
void ImpellerImageFilterRetain(
    ImpellerImageFilter IMPELLER_NULLABLE image_filter);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  image_filter  The image filter.
///
IMPELLER_EXPORT
void ImpellerImageFilterRelease(
    ImpellerImageFilter IMPELLER_NULLABLE image_filter);

//------------------------------------------------------------------------------
/// @brief      Creates an image filter that applies a Gaussian blur.
///
///             The Gaussian blur applied may be an approximation for
///             performance.
///
///
/// @param[in]  x_sigma    The x sigma.
/// @param[in]  y_sigma    The y sigma.
/// @param[in]  tile_mode  The tile mode.
///
/// @return     The image filter.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerImageFilter IMPELLER_NULLABLE
ImpellerImageFilterCreateBlurNew(float x_sigma,
                                 float y_sigma,
                                 ImpellerTileMode tile_mode);

//------------------------------------------------------------------------------
/// @brief      Creates an image filter that enhances the per-channel pixel
///             values to the maximum value in a circle around the pixel.
///
/// @param[in]  x_radius  The x radius.
/// @param[in]  y_radius  The y radius.
///
/// @return     The image filter.
///

IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerImageFilter IMPELLER_NULLABLE
ImpellerImageFilterCreateDilateNew(float x_radius, float y_radius);

//------------------------------------------------------------------------------
/// @brief      Creates an image filter that dampens the per-channel pixel
///             values to the minimum value in a circle around the pixel.
///
/// @param[in]  x_radius  The x radius.
/// @param[in]  y_radius  The y radius.
///
/// @return     The image filter.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerImageFilter IMPELLER_NULLABLE
ImpellerImageFilterCreateErodeNew(float x_radius, float y_radius);

//------------------------------------------------------------------------------
/// @brief      Creates an image filter that applies a transformation matrix to
///             the underlying image.
///
/// @param[in]  matrix    The transformation matrix.
/// @param[in]  sampling  The image sampling mode.
///
/// @return     The image filter.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerImageFilter IMPELLER_NULLABLE
ImpellerImageFilterCreateMatrixNew(
    const ImpellerMatrix* IMPELLER_NONNULL matrix,
    ImpellerTextureSampling sampling);

//------------------------------------------------------------------------------
/// @brief      Creates a composed filter that when applied is identical to
///             subsequently applying the inner and then the outer filters.
///
///             ```
///             destination = outer_filter(inner_filter(source))
///             ```
///
/// @param[in]  outer  The outer image filter.
/// @param[in]  inner  The inner image filter.
///
/// @return     The combined image filter.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerImageFilter IMPELLER_NULLABLE
ImpellerImageFilterCreateComposeNew(ImpellerImageFilter IMPELLER_NONNULL outer,
                                    ImpellerImageFilter IMPELLER_NONNULL inner);

//------------------------------------------------------------------------------
// Display List
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  display_list  The display list.
///
IMPELLER_EXPORT
void ImpellerDisplayListRetain(
    ImpellerDisplayList IMPELLER_NULLABLE display_list);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  display_list  The display list.
///
IMPELLER_EXPORT
void ImpellerDisplayListRelease(
    ImpellerDisplayList IMPELLER_NULLABLE display_list);

//------------------------------------------------------------------------------
// Display List Builder
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Create a new display list builder.
///
///             An optional cull rectangle may be specified. Impeller is allowed
///             to treat the contents outside this rectangle as being undefined.
///             This may aid performance optimizations.
///
/// @param[in]  cull_rect  The cull rectangle or NULL.
///
/// @return     The display list builder.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerDisplayListBuilder IMPELLER_NULLABLE
ImpellerDisplayListBuilderNew(const ImpellerRect* IMPELLER_NULLABLE cull_rect);

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  builder  The display list builder.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderRetain(
    ImpellerDisplayListBuilder IMPELLER_NULLABLE builder);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  builder  The display list builder.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderRelease(
    ImpellerDisplayListBuilder IMPELLER_NULLABLE builder);

//------------------------------------------------------------------------------
/// @brief      Create a new display list using the rendering intent already
///             encoded in the builder. The builder is reset after this call.
///
/// @param[in]  builder  The builder.
///
/// @return     The display list.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerDisplayList IMPELLER_NULLABLE
ImpellerDisplayListBuilderCreateDisplayListNew(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder);

//------------------------------------------------------------------------------
// Display List Builder: Managing the transformation stack.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Stashes the current transformation and clip state onto a save
///             stack.
///
/// @param[in]  builder  The builder.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderSave(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder);

//------------------------------------------------------------------------------
/// @brief      Stashes the current transformation and clip state onto a save
///             stack and creates and creates an offscreen layer onto which
///             subsequent rendering intent will be directed to.
///
///             On the balancing call to restore, the supplied paints filters
///             and blend modes will be used to composite the offscreen contents
///             back onto the display display list.
///
/// @param[in]  builder   The builder.
/// @param[in]  bounds    The bounds.
/// @param[in]  paint     The paint.
/// @param[in]  backdrop  The backdrop.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderSaveLayer(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerRect* IMPELLER_NONNULL bounds,
    ImpellerPaint IMPELLER_NULLABLE paint,
    ImpellerImageFilter IMPELLER_NULLABLE backdrop);

//------------------------------------------------------------------------------
/// @brief      Pops the last entry pushed onto the save stack using a call to
///             `ImpellerDisplayListBuilderSave` or
///             `ImpellerDisplayListBuilderSaveLayer`.
///
/// @param[in]  builder  The builder.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderRestore(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder);

//------------------------------------------------------------------------------
/// @brief      Apply a scale to the transformation matrix currently on top of
///             the save stack.
///
/// @param[in]  builder  The builder.
/// @param[in]  x_scale  The x scale.
/// @param[in]  y_scale  The y scale.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderScale(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    float x_scale,
    float y_scale);

//------------------------------------------------------------------------------
/// @brief      Apply a clockwise rotation to the transformation matrix
///             currently on top of the save stack.
///
/// @param[in]  builder        The builder.
/// @param[in]  angle_degrees  The angle in degrees.
///

IMPELLER_EXPORT
void ImpellerDisplayListBuilderRotate(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    float angle_degrees);

//------------------------------------------------------------------------------
/// @brief      Apply a translation to the transformation matrix currently on
///             top of the save stack.
///
/// @param[in]  builder        The builder.
/// @param[in]  x_translation  The x translation.
/// @param[in]  y_translation  The y translation.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderTranslate(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    float x_translation,
    float y_translation);

//------------------------------------------------------------------------------
/// @brief      Appends the the provided transformation to the transformation
///             already on the save stack.
///
/// @param[in]  builder    The builder.
/// @param[in]  transform  The transform to append.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderTransform(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerMatrix* IMPELLER_NONNULL transform);

//------------------------------------------------------------------------------
/// @brief      Clear the transformation on top of the save stack and replace it
///             with a new value.
///
/// @param[in]  builder    The builder.
/// @param[in]  transform  The new transform.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderSetTransform(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerMatrix* IMPELLER_NONNULL transform);

//------------------------------------------------------------------------------
/// @brief      Get the transformation currently built up on the top of the
///             transformation stack.
///
/// @param[in]  builder        The builder.
/// @param[out] out_transform  The transform.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderGetTransform(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    ImpellerMatrix* IMPELLER_NONNULL out_transform);

//------------------------------------------------------------------------------
/// @brief      Reset the transformation on top of the transformation stack to
///             identity.
///
/// @param[in]  builder  The builder.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderResetTransform(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder);

//------------------------------------------------------------------------------
/// @brief      Get the current size of the save stack.
///
/// @param[in]  builder  The builder.
///
/// @return     The save stack size.
///
IMPELLER_EXPORT
uint32_t ImpellerDisplayListBuilderGetSaveCount(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder);

//------------------------------------------------------------------------------
/// @brief      Effectively calls ImpellerDisplayListBuilderRestore till the
///             size of the save stack becomes a specified count.
///
/// @param[in]  builder  The builder.
/// @param[in]  count    The count.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderRestoreToCount(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    uint32_t count);

//------------------------------------------------------------------------------
// Display List Builder: Clipping
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Reduces the clip region to the intersection of the current clip
///             and the given rectangle taking into account the clip operation.
///
/// @param[in]  builder  The builder.
/// @param[in]  rect     The rectangle.
/// @param[in]  op       The operation.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderClipRect(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerRect* IMPELLER_NONNULL rect,
    ImpellerClipOperation op);

//------------------------------------------------------------------------------
/// @brief      Reduces the clip region to the intersection of the current clip
///             and the given oval taking into account the clip operation.
///
/// @param[in]  builder      The builder.
/// @param[in]  oval_bounds  The oval bounds.
/// @param[in]  op           The operation.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderClipOval(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerRect* IMPELLER_NONNULL oval_bounds,
    ImpellerClipOperation op);

//------------------------------------------------------------------------------
/// @brief      Reduces the clip region to the intersection of the current clip
///             and the given rounded rectangle taking into account the clip
///             operation.
///
/// @param[in]  builder  The builder.
/// @param[in]  rect     The rectangle.
/// @param[in]  radii    The radii.
/// @param[in]  op       The operation.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderClipRoundedRect(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerRect* IMPELLER_NONNULL rect,
    const ImpellerRoundingRadii* IMPELLER_NONNULL radii,
    ImpellerClipOperation op);

//------------------------------------------------------------------------------
/// @brief      Reduces the clip region to the intersection of the current clip
///             and the given path taking into account the clip operation.
///
/// @param[in]  builder  The builder.
/// @param[in]  path     The path.
/// @param[in]  op       The operation.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderClipPath(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    ImpellerPath IMPELLER_NONNULL path,
    ImpellerClipOperation op);

//------------------------------------------------------------------------------
// Display List Builder: Drawing Shapes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Fills the current clip with the specified paint.
///
/// @param[in]  builder  The builder.
/// @param[in]  paint    The paint.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawPaint(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    ImpellerPaint IMPELLER_NONNULL paint);

//------------------------------------------------------------------------------
/// @brief      Draws a line segment.
///
/// @param[in]  builder  The builder.
/// @param[in]  from     The starting point of the line.
/// @param[in]  to       The end point of the line.
/// @param[in]  paint    The paint.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawLine(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerPoint* IMPELLER_NONNULL from,
    const ImpellerPoint* IMPELLER_NONNULL to,
    ImpellerPaint IMPELLER_NONNULL paint);

//------------------------------------------------------------------------------
/// @brief      Draws a dash line segment.
///
/// @param[in]  builder     The builder.
/// @param[in]  from        The starting point of the line.
/// @param[in]  to          The end point of the line.
/// @param[in]  on_length   On length.
/// @param[in]  off_length  Off length.
/// @param[in]  paint       The paint.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawDashedLine(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerPoint* IMPELLER_NONNULL from,
    const ImpellerPoint* IMPELLER_NONNULL to,
    float on_length,
    float off_length,
    ImpellerPaint IMPELLER_NONNULL paint);

//------------------------------------------------------------------------------
/// @brief      Draws a rectangle.
///
/// @param[in]  builder  The builder.
/// @param[in]  rect     The rectangle.
/// @param[in]  paint    The paint.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawRect(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerRect* IMPELLER_NONNULL rect,
    ImpellerPaint IMPELLER_NONNULL paint);

//------------------------------------------------------------------------------
/// @brief      Draws an oval.
///
/// @param[in]  builder      The builder.
/// @param[in]  oval_bounds  The oval bounds.
/// @param[in]  paint        The paint.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawOval(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerRect* IMPELLER_NONNULL oval_bounds,
    ImpellerPaint IMPELLER_NONNULL paint);

//------------------------------------------------------------------------------
/// @brief      Draws a rounded rect.
///
/// @param[in]  builder  The builder.
/// @param[in]  rect     The rectangle.
/// @param[in]  radii    The radii.
/// @param[in]  paint    The paint.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawRoundedRect(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerRect* IMPELLER_NONNULL rect,
    const ImpellerRoundingRadii* IMPELLER_NONNULL radii,
    ImpellerPaint IMPELLER_NONNULL paint);

//------------------------------------------------------------------------------
/// @brief      Draws a shape that is the different between the specified
///             rectangles (each with configurable corner radii).
///
/// @param[in]  builder      The builder.
/// @param[in]  outer_rect   The outer rectangle.
/// @param[in]  outer_radii  The outer radii.
/// @param[in]  inner_rect   The inner rectangle.
/// @param[in]  inner_radii  The inner radii.
/// @param[in]  paint        The paint.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawRoundedRectDifference(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    const ImpellerRect* IMPELLER_NONNULL outer_rect,
    const ImpellerRoundingRadii* IMPELLER_NONNULL outer_radii,
    const ImpellerRect* IMPELLER_NONNULL inner_rect,
    const ImpellerRoundingRadii* IMPELLER_NONNULL inner_radii,
    ImpellerPaint IMPELLER_NONNULL paint);

//------------------------------------------------------------------------------
/// @brief      Draws the specified shape.
///
/// @param[in]  builder  The builder.
/// @param[in]  path     The path.
/// @param[in]  paint    The paint.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawPath(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    ImpellerPath IMPELLER_NONNULL path,
    ImpellerPaint IMPELLER_NONNULL paint);

//------------------------------------------------------------------------------
/// @brief      Flattens the contents of another display list into the one
///             currently being built.
///
/// @param[in]  builder       The builder.
/// @param[in]  display_list  The display list.
/// @param[in]  opacity       The opacity.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawDisplayList(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    ImpellerDisplayList IMPELLER_NONNULL display_list,
    float opacity);

//------------------------------------------------------------------------------
/// @brief      Draw a paragraph at the specified point.
///
/// @param[in]  builder    The builder.
/// @param[in]  paragraph  The paragraph.
/// @param[in]  point      The point.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawParagraph(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    ImpellerParagraph IMPELLER_NONNULL paragraph,
    const ImpellerPoint* IMPELLER_NONNULL point);

//------------------------------------------------------------------------------
// Display List Builder: Drawing Textures
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Draw a texture at the specified point.
///
/// @param[in]  builder   The builder.
/// @param[in]  texture   The texture.
/// @param[in]  point     The point.
/// @param[in]  sampling  The sampling.
/// @param[in]  paint     The paint.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawTexture(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    ImpellerTexture IMPELLER_NONNULL texture,
    const ImpellerPoint* IMPELLER_NONNULL point,
    ImpellerTextureSampling sampling,
    ImpellerPaint IMPELLER_NULLABLE paint);

//------------------------------------------------------------------------------
/// @brief      Draw a portion of texture at the specified location.
///
/// @param[in]  builder   The builder.
/// @param[in]  texture   The texture.
/// @param[in]  src_rect  The source rectangle.
/// @param[in]  dst_rect  The destination rectangle.
/// @param[in]  sampling  The sampling.
/// @param[in]  paint     The paint.
///
IMPELLER_EXPORT
void ImpellerDisplayListBuilderDrawTextureRect(
    ImpellerDisplayListBuilder IMPELLER_NONNULL builder,
    ImpellerTexture IMPELLER_NONNULL texture,
    const ImpellerRect* IMPELLER_NONNULL src_rect,
    const ImpellerRect* IMPELLER_NONNULL dst_rect,
    ImpellerTextureSampling sampling,
    ImpellerPaint IMPELLER_NULLABLE paint);

//------------------------------------------------------------------------------
// Typography Context
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Create a new typography contents.
///
/// @return     The typography context.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerTypographyContext IMPELLER_NULLABLE
ImpellerTypographyContextNew();

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  context  The typography context.
///
IMPELLER_EXPORT
void ImpellerTypographyContextRetain(
    ImpellerTypographyContext IMPELLER_NULLABLE context);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  context  The typography context.
///
IMPELLER_EXPORT
void ImpellerTypographyContextRelease(
    ImpellerTypographyContext IMPELLER_NULLABLE context);

//------------------------------------------------------------------------------
/// @brief      Register a custom font.
///
///             The following font formats are supported:
///             * OpenType font collections (.ttc extension)
///             * TrueType fonts: (.ttf extension)
///             * OpenType fonts: (.otf extension)
///
/// @warning    Web Open Font Formats (.woff and .woff2 extensions) are **not**
///             supported.
///
///             The font data is specified as a mapping. It is possible for the
///             release callback of the mapping to not be called even past the
///             destruction of the typography context. Care must be taken to not
///             collect the mapping till the release callback is invoked by
///             Impeller.
///
///             The family alias name can be NULL. In such cases, the font
///             family specified in paragraph styles must match the family that
///             is specified in the font data.
///
///             If the family name alias is not NULL, that family name must be
///             used in the paragraph style to reference glyphs from this font
///             instead of the one encoded in the font itself.
///
///             Multiple fonts (with glyphs for different styles) can be
///             specified with the same family.
///
/// @see        `ImpellerParagraphStyleSetFontFamily`
///
/// @param[in]  context                        The context.
/// @param[in]  contents                       The contents.
/// @param[in]  contents_on_release_user_data  The user data baton to be passed
///                                            to the contents release callback.
/// @param[in]  family_name_alias              The family name alias or NULL if
///                                            the one specified in the font
///                                            data is to be used.
///
/// @return     If the font could be successfully registered.
///
IMPELLER_EXPORT
bool ImpellerTypographyContextRegisterFont(
    ImpellerTypographyContext IMPELLER_NONNULL context,
    const ImpellerMapping* IMPELLER_NONNULL contents,
    void* IMPELLER_NULLABLE contents_on_release_user_data,
    const char* IMPELLER_NULLABLE family_name_alias);

//------------------------------------------------------------------------------
// Paragraph Style
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Create a new paragraph style.
///
/// @return     The paragraph style.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerParagraphStyle IMPELLER_NULLABLE
ImpellerParagraphStyleNew();

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  paragraph_style  The paragraph style.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleRetain(
    ImpellerParagraphStyle IMPELLER_NULLABLE paragraph_style);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  paragraph_style  The paragraph style.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleRelease(
    ImpellerParagraphStyle IMPELLER_NULLABLE paragraph_style);

//------------------------------------------------------------------------------
/// @brief      Set the paint used to render the text glyph contents.
///
/// @param[in]  paragraph_style  The paragraph style.
/// @param[in]  paint            The paint.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleSetForeground(
    ImpellerParagraphStyle IMPELLER_NONNULL paragraph_style,
    ImpellerPaint IMPELLER_NONNULL paint);

//------------------------------------------------------------------------------
/// @brief      Set the paint used to render the background of the text glyphs.
///
/// @param[in]  paragraph_style  The paragraph style.
/// @param[in]  paint            The paint.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleSetBackground(
    ImpellerParagraphStyle IMPELLER_NONNULL paragraph_style,
    ImpellerPaint IMPELLER_NONNULL paint);

//------------------------------------------------------------------------------
/// @brief      Set the weight of the font to select when rendering glyphs.
///
/// @param[in]  paragraph_style  The paragraph style.
/// @param[in]  weight           The weight.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleSetFontWeight(
    ImpellerParagraphStyle IMPELLER_NONNULL paragraph_style,
    ImpellerFontWeight weight);

//------------------------------------------------------------------------------
/// @brief      Set whether the glyphs should be bolded or italicized.
///
/// @param[in]  paragraph_style  The paragraph style.
/// @param[in]  style            The style.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleSetFontStyle(
    ImpellerParagraphStyle IMPELLER_NONNULL paragraph_style,
    ImpellerFontStyle style);

//------------------------------------------------------------------------------
/// @brief      Set the font family.
///
/// @param[in]  paragraph_style  The paragraph style.
/// @param[in]  family_name      The family name.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleSetFontFamily(
    ImpellerParagraphStyle IMPELLER_NONNULL paragraph_style,
    const char* IMPELLER_NONNULL family_name);

//------------------------------------------------------------------------------
/// @brief      Set the font size.
///
/// @param[in]  paragraph_style  The paragraph style.
/// @param[in]  size             The size.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleSetFontSize(
    ImpellerParagraphStyle IMPELLER_NONNULL paragraph_style,
    float size);

//------------------------------------------------------------------------------
/// @brief      The height of the text as a multiple of text size.
///
///             When height is 0.0, the line height will be determined by the
///             font's metrics directly, which may differ from the font size.
///             Otherwise the line height of the text will be a multiple of font
///             size, and be exactly fontSize * height logical pixels tall.
///
/// @param[in]  paragraph_style  The paragraph style.
/// @param[in]  height           The height.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleSetHeight(
    ImpellerParagraphStyle IMPELLER_NONNULL paragraph_style,
    float height);

//------------------------------------------------------------------------------
/// @brief      Set the alignment of text within the paragraph.
///
/// @param[in]  paragraph_style  The paragraph style.
/// @param[in]  align            The align.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleSetTextAlignment(
    ImpellerParagraphStyle IMPELLER_NONNULL paragraph_style,
    ImpellerTextAlignment align);

//------------------------------------------------------------------------------
/// @brief      Set the directionality of the text within the paragraph.
///
/// @param[in]  paragraph_style  The paragraph style.
/// @param[in]  direction        The direction.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleSetTextDirection(
    ImpellerParagraphStyle IMPELLER_NONNULL paragraph_style,
    ImpellerTextDirection direction);

//------------------------------------------------------------------------------
/// @brief      Set the maximum line count within the paragraph.
///
/// @param[in]  paragraph_style  The paragraph style.
/// @param[in]  max_lines        The maximum lines.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleSetMaxLines(
    ImpellerParagraphStyle IMPELLER_NONNULL paragraph_style,
    uint32_t max_lines);

//------------------------------------------------------------------------------
/// @brief      Set the paragraph locale.
///
/// @param[in]  paragraph_style  The paragraph style.
/// @param[in]  locale           The locale.
///
IMPELLER_EXPORT
void ImpellerParagraphStyleSetLocale(
    ImpellerParagraphStyle IMPELLER_NONNULL paragraph_style,
    const char* IMPELLER_NONNULL locale);

//------------------------------------------------------------------------------
// Paragraph Builder
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Create a new paragraph builder.
///
/// @param[in]  context  The context.
///
/// @return     The paragraph builder.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerParagraphBuilder IMPELLER_NULLABLE
ImpellerParagraphBuilderNew(ImpellerTypographyContext IMPELLER_NONNULL context);

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  paragraph_builder  The paragraph builder.
///
IMPELLER_EXPORT
void ImpellerParagraphBuilderRetain(
    ImpellerParagraphBuilder IMPELLER_NULLABLE paragraph_builder);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  paragraph_builder  The paragraph_builder.
///
IMPELLER_EXPORT
void ImpellerParagraphBuilderRelease(
    ImpellerParagraphBuilder IMPELLER_NULLABLE paragraph_builder);

//------------------------------------------------------------------------------
/// @brief      Push a new paragraph style onto the paragraph style stack
///             managed by the paragraph builder.
///
///             Not all paragraph styles can be combined. For instance, it does
///             not make sense to mix text alignment for different text runs
///             within a paragraph. In such cases, the preference of the the
///             first paragraph style on the style stack will take hold.
///
///             If text is pushed onto the paragraph builder without a style
///             previously pushed onto the stack, a default paragraph text style
///             will be used. This may not always be desirable because some
///             style element cannot be overridden. It is recommended that a
///             default paragraph style always be pushed onto the stack before
///             the addition of any text.
///
/// @param[in]  paragraph_builder  The paragraph builder.
/// @param[in]  style              The style.
///
IMPELLER_EXPORT
void ImpellerParagraphBuilderPushStyle(
    ImpellerParagraphBuilder IMPELLER_NONNULL paragraph_builder,
    ImpellerParagraphStyle IMPELLER_NONNULL style);

//------------------------------------------------------------------------------
/// @brief      Pop a previously pushed paragraph style from the paragraph style
///             stack.
///
/// @param[in]  paragraph_builder  The paragraph builder.
///
IMPELLER_EXPORT
void ImpellerParagraphBuilderPopStyle(
    ImpellerParagraphBuilder IMPELLER_NONNULL paragraph_builder);

//------------------------------------------------------------------------------
/// @brief      Add UTF-8 encoded text to the paragraph. The text will be styled
///             according to the paragraph style already on top of the paragraph
///             style stack.
///
/// @param[in]  paragraph_builder  The paragraph builder.
/// @param[in]  data               The data.
/// @param[in]  length             The length.
///
IMPELLER_EXPORT
void ImpellerParagraphBuilderAddText(
    ImpellerParagraphBuilder IMPELLER_NONNULL paragraph_builder,
    const uint8_t* IMPELLER_NULLABLE data,
    uint32_t length);

//------------------------------------------------------------------------------
/// @brief      Layout and build a new paragraph using the specified width. The
///             resulting paragraph is immutable. The paragraph builder must be
///             discarded and a new one created to build more paragraphs.
///
/// @param[in]  paragraph_builder  The paragraph builder.
/// @param[in]  width              The paragraph width.
///
/// @return     The paragraph if one can be created, NULL otherwise.
///
IMPELLER_EXPORT IMPELLER_NODISCARD ImpellerParagraph IMPELLER_NULLABLE
ImpellerParagraphBuilderBuildParagraphNew(
    ImpellerParagraphBuilder IMPELLER_NONNULL paragraph_builder,
    float width);

//------------------------------------------------------------------------------
// Paragraph
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// @brief      Retain a strong reference to the object. The object can be NULL
///             in which case this method is a no-op.
///
/// @param[in]  paragraph  The paragraph.
///
IMPELLER_EXPORT
void ImpellerParagraphRetain(ImpellerParagraph IMPELLER_NULLABLE paragraph);

//------------------------------------------------------------------------------
/// @brief      Release a previously retained reference to the object. The
///             object can be NULL in which case this method is a no-op.
///
/// @param[in]  paragraph  The paragraph.
///
IMPELLER_EXPORT
void ImpellerParagraphRelease(ImpellerParagraph IMPELLER_NULLABLE paragraph);

//------------------------------------------------------------------------------
/// @see        `ImpellerParagraphGetMinIntrinsicWidth`
///
/// @param[in]  paragraph  The paragraph.
///
///
/// @return     The width provided to the paragraph builder during the call to
///             layout. This is the maximum width any line in the laid out
///             paragraph can occupy. But, it is not necessarily the actual
///             width of the paragraph after layout.
///
IMPELLER_EXPORT
float ImpellerParagraphGetMaxWidth(
    ImpellerParagraph IMPELLER_NONNULL paragraph);

//------------------------------------------------------------------------------
/// @param[in]  paragraph  The paragraph.
///
/// @return     The height of the laid out paragraph. This is **not** a tight
///             bounding box and some glyphs may not reach the minimum location
///             they are allowed to reach.
///
IMPELLER_EXPORT
float ImpellerParagraphGetHeight(ImpellerParagraph IMPELLER_NONNULL paragraph);

//------------------------------------------------------------------------------
/// @param[in]  paragraph  The paragraph.
///
/// @return     The length of the longest line in the paragraph. This is the
///             horizontal distance between the left edge of the leftmost glyph
///             and the right edge of the rightmost glyph, in the longest line
///             in the paragraph.
///
IMPELLER_EXPORT
float ImpellerParagraphGetLongestLineWidth(
    ImpellerParagraph IMPELLER_NONNULL paragraph);

//------------------------------------------------------------------------------
/// @see        `ImpellerParagraphGetMaxWidth`
///
/// @param[in]  paragraph  The paragraph.
///
/// @return     The actual width of the longest line in the paragraph after
///             layout. This is expected to be less than or equal to
///             `ImpellerParagraphGetMaxWidth`.
///
IMPELLER_EXPORT
float ImpellerParagraphGetMinIntrinsicWidth(
    ImpellerParagraph IMPELLER_NONNULL paragraph);

//------------------------------------------------------------------------------
/// @param[in]  paragraph  The paragraph.
///
/// @return     The width of the paragraph without line breaking.
///
IMPELLER_EXPORT
float ImpellerParagraphGetMaxIntrinsicWidth(
    ImpellerParagraph IMPELLER_NONNULL paragraph);

//------------------------------------------------------------------------------
/// @param[in]  paragraph  The paragraph.
///
/// @return     The distance from the top of the paragraph to the ideographic
///             baseline of the first line when using ideographic fonts
///             (Japanese, Korean, etc...).
///
IMPELLER_EXPORT
float ImpellerParagraphGetIdeographicBaseline(
    ImpellerParagraph IMPELLER_NONNULL paragraph);

//------------------------------------------------------------------------------
/// @param[in]  paragraph  The paragraph.
///
/// @return     The distance from the top of the paragraph to the alphabetic
///             baseline of the first line when using alphabetic fonts (A-Z,
///             a-z, Greek, etc...).
///
IMPELLER_EXPORT
float ImpellerParagraphGetAlphabeticBaseline(
    ImpellerParagraph IMPELLER_NONNULL paragraph);

//------------------------------------------------------------------------------
/// @param[in]  paragraph  The paragraph.
///
/// @return     The number of lines visible in the paragraph after line
///             breaking.
///
IMPELLER_EXPORT
uint32_t ImpellerParagraphGetLineCount(
    ImpellerParagraph IMPELLER_NONNULL paragraph);

IMPELLER_EXTERN_C_END

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_IMPELLER_H_
