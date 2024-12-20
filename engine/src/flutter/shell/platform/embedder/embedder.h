// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// This file defines an Application Binary Interface (ABI), which requires more
// stability than regular code to remain functional for exchanging messages
// between different versions of the embedding and the engine, to allow for both
// forward and backward compatibility.
//
// Specifically,
// - The order, type, and size of the struct members below must remain the same,
//   and members should not be removed.
// - New structures that are part of the ABI must be defined with "size_t
//   struct_size;" as their first member, which should be initialized using
//   "sizeof(Type)".
// - Enum values must not change or be removed.
// - Enum members without explicit values must not be reordered.
// - Function signatures (names, argument counts, argument order, and argument
//   type) cannot change.
// - The core behavior of existing functions cannot change.
// - Instead of nesting structures by value within another structure/union,
//   prefer nesting by pointer. This ensures that adding members to the nested
//   struct does not break the ABI of the parent struct/union.
// - Instead of array of structures, prefer array of pointers to structures.
//   This ensures that array indexing does not break if members are added
//   to the structure.
//
// These changes are allowed:
// - Adding new struct members at the end of a structure as long as the struct
//   is not nested within another struct by value.
// - Adding new enum members with a new value.
// - Renaming a struct member as long as its type, size, and intent remain the
//   same.
// - Renaming an enum member as long as its value and intent remains the same.
//
// It is expected that struct members and implicitly-valued enums will not
// always be declared in an order that is optimal for the reader, since members
// will be added over time, and they can't be reordered.
//
// Existing functions should continue to appear from the caller's point of view
// to operate as they did when they were first introduced, so introduce a new
// function instead of modifying the core behavior of a function (and continue
// to support the existing function with the previous behavior).

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef FLUTTER_EXPORT
#define FLUTTER_EXPORT
#endif  // FLUTTER_EXPORT

#ifdef FLUTTER_API_SYMBOL_PREFIX
#define FLUTTER_EMBEDDING_CONCAT(a, b) a##b
#define FLUTTER_EMBEDDING_ADD_PREFIX(symbol, prefix) \
  FLUTTER_EMBEDDING_CONCAT(prefix, symbol)
#define FLUTTER_API_SYMBOL(symbol) \
  FLUTTER_EMBEDDING_ADD_PREFIX(symbol, FLUTTER_API_SYMBOL_PREFIX)
#else
#define FLUTTER_API_SYMBOL(symbol) symbol
#endif

#define FLUTTER_ENGINE_VERSION 1

typedef enum {
  kSuccess = 0,
  kInvalidLibraryVersion,
  kInvalidArguments,
  kInternalInconsistency,
} FlutterEngineResult;

typedef enum {
  kOpenGL,
  kSoftware,
  /// Metal is only supported on Darwin platforms (macOS / iOS).
  /// iOS version >= 10.0 (device), 13.0 (simulator)
  /// macOS version >= 10.14
  kMetal,
  kVulkan,
} FlutterRendererType;

/// Additional accessibility features that may be enabled by the platform.
/// Must match the `AccessibilityFeatures` enum in window.dart.
typedef enum {
  /// Indicate there is a running accessibility service which is changing the
  /// interaction model of the device.
  kFlutterAccessibilityFeatureAccessibleNavigation = 1 << 0,
  /// Indicate the platform is inverting the colors of the application.
  kFlutterAccessibilityFeatureInvertColors = 1 << 1,
  /// Request that animations be disabled or simplified.
  kFlutterAccessibilityFeatureDisableAnimations = 1 << 2,
  /// Request that text be rendered at a bold font weight.
  kFlutterAccessibilityFeatureBoldText = 1 << 3,
  /// Request that certain animations be simplified and parallax effects
  /// removed.
  kFlutterAccessibilityFeatureReduceMotion = 1 << 4,
  /// Request that UI be rendered with darker colors.
  kFlutterAccessibilityFeatureHighContrast = 1 << 5,
  /// Request to show on/off labels inside switches.
  kFlutterAccessibilityFeatureOnOffSwitchLabels = 1 << 6,
} FlutterAccessibilityFeature;

/// The set of possible actions that can be conveyed to a semantics node.
///
/// Must match the `SemanticsAction` enum in semantics.dart.
typedef enum {
  /// The equivalent of a user briefly tapping the screen with the finger
  /// without moving it.
  kFlutterSemanticsActionTap = 1 << 0,
  /// The equivalent of a user pressing and holding the screen with the finger
  /// for a few seconds without moving it.
  kFlutterSemanticsActionLongPress = 1 << 1,
  /// The equivalent of a user moving their finger across the screen from right
  /// to left.
  kFlutterSemanticsActionScrollLeft = 1 << 2,
  /// The equivalent of a user moving their finger across the screen from left
  /// to
  /// right.
  kFlutterSemanticsActionScrollRight = 1 << 3,
  /// The equivalent of a user moving their finger across the screen from bottom
  /// to top.
  kFlutterSemanticsActionScrollUp = 1 << 4,
  /// The equivalent of a user moving their finger across the screen from top to
  /// bottom.
  kFlutterSemanticsActionScrollDown = 1 << 5,
  /// Increase the value represented by the semantics node.
  kFlutterSemanticsActionIncrease = 1 << 6,
  /// Decrease the value represented by the semantics node.
  kFlutterSemanticsActionDecrease = 1 << 7,
  /// A request to fully show the semantics node on screen.
  kFlutterSemanticsActionShowOnScreen = 1 << 8,
  /// Move the cursor forward by one character.
  kFlutterSemanticsActionMoveCursorForwardByCharacter = 1 << 9,
  /// Move the cursor backward by one character.
  kFlutterSemanticsActionMoveCursorBackwardByCharacter = 1 << 10,
  /// Set the text selection to the given range.
  kFlutterSemanticsActionSetSelection = 1 << 11,
  /// Copy the current selection to the clipboard.
  kFlutterSemanticsActionCopy = 1 << 12,
  /// Cut the current selection and place it in the clipboard.
  kFlutterSemanticsActionCut = 1 << 13,
  /// Paste the current content of the clipboard.
  kFlutterSemanticsActionPaste = 1 << 14,
  /// Indicate that the node has gained accessibility focus.
  kFlutterSemanticsActionDidGainAccessibilityFocus = 1 << 15,
  /// Indicate that the node has lost accessibility focus.
  kFlutterSemanticsActionDidLoseAccessibilityFocus = 1 << 16,
  /// Indicate that the user has invoked a custom accessibility action.
  kFlutterSemanticsActionCustomAction = 1 << 17,
  /// A request that the node should be dismissed.
  kFlutterSemanticsActionDismiss = 1 << 18,
  /// Move the cursor forward by one word.
  kFlutterSemanticsActionMoveCursorForwardByWord = 1 << 19,
  /// Move the cursor backward by one word.
  kFlutterSemanticsActionMoveCursorBackwardByWord = 1 << 20,
  /// Replace the current text in the text field.
  kFlutterSemanticsActionSetText = 1 << 21,
  /// Request that the respective focusable widget gain input focus.
  kFlutterSemanticsActionFocus = 1 << 22,
  /// Request that scrolls the current scrollable container to a given scroll
  /// offset.
  kFlutterSemanticsActionScrollToOffset = 1 << 23,
} FlutterSemanticsAction;

/// The set of properties that may be associated with a semantics node.
///
/// Must match the `SemanticsFlag` enum in semantics.dart.
typedef enum {
  /// The semantics node has the quality of either being "checked" or
  /// "unchecked".
  kFlutterSemanticsFlagHasCheckedState = 1 << 0,
  /// Whether a semantics node is checked.
  kFlutterSemanticsFlagIsChecked = 1 << 1,
  /// Whether a semantics node is selected.
  kFlutterSemanticsFlagIsSelected = 1 << 2,
  /// Whether the semantic node represents a button.
  kFlutterSemanticsFlagIsButton = 1 << 3,
  /// Whether the semantic node represents a text field.
  kFlutterSemanticsFlagIsTextField = 1 << 4,
  /// Whether the semantic node currently holds the user's focus.
  kFlutterSemanticsFlagIsFocused = 1 << 5,
  /// The semantics node has the quality of either being "enabled" or
  /// "disabled".
  kFlutterSemanticsFlagHasEnabledState = 1 << 6,
  /// Whether a semantic node that hasEnabledState is currently enabled.
  kFlutterSemanticsFlagIsEnabled = 1 << 7,
  /// Whether a semantic node is in a mutually exclusive group.
  kFlutterSemanticsFlagIsInMutuallyExclusiveGroup = 1 << 8,
  /// Whether a semantic node is a header that divides content into sections.
  kFlutterSemanticsFlagIsHeader = 1 << 9,
  /// Whether the value of the semantics node is obscured.
  kFlutterSemanticsFlagIsObscured = 1 << 10,
  /// Whether the semantics node is the root of a subtree for which a route name
  /// should be announced.
  kFlutterSemanticsFlagScopesRoute = 1 << 11,
  /// Whether the semantics node label is the name of a visually distinct route.
  kFlutterSemanticsFlagNamesRoute = 1 << 12,
  /// Whether the semantics node is considered hidden.
  kFlutterSemanticsFlagIsHidden = 1 << 13,
  /// Whether the semantics node represents an image.
  kFlutterSemanticsFlagIsImage = 1 << 14,
  /// Whether the semantics node is a live region.
  kFlutterSemanticsFlagIsLiveRegion = 1 << 15,
  /// The semantics node has the quality of either being "on" or "off".
  kFlutterSemanticsFlagHasToggledState = 1 << 16,
  /// If true, the semantics node is "on". If false, the semantics node is
  /// "off".
  kFlutterSemanticsFlagIsToggled = 1 << 17,
  /// Whether the platform can scroll the semantics node when the user attempts
  /// to move the accessibility focus to an offscreen child.
  ///
  /// For example, a `ListView` widget has implicit scrolling so that users can
  /// easily move the accessibility focus to the next set of children. A
  /// `PageView` widget does not have implicit scrolling, so that users don't
  /// navigate to the next page when reaching the end of the current one.
  kFlutterSemanticsFlagHasImplicitScrolling = 1 << 18,
  /// Whether the value of the semantics node is coming from a multi-line text
  /// field.
  ///
  /// This is used for text fields to distinguish single-line text fields from
  /// multi-line ones.
  kFlutterSemanticsFlagIsMultiline = 1 << 19,
  /// Whether the semantic node is read only.
  ///
  /// Only applicable when kFlutterSemanticsFlagIsTextField flag is on.
  kFlutterSemanticsFlagIsReadOnly = 1 << 20,
  /// Whether the semantic node can hold the user's focus.
  kFlutterSemanticsFlagIsFocusable = 1 << 21,
  /// Whether the semantics node represents a link.
  kFlutterSemanticsFlagIsLink = 1 << 22,
  /// Whether the semantics node represents a slider.
  kFlutterSemanticsFlagIsSlider = 1 << 23,
  /// Whether the semantics node represents a keyboard key.
  kFlutterSemanticsFlagIsKeyboardKey = 1 << 24,
  /// Whether the semantics node represents a tristate checkbox in mixed state.
  kFlutterSemanticsFlagIsCheckStateMixed = 1 << 25,
  /// The semantics node has the quality of either being "expanded" or
  /// "collapsed".
  kFlutterSemanticsFlagHasExpandedState = 1 << 26,
  /// Whether a semantic node that hasExpandedState is currently expanded.
  kFlutterSemanticsFlagIsExpanded = 1 << 27,
  /// The semantics node has the quality of either being "selected" or
  /// "not selected".
  kFlutterSemanticsFlagHasSelectedState = 1 << 28,
} FlutterSemanticsFlag;

typedef enum {
  /// Text has unknown text direction.
  kFlutterTextDirectionUnknown = 0,
  /// Text is read from right to left.
  kFlutterTextDirectionRTL = 1,
  /// Text is read from left to right.
  kFlutterTextDirectionLTR = 2,
} FlutterTextDirection;

/// Valid values for priority of Thread.
typedef enum {
  /// Suitable for threads that shouldn't disrupt high priority work.
  kBackground = 0,
  /// Default priority level.
  kNormal = 1,
  /// Suitable for threads which generate data for the display.
  kDisplay = 2,
  /// Suitable for thread which raster data.
  kRaster = 3,
} FlutterThreadPriority;

typedef struct _FlutterEngine* FLUTTER_API_SYMBOL(FlutterEngine);

/// Unique identifier for views.
///
/// View IDs are generated by the embedder and are
/// opaque to the engine; the engine does not interpret view IDs in any way.
typedef int64_t FlutterViewId;

typedef struct {
  /// horizontal scale factor
  double scaleX;
  /// horizontal skew factor
  double skewX;
  /// horizontal translation
  double transX;
  /// vertical skew factor
  double skewY;
  /// vertical scale factor
  double scaleY;
  /// vertical translation
  double transY;
  /// input x-axis perspective factor
  double pers0;
  /// input y-axis perspective factor
  double pers1;
  /// perspective scale factor
  double pers2;
} FlutterTransformation;

typedef void (*VoidCallback)(void* /* user data */);

typedef enum {
  /// Specifies an OpenGL texture target type. Textures are specified using
  /// the FlutterOpenGLTexture struct.
  kFlutterOpenGLTargetTypeTexture,
  /// Specifies an OpenGL frame-buffer target type. Framebuffers are specified
  /// using the FlutterOpenGLFramebuffer struct.
  kFlutterOpenGLTargetTypeFramebuffer,
  /// Specifies an OpenGL on-screen surface target type. Surfaces are specified
  /// using the FlutterOpenGLSurface struct.
  kFlutterOpenGLTargetTypeSurface,
} FlutterOpenGLTargetType;

/// A pixel format to be used for software rendering.
///
/// A single pixel always stored as a POT number of bytes. (so in practice
/// either 1, 2, 4, 8, 16 bytes per pixel)
///
/// There are two kinds of pixel formats:
///   - formats where all components are 8 bits, called array formats
///     The component order as specified in the pixel format name is the
///     order of the components' bytes in memory, with the leftmost component
///     occupying the lowest memory address.
///
///   - all other formats are called packed formats, and the component order
///     as specified in the format name refers to the order from most
///     significant to least significant bits in the native type. for example,
///     for kFlutterSoftwarePixelFormatRGB565, R occupies the 5 most significant
///     bits, G the middle 6 bits, and B the 5 least significant bits.
///
/// Each pixel format in this list is documented with an example on how to get
/// the color components from the pixel.
/// - for packed formats, p is the pixel value as a word. For example, you can
///   get the pixel value for a RGB565 formatted buffer like this:
///   uint16_t p = ((const uint16_t*) allocation)[row_bytes * y / bpp + x];
///   (with bpp being the bytes per pixel, so 2 for RGB565)
///
/// - for array formats, p is a pointer to the pixel value. For example, you
///   can get the p for a RGBA8888 formatted buffer like this:
///   const uint8_t *p = ((const uint8_t*) allocation) + row_bytes*y + x*4;
typedef enum {
  /// Pixel with 8 bit grayscale value.
  /// The grayscale value is the luma value calculated from r, g, b
  /// according to BT.709. (gray = r*0.2126 + g*0.7152 + b*0.0722)
  kFlutterSoftwarePixelFormatGray8,

  /// Pixel with 5 bits red, 6 bits green, 5 bits blue, in 16-bit word.
  ///   r = (p >> 11) & 0x1F;
  ///   g = (p >> 5) & 0x3F;
  ///   b = p & 0x1F;
  ///
  /// On most (== little-endian) systems, this is equivalent to wayland format
  /// RGB565 (WL_DRM_FORMAT_RGB565, WL_SHM_FORMAT_RGB565).
  kFlutterSoftwarePixelFormatRGB565,

  /// Pixel with 4 bits each for alpha, red, green, blue; in 16-bit word.
  ///   r = (p >> 8) & 0xF;
  ///   g = (p >> 4) & 0xF;
  ///   b = p & 0xF;
  ///   a = (p >> 12) & 0xF;
  ///
  /// On most (== little-endian) systems, this is equivalent to wayland format
  /// RGBA4444 (WL_DRM_FORMAT_RGBA4444, WL_SHM_FORMAT_RGBA4444).
  kFlutterSoftwarePixelFormatRGBA4444,

  /// Pixel with 8 bits each for red, green, blue, alpha.
  ///   r = p[0];
  ///   g = p[1];
  ///   b = p[2];
  ///   a = p[3];
  ///
  /// This is equivalent to wayland format ABGR8888 (WL_DRM_FORMAT_ABGR8888,
  /// WL_SHM_FORMAT_ABGR8888).
  kFlutterSoftwarePixelFormatRGBA8888,

  /// Pixel with 8 bits each for red, green and blue and 8 unused bits.
  ///   r = p[0];
  ///   g = p[1];
  ///   b = p[2];
  ///
  /// This is equivalent to wayland format XBGR8888 (WL_DRM_FORMAT_XBGR8888,
  /// WL_SHM_FORMAT_XBGR8888).
  kFlutterSoftwarePixelFormatRGBX8888,

  /// Pixel with 8 bits each for blue, green, red and alpha.
  ///   r = p[2];
  ///   g = p[1];
  ///   b = p[0];
  ///   a = p[3];
  ///
  /// This is equivalent to wayland format ARGB8888 (WL_DRM_FORMAT_ARGB8888,
  /// WL_SHM_FORMAT_ARGB8888).
  kFlutterSoftwarePixelFormatBGRA8888,

  /// Either kFlutterSoftwarePixelFormatBGRA8888 or
  /// kFlutterSoftwarePixelFormatRGBA8888 depending on CPU endianess and OS.
  kFlutterSoftwarePixelFormatNative32,
} FlutterSoftwarePixelFormat;

typedef struct {
  /// Target texture of the active texture unit (example GL_TEXTURE_2D or
  /// GL_TEXTURE_RECTANGLE).
  uint32_t target;
  /// The name of the texture.
  uint32_t name;
  /// The texture format (example GL_RGBA8).
  uint32_t format;
  /// User data to be returned on the invocation of the destruction callback.
  void* user_data;
  /// Callback invoked (on an engine managed thread) that asks the embedder to
  /// collect the texture.
  VoidCallback destruction_callback;
  /// Optional parameters for texture height/width, default is 0, non-zero means
  /// the texture has the specified width/height. Usually, when the texture type
  /// is GL_TEXTURE_RECTANGLE, we need to specify the texture width/height to
  /// tell the embedder to scale when rendering.
  /// Width of the texture.
  size_t width;
  /// Height of the texture.
  size_t height;
} FlutterOpenGLTexture;

typedef struct {
  /// The format of the color attachment of the frame-buffer. For example,
  /// GL_RGBA8.
  ///
  /// In case of ambiguity when dealing with Window bound frame-buffers, 0 may
  /// be used.
  ///
  /// @bug      This field is incorrectly named as "target" when it actually
  ///           refers to a format.
  uint32_t target;

  /// The name of the framebuffer.
  uint32_t name;

  /// User data to be returned on the invocation of the destruction callback.
  void* user_data;

  /// Callback invoked (on an engine managed thread) that asks the embedder to
  /// collect the framebuffer.
  VoidCallback destruction_callback;
} FlutterOpenGLFramebuffer;

typedef bool (*FlutterOpenGLSurfaceCallback)(void* /* user data */,
                                             bool* /* opengl state changed */);

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterOpenGLSurface).
  size_t struct_size;

  /// User data to be passed to the make_current, clear_current and
  /// destruction callbacks.
  void* user_data;

  /// Callback invoked (on an engine-managed thread) that asks the embedder to
  /// make the surface current.
  ///
  /// Should return true if the operation succeeded, false if the surface could
  /// not be made current and rendering should be cancelled.
  ///
  /// The second parameter 'opengl state changed' should be set to true if
  /// any OpenGL API state is different than before this callback was called.
  /// In that case, Flutter will invalidate the internal OpenGL API state cache,
  /// which is a somewhat expensive operation.
  ///
  /// @attention required. (non-null)
  FlutterOpenGLSurfaceCallback make_current_callback;

  /// Callback invoked (on an engine-managed thread) when the current surface
  /// can be cleared.
  ///
  /// Should return true if the operation succeeded, false if an error ocurred.
  /// That error will be logged but otherwise not handled by the engine.
  ///
  /// The second parameter 'opengl state changed' is the same as with the
  /// @ref make_current_callback.
  ///
  /// The embedder might clear the surface here after it was previously made
  /// current. That's not required however, it's also possible to clear it in
  /// the destruction callback. There's no way to signal OpenGL state
  /// changes in the destruction callback though.
  ///
  /// @attention required. (non-null)
  FlutterOpenGLSurfaceCallback clear_current_callback;

  /// Callback invoked (on an engine-managed thread) that asks the embedder to
  /// collect the surface.
  ///
  /// @attention required. (non-null)
  VoidCallback destruction_callback;

  /// The surface format.
  ///
  /// Allowed values:
  ///   - GL_RGBA8
  ///   - GL_BGRA8_EXT
  uint32_t format;
} FlutterOpenGLSurface;

typedef bool (*BoolCallback)(void* /* user data */);
typedef FlutterTransformation (*TransformationCallback)(void* /* user data */);
typedef uint32_t (*UIntCallback)(void* /* user data */);
typedef bool (*SoftwareSurfacePresentCallback)(void* /* user data */,
                                               const void* /* allocation */,
                                               size_t /* row bytes */,
                                               size_t /* height */);
typedef void* (*ProcResolver)(void* /* user data */, const char* /* name */);
typedef bool (*TextureFrameCallback)(void* /* user data */,
                                     int64_t /* texture identifier */,
                                     size_t /* width */,
                                     size_t /* height */,
                                     FlutterOpenGLTexture* /* texture out */);
typedef void (*VsyncCallback)(void* /* user data */, intptr_t /* baton */);
typedef void (*OnPreEngineRestartCallback)(void* /* user data */);

/// A structure to represent the width and height.
typedef struct {
  double width;
  double height;
} FlutterSize;

/// A structure to represent the width and height.
///
/// See: \ref FlutterSize when the value are not integers.
typedef struct {
  uint32_t width;
  uint32_t height;
} FlutterUIntSize;

/// A structure to represent a rectangle.
typedef struct {
  double left;
  double top;
  double right;
  double bottom;
} FlutterRect;

/// A structure to represent a 2D point.
typedef struct {
  double x;
  double y;
} FlutterPoint;

/// A structure to represent a rounded rectangle.
typedef struct {
  FlutterRect rect;
  FlutterSize upper_left_corner_radius;
  FlutterSize upper_right_corner_radius;
  FlutterSize lower_right_corner_radius;
  FlutterSize lower_left_corner_radius;
} FlutterRoundedRect;

/// A structure to represent a damage region.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterDamage).
  size_t struct_size;
  /// The number of rectangles within the damage region.
  size_t num_rects;
  /// The actual damage region(s) in question.
  FlutterRect* damage;
} FlutterDamage;

/// This information is passed to the embedder when requesting a frame buffer
/// object.
///
/// See: \ref FlutterOpenGLRendererConfig.fbo_with_frame_info_callback,
/// \ref FlutterMetalRendererConfig.get_next_drawable_callback,
/// and \ref FlutterVulkanRendererConfig.get_next_image_callback.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterFrameInfo).
  size_t struct_size;
  /// The size of the surface that will be backed by the fbo.
  FlutterUIntSize size;
} FlutterFrameInfo;

/// Callback for when a frame buffer object is requested.
typedef uint32_t (*UIntFrameInfoCallback)(
    void* /* user data */,
    const FlutterFrameInfo* /* frame info */);

/// Callback for when a frame buffer object is requested with necessary
/// information for partial repaint.
typedef void (*FlutterFrameBufferWithDamageCallback)(
    void* /* user data */,
    const intptr_t /* fbo id */,
    FlutterDamage* /* existing damage */);

/// This information is passed to the embedder when a surface is presented.
///
/// See: \ref FlutterOpenGLRendererConfig.present_with_info.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterPresentInfo).
  size_t struct_size;
  /// Id of the fbo backing the surface that was presented.
  uint32_t fbo_id;
  /// Damage representing the area that the compositor needs to render.
  FlutterDamage frame_damage;
  /// Damage used to set the buffer's damage region.
  FlutterDamage buffer_damage;
} FlutterPresentInfo;

/// Callback for when a surface is presented.
typedef bool (*BoolPresentInfoCallback)(
    void* /* user data */,
    const FlutterPresentInfo* /* present info */);

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterOpenGLRendererConfig).
  size_t struct_size;
  BoolCallback make_current;
  BoolCallback clear_current;
  /// Specifying one (and only one) of `present` or `present_with_info` is
  /// required. Specifying both is an error and engine initialization will be
  /// terminated. The return value indicates success of the present call. If
  /// the intent is to use dirty region management, present_with_info must be
  /// defined as present will not succeed in communicating information about
  /// damage.
  BoolCallback present;
  /// Specifying one (and only one) of the `fbo_callback` or
  /// `fbo_with_frame_info_callback` is required. Specifying both is an error
  /// and engine intialization will be terminated. The return value indicates
  /// the id of the frame buffer object that flutter will obtain the gl surface
  /// from.
  UIntCallback fbo_callback;
  /// This is an optional callback. Flutter will ask the emebdder to create a GL
  /// context current on a background thread. If the embedder is able to do so,
  /// Flutter will assume that this context is in the same sharegroup as the
  /// main rendering context and use this context for asynchronous texture
  /// uploads. Though optional, it is recommended that all embedders set this
  /// callback as it will lead to better performance in texture handling.
  BoolCallback make_resource_current;
  /// By default, the renderer config assumes that the FBO does not change for
  /// the duration of the engine run. If this argument is true, the
  /// engine will ask the embedder for an updated FBO target (via an
  /// fbo_callback invocation) after a present call.
  bool fbo_reset_after_present;
  /// The transformation to apply to the render target before any rendering
  /// operations. This callback is optional.
  /// @attention      When using a custom compositor, the layer offset and sizes
  ///                 will be affected by this transformation. It will be
  ///                 embedder responsibility to render contents at the
  ///                 transformed offset and size. This is useful for embedders
  ///                 that want to render transformed contents directly into
  ///                 hardware overlay planes without having to apply extra
  ///                 transformations to layer contents (which may necessitate
  ///                 an expensive off-screen render pass).
  TransformationCallback surface_transformation;
  ProcResolver gl_proc_resolver;
  /// When the embedder specifies that a texture has a frame available, the
  /// engine will call this method (on an internal engine managed thread) so
  /// that external texture details can be supplied to the engine for subsequent
  /// composition.
  TextureFrameCallback gl_external_texture_frame_callback;
  /// Specifying one (and only one) of the `fbo_callback` or
  /// `fbo_with_frame_info_callback` is required. Specifying both is an error
  /// and engine intialization will be terminated. The return value indicates
  /// the id of the frame buffer object (fbo) that flutter will obtain the gl
  /// surface from. When using this variant, the embedder is passed a
  /// `FlutterFrameInfo` struct that indicates the properties of the surface
  /// that flutter will acquire from the returned fbo.
  UIntFrameInfoCallback fbo_with_frame_info_callback;
  /// Specifying one (and only one) of `present` or `present_with_info` is
  /// required. Specifying both is an error and engine initialization will be
  /// terminated. When using this variant, the embedder is passed a
  /// `FlutterPresentInfo` struct that the embedder can use to release any
  /// resources. The return value indicates success of the present call. This
  /// callback is essential for dirty region management. If not defined, all the
  /// pixels on the screen will be rendered at every frame (regardless of
  /// whether damage is actually being computed or not). This is because the
  /// information that is passed along to the callback contains the frame and
  /// buffer damage that are essential for dirty region management.
  BoolPresentInfoCallback present_with_info;
  /// Specifying this callback is a requirement for dirty region management.
  /// Dirty region management will only render the areas of the screen that have
  /// changed in between frames, greatly reducing rendering times and energy
  /// consumption. To take advantage of these benefits, it is necessary to
  /// define populate_existing_damage as a callback that takes user
  /// data, an FBO ID, and an existing damage FlutterDamage. The callback should
  /// use the given FBO ID to identify the FBO's exisiting damage (i.e. areas
  /// that have changed since the FBO was last used) and use it to populate the
  /// given existing damage variable. This callback is dependent on either
  /// fbo_callback or fbo_with_frame_info_callback being defined as they are
  /// responsible for providing populate_existing_damage with the FBO's
  /// ID. Not specifying populate_existing_damage will result in full
  /// repaint (i.e. rendering all the pixels on the screen at every frame).
  FlutterFrameBufferWithDamageCallback populate_existing_damage;
} FlutterOpenGLRendererConfig;

/// Alias for id<MTLDevice>.
typedef const void* FlutterMetalDeviceHandle;

/// Alias for id<MTLCommandQueue>.
typedef const void* FlutterMetalCommandQueueHandle;

/// Alias for id<MTLTexture>.
typedef const void* FlutterMetalTextureHandle;

/// Pixel format for the external texture.
typedef enum {
  kYUVA,
  kRGBA,
} FlutterMetalExternalTexturePixelFormat;

/// YUV color space for the YUV external texture.
typedef enum {
  kBT601FullRange,
  kBT601LimitedRange,
} FlutterMetalExternalTextureYUVColorSpace;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterMetalExternalTexture).
  size_t struct_size;
  /// Height of the texture.
  size_t width;
  /// Height of the texture.
  size_t height;
  /// The pixel format type of the external.
  FlutterMetalExternalTexturePixelFormat pixel_format;
  /// Represents the size of the `textures` array.
  size_t num_textures;
  /// Supported textures are YUVA and RGBA, in case of YUVA we expect 2 texture
  /// handles to be provided by the embedder, Y first and UV next. In case of
  /// RGBA only one should be passed.
  /// These are individually aliases for id<MTLTexture>. These textures are
  /// retained by the engine for the period of the composition. Once these
  /// textures have been unregistered via the
  /// `FlutterEngineUnregisterExternalTexture`, the embedder has to release
  /// these textures.
  FlutterMetalTextureHandle* textures;
  /// The YUV color space of the YUV external texture.
  FlutterMetalExternalTextureYUVColorSpace yuv_color_space;
} FlutterMetalExternalTexture;

/// Callback to provide an external texture for a given texture_id.
/// See: external_texture_frame_callback.
typedef bool (*FlutterMetalTextureFrameCallback)(
    void* /* user data */,
    int64_t /* texture identifier */,
    size_t /* width */,
    size_t /* height */,
    FlutterMetalExternalTexture* /* texture out */);

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterMetalTexture).
  size_t struct_size;
  /// Embedder provided unique identifier to the texture buffer. Given that the
  /// `texture` handle is passed to the engine to render to, the texture buffer
  /// is itself owned by the embedder. This `texture_id` is then also given to
  /// the embedder in the present callback.
  int64_t texture_id;
  /// Handle to the MTLTexture that is owned by the embedder. Engine will render
  /// the frame into this texture.
  ///
  /// A NULL texture is considered invalid.
  FlutterMetalTextureHandle texture;
  /// A baton that is not interpreted by the engine in any way. It will be given
  /// back to the embedder in the destruction callback below. Embedder resources
  /// may be associated with this baton.
  void* user_data;
  /// The callback invoked by the engine when it no longer needs this backing
  /// store.
  VoidCallback destruction_callback;
} FlutterMetalTexture;

/// Callback for when a metal texture is requested.
typedef FlutterMetalTexture (*FlutterMetalTextureCallback)(
    void* /* user data */,
    const FlutterFrameInfo* /* frame info */);

/// Callback for when a metal texture is presented. The texture_id here
/// corresponds to the texture_id provided by the embedder in the
/// `FlutterMetalTextureCallback` callback.
typedef bool (*FlutterMetalPresentCallback)(
    void* /* user data */,
    const FlutterMetalTexture* /* texture */);

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterMetalRendererConfig).
  size_t struct_size;
  /// Alias for id<MTLDevice>.
  FlutterMetalDeviceHandle device;
  /// Alias for id<MTLCommandQueue>.
  FlutterMetalCommandQueueHandle present_command_queue;
  /// The callback that gets invoked when the engine requests the embedder for a
  /// texture to render to.
  ///
  /// Not used if a FlutterCompositor is supplied in FlutterProjectArgs.
  FlutterMetalTextureCallback get_next_drawable_callback;
  /// The callback presented to the embedder to present a fully populated metal
  /// texture to the user.
  ///
  /// Not used if a FlutterCompositor is supplied in FlutterProjectArgs.
  FlutterMetalPresentCallback present_drawable_callback;
  /// When the embedder specifies that a texture has a frame available, the
  /// engine will call this method (on an internal engine managed thread) so
  /// that external texture details can be supplied to the engine for subsequent
  /// composition.
  FlutterMetalTextureFrameCallback external_texture_frame_callback;
} FlutterMetalRendererConfig;

/// Alias for VkInstance.
typedef void* FlutterVulkanInstanceHandle;

/// Alias for VkPhysicalDevice.
typedef void* FlutterVulkanPhysicalDeviceHandle;

/// Alias for VkDevice.
typedef void* FlutterVulkanDeviceHandle;

/// Alias for VkQueue.
typedef void* FlutterVulkanQueueHandle;

/// Alias for VkImage.
typedef uint64_t FlutterVulkanImageHandle;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterVulkanImage).
  size_t struct_size;
  /// Handle to the VkImage that is owned by the embedder. The engine will
  /// bind this image for writing the frame.
  FlutterVulkanImageHandle image;
  /// The VkFormat of the image (for example: VK_FORMAT_R8G8B8A8_UNORM).
  uint32_t format;
} FlutterVulkanImage;

/// Callback to fetch a Vulkan function pointer for a given instance. Normally,
/// this should return the results of vkGetInstanceProcAddr.
typedef void* (*FlutterVulkanInstanceProcAddressCallback)(
    void* /* user data */,
    FlutterVulkanInstanceHandle /* instance */,
    const char* /* name */);

/// Callback for when a VkImage is requested.
typedef FlutterVulkanImage (*FlutterVulkanImageCallback)(
    void* /* user data */,
    const FlutterFrameInfo* /* frame info */);

/// Callback for when a VkImage has been written to and is ready for use by the
/// embedder.
typedef bool (*FlutterVulkanPresentCallback)(
    void* /* user data */,
    const FlutterVulkanImage* /* image */);

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterVulkanRendererConfig).
  size_t struct_size;

  /// The Vulkan API version. This should match the value set in
  /// VkApplicationInfo::apiVersion when the VkInstance was created.
  uint32_t version;
  /// VkInstance handle. Must not be destroyed before `FlutterEngineShutdown` is
  /// called.
  FlutterVulkanInstanceHandle instance;
  /// VkPhysicalDevice handle.
  FlutterVulkanPhysicalDeviceHandle physical_device;
  /// VkDevice handle. Must not be destroyed before `FlutterEngineShutdown` is
  /// called.
  FlutterVulkanDeviceHandle device;
  /// The queue family index of the VkQueue supplied in the next field.
  uint32_t queue_family_index;
  /// VkQueue handle.
  /// The queue should not be used without protection from a mutex to make sure
  /// it is not used simultaneously with other threads. That mutex should match
  /// the one injected via the |get_instance_proc_address_callback|.
  /// There is a proposal to remove the need for the mutex at
  /// https://github.com/flutter/flutter/issues/134573.
  FlutterVulkanQueueHandle queue;
  /// The number of instance extensions available for enumerating in the next
  /// field.
  size_t enabled_instance_extension_count;
  /// Array of enabled instance extension names. This should match the names
  /// passed to `VkInstanceCreateInfo.ppEnabledExtensionNames` when the instance
  /// was created, but any subset of enabled instance extensions may be
  /// specified.
  /// This field is optional; `nullptr` may be specified.
  /// This memory is only accessed during the call to FlutterEngineInitialize.
  const char** enabled_instance_extensions;
  /// The number of device extensions available for enumerating in the next
  /// field.
  size_t enabled_device_extension_count;
  /// Array of enabled logical device extension names. This should match the
  /// names passed to `VkDeviceCreateInfo.ppEnabledExtensionNames` when the
  /// logical device was created, but any subset of enabled logical device
  /// extensions may be specified.
  /// This field is optional; `nullptr` may be specified.
  /// This memory is only accessed during the call to FlutterEngineInitialize.
  /// For example: VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME
  const char** enabled_device_extensions;
  /// The callback invoked when resolving Vulkan function pointers.
  /// At a bare minimum this should be used to swap out any calls that operate
  /// on vkQueue's for threadsafe variants that obtain locks for their duration.
  /// The functions to swap out are "vkQueueSubmit" and "vkQueueWaitIdle".  An
  /// example of how to do that can be found in the test
  /// "EmbedderTest.CanSwapOutVulkanCalls" unit-test in
  /// //shell/platform/embedder/tests/embedder_vk_unittests.cc.
  FlutterVulkanInstanceProcAddressCallback get_instance_proc_address_callback;
  /// The callback invoked when the engine requests a VkImage from the embedder
  /// for rendering the next frame.
  /// Not used if a FlutterCompositor is supplied in FlutterProjectArgs.
  FlutterVulkanImageCallback get_next_image_callback;
  /// The callback invoked when a VkImage has been written to and is ready for
  /// use by the embedder. Prior to calling this callback, the engine performs
  /// a host sync, and so the VkImage can be used in a pipeline by the embedder
  /// without any additional synchronization.
  /// Not used if a FlutterCompositor is supplied in FlutterProjectArgs.
  FlutterVulkanPresentCallback present_image_callback;

} FlutterVulkanRendererConfig;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterSoftwareRendererConfig).
  size_t struct_size;
  /// The callback presented to the embedder to present a fully populated buffer
  /// to the user. The pixel format of the buffer is the native 32-bit RGBA
  /// format. The buffer is owned by the Flutter engine and must be copied in
  /// this callback if needed.
  SoftwareSurfacePresentCallback surface_present_callback;
} FlutterSoftwareRendererConfig;

typedef struct {
  FlutterRendererType type;
  union {
    FlutterOpenGLRendererConfig open_gl;
    FlutterSoftwareRendererConfig software;
    FlutterMetalRendererConfig metal;
    FlutterVulkanRendererConfig vulkan;
  };
} FlutterRendererConfig;

/// Display refers to a graphics hardware system consisting of a framebuffer,
/// typically a monitor or a screen. This ID is unique per display and is
/// stable until the Flutter application restarts.
typedef uint64_t FlutterEngineDisplayId;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterWindowMetricsEvent).
  size_t struct_size;
  /// Physical width of the window.
  size_t width;
  /// Physical height of the window.
  size_t height;
  /// Scale factor for the physical screen.
  double pixel_ratio;
  /// Horizontal physical location of the left side of the window on the screen.
  size_t left;
  /// Vertical physical location of the top of the window on the screen.
  size_t top;
  /// Top inset of window.
  double physical_view_inset_top;
  /// Right inset of window.
  double physical_view_inset_right;
  /// Bottom inset of window.
  double physical_view_inset_bottom;
  /// Left inset of window.
  double physical_view_inset_left;
  /// The identifier of the display the view is rendering on.
  FlutterEngineDisplayId display_id;
  /// The view that this event is describing.
  int64_t view_id;
} FlutterWindowMetricsEvent;

typedef struct {
  /// The size of this struct.
  /// Must be sizeof(FlutterAddViewResult).
  size_t struct_size;

  /// True if the add view operation succeeded.
  bool added;

  /// The |FlutterAddViewInfo.user_data|.
  void* user_data;
} FlutterAddViewResult;

/// The callback invoked by the engine when the engine has attempted to add a
/// view.
///
/// The |FlutterAddViewResult| is only guaranteed to be valid during this
/// callback.
typedef void (*FlutterAddViewCallback)(const FlutterAddViewResult* result);

typedef struct {
  /// The size of this struct.
  /// Must be sizeof(FlutterAddViewInfo).
  size_t struct_size;

  /// The identifier for the view to add. This must be unique.
  FlutterViewId view_id;

  /// The view's properties.
  ///
  /// The metric's |view_id| must match this struct's |view_id|.
  const FlutterWindowMetricsEvent* view_metrics;

  /// A baton that is not interpreted by the engine in any way. It will be given
  /// back to the embedder in |add_view_callback|. Embedder resources may be
  /// associated with this baton.
  void* user_data;

  /// Called once the engine has attempted to add the view. This callback is
  /// required.
  ///
  /// The embedder/app must not use the view until the callback is invoked with
  /// an `added` value of `true`.
  ///
  /// This callback is invoked on an internal engine managed thread. Embedders
  /// must re-thread if necessary.
  FlutterAddViewCallback add_view_callback;
} FlutterAddViewInfo;

typedef struct {
  /// The size of this struct.
  /// Must be sizeof(FlutterRemoveViewResult).
  size_t struct_size;

  /// True if the remove view operation succeeded.
  bool removed;

  /// The |FlutterRemoveViewInfo.user_data|.
  void* user_data;
} FlutterRemoveViewResult;

/// The callback invoked by the engine when the engine has attempted to remove
/// a view.
///
/// The |FlutterRemoveViewResult| is only guaranteed to be valid during this
/// callback.
typedef void (*FlutterRemoveViewCallback)(
    const FlutterRemoveViewResult* /* result */);

typedef struct {
  /// The size of this struct.
  /// Must be sizeof(FlutterRemoveViewInfo).
  size_t struct_size;

  /// The identifier for the view to remove.
  ///
  /// The implicit view cannot be removed if it is enabled.
  FlutterViewId view_id;

  /// A baton that is not interpreted by the engine in any way.
  /// It will be given back to the embedder in |remove_view_callback|.
  /// Embedder resources may be associated with this baton.
  void* user_data;

  /// Called once the engine has attempted to remove the view.
  /// This callback is required.
  ///
  /// The embedder must not destroy the underlying surface until the callback is
  /// invoked with a `removed` value of `true`.
  ///
  /// This callback is invoked on an internal engine managed thread.
  /// Embedders must re-thread if necessary.
  ///
  /// The |result| argument will be deallocated when the callback returns.
  FlutterRemoveViewCallback remove_view_callback;
} FlutterRemoveViewInfo;

/// The phase of the pointer event.
typedef enum {
  kCancel,
  /// The pointer, which must have been down (see kDown), is now up.
  ///
  /// For touch, this means that the pointer is no longer in contact with the
  /// screen. For a mouse, it means the last button was released. Note that if
  /// any other buttons are still pressed when one button is released, that
  /// should be sent as a kMove rather than a kUp.
  kUp,
  /// The pointer, which must have been up, is now down.
  ///
  /// For touch, this means that the pointer has come into contact with the
  /// screen. For a mouse, it means a button is now pressed. Note that if any
  /// other buttons are already pressed when a new button is pressed, that
  /// should be sent as a kMove rather than a kDown.
  kDown,
  /// The pointer moved while down.
  ///
  /// This is also used for changes in button state that don't cause a kDown or
  /// kUp, such as releasing one of two pressed buttons.
  kMove,
  /// The pointer is now sending input to Flutter. For instance, a mouse has
  /// entered the area where the Flutter content is displayed.
  ///
  /// A pointer should always be added before sending any other events.
  kAdd,
  /// The pointer is no longer sending input to Flutter. For instance, a mouse
  /// has left the area where the Flutter content is displayed.
  ///
  /// A removed pointer should no longer send events until sending a new kAdd.
  kRemove,
  /// The pointer moved while up.
  kHover,
  /// A pan/zoom started on this pointer.
  kPanZoomStart,
  /// The pan/zoom updated.
  kPanZoomUpdate,
  /// The pan/zoom ended.
  kPanZoomEnd,
} FlutterPointerPhase;

/// The device type that created a pointer event.
typedef enum {
  kFlutterPointerDeviceKindMouse = 1,
  kFlutterPointerDeviceKindTouch,
  kFlutterPointerDeviceKindStylus,
  kFlutterPointerDeviceKindTrackpad,
} FlutterPointerDeviceKind;

/// Flags for the `buttons` field of `FlutterPointerEvent` when `device_kind`
/// is `kFlutterPointerDeviceKindMouse`.
typedef enum {
  kFlutterPointerButtonMousePrimary = 1 << 0,
  kFlutterPointerButtonMouseSecondary = 1 << 1,
  kFlutterPointerButtonMouseMiddle = 1 << 2,
  kFlutterPointerButtonMouseBack = 1 << 3,
  kFlutterPointerButtonMouseForward = 1 << 4,
  /// If a mouse has more than five buttons, send higher bit shifted values
  /// corresponding to the button number: 1 << 5 for the 6th, etc.
} FlutterPointerMouseButtons;

/// The type of a pointer signal.
typedef enum {
  kFlutterPointerSignalKindNone,
  kFlutterPointerSignalKindScroll,
  kFlutterPointerSignalKindScrollInertiaCancel,
  kFlutterPointerSignalKindScale,
} FlutterPointerSignalKind;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterPointerEvent).
  size_t struct_size;
  FlutterPointerPhase phase;
  /// The timestamp at which the pointer event was generated. The timestamp
  /// should be specified in microseconds and the clock should be the same as
  /// that used by `FlutterEngineGetCurrentTime`.
  size_t timestamp;
  /// The x coordinate of the pointer event in physical pixels.
  double x;
  /// The y coordinate of the pointer event in physical pixels.
  double y;
  /// An optional device identifier. If this is not specified, it is assumed
  /// that the embedder has no multi-touch capability.
  int32_t device;
  FlutterPointerSignalKind signal_kind;
  /// The x offset of the scroll in physical pixels.
  double scroll_delta_x;
  /// The y offset of the scroll in physical pixels.
  double scroll_delta_y;
  /// The type of the device generating this event.
  /// Backwards compatibility note: If this is not set, the device will be
  /// treated as a mouse, with the primary button set for `kDown` and `kMove`.
  /// If set explicitly to `kFlutterPointerDeviceKindMouse`, you must set the
  /// correct buttons.
  FlutterPointerDeviceKind device_kind;
  /// The buttons currently pressed, if any.
  int64_t buttons;
  /// The x offset of the pan/zoom in physical pixels.
  double pan_x;
  /// The y offset of the pan/zoom in physical pixels.
  double pan_y;
  /// The scale of the pan/zoom, where 1.0 is the initial scale.
  double scale;
  /// The rotation of the pan/zoom in radians, where 0.0 is the initial angle.
  double rotation;
  /// The identifier of the view that received the pointer event.
  FlutterViewId view_id;
} FlutterPointerEvent;

typedef enum {
  kFlutterKeyEventTypeUp = 1,
  kFlutterKeyEventTypeDown,
  kFlutterKeyEventTypeRepeat,
} FlutterKeyEventType;

typedef enum {
  kFlutterKeyEventDeviceTypeKeyboard = 1,
  kFlutterKeyEventDeviceTypeDirectionalPad,
  kFlutterKeyEventDeviceTypeGamepad,
  kFlutterKeyEventDeviceTypeJoystick,
  kFlutterKeyEventDeviceTypeHdmi,
} FlutterKeyEventDeviceType;

/// A structure to represent a key event.
///
/// Sending `FlutterKeyEvent` via `FlutterEngineSendKeyEvent` results in a
/// corresponding `FlutterKeyEvent` to be dispatched in the framework. It is
/// embedder's responsibility to ensure the regularity of sent events, since the
/// framework only performs simple one-to-one mapping. The events must conform
/// the following rules:
///
///  * Each key press sequence shall consist of one key down event (`kind` being
///    `kFlutterKeyEventTypeDown`), zero or more repeat events, and one key up
///    event, representing a physical key button being pressed, held, and
///    released.
///  * All events throughout a key press sequence shall have the same `physical`
///    and `logical`. Having different `character`s is allowed.
///
/// A `FlutterKeyEvent` with `physical` 0 and `logical` 0 is an empty event.
/// This is the only case either `physical` or `logical` can be 0. An empty
/// event must be sent if a key message should be converted to no
/// `FlutterKeyEvent`s, for example, when a key down message is received for a
/// key that has already been pressed according to the record. This is to ensure
/// some `FlutterKeyEvent` arrives at the framework before raw key message.
/// See https://github.com/flutter/flutter/issues/87230.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterKeyEvent).
  size_t struct_size;
  /// The timestamp at which the key event was generated. The timestamp should
  /// be specified in microseconds and the clock should be the same as that used
  /// by `FlutterEngineGetCurrentTime`.
  double timestamp;
  /// The event kind.
  FlutterKeyEventType type;
  /// The USB HID code for the physical key of the event.
  ///
  /// For the full definition and list of pre-defined physical keys, see
  /// `PhysicalKeyboardKey` from the framework.
  ///
  /// The only case that `physical` might be 0 is when this is an empty event.
  /// See `FlutterKeyEvent` for introduction.
  uint64_t physical;
  /// The key ID for the logical key of this event.
  ///
  /// For the full definition and a list of pre-defined logical keys, see
  /// `LogicalKeyboardKey` from the framework.
  ///
  /// The only case that `logical` might be 0 is when this is an empty event.
  /// See `FlutterKeyEvent` for introduction.
  uint64_t logical;
  /// Null-terminated character input from the event. Can be null. Ignored for
  /// up events.
  const char* character;
  /// True if this event does not correspond to a native event.
  ///
  /// The embedder is likely to skip events and/or construct new events that do
  /// not correspond to any native events in order to conform the regularity
  /// of events (as documented in `FlutterKeyEvent`). An example is when a key
  /// up is missed due to loss of window focus, on a platform that provides
  /// query to key pressing status, the embedder might realize that the key has
  /// been released at the next key event, and should construct a synthesized up
  /// event immediately before the actual event.
  ///
  /// An event being synthesized means that the `timestamp` might greatly
  /// deviate from the actual time when the event occurs physically.
  bool synthesized;
  /// The source device for the key event.
  FlutterKeyEventDeviceType device_type;
} FlutterKeyEvent;

typedef void (*FlutterKeyEventCallback)(bool /* handled */,
                                        void* /* user_data */);

struct _FlutterPlatformMessageResponseHandle;
typedef struct _FlutterPlatformMessageResponseHandle
    FlutterPlatformMessageResponseHandle;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterPlatformMessage).
  size_t struct_size;
  const char* channel;
  const uint8_t* message;
  size_t message_size;
  /// The response handle on which to invoke
  /// `FlutterEngineSendPlatformMessageResponse` when the response is ready.
  /// `FlutterEngineSendPlatformMessageResponse` must be called for all messages
  /// received by the embedder. Failure to call
  /// `FlutterEngineSendPlatformMessageResponse` will cause a memory leak. It is
  /// not safe to send multiple responses on a single response object.
  const FlutterPlatformMessageResponseHandle* response_handle;
} FlutterPlatformMessage;

typedef void (*FlutterPlatformMessageCallback)(
    const FlutterPlatformMessage* /* message*/,
    void* /* user data */);

typedef void (*FlutterDataCallback)(const uint8_t* /* data */,
                                    size_t /* size */,
                                    void* /* user data */);

/// The identifier of the platform view. This identifier is specified by the
/// application when a platform view is added to the scene via the
/// `SceneBuilder.addPlatformView` call.
typedef int64_t FlutterPlatformViewIdentifier;

/// `FlutterSemanticsNode` ID used as a sentinel to signal the end of a batch of
/// semantics node updates. This is unused if using
/// `FlutterUpdateSemanticsCallback2`.
FLUTTER_EXPORT
extern const int32_t kFlutterSemanticsNodeIdBatchEnd;

// The enumeration of possible string attributes that affect how assistive
// technologies announce a string.
//
// See dart:ui's implementers of the StringAttribute abstract class.
typedef enum {
  // Indicates the string should be announced character by character.
  kSpellOut,
  // Indicates the string should be announced using the specified locale.
  kLocale,
} FlutterStringAttributeType;

// Indicates the assistive technology should announce out the string character
// by character.
//
// See dart:ui's SpellOutStringAttribute.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterSpellOutStringAttribute).
  size_t struct_size;
} FlutterSpellOutStringAttribute;

// Indicates the assistive technology should announce the string using the
// specified locale.
//
// See dart:ui's LocaleStringAttribute.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterLocaleStringAttribute).
  size_t struct_size;
  // The locale of this attribute.
  const char* locale;
} FlutterLocaleStringAttribute;

// Indicates how the assistive technology should treat the string.
//
// See dart:ui's StringAttribute.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterStringAttribute).
  size_t struct_size;
  // The position this attribute starts.
  size_t start;
  // The next position after the attribute ends.
  size_t end;
  /// The type of the attribute described by the subsequent union.
  FlutterStringAttributeType type;
  union {
    // Indicates the string should be announced character by character.
    const FlutterSpellOutStringAttribute* spell_out;
    // Indicates the string should be announced using the specified locale.
    const FlutterLocaleStringAttribute* locale;
  };
} FlutterStringAttribute;

/// A node that represents some semantic data.
///
/// The semantics tree is maintained during the semantics phase of the pipeline
/// (i.e., during PipelineOwner.flushSemantics), which happens after
/// compositing. Updates are then pushed to embedders via the registered
/// `FlutterUpdateSemanticsCallback`.
///
/// @deprecated     Use `FlutterSemanticsNode2` instead. In order to preserve
///                 ABI compatibility for existing users, no new fields will be
///                 added to this struct. New fields will continue to be added
///                 to `FlutterSemanticsNode2`.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterSemanticsNode).
  size_t struct_size;
  /// The unique identifier for this node.
  int32_t id;
  /// The set of semantics flags associated with this node.
  FlutterSemanticsFlag flags;
  /// The set of semantics actions applicable to this node.
  FlutterSemanticsAction actions;
  /// The position at which the text selection originates.
  int32_t text_selection_base;
  /// The position at which the text selection terminates.
  int32_t text_selection_extent;
  /// The total number of scrollable children that contribute to semantics.
  int32_t scroll_child_count;
  /// The index of the first visible semantic child of a scroll node.
  int32_t scroll_index;
  /// The current scrolling position in logical pixels if the node is
  /// scrollable.
  double scroll_position;
  /// The maximum in-range value for `scrollPosition` if the node is scrollable.
  double scroll_extent_max;
  /// The minimum in-range value for `scrollPosition` if the node is scrollable.
  double scroll_extent_min;
  /// The elevation along the z-axis at which the rect of this semantics node is
  /// located above its parent.
  double elevation;
  /// Describes how much space the semantics node takes up along the z-axis.
  double thickness;
  /// A textual description of the node.
  const char* label;
  /// A brief description of the result of performing an action on the node.
  const char* hint;
  /// A textual description of the current value of the node.
  const char* value;
  /// A value that `value` will have after a kFlutterSemanticsActionIncrease`
  /// action has been performed.
  const char* increased_value;
  /// A value that `value` will have after a kFlutterSemanticsActionDecrease`
  /// action has been performed.
  const char* decreased_value;
  /// The reading direction for `label`, `value`, `hint`, `increasedValue`,
  /// `decreasedValue`, and `tooltip`.
  FlutterTextDirection text_direction;
  /// The bounding box for this node in its coordinate system.
  FlutterRect rect;
  /// The transform from this node's coordinate system to its parent's
  /// coordinate system.
  FlutterTransformation transform;
  /// The number of children this node has.
  size_t child_count;
  /// Array of child node IDs in traversal order. Has length `child_count`.
  const int32_t* children_in_traversal_order;
  /// Array of child node IDs in hit test order. Has length `child_count`.
  const int32_t* children_in_hit_test_order;
  /// The number of custom accessibility action associated with this node.
  size_t custom_accessibility_actions_count;
  /// Array of `FlutterSemanticsCustomAction` IDs associated with this node.
  /// Has length `custom_accessibility_actions_count`.
  const int32_t* custom_accessibility_actions;
  /// Identifier of the platform view associated with this semantics node, or
  /// -1 if none.
  FlutterPlatformViewIdentifier platform_view_id;
  /// A textual tooltip attached to the node.
  const char* tooltip;
} FlutterSemanticsNode;

/// A node in the Flutter semantics tree.
///
/// The semantics tree is maintained during the semantics phase of the pipeline
/// (i.e., during PipelineOwner.flushSemantics), which happens after
/// compositing. Updates are then pushed to embedders via the registered
/// `FlutterUpdateSemanticsCallback2`.
///
/// @see https://api.flutter.dev/flutter/semantics/SemanticsNode-class.html
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterSemanticsNode).
  size_t struct_size;
  /// The unique identifier for this node.
  int32_t id;
  /// The set of semantics flags associated with this node.
  FlutterSemanticsFlag flags;
  /// The set of semantics actions applicable to this node.
  FlutterSemanticsAction actions;
  /// The position at which the text selection originates.
  int32_t text_selection_base;
  /// The position at which the text selection terminates.
  int32_t text_selection_extent;
  /// The total number of scrollable children that contribute to semantics.
  int32_t scroll_child_count;
  /// The index of the first visible semantic child of a scroll node.
  int32_t scroll_index;
  /// The current scrolling position in logical pixels if the node is
  /// scrollable.
  double scroll_position;
  /// The maximum in-range value for `scrollPosition` if the node is scrollable.
  double scroll_extent_max;
  /// The minimum in-range value for `scrollPosition` if the node is scrollable.
  double scroll_extent_min;
  /// The elevation along the z-axis at which the rect of this semantics node is
  /// located above its parent.
  double elevation;
  /// Describes how much space the semantics node takes up along the z-axis.
  double thickness;
  /// A textual description of the node.
  const char* label;
  /// A brief description of the result of performing an action on the node.
  const char* hint;
  /// A textual description of the current value of the node.
  const char* value;
  /// A value that `value` will have after a kFlutterSemanticsActionIncrease`
  /// action has been performed.
  const char* increased_value;
  /// A value that `value` will have after a kFlutterSemanticsActionDecrease`
  /// action has been performed.
  const char* decreased_value;
  /// The reading direction for `label`, `value`, `hint`, `increasedValue`,
  /// `decreasedValue`, and `tooltip`.
  FlutterTextDirection text_direction;
  /// The bounding box for this node in its coordinate system.
  FlutterRect rect;
  /// The transform from this node's coordinate system to its parent's
  /// coordinate system.
  FlutterTransformation transform;
  /// The number of children this node has.
  size_t child_count;
  /// Array of child node IDs in traversal order. Has length `child_count`.
  const int32_t* children_in_traversal_order;
  /// Array of child node IDs in hit test order. Has length `child_count`.
  const int32_t* children_in_hit_test_order;
  /// The number of custom accessibility action associated with this node.
  size_t custom_accessibility_actions_count;
  /// Array of `FlutterSemanticsCustomAction` IDs associated with this node.
  /// Has length `custom_accessibility_actions_count`.
  const int32_t* custom_accessibility_actions;
  /// Identifier of the platform view associated with this semantics node, or
  /// -1 if none.
  FlutterPlatformViewIdentifier platform_view_id;
  /// A textual tooltip attached to the node.
  const char* tooltip;
  // The number of string attributes associated with the `label`.
  size_t label_attribute_count;
  // Array of string attributes associated with the `label`.
  // Has length `label_attribute_count`.
  const FlutterStringAttribute** label_attributes;
  // The number of string attributes associated with the `hint`.
  size_t hint_attribute_count;
  // Array of string attributes associated with the `hint`.
  // Has length `hint_attribute_count`.
  const FlutterStringAttribute** hint_attributes;
  // The number of string attributes associated with the `value`.
  size_t value_attribute_count;
  // Array of string attributes associated with the `value`.
  // Has length `value_attribute_count`.
  const FlutterStringAttribute** value_attributes;
  // The number of string attributes associated with the `increased_value`.
  size_t increased_value_attribute_count;
  // Array of string attributes associated with the `increased_value`.
  // Has length `increased_value_attribute_count`.
  const FlutterStringAttribute** increased_value_attributes;
  // The number of string attributes associated with the `decreased_value`.
  size_t decreased_value_attribute_count;
  // Array of string attributes associated with the `decreased_value`.
  // Has length `decreased_value_attribute_count`.
  const FlutterStringAttribute** decreased_value_attributes;
} FlutterSemanticsNode2;

/// `FlutterSemanticsCustomAction` ID used as a sentinel to signal the end of a
/// batch of semantics custom action updates. This is unused if using
/// `FlutterUpdateSemanticsCallback2`.
FLUTTER_EXPORT
extern const int32_t kFlutterSemanticsCustomActionIdBatchEnd;

/// A custom semantics action, or action override.
///
/// Custom actions can be registered by applications in order to provide
/// semantic actions other than the standard actions available through the
/// `FlutterSemanticsAction` enum.
///
/// Action overrides are custom actions that the application developer requests
/// to be used in place of the standard actions in the `FlutterSemanticsAction`
/// enum.
///
/// @deprecated     Use `FlutterSemanticsCustomAction2` instead. In order to
///                 preserve ABI compatility for existing users, no new fields
///                 will be added to this struct. New fields will continue to
///                 be added to `FlutterSemanticsCustomAction2`.
typedef struct {
  /// The size of the struct. Must be sizeof(FlutterSemanticsCustomAction).
  size_t struct_size;
  /// The unique custom action or action override ID.
  int32_t id;
  /// For overridden standard actions, corresponds to the
  /// `FlutterSemanticsAction` to override.
  FlutterSemanticsAction override_action;
  /// The user-readable name of this custom semantics action.
  const char* label;
  /// The hint description of this custom semantics action.
  const char* hint;
} FlutterSemanticsCustomAction;

/// A custom semantics action, or action override.
///
/// Custom actions can be registered by applications in order to provide
/// semantic actions other than the standard actions available through the
/// `FlutterSemanticsAction` enum.
///
/// Action overrides are custom actions that the application developer requests
/// to be used in place of the standard actions in the `FlutterSemanticsAction`
/// enum.
///
/// @see
/// https://api.flutter.dev/flutter/semantics/CustomSemanticsAction-class.html
typedef struct {
  /// The size of the struct. Must be sizeof(FlutterSemanticsCustomAction).
  size_t struct_size;
  /// The unique custom action or action override ID.
  int32_t id;
  /// For overridden standard actions, corresponds to the
  /// `FlutterSemanticsAction` to override.
  FlutterSemanticsAction override_action;
  /// The user-readable name of this custom semantics action.
  const char* label;
  /// The hint description of this custom semantics action.
  const char* hint;
} FlutterSemanticsCustomAction2;

/// A batch of updates to semantics nodes and custom actions.
///
/// @deprecated     Use `FlutterSemanticsUpdate2` instead. Adding members
///                 to `FlutterSemanticsNode` or `FlutterSemanticsCustomAction`
///                 breaks the ABI of this struct.
typedef struct {
  /// The size of the struct. Must be sizeof(FlutterSemanticsUpdate).
  size_t struct_size;
  /// The number of semantics node updates.
  size_t nodes_count;
  // Array of semantics nodes. Has length `nodes_count`.
  FlutterSemanticsNode* nodes;
  /// The number of semantics custom action updates.
  size_t custom_actions_count;
  /// Array of semantics custom actions. Has length `custom_actions_count`.
  FlutterSemanticsCustomAction* custom_actions;
} FlutterSemanticsUpdate;

/// A batch of updates to semantics nodes and custom actions.
typedef struct {
  /// The size of the struct. Must be sizeof(FlutterSemanticsUpdate2).
  size_t struct_size;
  /// The number of semantics node updates.
  size_t node_count;
  // Array of semantics node pointers. Has length `node_count`.
  FlutterSemanticsNode2** nodes;
  /// The number of semantics custom action updates.
  size_t custom_action_count;
  /// Array of semantics custom action pointers. Has length
  /// `custom_action_count`.
  FlutterSemanticsCustomAction2** custom_actions;
} FlutterSemanticsUpdate2;

typedef void (*FlutterUpdateSemanticsNodeCallback)(
    const FlutterSemanticsNode* /* semantics node */,
    void* /* user data */);

typedef void (*FlutterUpdateSemanticsCustomActionCallback)(
    const FlutterSemanticsCustomAction* /* semantics custom action */,
    void* /* user data */);

typedef void (*FlutterUpdateSemanticsCallback)(
    const FlutterSemanticsUpdate* /* semantics update */,
    void* /* user data*/);

typedef void (*FlutterUpdateSemanticsCallback2)(
    const FlutterSemanticsUpdate2* /* semantics update */,
    void* /* user data*/);

/// An update to whether a message channel has a listener set or not.
typedef struct {
  /// The size of the struct. Must be sizeof(FlutterChannelUpdate).
  size_t struct_size;
  /// The name of the channel.
  const char* channel;
  /// True if a listener has been set, false if one has been cleared.
  bool listening;
} FlutterChannelUpdate;

typedef void (*FlutterChannelUpdateCallback)(
    const FlutterChannelUpdate* /* channel update */,
    void* /* user data */);

typedef struct _FlutterTaskRunner* FlutterTaskRunner;

typedef struct {
  FlutterTaskRunner runner;
  uint64_t task;
} FlutterTask;

typedef void (*FlutterTaskRunnerPostTaskCallback)(
    FlutterTask /* task */,
    uint64_t /* target time nanos */,
    void* /* user data */);

/// An interface used by the Flutter engine to execute tasks at the target time
/// on a specified thread. There should be a 1-1 relationship between a thread
/// and a task runner. It is undefined behavior to run a task on a thread that
/// is not associated with its task runner.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterTaskRunnerDescription).
  size_t struct_size;
  void* user_data;
  /// May be called from any thread. Should return true if tasks posted on the
  /// calling thread will be run on that same thread.
  ///
  /// @attention     This field is required.
  BoolCallback runs_task_on_current_thread_callback;
  /// May be called from any thread. The given task should be executed by the
  /// embedder on the thread associated with that task runner by calling
  /// `FlutterEngineRunTask` at the given target time. The system monotonic
  /// clock should be used for the target time. The target time is the absolute
  /// time from epoch (NOT a delta) at which the task must be returned back to
  /// the engine on the correct thread. If the embedder needs to calculate a
  /// delta, `FlutterEngineGetCurrentTime` may be called and the difference used
  /// as the delta.
  ///
  /// @attention     This field is required.
  FlutterTaskRunnerPostTaskCallback post_task_callback;
  /// A unique identifier for the task runner. If multiple task runners service
  /// tasks on the same thread, their identifiers must match.
  size_t identifier;
  /// The callback invoked when the task runner is destroyed.
  VoidCallback destruction_callback;
} FlutterTaskRunnerDescription;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterCustomTaskRunners).
  size_t struct_size;
  /// Specify the task runner for the thread on which the `FlutterEngineRun`
  /// call is made. The same task runner description can be specified for both
  /// the render and platform task runners. This makes the Flutter engine use
  /// the same thread for both task runners.
  const FlutterTaskRunnerDescription* platform_task_runner;
  /// Specify the task runner for the thread on which the render tasks will be
  /// run. The same task runner description can be specified for both the render
  /// and platform task runners. This makes the Flutter engine use the same
  /// thread for both task runners.
  const FlutterTaskRunnerDescription* render_task_runner;
  /// Specify a callback that is used to set the thread priority for embedder
  /// task runners.
  void (*thread_priority_setter)(FlutterThreadPriority);
} FlutterCustomTaskRunners;

typedef struct {
  /// The type of the OpenGL backing store. Currently, it can either be a
  /// texture or a framebuffer.
  FlutterOpenGLTargetType type;
  union {
    /// A texture for Flutter to render into.
    FlutterOpenGLTexture texture;
    /// A framebuffer for Flutter to render into. The embedder must ensure that
    /// the framebuffer is complete.
    FlutterOpenGLFramebuffer framebuffer;
    /// A surface for Flutter to render into. Basically a wrapper around
    /// a closure that'll be called when the surface should be made current.
    FlutterOpenGLSurface surface;
  };
} FlutterOpenGLBackingStore;

typedef struct {
  /// A pointer to the raw bytes of the allocation described by this software
  /// backing store.
  const void* allocation;
  /// The number of bytes in a single row of the allocation.
  size_t row_bytes;
  /// The number of rows in the allocation.
  size_t height;
  /// A baton that is not interpreted by the engine in any way. It will be given
  /// back to the embedder in the destruction callback below. Embedder resources
  /// may be associated with this baton.
  void* user_data;
  /// The callback invoked by the engine when it no longer needs this backing
  /// store.
  VoidCallback destruction_callback;
} FlutterSoftwareBackingStore;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterSoftwareBackingStore2).
  size_t struct_size;
  /// A pointer to the raw bytes of the allocation described by this software
  /// backing store.
  const void* allocation;
  /// The number of bytes in a single row of the allocation.
  size_t row_bytes;
  /// The number of rows in the allocation.
  size_t height;
  /// A baton that is not interpreted by the engine in any way. It will be given
  /// back to the embedder in the destruction callback below. Embedder resources
  /// may be associated with this baton.
  void* user_data;
  /// The callback invoked by the engine when it no longer needs this backing
  /// store.
  VoidCallback destruction_callback;
  /// The pixel format that the engine should use to render into the allocation.
  ///
  /// On Linux, kFlutterSoftwarePixelFormatBGRA8888 is most commonly used.
  FlutterSoftwarePixelFormat pixel_format;
} FlutterSoftwareBackingStore2;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterMetalBackingStore).
  size_t struct_size;
  union {
    // A Metal texture for Flutter to render into. Ownership is not transferred
    // to Flutter; the texture is CFRetained on successfully being passed in and
    // CFReleased when no longer used.
    FlutterMetalTexture texture;
  };
} FlutterMetalBackingStore;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterVulkanBackingStore).
  size_t struct_size;
  /// The image that the layer will be rendered to. This image must already be
  /// available for the engine to bind for writing when it's given to the engine
  /// via the backing store creation callback. The engine will perform a host
  /// sync for all layers prior to calling the compositor present callback, and
  /// so the written layer images can be freely bound by the embedder without
  /// any additional synchronization.
  const FlutterVulkanImage* image;
  /// A baton that is not interpreted by the engine in any way. It will be given
  /// back to the embedder in the destruction callback below. Embedder resources
  /// may be associated with this baton.
  void* user_data;
  /// The callback invoked by the engine when it no longer needs this backing
  /// store.
  VoidCallback destruction_callback;
} FlutterVulkanBackingStore;

typedef enum {
  /// Indicates that the Flutter application requested that an opacity be
  /// applied to the platform view.
  kFlutterPlatformViewMutationTypeOpacity,
  /// Indicates that the Flutter application requested that the platform view be
  /// clipped using a rectangle.
  kFlutterPlatformViewMutationTypeClipRect,
  /// Indicates that the Flutter application requested that the platform view be
  /// clipped using a rounded rectangle.
  kFlutterPlatformViewMutationTypeClipRoundedRect,
  /// Indicates that the Flutter application requested that the platform view be
  /// transformed before composition.
  kFlutterPlatformViewMutationTypeTransformation,
} FlutterPlatformViewMutationType;

typedef struct {
  /// The type of the mutation described by the subsequent union.
  FlutterPlatformViewMutationType type;
  union {
    double opacity;
    FlutterRect clip_rect;
    FlutterRoundedRect clip_rounded_rect;
    FlutterTransformation transformation;
  };
} FlutterPlatformViewMutation;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterPlatformView).
  size_t struct_size;
  /// The identifier of this platform view. This identifier is specified by the
  /// application when a platform view is added to the scene via the
  /// `SceneBuilder.addPlatformView` call.
  FlutterPlatformViewIdentifier identifier;
  /// The number of mutations to be applied to the platform view by the embedder
  /// before on-screen composition.
  size_t mutations_count;
  /// The mutations to be applied by this platform view before it is composited
  /// on-screen. The Flutter application may transform the platform view but
  /// these transformations cannot be affected by the Flutter compositor because
  /// it does not render platform views. Since the embedder is responsible for
  /// composition of these views, it is also the embedder's responsibility to
  /// affect the appropriate transformation.
  ///
  /// The mutations must be applied in order. The mutations done in the
  /// collection don't take into account the device pixel ratio or the root
  /// surface transformation. If these exist, the first mutation in the list
  /// will be a transformation mutation to make sure subsequent mutations are in
  /// the correct coordinate space.
  const FlutterPlatformViewMutation** mutations;
} FlutterPlatformView;

typedef enum {
  /// Specifies an OpenGL backing store. Can either be an OpenGL texture or
  /// framebuffer.
  kFlutterBackingStoreTypeOpenGL,
  /// Specified an software allocation for Flutter to render into using the CPU.
  kFlutterBackingStoreTypeSoftware,
  /// Specifies a Metal backing store. This is backed by a Metal texture.
  kFlutterBackingStoreTypeMetal,
  /// Specifies a Vulkan backing store. This is backed by a Vulkan VkImage.
  kFlutterBackingStoreTypeVulkan,
  /// Specifies a allocation that the engine should render into using
  /// software rendering.
  kFlutterBackingStoreTypeSoftware2,
} FlutterBackingStoreType;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterBackingStore).
  size_t struct_size;
  /// A baton that is not interpreted by the engine in any way. The embedder may
  /// use this to associate resources that are tied to the lifecycle of the
  /// `FlutterBackingStore`.
  void* user_data;
  /// Specifies the type of backing store.
  FlutterBackingStoreType type;
  /// Indicates if this backing store was updated since the last time it was
  /// associated with a presented layer.
  bool did_update;
  union {
    /// The description of the OpenGL backing store.
    FlutterOpenGLBackingStore open_gl;
    /// The description of the software backing store.
    FlutterSoftwareBackingStore software;
    /// The description of the software backing store.
    FlutterSoftwareBackingStore2 software2;
    // The description of the Metal backing store.
    FlutterMetalBackingStore metal;
    // The description of the Vulkan backing store.
    FlutterVulkanBackingStore vulkan;
  };
} FlutterBackingStore;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterBackingStoreConfig).
  size_t struct_size;
  /// The size of the render target the engine expects to render into.
  FlutterSize size;
  /// The identifier for the view that the engine will use this backing store to
  /// render into.
  FlutterViewId view_id;
} FlutterBackingStoreConfig;

typedef enum {
  /// Indicates that the contents of this layer are rendered by Flutter into a
  /// backing store.
  kFlutterLayerContentTypeBackingStore,
  /// Indicates that the contents of this layer are determined by the embedder.
  kFlutterLayerContentTypePlatformView,
} FlutterLayerContentType;

/// A region represented by a collection of non-overlapping rectangles.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterRegion).
  size_t struct_size;
  /// Number of rectangles in the region.
  size_t rects_count;
  /// The rectangles that make up the region.
  FlutterRect* rects;
} FlutterRegion;

/// Contains additional information about the backing store provided
/// during presentation to the embedder.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterBackingStorePresentInfo).
  size_t struct_size;

  /// The area of the backing store that contains Flutter contents. Pixels
  /// outside of this area are transparent and the embedder may choose not
  /// to render them. Coordinates are in physical pixels.
  FlutterRegion* paint_region;
} FlutterBackingStorePresentInfo;

typedef struct {
  /// This size of this struct. Must be sizeof(FlutterLayer).
  size_t struct_size;
  /// Each layer displays contents in one way or another. The type indicates
  /// whether those contents are specified by Flutter or the embedder.
  FlutterLayerContentType type;
  union {
    /// Indicates that the contents of this layer are rendered by Flutter into a
    /// backing store.
    const FlutterBackingStore* backing_store;
    /// Indicates that the contents of this layer are determined by the
    /// embedder.
    const FlutterPlatformView* platform_view;
  };
  /// The offset of this layer (in physical pixels) relative to the top left of
  /// the root surface used by the engine.
  FlutterPoint offset;
  /// The size of the layer (in physical pixels).
  FlutterSize size;

  /// Extra information for the backing store that the embedder may
  /// use during presentation.
  FlutterBackingStorePresentInfo* backing_store_present_info;

  // Time in nanoseconds at which this frame is scheduled to be presented. 0 if
  // not known. See FlutterEngineGetCurrentTime().
  uint64_t presentation_time;
} FlutterLayer;

typedef struct {
  /// The size of this struct.
  /// Must be sizeof(FlutterPresentViewInfo).
  size_t struct_size;

  /// The identifier of the target view.
  FlutterViewId view_id;

  /// The layers that should be composited onto the view.
  const FlutterLayer** layers;

  /// The count of layers.
  size_t layers_count;

  /// The |FlutterCompositor.user_data|.
  void* user_data;
} FlutterPresentViewInfo;

typedef bool (*FlutterBackingStoreCreateCallback)(
    const FlutterBackingStoreConfig* config,
    FlutterBackingStore* backing_store_out,
    void* user_data);

typedef bool (*FlutterBackingStoreCollectCallback)(
    const FlutterBackingStore* renderer,
    void* user_data);

typedef bool (*FlutterLayersPresentCallback)(const FlutterLayer** layers,
                                             size_t layers_count,
                                             void* user_data);

/// The callback invoked when the embedder should present to a view.
///
/// The |FlutterPresentViewInfo| will be deallocated once the callback returns.
typedef bool (*FlutterPresentViewCallback)(
    const FlutterPresentViewInfo* /* present info */);

typedef struct {
  /// This size of this struct. Must be sizeof(FlutterCompositor).
  size_t struct_size;
  /// A baton that in not interpreted by the engine in any way. If it passed
  /// back to the embedder in `FlutterCompositor.create_backing_store_callback`,
  /// `FlutterCompositor.collect_backing_store_callback`,
  /// `FlutterCompositor.present_layers_callback`, and
  /// `FlutterCompositor.present_view_callback`.
  void* user_data;
  /// A callback invoked by the engine to obtain a backing store for a specific
  /// `FlutterLayer`.
  ///
  /// On ABI stability: Callers must take care to restrict access within
  /// `FlutterBackingStore::struct_size` when specifying a new backing store to
  /// the engine. This only matters if the embedder expects to be used with
  /// engines older than the version whose headers it used during compilation.
  ///
  /// The callback should return true if the operation was successful.
  FlutterBackingStoreCreateCallback create_backing_store_callback;
  /// A callback invoked by the engine to release the backing store. The
  /// embedder may collect any resources associated with the backing store.
  ///
  /// The callback should return true if the operation was successful.
  FlutterBackingStoreCollectCallback collect_backing_store_callback;
  /// Callback invoked by the engine to composite the contents of each layer
  /// onto the implicit view.
  ///
  /// DEPRECATED: Use `present_view_callback` to support multiple views.
  /// If this callback is provided, `FlutterEngineAddView` and
  /// `FlutterEngineRemoveView` should not be used.
  ///
  /// Only one of `present_layers_callback` and `present_view_callback` may be
  /// provided. Providing both is an error and engine initialization will
  /// terminate.
  ///
  /// The callback should return true if the operation was successful.
  FlutterLayersPresentCallback present_layers_callback;
  /// Avoid caching backing stores provided by this compositor.
  ///
  /// The engine has an internal backing store cache. Instead of
  /// creating & destroying backing stores for every frame, created
  /// backing stores are automatically reused for subsequent frames.
  ///
  /// If you wish to change this behavior and destroy backing stores after
  /// they've been used once, and create new backing stores for every frame,
  /// you can set this bool to true.
  bool avoid_backing_store_cache;
  /// Callback invoked by the engine to composite the contents of each layer
  /// onto the specified view.
  ///
  /// Only one of `present_layers_callback` and `present_view_callback` may be
  /// provided. Providing both is an error and engine initialization will
  /// terminate.
  ///
  /// The callback should return true if the operation was successful.
  FlutterPresentViewCallback present_view_callback;
} FlutterCompositor;

typedef struct {
  /// This size of this struct. Must be sizeof(FlutterLocale).
  size_t struct_size;
  /// The language code of the locale. For example, "en". This is a required
  /// field. The string must be null terminated. It may be collected after the
  /// call to `FlutterEngineUpdateLocales`.
  const char* language_code;
  /// The country code of the locale. For example, "US". This is a an optional
  /// field. The string must be null terminated if present. It may be collected
  /// after the call to `FlutterEngineUpdateLocales`. If not present, a
  /// `nullptr` may be specified.
  const char* country_code;
  /// The script code of the locale. This is a an optional field. The string
  /// must be null terminated if present. It may be collected after the call to
  /// `FlutterEngineUpdateLocales`. If not present, a `nullptr` may be
  /// specified.
  const char* script_code;
  /// The variant code of the locale. This is a an optional field. The string
  /// must be null terminated if present. It may be collected after the call to
  /// `FlutterEngineUpdateLocales`. If not present, a `nullptr` may be
  /// specified.
  const char* variant_code;
} FlutterLocale;

/// Callback that returns the system locale.
///
/// Embedders that implement this callback should return the `FlutterLocale`
/// from the `supported_locales` list that most closely matches the
/// user/device's preferred locale.
///
/// This callback does not currently provide the user_data baton.
/// https://github.com/flutter/flutter/issues/79826
typedef const FlutterLocale* (*FlutterComputePlatformResolvedLocaleCallback)(
    const FlutterLocale** /* supported_locales*/,
    size_t /* Number of locales*/);

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterEngineDisplay).
  size_t struct_size;

  FlutterEngineDisplayId display_id;

  /// This is set to true if the embedder only has one display. In cases where
  /// this is set to true, the value of display_id is ignored. In cases where
  /// this is not set to true, it is expected that a valid display_id be
  /// provided.
  bool single_display;

  /// This represents the refresh period in frames per second. This value may be
  /// zero if the device is not running or unavailable or unknown.
  double refresh_rate;

  /// The width of the display, in physical pixels.
  size_t width;

  /// The height of the display, in physical pixels.
  size_t height;

  /// The pixel ratio of the display, which is used to convert physical pixels
  /// to logical pixels.
  double device_pixel_ratio;
} FlutterEngineDisplay;

/// The update type parameter that is passed to
/// `FlutterEngineNotifyDisplayUpdate`.
typedef enum {
  /// `FlutterEngineDisplay`s that were active during start-up. A display is
  /// considered active if:
  ///    1. The frame buffer hardware is connected.
  ///    2. The display is drawable, e.g. it isn't being mirrored from another
  ///    connected display or sleeping.
  kFlutterEngineDisplaysUpdateTypeStartup,
  kFlutterEngineDisplaysUpdateTypeCount,
} FlutterEngineDisplaysUpdateType;

typedef int64_t FlutterEngineDartPort;

typedef enum {
  kFlutterEngineDartObjectTypeNull,
  kFlutterEngineDartObjectTypeBool,
  kFlutterEngineDartObjectTypeInt32,
  kFlutterEngineDartObjectTypeInt64,
  kFlutterEngineDartObjectTypeDouble,
  kFlutterEngineDartObjectTypeString,
  /// The object will be made available to Dart code as an instance of
  /// Uint8List.
  kFlutterEngineDartObjectTypeBuffer,
} FlutterEngineDartObjectType;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterEngineDartBuffer).
  size_t struct_size;
  /// An opaque baton passed back to the embedder when the
  /// buffer_collect_callback is invoked. The engine does not interpret this
  /// field in any way.
  void* user_data;
  /// This is an optional field.
  ///
  /// When specified, the engine will assume that the buffer is owned by the
  /// embedder. When the data is no longer needed by any isolate, this callback
  /// will be made on an internal engine managed thread. The embedder is free to
  /// collect the buffer here. When this field is specified, it is the embedders
  /// responsibility to keep the buffer alive and not modify it till this
  /// callback is invoked by the engine. The user data specified in the callback
  /// is the value of `user_data` field in this struct.
  ///
  /// When NOT specified, the VM creates an internal copy of the buffer. The
  /// caller is free to modify the buffer as necessary or collect it immediately
  /// after the call to `FlutterEnginePostDartObject`.
  ///
  /// @attention      The buffer_collect_callback is will only be invoked by the
  ///                 engine when the `FlutterEnginePostDartObject` method
  ///                 returns kSuccess. In case of non-successful calls to this
  ///                 method, it is the embedders responsibility to collect the
  ///                 buffer.
  VoidCallback buffer_collect_callback;
  /// A pointer to the bytes of the buffer. When the buffer is owned by the
  /// embedder (by specifying the `buffer_collect_callback`), Dart code may
  /// modify that embedder owned buffer. For this reason, it is important that
  /// this buffer not have page protections that restrict writing to this
  /// buffer.
  uint8_t* buffer;
  /// The size of the buffer.
  size_t buffer_size;
} FlutterEngineDartBuffer;

/// This struct specifies the native representation of a Dart object that can be
/// sent via a send port to any isolate in the VM that has the corresponding
/// receive port.
///
/// All fields in this struct are copied out in the call to
/// `FlutterEnginePostDartObject` and the caller is free to reuse or collect
/// this struct after that call.
typedef struct {
  FlutterEngineDartObjectType type;
  union {
    bool bool_value;
    int32_t int32_value;
    int64_t int64_value;
    double double_value;
    /// A null terminated string. This string will be copied by the VM in the
    /// call to `FlutterEnginePostDartObject` and must be collected by the
    /// embedder after that call is made.
    const char* string_value;
    const FlutterEngineDartBuffer* buffer_value;
  };
} FlutterEngineDartObject;

/// This enum allows embedders to determine the type of the engine thread in the
/// FlutterNativeThreadCallback. Based on the thread type, the embedder may be
/// able to tweak the thread priorities for optimum performance.
typedef enum {
  /// The Flutter Engine considers the thread on which the FlutterEngineRun call
  /// is made to be the platform thread. There is only one such thread per
  /// engine instance.
  kFlutterNativeThreadTypePlatform,
  /// This is the thread the Flutter Engine uses to execute rendering commands
  /// based on the selected client rendering API. There is only one such thread
  /// per engine instance.
  kFlutterNativeThreadTypeRender,
  /// This is a dedicated thread on which the root Dart isolate is serviced.
  /// There is only one such thread per engine instance.
  kFlutterNativeThreadTypeUI,
  /// Multiple threads are used by the Flutter engine to perform long running
  /// background tasks.
  kFlutterNativeThreadTypeWorker,
} FlutterNativeThreadType;

/// A callback made by the engine in response to
/// `FlutterEnginePostCallbackOnAllNativeThreads` on all internal thread.
typedef void (*FlutterNativeThreadCallback)(FlutterNativeThreadType type,
                                            void* user_data);

/// AOT data source type.
typedef enum {
  kFlutterEngineAOTDataSourceTypeElfPath
} FlutterEngineAOTDataSourceType;

/// This struct specifies one of the various locations the engine can look for
/// AOT data sources.
typedef struct {
  FlutterEngineAOTDataSourceType type;
  union {
    /// Absolute path to an ELF library file.
    const char* elf_path;
  };
} FlutterEngineAOTDataSource;

// Logging callback for Dart application messages.
//
// The `tag` parameter contains a null-terminated string containing a logging
// tag or component name that can be used to identify system log messages from
// the app. The `message` parameter contains a null-terminated string
// containing the message to be logged. `user_data` is a user data baton passed
// in `FlutterEngineRun`.
typedef void (*FlutterLogMessageCallback)(const char* /* tag */,
                                          const char* /* message */,
                                          void* /* user_data */);

/// An opaque object that describes the AOT data that can be used to launch a
/// FlutterEngine instance in AOT mode.
typedef struct _FlutterEngineAOTData* FlutterEngineAOTData;

typedef struct {
  /// The size of this struct. Must be sizeof(FlutterProjectArgs).
  size_t struct_size;
  /// The path to the Flutter assets directory containing project assets. The
  /// string can be collected after the call to `FlutterEngineRun` returns. The
  /// string must be NULL terminated.
  const char* assets_path;
  /// The path to the Dart file containing the `main` entry point.
  /// The string can be collected after the call to `FlutterEngineRun` returns.
  /// The string must be NULL terminated.
  ///
  /// @deprecated     As of Dart 2, running from Dart source is no longer
  ///                 supported. Dart code should now be compiled to kernel form
  ///                 and will be loaded by from `kernel_blob.bin` in the assets
  ///                 directory. This struct member is retained for ABI
  ///                 stability.
  const char* main_path__unused__;
  /// The path to the `.packages` file for the project. The string can be
  /// collected after the call to `FlutterEngineRun` returns. The string must be
  /// NULL terminated.
  ///
  /// @deprecated    As of Dart 2, running from Dart source is no longer
  ///                supported. Dart code should now be compiled to kernel form
  ///                and will be loaded by from `kernel_blob.bin` in the assets
  ///                directory. This struct member is retained for ABI
  ///                stability.
  const char* packages_path__unused__;
  /// The path to the `icudtl.dat` file for the project. The string can be
  /// collected after the call to `FlutterEngineRun` returns. The string must
  /// be NULL terminated.
  const char* icu_data_path;
  /// The command line argument count used to initialize the project.
  int command_line_argc;
  /// The command line arguments used to initialize the project. The strings can
  /// be collected after the call to `FlutterEngineRun` returns. The strings
  /// must be `NULL` terminated.
  ///
  /// @attention     The first item in the command line (if specified at all) is
  ///                interpreted as the executable name. So if an engine flag
  ///                needs to be passed into the same, it needs to not be the
  ///                very first item in the list.
  ///
  /// The set of engine flags are only meant to control
  /// unstable features in the engine. Deployed applications should not pass any
  /// command line arguments at all as they may affect engine stability at
  /// runtime in the presence of un-sanitized input. The list of currently
  /// recognized engine flags and their descriptions can be retrieved from the
  /// `switches.h` engine source file.
  const char* const* command_line_argv;
  /// The callback invoked by the engine in order to give the embedder the
  /// chance to respond to platform messages from the Dart application.
  /// The callback will be invoked on the thread on which the `FlutterEngineRun`
  /// call is made. The second parameter, `user_data`, is supplied when
  /// `FlutterEngineRun` or `FlutterEngineInitialize` is called.
  FlutterPlatformMessageCallback platform_message_callback;
  /// The VM snapshot data buffer used in AOT operation. This buffer must be
  /// mapped in as read-only. For more information refer to the documentation on
  /// the Wiki at
  /// https://github.com/flutter/flutter/wiki/Flutter-engine-operation-in-AOT-Mode
  const uint8_t* vm_snapshot_data;
  /// The size of the VM snapshot data buffer.  If vm_snapshot_data is a symbol
  /// reference, 0 may be passed here.
  size_t vm_snapshot_data_size;
  /// The VM snapshot instructions buffer used in AOT operation. This buffer
  /// must be mapped in as read-execute. For more information refer to the
  /// documentation on the Wiki at
  /// https://github.com/flutter/flutter/wiki/Flutter-engine-operation-in-AOT-Mode
  const uint8_t* vm_snapshot_instructions;
  /// The size of the VM snapshot instructions buffer. If
  /// vm_snapshot_instructions is a symbol reference, 0 may be passed here.
  size_t vm_snapshot_instructions_size;
  /// The isolate snapshot data buffer used in AOT operation. This buffer must
  /// be mapped in as read-only. For more information refer to the documentation
  /// on the Wiki at
  /// https://github.com/flutter/flutter/wiki/Flutter-engine-operation-in-AOT-Mode
  const uint8_t* isolate_snapshot_data;
  /// The size of the isolate snapshot data buffer.  If isolate_snapshot_data is
  /// a symbol reference, 0 may be passed here.
  size_t isolate_snapshot_data_size;
  /// The isolate snapshot instructions buffer used in AOT operation. This
  /// buffer must be mapped in as read-execute. For more information refer to
  /// the documentation on the Wiki at
  /// https://github.com/flutter/flutter/wiki/Flutter-engine-operation-in-AOT-Mode
  const uint8_t* isolate_snapshot_instructions;
  /// The size of the isolate snapshot instructions buffer. If
  /// isolate_snapshot_instructions is a symbol reference, 0 may be passed here.
  size_t isolate_snapshot_instructions_size;
  /// The callback invoked by the engine in root isolate scope. Called
  /// immediately after the root isolate has been created and marked runnable.
  VoidCallback root_isolate_create_callback;
  /// The legacy callback invoked by the engine in order to give the embedder
  /// the chance to respond to semantics node updates from the Dart application.
  /// Semantics node updates are sent in batches terminated by a 'batch end'
  /// callback that is passed a sentinel `FlutterSemanticsNode` whose `id` field
  /// has the value `kFlutterSemanticsNodeIdBatchEnd`.
  ///
  /// The callback will be invoked on the thread on which the `FlutterEngineRun`
  /// call is made.
  ///
  /// @deprecated    Use `update_semantics_callback2` instead. Only one of
  ///                `update_semantics_node_callback`,
  ///                `update_semantics_callback`, and
  ///                `update_semantics_callback2` may be provided; the others
  ///                should be set to null.
  ///
  ///                This callback is incompatible with multiple views. If this
  ///                callback is provided, `FlutterEngineAddView` and
  ///                `FlutterEngineRemoveView` should not be used.
  FlutterUpdateSemanticsNodeCallback update_semantics_node_callback;
  /// The legacy callback invoked by the engine in order to give the embedder
  /// the chance to respond to updates to semantics custom actions from the Dart
  /// application. Custom action updates are sent in batches terminated by a
  /// 'batch end' callback that is passed a sentinel
  /// `FlutterSemanticsCustomAction` whose `id` field has the value
  /// `kFlutterSemanticsCustomActionIdBatchEnd`.
  ///
  /// The callback will be invoked on the thread on which the `FlutterEngineRun`
  /// call is made.
  ///
  /// @deprecated    Use `update_semantics_callback2` instead. Only one of
  ///                `update_semantics_node_callback`,
  ///                `update_semantics_callback`, and
  ///                `update_semantics_callback2` may be provided; the others
  ///                should be set to null.
  ///
  ///                This callback is incompatible with multiple views. If this
  ///                callback is provided, `FlutterEngineAddView` and
  ///                `FlutterEngineRemoveView` should not be used.
  FlutterUpdateSemanticsCustomActionCallback
      update_semantics_custom_action_callback;
  /// Path to a directory used to store data that is cached across runs of a
  /// Flutter application (such as compiled shader programs used by Skia).
  /// This is optional.  The string must be NULL terminated.
  ///
  // This is different from the cache-path-dir argument defined in switches.h,
  // which is used in `flutter::Settings` as `temp_directory_path`.
  const char* persistent_cache_path;

  /// If true, the engine would only read the existing cache, but not write new
  /// ones.
  bool is_persistent_cache_read_only;

  /// A callback that gets invoked by the engine when it attempts to wait for a
  /// platform vsync event. The engine will give the platform a baton that needs
  /// to be returned back to the engine via `FlutterEngineOnVsync`. All batons
  /// must be retured to the engine before initializing a
  /// `FlutterEngineShutdown`. Not doing the same will result in a memory leak.
  /// While the call to `FlutterEngineOnVsync` must occur on the thread that
  /// made the call to `FlutterEngineRun`, the engine will make this callback on
  /// an internal engine-managed thread. If the components accessed on the
  /// embedder are not thread safe, the appropriate re-threading must be done.
  VsyncCallback vsync_callback;

  /// The name of a custom Dart entrypoint. This is optional and specifying a
  /// null or empty entrypoint makes the engine look for a method named "main"
  /// in the root library of the application.
  ///
  /// Care must be taken to ensure that the custom entrypoint is not tree-shaken
  /// away. Usually, this is done using the `@pragma('vm:entry-point')`
  /// decoration.
  const char* custom_dart_entrypoint;

  /// Typically the Flutter engine create and manages its internal threads. This
  /// optional argument allows for the specification of task runner interfaces
  /// to event loops managed by the embedder on threads it creates.
  const FlutterCustomTaskRunners* custom_task_runners;

  /// All `FlutterEngine` instances in the process share the same Dart VM. When
  /// the first engine is launched, it starts the Dart VM as well. It used to be
  /// the case that it was not possible to shutdown the Dart VM cleanly and
  /// start it back up in the process in a safe manner. This issue has since
  /// been patched. Unfortunately, applications already began to make use of the
  /// fact that shutting down the Flutter engine instance left a running VM in
  /// the process. Since a Flutter engine could be launched on any thread,
  /// applications would "warm up" the VM on another thread by launching
  /// an engine with no isolates and then shutting it down immediately. The main
  /// Flutter application could then be started on the main thread without
  /// having to incur the Dart VM startup costs at that time. With the new
  /// behavior, this "optimization" immediately becomes massive performance
  /// pessimization as the VM would be started up in the "warm up" phase, shut
  /// down there and then started again on the main thread. Changing this
  /// behavior was deemed to be an unacceptable breaking change. Embedders that
  /// wish to shutdown the Dart VM when the last engine is terminated in the
  /// process should opt into this behavior by setting this flag to true.
  bool shutdown_dart_vm_when_done;

  /// Typically, Flutter renders the layer hierarchy into a single root surface.
  /// However, when embedders need to interleave their own contents within the
  /// Flutter layer hierarchy, their applications can push platform views within
  /// the Flutter scene. This is done using the `SceneBuilder.addPlatformView`
  /// call. When this happens, the Flutter rasterizer divides the effective view
  /// hierarchy into multiple layers. Each layer gets its own backing store and
  /// Flutter renders into the same. Once the layers contents have been
  /// fulfilled, the embedder is asked to composite these layers on-screen. At
  /// this point, it can interleave its own contents within the effective
  /// hierarchy. The interface for the specification of these layer backing
  /// stores and the hooks to listen for the composition of layers on-screen can
  /// be controlled using this field. This field is completely optional. In its
  /// absence, platforms views in the scene are ignored and Flutter renders to
  /// the root surface as normal.
  const FlutterCompositor* compositor;

  /// Max size of the old gen heap for the Dart VM in MB, or 0 for unlimited, -1
  /// for default value.
  ///
  /// See also:
  /// https://github.com/dart-lang/sdk/blob/ca64509108b3e7219c50d6c52877c85ab6a35ff2/runtime/vm/flag_list.h#L150
  int64_t dart_old_gen_heap_size;

  /// The AOT data to be used in AOT operation.
  ///
  /// Embedders should instantiate and destroy this object via the
  /// FlutterEngineCreateAOTData and FlutterEngineCollectAOTData methods.
  ///
  /// Embedders can provide either snapshot buffers or aot_data, but not both.
  FlutterEngineAOTData aot_data;

  /// A callback that computes the locale the platform would natively resolve
  /// to.
  ///
  /// The input parameter is an array of FlutterLocales which represent the
  /// locales supported by the app. One of the input supported locales should
  /// be selected and returned to best match with the user/device's preferred
  /// locale. The implementation should produce a result that as closely
  /// matches what the platform would natively resolve to as possible.
  FlutterComputePlatformResolvedLocaleCallback
      compute_platform_resolved_locale_callback;

  /// The command line argument count for arguments passed through to the Dart
  /// entrypoint.
  int dart_entrypoint_argc;

  /// The command line arguments passed through to the Dart entrypoint. The
  /// strings must be `NULL` terminated.
  ///
  /// The strings will be copied out and so any strings passed in here can
  /// be safely collected after initializing the engine with
  /// `FlutterProjectArgs`.
  const char* const* dart_entrypoint_argv;

  // Logging callback for Dart application messages.
  //
  // This callback is used by embedder to log print messages from the running
  // Flutter application. This callback is made on an internal engine managed
  // thread and embedders must re-thread if necessary. Performing blocking calls
  // in this callback may introduce application jank.
  FlutterLogMessageCallback log_message_callback;

  // A tag string associated with application log messages.
  //
  // A log message tag string that can be used convey application, subsystem,
  // or component name to embedder's logger. This string will be passed to to
  // callbacks on `log_message_callback`. Defaults to "flutter" if unspecified.
  const char* log_tag;

  // A callback that is invoked right before the engine is restarted.
  //
  // This optional callback is typically used to reset states to as if the
  // engine has just been started, and usually indicates the user has requested
  // a hot restart (Shift-R in the Flutter CLI.) It is not called the first time
  // the engine starts.
  //
  // The first argument is the `user_data` from `FlutterEngineInitialize`.
  OnPreEngineRestartCallback on_pre_engine_restart_callback;

  /// The callback invoked by the engine in order to give the embedder the
  /// chance to respond to updates to semantics nodes and custom actions from
  /// the Dart application.
  ///
  /// The callback will be invoked on the thread on which the `FlutterEngineRun`
  /// call is made.
  ///
  /// @deprecated    Use `update_semantics_callback2` instead. Only one of
  ///                `update_semantics_node_callback`,
  ///                `update_semantics_callback`, and
  ///                `update_semantics_callback2` may be provided; the others
  ///                must be set to null.
  ///
  ///                This callback is incompatible with multiple views. If this
  ///                callback is provided, `FlutterEngineAddView` and
  ///                `FlutterEngineRemoveView` should not be used.
  FlutterUpdateSemanticsCallback update_semantics_callback;

  /// The callback invoked by the engine in order to give the embedder the
  /// chance to respond to updates to semantics nodes and custom actions from
  /// the Dart application.
  ///
  /// The callback will be invoked on the thread on which the `FlutterEngineRun`
  /// call is made.
  ///
  /// Only one of `update_semantics_node_callback`, `update_semantics_callback`,
  /// and `update_semantics_callback2` may be provided; the others must be set
  /// to null.
  FlutterUpdateSemanticsCallback2 update_semantics_callback2;

  /// The callback invoked by the engine in response to a channel listener
  /// being registered on the framework side. The callback is invoked from
  /// a task posted to the platform thread.
  FlutterChannelUpdateCallback channel_update_callback;
} FlutterProjectArgs;

#ifndef FLUTTER_ENGINE_NO_PROTOTYPES

// NOLINTBEGIN(google-objc-function-naming)

//------------------------------------------------------------------------------
/// @brief      Creates the necessary data structures to launch a Flutter Dart
///             application in AOT mode. The data may only be collected after
///             all FlutterEngine instances launched using this data have been
///             terminated.
///
/// @param[in]  source    The source of the AOT data.
/// @param[out] data_out  The AOT data on success. Unchanged on failure.
///
/// @return     Returns if the AOT data could be successfully resolved.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineCreateAOTData(
    const FlutterEngineAOTDataSource* source,
    FlutterEngineAOTData* data_out);

//------------------------------------------------------------------------------
/// @brief      Collects the AOT data.
///
/// @warning    The embedder must ensure that this call is made only after all
///             FlutterEngine instances launched using this data have been
///             terminated, and that all of those instances were launched with
///             the FlutterProjectArgs::shutdown_dart_vm_when_done flag set to
///             true.
///
/// @param[in]  data   The data to collect.
///
/// @return     Returns if the AOT data was successfully collected.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineCollectAOTData(FlutterEngineAOTData data);

//------------------------------------------------------------------------------
/// @brief      Initialize and run a Flutter engine instance and return a handle
///             to it. This is a convenience method for the pair of calls to
///             `FlutterEngineInitialize` and `FlutterEngineRunInitialized`.
///
/// @note       This method of running a Flutter engine works well except in
///             cases where the embedder specifies custom task runners via
///             `FlutterProjectArgs::custom_task_runners`. In such cases, the
///             engine may need the embedder to post tasks back to it before
///             `FlutterEngineRun` has returned. Embedders can only post tasks
///             to the engine if they have a handle to the engine. In such
///             cases, embedders are advised to get the engine handle via the
///             `FlutterInitializeCall`. Then they can call
///             `FlutterEngineRunInitialized` knowing that they will be able to
///             service custom tasks on other threads with the engine handle.
///
/// @param[in]  version    The Flutter embedder API version. Must be
///                        FLUTTER_ENGINE_VERSION.
/// @param[in]  config     The renderer configuration.
/// @param[in]  args       The Flutter project arguments.
/// @param      user_data  A user data baton passed back to embedders in
///                        callbacks.
/// @param[out] engine_out The engine handle on successful engine creation.
///
/// @return     The result of the call to run the Flutter engine.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineRun(size_t version,
                                     const FlutterRendererConfig* config,
                                     const FlutterProjectArgs* args,
                                     void* user_data,
                                     FLUTTER_API_SYMBOL(FlutterEngine) *
                                         engine_out);

//------------------------------------------------------------------------------
/// @brief      Shuts down a Flutter engine instance. The engine handle is no
///             longer valid for any calls in the embedder API after this point.
///             Making additional calls with this handle is undefined behavior.
///
/// @note       This de-initializes the Flutter engine instance (via an implicit
///             call to `FlutterEngineDeinitialize`) if necessary.
///
/// @param[in]  engine  The Flutter engine instance to collect.
///
/// @return     The result of the call to shutdown the Flutter engine instance.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineShutdown(FLUTTER_API_SYMBOL(FlutterEngine)
                                              engine);

//------------------------------------------------------------------------------
/// @brief      Initialize a Flutter engine instance. This does not run the
///             Flutter application code till the `FlutterEngineRunInitialized`
///             call is made. Besides Flutter application code, no tasks are
///             scheduled on embedder managed task runners either. This allows
///             embedders providing custom task runners to the Flutter engine to
///             obtain a handle to the Flutter engine before the engine can post
///             tasks on these task runners.
///
/// @param[in]  version    The Flutter embedder API version. Must be
///                        FLUTTER_ENGINE_VERSION.
/// @param[in]  config     The renderer configuration.
/// @param[in]  args       The Flutter project arguments.
/// @param      user_data  A user data baton passed back to embedders in
///                        callbacks.
/// @param[out] engine_out The engine handle on successful engine creation.
///
/// @return     The result of the call to initialize the Flutter engine.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineInitialize(size_t version,
                                            const FlutterRendererConfig* config,
                                            const FlutterProjectArgs* args,
                                            void* user_data,
                                            FLUTTER_API_SYMBOL(FlutterEngine) *
                                                engine_out);

//------------------------------------------------------------------------------
/// @brief      Stops running the Flutter engine instance. After this call, the
///             embedder is also guaranteed that no more calls to post tasks
///             onto custom task runners specified by the embedder are made. The
///             Flutter engine handle still needs to be collected via a call to
///             `FlutterEngineShutdown`.
///
/// @param[in]  engine    The running engine instance to de-initialize.
///
/// @return     The result of the call to de-initialize the Flutter engine.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineDeinitialize(FLUTTER_API_SYMBOL(FlutterEngine)
                                                  engine);

//------------------------------------------------------------------------------
/// @brief      Runs an initialized engine instance. An engine can be
///             initialized via `FlutterEngineInitialize`. An initialized
///             instance can only be run once. During and after this call,
///             custom task runners supplied by the embedder are expected to
///             start servicing tasks.
///
/// @param[in]  engine  An initialized engine instance that has not previously
///                     been run.
///
/// @return     The result of the call to run the initialized Flutter
///             engine instance.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineRunInitialized(
    FLUTTER_API_SYMBOL(FlutterEngine) engine);

//------------------------------------------------------------------------------
/// @brief      Adds a view.
///
///             This is an asynchronous operation. The view should not be used
///             until the |info.add_view_callback| is invoked with an |added|
///             value of true. The embedder should prepare resources in advance
///             but be ready to clean up on failure.
///
///             A frame is scheduled if the operation succeeds.
///
///             The callback is invoked on a thread managed by the engine. The
///             embedder should re-thread if needed.
///
///             Attempting to add the implicit view will fail and will return
///             kInvalidArguments. Attempting to add a view with an already
///             existing view ID will fail, and |info.add_view_callback| will be
///             invoked with an |added| value of false.
///
/// @param[in]  engine  A running engine instance.
/// @param[in]  info    The add view arguments. This can be deallocated
///                     once |FlutterEngineAddView| returns, before
///                     |add_view_callback| is invoked.
///
/// @return     The result of *starting* the asynchronous operation. If
///             `kSuccess`, the |add_view_callback| will be invoked.
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineAddView(FLUTTER_API_SYMBOL(FlutterEngine)
                                             engine,
                                         const FlutterAddViewInfo* info);

//------------------------------------------------------------------------------
/// @brief      Removes a view.
///
///             This is an asynchronous operation. The view's resources must not
///             be cleaned up until |info.remove_view_callback| is invoked with
///             a |removed| value of true.
///
///             The callback is invoked on a thread managed by the engine. The
///             embedder should re-thread if needed.
///
///             Attempting to remove the implicit view will fail and will return
///             kInvalidArguments. Attempting to remove a view with a
///             non-existent view ID will fail, and |info.remove_view_callback|
///             will be invoked with a |removed| value of false.
///
/// @param[in]  engine  A running engine instance.
/// @param[in]  info    The remove view arguments. This can be deallocated
///                     once |FlutterEngineRemoveView| returns, before
///                     |remove_view_callback| is invoked.
///
/// @return     The result of *starting* the asynchronous operation. If
///             `kSuccess`, the |remove_view_callback| will be invoked.
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineRemoveView(FLUTTER_API_SYMBOL(FlutterEngine)
                                                engine,
                                            const FlutterRemoveViewInfo* info);

FLUTTER_EXPORT
FlutterEngineResult FlutterEngineSendWindowMetricsEvent(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterWindowMetricsEvent* event);

FLUTTER_EXPORT
FlutterEngineResult FlutterEngineSendPointerEvent(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterPointerEvent* events,
    size_t events_count);

//------------------------------------------------------------------------------
/// @brief      Sends a key event to the engine. The framework will decide
///             whether to handle this event in a synchronous fashion, although
///             due to technical limitation, the result is always reported
///             asynchronously. The `callback` is guaranteed to be called
///             exactly once.
///
/// @param[in]  engine         A running engine instance.
/// @param[in]  event          The event data to be sent. This function will no
///                            longer access `event` after returning.
/// @param[in]  callback       The callback invoked by the engine when the
///                            Flutter application has decided whether it
///                            handles this event. Accepts nullptr.
/// @param[in]  user_data      The context associated with the callback. The
///                            exact same value will used to invoke `callback`.
///                            Accepts nullptr.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineSendKeyEvent(FLUTTER_API_SYMBOL(FlutterEngine)
                                                  engine,
                                              const FlutterKeyEvent* event,
                                              FlutterKeyEventCallback callback,
                                              void* user_data);

FLUTTER_EXPORT
FlutterEngineResult FlutterEngineSendPlatformMessage(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterPlatformMessage* message);

//------------------------------------------------------------------------------
/// @brief     Creates a platform message response handle that allows the
///            embedder to set a native callback for a response to a message.
///            This handle may be set on the `response_handle` field of any
///            `FlutterPlatformMessage` sent to the engine.
///
///            The handle must be collected via a call to
///            `FlutterPlatformMessageReleaseResponseHandle`. This may be done
///            immediately after a call to `FlutterEngineSendPlatformMessage`
///            with a platform message whose response handle contains the handle
///            created using this call. In case a handle is created but never
///            sent in a message, the release call must still be made. Not
///            calling release on the handle results in a small memory leak.
///
///            The user data baton passed to the data callback is the one
///            specified in this call as the third argument.
///
/// @see       FlutterPlatformMessageReleaseResponseHandle()
///
/// @param[in]  engine         A running engine instance.
/// @param[in]  data_callback  The callback invoked by the engine when the
///                            Flutter application send a response on the
///                            handle.
/// @param[in]  user_data      The user data associated with the data callback.
/// @param[out] response_out   The response handle created when this call is
///                            successful.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterPlatformMessageCreateResponseHandle(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterDataCallback data_callback,
    void* user_data,
    FlutterPlatformMessageResponseHandle** response_out);

//------------------------------------------------------------------------------
/// @brief      Collects the handle created using
///             `FlutterPlatformMessageCreateResponseHandle`.
///
/// @see        FlutterPlatformMessageCreateResponseHandle()
///
/// @param[in]  engine     A running engine instance.
/// @param[in]  response   The platform message response handle to collect.
///                        These handles are created using
///                        `FlutterPlatformMessageCreateResponseHandle()`.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterPlatformMessageReleaseResponseHandle(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterPlatformMessageResponseHandle* response);

//------------------------------------------------------------------------------
/// @brief      Send a response from the native side to a platform message from
///             the Dart Flutter application.
///
/// @param[in]  engine       The running engine instance.
/// @param[in]  handle       The platform message response handle.
/// @param[in]  data         The data to associate with the platform message
///                          response.
/// @param[in]  data_length  The length of the platform message response data.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineSendPlatformMessageResponse(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterPlatformMessageResponseHandle* handle,
    const uint8_t* data,
    size_t data_length);

//------------------------------------------------------------------------------
/// @brief      This API is only meant to be used by platforms that need to
///             flush tasks on a message loop not controlled by the Flutter
///             engine.
///
/// @deprecated This API will be deprecated and is not part of the stable API.
///             Please use the custom task runners API by setting an
///             appropriate `FlutterProjectArgs::custom_task_runners`
///             interface. This will yield better performance and the
///             interface is stable.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult __FlutterEngineFlushPendingTasksNow();

//------------------------------------------------------------------------------
/// @brief      Register an external texture with a unique (per engine)
///             identifier. Only rendering backends that support external
///             textures accept external texture registrations. After the
///             external texture is registered, the application can mark that a
///             frame is available by calling
///             `FlutterEngineMarkExternalTextureFrameAvailable`.
///
/// @see        FlutterEngineUnregisterExternalTexture()
/// @see        FlutterEngineMarkExternalTextureFrameAvailable()
///
/// @param[in]  engine              A running engine instance.
/// @param[in]  texture_identifier  The identifier of the texture to register
///                                 with the engine. The embedder may supply new
///                                 frames to this texture using the same
///                                 identifier.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineRegisterExternalTexture(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    int64_t texture_identifier);

//------------------------------------------------------------------------------
/// @brief      Unregister a previous texture registration.
///
/// @see        FlutterEngineRegisterExternalTexture()
/// @see        FlutterEngineMarkExternalTextureFrameAvailable()
///
/// @param[in]  engine              A running engine instance.
/// @param[in]  texture_identifier  The identifier of the texture for which new
///                                 frame will not be available.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineUnregisterExternalTexture(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    int64_t texture_identifier);

//------------------------------------------------------------------------------
/// @brief      Mark that a new texture frame is available for a given texture
///             identifier.
///
/// @see        FlutterEngineRegisterExternalTexture()
/// @see        FlutterEngineUnregisterExternalTexture()
///
/// @param[in]  engine              A running engine instance.
/// @param[in]  texture_identifier  The identifier of the texture whose frame
///                                 has been updated.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineMarkExternalTextureFrameAvailable(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    int64_t texture_identifier);

//------------------------------------------------------------------------------
/// @brief      Enable or disable accessibility semantics.
///
/// @param[in]  engine     A running engine instance.
/// @param[in]  enabled    When enabled, changes to the semantic contents of the
///                        window are sent via the
///                        `FlutterUpdateSemanticsCallback2` registered to
///                        `update_semantics_callback2` in
///                        `FlutterProjectArgs`.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineUpdateSemanticsEnabled(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    bool enabled);

//------------------------------------------------------------------------------
/// @brief      Sets additional accessibility features.
///
/// @param[in]  engine     A running engine instance
/// @param[in]  features   The accessibility features to set.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineUpdateAccessibilityFeatures(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterAccessibilityFeature features);

//------------------------------------------------------------------------------
/// @brief      Dispatch a semantics action to the specified semantics node.
///
/// @param[in]  engine       A running engine instance.
/// @param[in]  node_id      The semantics node identifier.
/// @param[in]  action       The semantics action.
/// @param[in]  data         Data associated with the action.
/// @param[in]  data_length  The data length.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineDispatchSemanticsAction(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    uint64_t node_id,
    FlutterSemanticsAction action,
    const uint8_t* data,
    size_t data_length);

//------------------------------------------------------------------------------
/// @brief      Notify the engine that a vsync event occurred. A baton passed to
///             the platform via the vsync callback must be returned. This call
///             must be made on the thread on which the call to
///             `FlutterEngineRun` was made.
///
/// @see        FlutterEngineGetCurrentTime()
///
/// @attention  That frame timepoints are in nanoseconds.
///
/// @attention  The system monotonic clock is used as the timebase.
///
/// @param[in]  engine.                  A running engine instance.
/// @param[in]  baton                    The baton supplied by the engine.
/// @param[in]  frame_start_time_nanos   The point at which the vsync event
///                                      occurred or will occur. If the time
///                                      point is in the future, the engine will
///                                      wait till that point to begin its frame
///                                      workload.
/// @param[in]  frame_target_time_nanos  The point at which the embedder
///                                      anticipates the next vsync to occur.
///                                      This is a hint the engine uses to
///                                      schedule Dart VM garbage collection in
///                                      periods in which the various threads
///                                      are most likely to be idle. For
///                                      example, for a 60Hz display, embedders
///                                      should add 16.6 * 1e6 to the frame time
///                                      field.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineOnVsync(FLUTTER_API_SYMBOL(FlutterEngine)
                                             engine,
                                         intptr_t baton,
                                         uint64_t frame_start_time_nanos,
                                         uint64_t frame_target_time_nanos);

//------------------------------------------------------------------------------
/// @brief      Reloads the system fonts in engine.
///
/// @param[in]  engine.                  A running engine instance.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineReloadSystemFonts(
    FLUTTER_API_SYMBOL(FlutterEngine) engine);

//------------------------------------------------------------------------------
/// @brief      A profiling utility. Logs a trace duration begin event to the
///             timeline. If the timeline is unavailable or disabled, this has
///             no effect. Must be balanced with an duration end event (via
///             `FlutterEngineTraceEventDurationEnd`) with the same name on the
///             same thread. Can be called on any thread. Strings passed into
///             the function will NOT be copied when added to the timeline. Only
///             string literals may be passed in.
///
/// @param[in]  name  The name of the trace event.
///
FLUTTER_EXPORT
void FlutterEngineTraceEventDurationBegin(const char* name);

//-----------------------------------------------------------------------------
/// @brief      A profiling utility. Logs a trace duration end event to the
///             timeline. If the timeline is unavailable or disabled, this has
///             no effect. This call must be preceded by a trace duration begin
///             call (via `FlutterEngineTraceEventDurationBegin`) with the same
///             name on the same thread. Can be called on any thread. Strings
///             passed into the function will NOT be copied when added to the
///             timeline. Only string literals may be passed in.
///
/// @param[in]  name  The name of the trace event.
///
FLUTTER_EXPORT
void FlutterEngineTraceEventDurationEnd(const char* name);

//-----------------------------------------------------------------------------
/// @brief      A profiling utility. Logs a trace duration instant event to the
///             timeline. If the timeline is unavailable or disabled, this has
///             no effect. Can be called on any thread. Strings passed into the
///             function will NOT be copied when added to the timeline. Only
///             string literals may be passed in.
///
/// @param[in]  name  The name of the trace event.
///
FLUTTER_EXPORT
void FlutterEngineTraceEventInstant(const char* name);

//------------------------------------------------------------------------------
/// @brief      Posts a task onto the Flutter render thread. Typically, this may
///             be called from any thread as long as a `FlutterEngineShutdown`
///             on the specific engine has not already been initiated.
///
/// @param[in]  engine         A running engine instance.
/// @param[in]  callback       The callback to execute on the render thread.
/// @param      callback_data  The callback context.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEnginePostRenderThreadTask(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    VoidCallback callback,
    void* callback_data);

//------------------------------------------------------------------------------
/// @brief      Get the current time in nanoseconds from the clock used by the
///             flutter engine. This is the system monotonic clock.
///
/// @return     The current time in nanoseconds.
///
FLUTTER_EXPORT
uint64_t FlutterEngineGetCurrentTime();

//------------------------------------------------------------------------------
/// @brief      Inform the engine to run the specified task. This task has been
///             given to the engine via the
///             `FlutterTaskRunnerDescription.post_task_callback`. This call
///             must only be made at the target time specified in that callback.
///             Running the task before that time is undefined behavior.
///
/// @param[in]  engine     A running engine instance.
/// @param[in]  task       the task handle.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineRunTask(FLUTTER_API_SYMBOL(FlutterEngine)
                                             engine,
                                         const FlutterTask* task);

//------------------------------------------------------------------------------
/// @brief      Notify a running engine instance that the locale has been
///             updated. The preferred locale must be the first item in the list
///             of locales supplied. The other entries will be used as a
///             fallback.
///
/// @param[in]  engine         A running engine instance.
/// @param[in]  locales        The updated locales in the order of preference.
/// @param[in]  locales_count  The count of locales supplied.
///
/// @return     Whether the locale updates were applied.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineUpdateLocales(FLUTTER_API_SYMBOL(FlutterEngine)
                                                   engine,
                                               const FlutterLocale** locales,
                                               size_t locales_count);

//------------------------------------------------------------------------------
/// @brief      Returns if the Flutter engine instance will run AOT compiled
///             Dart code. This call has no threading restrictions.
///
///             For embedder code that is configured for both AOT and JIT mode
///             Dart execution based on the Flutter engine being linked to, this
///             runtime check may be used to appropriately configure the
///             `FlutterProjectArgs`. In JIT mode execution, the kernel
///             snapshots must be present in the Flutter assets directory
///             specified in the `FlutterProjectArgs`. For AOT execution, the
///             fields `vm_snapshot_data`, `vm_snapshot_instructions`,
///             `isolate_snapshot_data` and `isolate_snapshot_instructions`
///             (along with their size fields) must be specified in
///             `FlutterProjectArgs`.
///
/// @return     True, if AOT Dart code is run. JIT otherwise.
///
FLUTTER_EXPORT
bool FlutterEngineRunsAOTCompiledDartCode(void);

//------------------------------------------------------------------------------
/// @brief      Posts a Dart object to specified send port. The corresponding
///             receive port for send port can be in any isolate running in the
///             VM. This isolate can also be the root isolate for an
///             unrelated engine. The engine parameter is necessary only to
///             ensure the call is not made when no engine (and hence no VM) is
///             running.
///
///             Unlike the platform messages mechanism, there are no threading
///             restrictions when using this API. Message can be posted on any
///             thread and they will be made available to isolate on which the
///             corresponding send port is listening.
///
///             However, it is the embedders responsibility to ensure that the
///             call is not made during an ongoing call the
///             `FlutterEngineDeinitialize` or `FlutterEngineShutdown` on
///             another thread.
///
/// @param[in]  engine     A running engine instance.
/// @param[in]  port       The send port to send the object to.
/// @param[in]  object     The object to send to the isolate with the
///                        corresponding receive port.
///
/// @return     If the message was posted to the send port.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEnginePostDartObject(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterEngineDartPort port,
    const FlutterEngineDartObject* object);

//------------------------------------------------------------------------------
/// @brief      Posts a low memory notification to a running engine instance.
///             The engine will do its best to release non-critical resources in
///             response. It is not guaranteed that the resource would have been
///             collected by the time this call returns however. The
///             notification is posted to engine subsystems that may be
///             operating on other threads.
///
///             Flutter applications can respond to these notifications by
///             setting `WidgetsBindingObserver.didHaveMemoryPressure`
///             observers.
///
/// @param[in]  engine     A running engine instance.
///
/// @return     If the low memory notification was sent to the running engine
///             instance.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineNotifyLowMemoryWarning(
    FLUTTER_API_SYMBOL(FlutterEngine) engine);

//------------------------------------------------------------------------------
/// @brief      Schedule a callback to be run on all engine managed threads.
///             The engine will attempt to service this callback the next time
///             the message loop for each managed thread is idle. Since the
///             engine manages the entire lifecycle of multiple threads, there
///             is no opportunity for the embedders to finely tune the
///             priorities of threads directly, or, perform other thread
///             specific configuration (for example, setting thread names for
///             tracing). This callback gives embedders a chance to affect such
///             tuning.
///
/// @attention  This call is expensive and must be made as few times as
///             possible. The callback must also return immediately as not doing
///             so may risk performance issues (especially for callbacks of type
///             kFlutterNativeThreadTypeUI and kFlutterNativeThreadTypeRender).
///
/// @attention  Some callbacks (especially the ones of type
///             kFlutterNativeThreadTypeWorker) may be called after the
///             FlutterEngine instance has shut down. Embedders must be careful
///             in handling the lifecycle of objects associated with the user
///             data baton.
///
/// @attention  In case there are multiple running Flutter engine instances,
///             their workers are shared.
///
/// @param[in]  engine     A running engine instance.
/// @param[in]  callback   The callback that will get called multiple times on
///                        each engine managed thread.
/// @param[in]  user_data  A baton passed by the engine to the callback. This
///                        baton is not interpreted by the engine in any way.
///
/// @return     Returns if the callback was successfully posted to all threads.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEnginePostCallbackOnAllNativeThreads(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterNativeThreadCallback callback,
    void* user_data);

//------------------------------------------------------------------------------
/// @brief    Posts updates corresponding to display changes to a running engine
///           instance.
///
/// @param[in] update_type      The type of update pushed to the engine.
/// @param[in] displays         The displays affected by this update.
/// @param[in] display_count    Size of the displays array, must be at least 1.
///
/// @return the result of the call made to the engine.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineNotifyDisplayUpdate(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterEngineDisplaysUpdateType update_type,
    const FlutterEngineDisplay* displays,
    size_t display_count);

//------------------------------------------------------------------------------
/// @brief      Schedule a new frame to redraw the content.
///
/// @param[in]  engine     A running engine instance.
///
/// @return the result of the call made to the engine.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineScheduleFrame(FLUTTER_API_SYMBOL(FlutterEngine)
                                                   engine);

//------------------------------------------------------------------------------
/// @brief      Schedule a callback to be called after the next frame is drawn.
///             This must be called from the platform thread. The callback is
///             executed only once from the raster thread; embedders must
///             re-thread if necessary. Performing blocking calls
///             in this callback may introduce application jank.
///
/// @param[in]  engine     A running engine instance.
/// @param[in]  callback   The callback to execute.
/// @param[in]  user_data  A baton passed by the engine to the callback. This
///                        baton is not interpreted by the engine in any way.
///
/// @return     The result of the call.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineSetNextFrameCallback(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    VoidCallback callback,
    void* user_data);

#endif  // !FLUTTER_ENGINE_NO_PROTOTYPES

// Typedefs for the function pointers in FlutterEngineProcTable.
typedef FlutterEngineResult (*FlutterEngineCreateAOTDataFnPtr)(
    const FlutterEngineAOTDataSource* source,
    FlutterEngineAOTData* data_out);
typedef FlutterEngineResult (*FlutterEngineCollectAOTDataFnPtr)(
    FlutterEngineAOTData data);
typedef FlutterEngineResult (*FlutterEngineRunFnPtr)(
    size_t version,
    const FlutterRendererConfig* config,
    const FlutterProjectArgs* args,
    void* user_data,
    FLUTTER_API_SYMBOL(FlutterEngine) * engine_out);
typedef FlutterEngineResult (*FlutterEngineShutdownFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine);
typedef FlutterEngineResult (*FlutterEngineInitializeFnPtr)(
    size_t version,
    const FlutterRendererConfig* config,
    const FlutterProjectArgs* args,
    void* user_data,
    FLUTTER_API_SYMBOL(FlutterEngine) * engine_out);
typedef FlutterEngineResult (*FlutterEngineDeinitializeFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine);
typedef FlutterEngineResult (*FlutterEngineRunInitializedFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine);
typedef FlutterEngineResult (*FlutterEngineSendWindowMetricsEventFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterWindowMetricsEvent* event);
typedef FlutterEngineResult (*FlutterEngineSendPointerEventFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterPointerEvent* events,
    size_t events_count);
typedef FlutterEngineResult (*FlutterEngineSendKeyEventFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterKeyEvent* event,
    FlutterKeyEventCallback callback,
    void* user_data);
typedef FlutterEngineResult (*FlutterEngineSendPlatformMessageFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterPlatformMessage* message);
typedef FlutterEngineResult (
    *FlutterEnginePlatformMessageCreateResponseHandleFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterDataCallback data_callback,
    void* user_data,
    FlutterPlatformMessageResponseHandle** response_out);
typedef FlutterEngineResult (
    *FlutterEnginePlatformMessageReleaseResponseHandleFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterPlatformMessageResponseHandle* response);
typedef FlutterEngineResult (*FlutterEngineSendPlatformMessageResponseFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterPlatformMessageResponseHandle* handle,
    const uint8_t* data,
    size_t data_length);
typedef FlutterEngineResult (*FlutterEngineRegisterExternalTextureFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    int64_t texture_identifier);
typedef FlutterEngineResult (*FlutterEngineUnregisterExternalTextureFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    int64_t texture_identifier);
typedef FlutterEngineResult (
    *FlutterEngineMarkExternalTextureFrameAvailableFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    int64_t texture_identifier);
typedef FlutterEngineResult (*FlutterEngineUpdateSemanticsEnabledFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    bool enabled);
typedef FlutterEngineResult (*FlutterEngineUpdateAccessibilityFeaturesFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterAccessibilityFeature features);
typedef FlutterEngineResult (*FlutterEngineDispatchSemanticsActionFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    uint64_t id,
    FlutterSemanticsAction action,
    const uint8_t* data,
    size_t data_length);
typedef FlutterEngineResult (*FlutterEngineOnVsyncFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    intptr_t baton,
    uint64_t frame_start_time_nanos,
    uint64_t frame_target_time_nanos);
typedef FlutterEngineResult (*FlutterEngineReloadSystemFontsFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine);
typedef void (*FlutterEngineTraceEventDurationBeginFnPtr)(const char* name);
typedef void (*FlutterEngineTraceEventDurationEndFnPtr)(const char* name);
typedef void (*FlutterEngineTraceEventInstantFnPtr)(const char* name);
typedef FlutterEngineResult (*FlutterEnginePostRenderThreadTaskFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    VoidCallback callback,
    void* callback_data);
typedef uint64_t (*FlutterEngineGetCurrentTimeFnPtr)();
typedef FlutterEngineResult (*FlutterEngineRunTaskFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterTask* task);
typedef FlutterEngineResult (*FlutterEngineUpdateLocalesFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterLocale** locales,
    size_t locales_count);
typedef bool (*FlutterEngineRunsAOTCompiledDartCodeFnPtr)(void);
typedef FlutterEngineResult (*FlutterEnginePostDartObjectFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterEngineDartPort port,
    const FlutterEngineDartObject* object);
typedef FlutterEngineResult (*FlutterEngineNotifyLowMemoryWarningFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine);
typedef FlutterEngineResult (*FlutterEnginePostCallbackOnAllNativeThreadsFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterNativeThreadCallback callback,
    void* user_data);
typedef FlutterEngineResult (*FlutterEngineNotifyDisplayUpdateFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterEngineDisplaysUpdateType update_type,
    const FlutterEngineDisplay* displays,
    size_t display_count);
typedef FlutterEngineResult (*FlutterEngineScheduleFrameFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine);
typedef FlutterEngineResult (*FlutterEngineSetNextFrameCallbackFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    VoidCallback callback,
    void* user_data);
typedef FlutterEngineResult (*FlutterEngineAddViewFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterAddViewInfo* info);
typedef FlutterEngineResult (*FlutterEngineRemoveViewFnPtr)(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterRemoveViewInfo* info);

/// Function-pointer-based versions of the APIs above.
typedef struct {
  /// The size of this struct. Must be sizeof(FlutterEngineProcs).
  size_t struct_size;

  FlutterEngineCreateAOTDataFnPtr CreateAOTData;
  FlutterEngineCollectAOTDataFnPtr CollectAOTData;
  FlutterEngineRunFnPtr Run;
  FlutterEngineShutdownFnPtr Shutdown;
  FlutterEngineInitializeFnPtr Initialize;
  FlutterEngineDeinitializeFnPtr Deinitialize;
  FlutterEngineRunInitializedFnPtr RunInitialized;
  FlutterEngineSendWindowMetricsEventFnPtr SendWindowMetricsEvent;
  FlutterEngineSendPointerEventFnPtr SendPointerEvent;
  FlutterEngineSendKeyEventFnPtr SendKeyEvent;
  FlutterEngineSendPlatformMessageFnPtr SendPlatformMessage;
  FlutterEnginePlatformMessageCreateResponseHandleFnPtr
      PlatformMessageCreateResponseHandle;
  FlutterEnginePlatformMessageReleaseResponseHandleFnPtr
      PlatformMessageReleaseResponseHandle;
  FlutterEngineSendPlatformMessageResponseFnPtr SendPlatformMessageResponse;
  FlutterEngineRegisterExternalTextureFnPtr RegisterExternalTexture;
  FlutterEngineUnregisterExternalTextureFnPtr UnregisterExternalTexture;
  FlutterEngineMarkExternalTextureFrameAvailableFnPtr
      MarkExternalTextureFrameAvailable;
  FlutterEngineUpdateSemanticsEnabledFnPtr UpdateSemanticsEnabled;
  FlutterEngineUpdateAccessibilityFeaturesFnPtr UpdateAccessibilityFeatures;
  FlutterEngineDispatchSemanticsActionFnPtr DispatchSemanticsAction;
  FlutterEngineOnVsyncFnPtr OnVsync;
  FlutterEngineReloadSystemFontsFnPtr ReloadSystemFonts;
  FlutterEngineTraceEventDurationBeginFnPtr TraceEventDurationBegin;
  FlutterEngineTraceEventDurationEndFnPtr TraceEventDurationEnd;
  FlutterEngineTraceEventInstantFnPtr TraceEventInstant;
  FlutterEnginePostRenderThreadTaskFnPtr PostRenderThreadTask;
  FlutterEngineGetCurrentTimeFnPtr GetCurrentTime;
  FlutterEngineRunTaskFnPtr RunTask;
  FlutterEngineUpdateLocalesFnPtr UpdateLocales;
  FlutterEngineRunsAOTCompiledDartCodeFnPtr RunsAOTCompiledDartCode;
  FlutterEnginePostDartObjectFnPtr PostDartObject;
  FlutterEngineNotifyLowMemoryWarningFnPtr NotifyLowMemoryWarning;
  FlutterEnginePostCallbackOnAllNativeThreadsFnPtr
      PostCallbackOnAllNativeThreads;
  FlutterEngineNotifyDisplayUpdateFnPtr NotifyDisplayUpdate;
  FlutterEngineScheduleFrameFnPtr ScheduleFrame;
  FlutterEngineSetNextFrameCallbackFnPtr SetNextFrameCallback;
  FlutterEngineAddViewFnPtr AddView;
  FlutterEngineRemoveViewFnPtr RemoveView;
} FlutterEngineProcTable;

//------------------------------------------------------------------------------
/// @brief      Gets the table of engine function pointers.
///
/// @param[out] table   The table to fill with pointers. This should be
///                     zero-initialized, except for struct_size.
///
/// @return     Returns whether the table was successfully populated.
///
FLUTTER_EXPORT
FlutterEngineResult FlutterEngineGetProcAddresses(
    FlutterEngineProcTable* table);

// NOLINTEND(google-objc-function-naming)

#if defined(__cplusplus)
}  // extern "C"
#endif

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_H_
