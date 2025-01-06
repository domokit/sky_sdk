// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'text_input.dart';
library;

import 'package:flutter/foundation.dart';

import 'message_codec.dart';
import 'system_channels.dart';

/// An interface into Android's stylus handwriting text input.
///
/// Allows handwriting directly on top of a text input using a stylus.
///
/// See also:
///
///  * [EditableText.stylusHandwritingEnabled], which controls whether Flutter's
///    built-in text fields support handwriting input.
///  * [SystemChannels.scribe], which is the [MethodChannel] used by this
///    class, and which has a list of the methods that this class handles.
///  * <https://developer.android.com/develop/ui/views/touch-and-input/stylus-input/stylus-input-in-text-fields>,
///    which is the Android documentation explaining the Scribe feature.
abstract final class Scribe {
  static const MethodChannel _channel = SystemChannels.scribe;

  /// A convenience method to check if the device currently supports Scribe
  /// stylus handwriting input.
  ///
  /// Call this each time before calling [startStylusHandwriting] to make sure
  /// it's available.
  ///
  /// {@tool snippet}
  /// This example shows using [isFeatureAvailable] to confirm that
  /// [startStylusHandwriting] can be called.
  ///
  /// ```dart
  /// if (!await Scribe.isFeatureAvailable()) {
  ///   // The device doesn't support stylus input right now, or maybe at all.
  ///   return;
  /// }
  ///
  /// // Scribe is supported, so start it.
  /// Scribe.startStylusHandwriting();
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  /// * [isStylusHandwritingAvailable], which is similar, but throws an error
  ///   when called by an unsupported API level. It directly corresponds to the
  ///   underlying Android API
  ///   <https://developer.android.com/reference/android/view/inputmethod/InputMethodManager#isStylusHandwritingAvailable()>.
  ///  * [EditableText.stylusHandwritingEnabled], which controls whether
  ///    Flutter's built-in text fields support handwriting input.
  static Future<bool> isFeatureAvailable() async {
    final bool? result = await _channel.invokeMethod<bool?>('Scribe.isFeatureAvailable');

    if (result == null) {
      throw FlutterError('MethodChannel.invokeMethod unexpectedly returned null.');
    }

    return result;
  }

  /// Returns true if the InputMethodManager supports Scribe stylus handwriting
  /// input, false otherwise.
  ///
  /// Call this each time before calling [startStylusHandwriting] to make sure
  /// it's available.
  ///
  /// Supported on Android API 34 and above. If called by an unsupported API
  /// level, a [PlatformException] will be thrown. To avoid error handling, use
  /// the convenience method [isFeatureAvailable] instead.
  ///
  /// {@tool snippet}
  /// This example shows using [isStylusHandwritingAvailable] to confirm that
  /// [startStylusHandwriting] can be called.
  ///
  /// ```dart
  /// try {
  ///   if (!await Scribe.isStylusHandwritingAvailable()) {
  ///     // If isStylusHandwritingAvailable returns false then the device's API level
  ///     // supports Scribe, but for some other reason it's not able to accept stylus
  ///     // input right now.
  ///     return;
  ///   }
  /// } on PlatformException catch (exception) {
  ///   if (exception.message == 'Requires API level 34 or higher.') {
  ///     // The device's API level is too low to support Scribe.
  ///     return;
  ///   }
  ///   // Any other exception is unexpected and should not be caught here.
  ///   rethrow;
  /// }
  ///
  /// // Scribe is supported, so start it.
  /// Scribe.startStylusHandwriting();
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  /// * <https://developer.android.com/reference/android/view/inputmethod/InputMethodManager#isStylusHandwritingAvailable()>,
  ///   which is the corresponding API on Android that this method attempts to
  ///   mirror.
  static Future<bool> isStylusHandwritingAvailable() async {
    final bool? result = await _channel.invokeMethod<bool?>('Scribe.isStylusHandwritingAvailable');

    if (result == null) {
      throw FlutterError('MethodChannel.invokeMethod unexpectedly returned null.');
    }

    return result;
  }

  /// Tell Android to begin receiving stylus handwriting input.
  ///
  /// This is typically called after detecting a [PointerDownEvent] from a
  /// [PointerDeviceKind.stylus] on an active text field, indicating the start
  /// of stylus handwriting input. If there is no active [TextInputConnection],
  /// the call will be ignored.
  ///
  /// Call [isFeatureAvailable] each time before calling this to make sure that
  /// stylus handwriting input is supported and available.
  ///
  /// Supported on Android API 33 and above.
  ///
  /// See also:
  ///
  /// * <https://developer.android.com/reference/android/view/inputmethod/InputMethodManager#startStylusHandwriting(android.view.View)>,
  ///   which is the corresponding API on Android that this method attempts to
  ///   mirror.
  ///  * [EditableText.stylusHandwritingEnabled], which controls whether
  ///    Flutter's built-in text fields support handwriting input.
  static Future<void> startStylusHandwriting() {
    return _channel.invokeMethod<void>('Scribe.startStylusHandwriting');
  }
}
