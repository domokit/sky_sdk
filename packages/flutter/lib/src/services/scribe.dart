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
/// This is typically used by implemeting the methods in [ScribeClient] in a
/// class, usually a [State], and setting an instance of it to [client]. The
/// relevant methods on [ScribeClient] will be called in response to method
/// channel calls on [SystemChannels.scribe].
///
/// See also:
///
///  * [EditableText.stylusHandwritingEnabled], which controls whether Flutter's
///    built-in text fields support handwriting input.
///  * [SystemChannels.scribe], which is the [MethodChannel] used by this
///    class, and which has a list of the methods that this class handles.
///  * <https://developer.android.com/develop/ui/views/touch-and-input/stylus-input/stylus-input-in-text-fields>,
///    which is the Android documentation explaining the Scribe feature.
final class Scribe {
  Scribe._() {
    _channel.setMethodCallHandler(_loudlyHandleScribeInputInvocation);
  }

  static const MethodChannel _channel = SystemChannels.scribe;
  static final Scribe _instance = Scribe._();

  final Set<ScribeClient> _scribeClients = <ScribeClient>{};

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

  static Rect _getSelectionArea(List<dynamic> args, double devicePixelRatio) {
    final Map<dynamic, dynamic> argsMap = args.first as Map<dynamic, dynamic>;
    final Map<dynamic, dynamic> selectionAreaMap =
        argsMap['selectionArea'] as Map<dynamic, dynamic>;
    final Map<String, double> selectionAreaJson = selectionAreaMap.cast<String, double>();
    return Rect.fromLTRB(
      // Flutter uses logical pixels while Android uses physical pixels, so we
      // need to divide by the devicePixelRatio to convert.
      selectionAreaJson['left']! / devicePixelRatio,
      selectionAreaJson['top']! / devicePixelRatio,
      selectionAreaJson['right']! / devicePixelRatio,
      selectionAreaJson['bottom']! / devicePixelRatio,
    );
  }

  static void registerScribeClient(ScribeClient scribeClient) {
    _instance._scribeClients.add(scribeClient);
    // TODO(justinmc): Support Scribe hover icon by sending the Rect of each
    // ScribeClient currently on screen to the engine.
    // https://github.com/flutter/flutter/issues/155948
  }

  static void unregisterScribeClient(ScribeClient scribeClient) {
    _instance._scribeClients.remove(scribeClient);
  }

  ScribeClient? get activeScribeClient {
    final Iterable<ScribeClient> activeClients = _scribeClients.where((ScribeClient client) {
      return client.isActive;
    });
    assert(activeClients.length <= 1);
    return activeClients.firstOrNull;
  }

  Future<dynamic> _loudlyHandleScribeInputInvocation(MethodCall call) async {
    try {
      return await _handleScribeInputInvocation(call);
    } catch (exception, stack) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: exception,
          stack: stack,
          library: 'services library',
          context: ErrorDescription('during method call ${call.method}'),
          informationCollector:
              () => <DiagnosticsNode>[
                DiagnosticsProperty<MethodCall>(
                  'call',
                  call,
                  style: DiagnosticsTreeStyle.errorProperty,
                ),
              ],
        ),
      );
      rethrow;
    }
  }

  Future<dynamic> _handleScribeInputInvocation(MethodCall methodCall) async {
    switch (methodCall.method) {
      case 'ScribeClient.performHandwritingGesture':
        assert(activeScribeClient != null);
        print('Justin ScribeClient.performHandwritingGesture with args ${methodCall.arguments}');
        // TODO(justinmc): Create enums and stuff to parse the argument.
        return activeScribeClient!.performHandwritingGesture();
      case 'ScribeClient.previewHandwritingGesture':
        assert(activeScribeClient != null);
        print('Justin ScribeClient.previewHandwritingGesture with args ${methodCall.arguments}');
        return activeScribeClient!.previewHandwritingGesture();
    }
  }
}

/// An interface into Android's stylus handwriting text input.

/// A mixin that to receive focus from the engine.
///
/// This is currently only used to handle UIIndirectScribbleInteraction.
mixin ScribeClient {
  bool get isActive;

  // TODO(justinmc): Double check about providing an implementation and breaking
  // changes. If you add a new method, will users break?
  /// Called when Android receives a handwriting gesture.
  ///
  /// See also:
  ///
  ///  * Android's
  ///    [InputConnection.performHandwritingGesture](https://developer.android.com/reference/android/view/inputmethod/InputConnection#performHandwritingGesture(android.view.inputmethod.HandwritingGesture,%20java.util.concurrent.Executor,%20java.util.function.IntConsumer))
  ///    method, from which the engine calls through to this method.
  Future<bool> performHandwritingGesture() {
    throw UnimplementedError();
  }

  void previewHandwritingGesture() {
    throw UnimplementedError();
  }

  // TODO(justinmc): Is there a cleaner way to adjust for the device pixel ratio?
  double get devicePixelRatio;
}
