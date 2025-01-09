// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import 'dom.dart';
import 'services.dart';

/// Handles clipboard related platform messages.
class ClipboardMessageHandler {
  /// Helper to handle copy to clipboard functionality.
  CopyToClipboardStrategy _copyToClipboardStrategy = CopyToClipboardStrategy();

  /// Helper to handle copy to clipboard functionality.
  PasteFromClipboardStrategy _pasteFromClipboardStrategy = PasteFromClipboardStrategy();

  /// Handles the platform message which stores the given text to the clipboard.
  void setDataMethodCall(MethodCall methodCall, ui.PlatformMessageResponseCallback? callback) {
    const MethodCodec codec = JSONMethodCodec();
    bool errorEnvelopeEncoded = false;
    _copyToClipboardStrategy
        .setData((methodCall.arguments as Map<String, Object?>)['text'] as String?)
        .then((bool success) {
          if (success) {
            callback!(codec.encodeSuccessEnvelope(true));
          } else {
            callback!(
              codec.encodeErrorEnvelope(code: 'copy_fail', message: 'Clipboard.setData failed'),
            );
            errorEnvelopeEncoded = true;
          }
        })
        .catchError((dynamic _) {
          // Don't encode a duplicate reply if we already failed and an error
          // was already encoded.
          if (!errorEnvelopeEncoded) {
            callback!(
              codec.encodeErrorEnvelope(code: 'copy_fail', message: 'Clipboard.setData failed'),
            );
          }
        });
  }

  /// Handles the platform message which retrieves text data from the clipboard.
  void getDataMethodCall(ui.PlatformMessageResponseCallback? callback) {
    const MethodCodec codec = JSONMethodCodec();
    _pasteFromClipboardStrategy
        .getData()
        .then((String data) {
          final Map<String, dynamic> map = <String, dynamic>{'text': data};
          callback!(codec.encodeSuccessEnvelope(map));
        })
        .catchError((dynamic error) {
          if (error is UnimplementedError) {
            // Clipboard.getData not supported.
            // Passing [null] to [callback] indicates that the platform message isn't
            // implemented. Look at [MethodChannel.invokeMethod] to see how [null] is
            // handled.
            Future<void>.delayed(Duration.zero).then((_) {
              if (callback != null) {
                callback(null);
              }
            });
            return;
          }
          _reportGetDataFailure(callback, codec, error);
        });
  }

  /// Handles the platform message which asks if the clipboard contains
  /// pasteable strings.
  void hasStringsMethodCall(ui.PlatformMessageResponseCallback? callback) {
    const MethodCodec codec = JSONMethodCodec();
    _pasteFromClipboardStrategy
        .getData()
        .then((String data) {
          final Map<String, dynamic> map = <String, dynamic>{'value': data.isNotEmpty};
          callback!(codec.encodeSuccessEnvelope(map));
        })
        .catchError((dynamic error) {
          if (error is UnimplementedError) {
            // Clipboard.hasStrings not supported.
            // Passing [null] to [callback] indicates that the platform message isn't
            // implemented. Look at [MethodChannel.invokeMethod] to see how [null] is
            // handled.
            Future<void>.delayed(Duration.zero).then((_) {
              if (callback != null) {
                callback(null);
              }
            });
            return;
          }
          final Map<String, dynamic> map = <String, dynamic>{'value': false};
          callback!(codec.encodeSuccessEnvelope(map));
        });
  }

  void _reportGetDataFailure(
    ui.PlatformMessageResponseCallback? callback,
    MethodCodec codec,
    dynamic error,
  ) {
    print('Could not get text from clipboard: $error');
    callback!(codec.encodeErrorEnvelope(code: 'paste_fail', message: 'Clipboard.getData failed'));
  }

  /// Methods used by tests.
  set pasteFromClipboardStrategy(PasteFromClipboardStrategy strategy) {
    _pasteFromClipboardStrategy = strategy;
  }

  set copyToClipboardStrategy(CopyToClipboardStrategy strategy) {
    _copyToClipboardStrategy = strategy;
  }
}

/// Provides functionality for writing text to clipboard.
///
/// A concrete implementation is picked at runtime based on the available
/// APIs and the browser.
abstract class CopyToClipboardStrategy {
  factory CopyToClipboardStrategy() {
    return domWindow.navigator.clipboard != null
        ? ClipboardAPICopyStrategy()
        : ExecCommandCopyStrategy();
  }

  /// Places the text onto the browser Clipboard.
  ///
  /// Returns `true` for a successful action.
  ///
  /// Returns `false` for an uncessful action or when there is an excaption.
  Future<bool> setData(String? text);
}

/// Provides functionality for reading text from clipboard.
///
/// A concrete implementation is picked at runtime based on the available
/// APIs and the browser.
abstract class PasteFromClipboardStrategy {
  factory PasteFromClipboardStrategy() {
    return (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox ||
            domWindow.navigator.clipboard == null)
        ? ExecCommandPasteStrategy()
        : ClipboardAPIPasteStrategy();
  }

  /// Returns text from the system Clipboard.
  Future<String> getData();
}

/// Provides copy functionality for browsers which supports ClipboardAPI.
///
/// Works on Chrome and Firefox browsers.
/// See: https://developer.mozilla.org/en-US/docs/Web/API/Clipboard_API
class ClipboardAPICopyStrategy implements CopyToClipboardStrategy {
  @override
  Future<bool> setData(String? text) async {
    try {
      await domWindow.navigator.clipboard!.writeText(text!);
    } catch (error) {
      print('copy is not successful $error');
      return Future<bool>.value(false);
    }
    return Future<bool>.value(true);
  }
}

/// Provides paste functionality for browsers which supports `clipboard.readText`.
///
/// Works on Chrome. Firefox only supports `readText` if the target element is
/// in content editable mode.
/// See: https://developer.mozilla.org/en-US/docs/Web/Guide/HTML/Editable_content
/// See: https://developer.mozilla.org/en-US/docs/Web/API/Clipboard_API
class ClipboardAPIPasteStrategy implements PasteFromClipboardStrategy {
  @override
  Future<String> getData() async {
    return domWindow.navigator.clipboard!.readText();
  }
}

/// Provides a fallback strategy for browsers which does not support ClipboardAPI.
class ExecCommandCopyStrategy implements CopyToClipboardStrategy {
  @override
  Future<bool> setData(String? text) {
    return Future<bool>.value(_setDataSync(text));
  }

  bool _setDataSync(String? text) {
    // Copy content to clipboard with execCommand.
    // See: https://developers.google.com/web/updates/2015/04/cut-and-copy-commands
    final DomHTMLTextAreaElement tempTextArea = _appendTemporaryTextArea();
    tempTextArea.value = text;
    tempTextArea.focusWithoutScroll();
    tempTextArea.select();
    bool result = false;
    try {
      result = domDocument.execCommand('copy');
      if (!result) {
        print('copy is not successful');
      }
    } catch (error) {
      print('copy is not successful $error');
    } finally {
      _removeTemporaryTextArea(tempTextArea);
    }
    return result;
  }

  DomHTMLTextAreaElement _appendTemporaryTextArea() {
    final DomHTMLTextAreaElement tempElement = createDomHTMLTextAreaElement();
    final DomCSSStyleDeclaration elementStyle = tempElement.style;
    elementStyle
      ..position = 'absolute'
      ..top = '-99999px'
      ..left = '-99999px'
      ..opacity = '0'
      ..color = 'transparent'
      ..backgroundColor = 'transparent'
      ..background = 'transparent';

    domDocument.body!.append(tempElement);

    return tempElement;
  }

  void _removeTemporaryTextArea(DomHTMLElement element) {
    element.remove();
  }
}

/// Provides a fallback strategy for browsers which does not support ClipboardAPI.
class ExecCommandPasteStrategy implements PasteFromClipboardStrategy {
  @override
  Future<String> getData() {
    // TODO(mdebbar): https://github.com/flutter/flutter/issues/48581
    return Future<String>.error(UnimplementedError('Paste is not implemented for this browser.'));
  }
}
