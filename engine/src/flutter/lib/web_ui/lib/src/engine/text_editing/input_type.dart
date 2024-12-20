// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../dom.dart';

/// Various types of inputs used in text fields.
///
/// These types are coming from Flutter's [TextInputType]. Currently, we don't
/// support all the types. We fallback to [EngineInputType.text] when Flutter
/// sends a type that isn't supported.
// TODO(mdebbar): Support more types.
abstract class EngineInputType {
  const EngineInputType();

  static EngineInputType fromName(String name, {bool isDecimal = false, bool isMultiline = false}) {
    switch (name) {
      case 'TextInputType.number':
        return isDecimal ? decimal : number;
      case 'TextInputType.phone':
        return phone;
      case 'TextInputType.emailAddress':
        return emailAddress;
      case 'TextInputType.url':
        return url;
      case 'TextInputType.multiline':
        return multiline;
      case 'TextInputType.none':
        return isMultiline ? multilineNone : none;
      case 'TextInputType.text':
      default:
        return text;
    }
  }

  /// No text input.
  static const NoTextInputType none = NoTextInputType();

  /// Multi-line no text input.
  static const MultilineNoTextInputType multilineNone = MultilineNoTextInputType();

  /// Single-line text input type.
  static const TextInputType text = TextInputType();

  /// Numeric input type.
  static const NumberInputType number = NumberInputType();

  /// Decimal input type.
  static const DecimalInputType decimal = DecimalInputType();

  /// Phone number input type.
  static const PhoneInputType phone = PhoneInputType();

  /// Email address input type.
  static const EmailInputType emailAddress = EmailInputType();

  /// URL input type.
  static const UrlInputType url = UrlInputType();

  /// Multi-line text input type.
  static const MultilineInputType multiline = MultilineInputType();

  /// The HTML `inputmode` attribute to be set on the DOM element.
  ///
  /// This HTML attribute helps the browser decide what kind of keyboard works
  /// best for this text field.
  ///
  /// For various `inputmode` values supported by browsers, see:
  /// <https://developer.mozilla.org/en-US/docs/Web/HTML/Global_attributes/inputmode>.
  String? get inputmodeAttribute;

  /// Create the appropriate DOM element for this input type.
  DomHTMLElement createDomElement() => createDomHTMLInputElement();

  /// Given a [domElement], set attributes that are specific to this input type.
  void configureInputMode(DomHTMLElement domElement) {
    if (inputmodeAttribute == null) {
      return;
    }

    // Only apply `inputmode` in mobile browsers so that the right virtual
    // keyboard shows up.
    if (ui_web.browser.operatingSystem == ui_web.OperatingSystem.iOs ||
        ui_web.browser.operatingSystem == ui_web.OperatingSystem.android ||
        inputmodeAttribute == EngineInputType.none.inputmodeAttribute) {
      domElement.setAttribute('inputmode', inputmodeAttribute!);
    }
  }
}

/// No text input.
class NoTextInputType extends EngineInputType {
  const NoTextInputType();

  @override
  String get inputmodeAttribute => 'none';
}

/// See: https://github.com/flutter/flutter/issues/125875
/// Multi-line no text input from system virtual keyboard.
///
/// Use this for inputting multiple lines with a customized keyboard.
///
/// When Flutter uses a custom virtual keyboard, it sends [TextInputType.none]
/// with a [isMultiline] flag to block the system virtual keyboard.
///
/// For [MultilineNoTextInputType] (mapped to [TextInputType.none] with
/// [isMultiline] = true), it creates a <textarea> element with the
/// inputmode="none" attribute.
///
/// For [NoTextInputType] (mapped to [TextInputType.none] with
/// [isMultiline] = false), it creates an <input> element with the
/// inputmode="none" attribute.
class MultilineNoTextInputType extends MultilineInputType {
  const MultilineNoTextInputType();

  @override
  String? get inputmodeAttribute => 'none';

  @override
  DomHTMLElement createDomElement() => createDomHTMLTextAreaElement();
}

/// Single-line text input type.
class TextInputType extends EngineInputType {
  const TextInputType();

  @override
  String? get inputmodeAttribute => null;
}

/// Numeric input type.
///
/// Input keyboard with only the digits 0–9.
class NumberInputType extends EngineInputType {
  const NumberInputType();

  @override
  String get inputmodeAttribute => 'numeric';
}

/// Decimal input type.
///
/// Input keyboard with containing the digits 0–9 and a decimal separator.
/// Separator can be `.`, `,` depending on the locale.
class DecimalInputType extends EngineInputType {
  const DecimalInputType();

  @override
  String get inputmodeAttribute => 'decimal';
}

/// Phone number input type.
class PhoneInputType extends EngineInputType {
  const PhoneInputType();

  @override
  String get inputmodeAttribute => 'tel';
}

/// Email address input type.
class EmailInputType extends EngineInputType {
  const EmailInputType();

  @override
  String get inputmodeAttribute => 'email';
}

/// URL input type.
class UrlInputType extends EngineInputType {
  const UrlInputType();

  @override
  String get inputmodeAttribute => 'url';
}

/// Multi-line text input type.
class MultilineInputType extends EngineInputType {
  const MultilineInputType();

  @override
  String? get inputmodeAttribute => null;

  @override
  DomHTMLElement createDomElement() => createDomHTMLTextAreaElement();
}
