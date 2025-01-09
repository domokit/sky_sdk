// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../dom.dart';

/// Various input action types used in text fields.
///
/// These types are coming from Flutter's [TextInputAction]. Currently, the web doesn't
/// support all the types. We fallback to [EngineInputAction.none] when Flutter
/// sends a type that isn't supported.
abstract class EngineInputAction {
  const EngineInputAction();

  static EngineInputAction fromName(String name) {
    switch (name) {
      case 'TextInputAction.continueAction':
      case 'TextInputAction.next':
        return next;
      case 'TextInputAction.previous':
        return previous;
      case 'TextInputAction.done':
        return done;
      case 'TextInputAction.go':
        return go;
      case 'TextInputAction.newline':
        return enter;
      case 'TextInputAction.search':
        return search;
      case 'TextInputAction.send':
        return send;
      case 'TextInputAction.emergencyCall':
      case 'TextInputAction.join':
      case 'TextInputAction.none':
      case 'TextInputAction.route':
      case 'TextInputAction.unspecified':
      default:
        return none;
    }
  }

  /// No input action
  static const NoInputAction none = NoInputAction();

  /// Action to go to next
  static const NextInputAction next = NextInputAction();

  /// Action to go to previous
  static const PreviousInputAction previous = PreviousInputAction();

  /// Action to be finished
  static const DoneInputAction done = DoneInputAction();

  /// Action to Go
  static const GoInputAction go = GoInputAction();

  /// Action to insert newline
  static const EnterInputAction enter = EnterInputAction();

  /// Action to search
  static const SearchInputAction search = SearchInputAction();

  /// Action to send
  static const SendInputAction send = SendInputAction();

  /// The HTML `enterkeyhint` attribute to be set on the DOM element.
  ///
  /// This HTML attribute helps the browser decide what kind of keyboard action
  /// to use for this text field
  ///
  /// For various `enterkeyhint` values supported by browsers, see:
  /// <https://developer.mozilla.org/en-US/docs/Web/HTML/Global_attributes/enterkeyhint>.
  String? get enterkeyhintAttribute;

  /// Given a [domElement], set attributes that are specific to this input action.
  void configureInputAction(DomHTMLElement domElement) {
    if (enterkeyhintAttribute == null) {
      return;
    }

    // Only apply `enterkeyhint` in mobile browsers so that the right virtual
    // keyboard shows up.
    if (ui_web.browser.operatingSystem == ui_web.OperatingSystem.iOs ||
        ui_web.browser.operatingSystem == ui_web.OperatingSystem.android ||
        enterkeyhintAttribute == EngineInputAction.none.enterkeyhintAttribute) {
      domElement.setAttribute('enterkeyhint', enterkeyhintAttribute!);
    }
  }
}

/// No action specified
class NoInputAction extends EngineInputAction {
  const NoInputAction();

  @override
  String? get enterkeyhintAttribute => null;
}

/// Typically inserting a new line.
class EnterInputAction extends EngineInputAction {
  const EnterInputAction();

  @override
  String? get enterkeyhintAttribute => 'enter';
}

/// Typically meaning there is nothing more to input and the input method editor (IME) will be closed.
class DoneInputAction extends EngineInputAction {
  const DoneInputAction();

  @override
  String? get enterkeyhintAttribute => 'done';
}

/// Typically meaning to take the user to the target of the text they typed.
class GoInputAction extends EngineInputAction {
  const GoInputAction();

  @override
  String? get enterkeyhintAttribute => 'go';
}

/// Typically taking the user to the next field that will accept text.
class NextInputAction extends EngineInputAction {
  const NextInputAction();

  @override
  String? get enterkeyhintAttribute => 'next';
}

/// Typically taking the user to the previous field that will accept text.
class PreviousInputAction extends EngineInputAction {
  const PreviousInputAction();

  @override
  String? get enterkeyhintAttribute => 'previous';
}

/// Typically taking the user to the results of searching for the text they have typed.
class SearchInputAction extends EngineInputAction {
  const SearchInputAction();

  @override
  String? get enterkeyhintAttribute => 'search';
}

/// Typically delivering the text to its target.
class SendInputAction extends EngineInputAction {
  const SendInputAction();

  @override
  String? get enterkeyhintAttribute => 'send';
}
