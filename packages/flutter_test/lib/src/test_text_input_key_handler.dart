// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'binding.dart';

/// Processes text input events that were not handled by the framework.
abstract class TestTextInputKeyHandler {
  /// Process key down event that was not handled by the framework.
  Future<void> handleKeyDownEvent(LogicalKeyboardKey key);

  /// Process key up event that was not handled by the framework.
  Future<void> handleKeyUpEvent(LogicalKeyboardKey key);
}

/// MacOS specific key input handler. This class translates standard macOS text editing shortcuts
/// into appropriate selectors similarly to what NSTextInputContext does in Flutter Engine.
class MacOSTestTextInputKeyHandler extends TestTextInputKeyHandler {
  /// Create a new macOS specific text input handler.
  MacOSTestTextInputKeyHandler(this.client);

  /// ClientId of TextInput
  final int client;

  Future<void> _sendSelectors(List<String> selectors) async {
    await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
      SystemChannels.textInput.name,
      SystemChannels.textInput.codec.encodeMethodCall(
        MethodCall('TextInputClient.performSelectors', <dynamic>[client, selectors]),
      ),
      (ByteData? data) {
        /* response from framework is discarded */
      },
    );
  }

  // These combinations must match NSStandardKeyBindingResponding.
  static final Map<SingleActivator, List<String>>
  _macOSActivatorToSelectors = <SingleActivator, List<String>>{
    for (final bool pressShift in const <bool>[true, false]) ...<SingleActivator, List<String>>{
      SingleActivator(LogicalKeyboardKey.backspace, shift: pressShift): <String>['deleteBackward:'],
      SingleActivator(LogicalKeyboardKey.backspace, alt: true, shift: pressShift): <String>[
        'deleteWordBackward:',
      ],
      SingleActivator(LogicalKeyboardKey.backspace, meta: true, shift: pressShift): <String>[
        'deleteToBeginningOfLine:',
      ],
      SingleActivator(LogicalKeyboardKey.backspace, control: true, shift: pressShift): <String>[
        'deleteBackwardByDecomposingPreviousCharacter:',
      ],
      SingleActivator(LogicalKeyboardKey.delete, shift: pressShift): <String>['deleteForward:'],
      SingleActivator(LogicalKeyboardKey.delete, alt: true, shift: pressShift): <String>[
        'deleteWordForward:',
      ],
      SingleActivator(LogicalKeyboardKey.delete, meta: true, shift: pressShift): <String>[
        'deleteToEndOfLine:',
      ],
    },
    const SingleActivator(LogicalKeyboardKey.arrowLeft): <String>['moveLeft:'],
    const SingleActivator(LogicalKeyboardKey.arrowRight): <String>['moveRight:'],
    const SingleActivator(LogicalKeyboardKey.arrowUp): <String>['moveUp:'],
    const SingleActivator(LogicalKeyboardKey.arrowDown): <String>['moveDown:'],
    const SingleActivator(LogicalKeyboardKey.arrowLeft, shift: true): <String>[
      'moveLeftAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowRight, shift: true): <String>[
      'moveRightAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowUp, shift: true): <String>[
      'moveUpAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowDown, shift: true): <String>[
      'moveDownAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowLeft, alt: true): <String>['moveWordLeft:'],
    const SingleActivator(LogicalKeyboardKey.arrowRight, alt: true): <String>['moveWordRight:'],
    const SingleActivator(LogicalKeyboardKey.arrowUp, alt: true): <String>[
      'moveBackward:',
      'moveToBeginningOfParagraph:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowDown, alt: true): <String>[
      'moveForward:',
      'moveToEndOfParagraph:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowLeft, alt: true, shift: true): <String>[
      'moveWordLeftAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowRight, alt: true, shift: true): <String>[
      'moveWordRightAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowUp, alt: true, shift: true): <String>[
      'moveParagraphBackwardAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowDown, alt: true, shift: true): <String>[
      'moveParagraphForwardAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowLeft, meta: true): <String>[
      'moveToLeftEndOfLine:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowRight, meta: true): <String>[
      'moveToRightEndOfLine:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowUp, meta: true): <String>[
      'moveToBeginningOfDocument:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowDown, meta: true): <String>[
      'moveToEndOfDocument:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowLeft, meta: true, shift: true): <String>[
      'moveToLeftEndOfLineAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowRight, meta: true, shift: true): <String>[
      'moveToRightEndOfLineAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowUp, meta: true, shift: true): <String>[
      'moveToBeginningOfDocumentAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.arrowDown, meta: true, shift: true): <String>[
      'moveToEndOfDocumentAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.keyA, control: true, shift: true): <String>[
      'moveToBeginningOfParagraphAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.keyA, control: true): <String>[
      'moveToBeginningOfParagraph:',
    ],
    const SingleActivator(LogicalKeyboardKey.keyB, control: true, shift: true): <String>[
      'moveBackwardAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.keyB, control: true): <String>['moveBackward:'],
    const SingleActivator(LogicalKeyboardKey.keyE, control: true, shift: true): <String>[
      'moveToEndOfParagraphAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.keyE, control: true): <String>[
      'moveToEndOfParagraph:',
    ],
    const SingleActivator(LogicalKeyboardKey.keyF, control: true, shift: true): <String>[
      'moveForwardAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.keyF, control: true): <String>['moveForward:'],
    const SingleActivator(LogicalKeyboardKey.keyK, control: true): <String>[
      'deleteToEndOfParagraph',
    ],
    const SingleActivator(LogicalKeyboardKey.keyL, control: true): <String>[
      'centerSelectionInVisibleArea',
    ],
    const SingleActivator(LogicalKeyboardKey.keyN, control: true): <String>['moveDown:'],
    const SingleActivator(LogicalKeyboardKey.keyN, control: true, shift: true): <String>[
      'moveDownAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.keyO, control: true): <String>[
      'insertNewlineIgnoringFieldEditor:',
    ],
    const SingleActivator(LogicalKeyboardKey.keyP, control: true): <String>['moveUp:'],
    const SingleActivator(LogicalKeyboardKey.keyP, control: true, shift: true): <String>[
      'moveUpAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.keyT, control: true): <String>['transpose:'],
    const SingleActivator(LogicalKeyboardKey.keyV, control: true): <String>['pageDown:'],
    const SingleActivator(LogicalKeyboardKey.keyV, control: true, shift: true): <String>[
      'pageDownAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.keyY, control: true): <String>['yank:'],
    const SingleActivator(LogicalKeyboardKey.quoteSingle, control: true): <String>[
      'insertSingleQuoteIgnoringSubstitution:',
    ],
    const SingleActivator(LogicalKeyboardKey.quote, control: true): <String>[
      'insertDoubleQuoteIgnoringSubstitution:',
    ],
    const SingleActivator(LogicalKeyboardKey.home): <String>['scrollToBeginningOfDocument:'],
    const SingleActivator(LogicalKeyboardKey.end): <String>['scrollToEndOfDocument:'],
    const SingleActivator(LogicalKeyboardKey.home, shift: true): <String>[
      'moveToBeginningOfDocumentAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.end, shift: true): <String>[
      'moveToEndOfDocumentAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.pageUp): <String>['scrollPageUp:'],
    const SingleActivator(LogicalKeyboardKey.pageDown): <String>['scrollPageDown:'],
    const SingleActivator(LogicalKeyboardKey.pageUp, shift: true): <String>[
      'pageUpAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.pageDown, shift: true): <String>[
      'pageDownAndModifySelection:',
    ],
    const SingleActivator(LogicalKeyboardKey.escape): <String>['cancelOperation:'],
    const SingleActivator(LogicalKeyboardKey.enter): <String>['insertNewline:'],
    const SingleActivator(LogicalKeyboardKey.enter, alt: true): <String>[
      'insertNewlineIgnoringFieldEditor:',
    ],
    const SingleActivator(LogicalKeyboardKey.enter, control: true): <String>['insertLineBreak:'],
    const SingleActivator(LogicalKeyboardKey.tab): <String>['insertTab:'],
    const SingleActivator(LogicalKeyboardKey.tab, alt: true): <String>[
      'insertTabIgnoringFieldEditor:',
    ],
    const SingleActivator(LogicalKeyboardKey.tab, shift: true): <String>['insertBacktab:'],
  };

  @override
  Future<void> handleKeyDownEvent(LogicalKeyboardKey key) async {
    switch (key) {
      case LogicalKeyboardKey.shift:
      case LogicalKeyboardKey.shiftLeft:
      case LogicalKeyboardKey.shiftRight:
        _shift = true;
      case LogicalKeyboardKey.alt:
      case LogicalKeyboardKey.altLeft:
      case LogicalKeyboardKey.altRight:
        _alt = true;
      case LogicalKeyboardKey.meta:
      case LogicalKeyboardKey.metaLeft:
      case LogicalKeyboardKey.metaRight:
        _meta = true;
      case LogicalKeyboardKey.control:
      case LogicalKeyboardKey.controlLeft:
      case LogicalKeyboardKey.controlRight:
        _control = true;
      default:
        for (final MapEntry<SingleActivator, List<String>> entry
            in _macOSActivatorToSelectors.entries) {
          final SingleActivator activator = entry.key;
          if (activator.triggers.first == key &&
              activator.shift == _shift &&
              activator.alt == _alt &&
              activator.meta == _meta &&
              activator.control == _control) {
            await _sendSelectors(entry.value);
            return;
          }
        }
    }
  }

  @override
  Future<void> handleKeyUpEvent(LogicalKeyboardKey key) async {
    switch (key) {
      case LogicalKeyboardKey.shift:
      case LogicalKeyboardKey.shiftLeft:
      case LogicalKeyboardKey.shiftRight:
        _shift = false;
      case LogicalKeyboardKey.alt:
      case LogicalKeyboardKey.altLeft:
      case LogicalKeyboardKey.altRight:
        _alt = false;
      case LogicalKeyboardKey.meta:
      case LogicalKeyboardKey.metaLeft:
      case LogicalKeyboardKey.metaRight:
        _meta = false;
      case LogicalKeyboardKey.control:
      case LogicalKeyboardKey.controlLeft:
      case LogicalKeyboardKey.controlRight:
        _control = false;
    }
  }

  bool _shift = false;
  bool _alt = false;
  bool _meta = false;
  bool _control = false;
}
