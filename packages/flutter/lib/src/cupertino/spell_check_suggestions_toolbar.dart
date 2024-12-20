// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart' show SelectionChangedCause, SuggestionSpan;
import 'package:flutter/widgets.dart';

import 'debug.dart';
import 'localizations.dart';
import 'text_selection_toolbar.dart';
import 'text_selection_toolbar_button.dart';

/// iOS only shows 3 spell check suggestions in the toolbar.
const int _kMaxSuggestions = 3;

/// The default spell check suggestions toolbar for iOS.
///
/// Tries to position itself below the [anchors], but if it doesn't fit, then it
/// readjusts to fit above bottom view insets.
///
/// See also:
///  * [SpellCheckSuggestionsToolbar], which is similar but for both the
///    Material and Cupertino libraries.
class CupertinoSpellCheckSuggestionsToolbar extends StatelessWidget {
  /// Constructs a [CupertinoSpellCheckSuggestionsToolbar].
  ///
  /// [buttonItems] must not contain more than three items.
  const CupertinoSpellCheckSuggestionsToolbar({
    super.key,
    required this.anchors,
    required this.buttonItems,
  }) : assert(buttonItems.length <= _kMaxSuggestions);

  /// Constructs a [CupertinoSpellCheckSuggestionsToolbar] with the default
  /// children for an [EditableText].
  ///
  /// See also:
  ///  * [SpellCheckSuggestionsToolbar.editableText], which is similar but
  ///    builds an Android-style toolbar.
  CupertinoSpellCheckSuggestionsToolbar.editableText({
    super.key,
    required EditableTextState editableTextState,
  }) : buttonItems = buildButtonItems(editableTextState) ?? <ContextMenuButtonItem>[],
       anchors = editableTextState.contextMenuAnchors;

  /// The location on which to anchor the menu.
  final TextSelectionToolbarAnchors anchors;

  /// The [ContextMenuButtonItem]s that will be turned into the correct button
  /// widgets and displayed in the spell check suggestions toolbar.
  ///
  /// Must not contain more than three items.
  ///
  /// See also:
  ///
  ///  * [AdaptiveTextSelectionToolbar.buttonItems], the list of
  ///    [ContextMenuButtonItem]s that are used to build the buttons of the
  ///    text selection toolbar.
  ///  * [SpellCheckSuggestionsToolbar.buttonItems], the list of
  ///    [ContextMenuButtonItem]s used to build the Material style spell check
  ///    suggestions toolbar.
  final List<ContextMenuButtonItem> buttonItems;

  /// Builds the button items for the toolbar based on the available
  /// spell check suggestions.
  static List<ContextMenuButtonItem>? buildButtonItems(EditableTextState editableTextState) {
    // Determine if composing region is misspelled.
    final SuggestionSpan? spanAtCursorIndex = editableTextState.findSuggestionSpanAtCursorIndex(
      editableTextState.currentTextEditingValue.selection.baseOffset,
    );

    if (spanAtCursorIndex == null) {
      return null;
    }
    if (spanAtCursorIndex.suggestions.isEmpty) {
      assert(debugCheckHasCupertinoLocalizations(editableTextState.context));
      final CupertinoLocalizations localizations = CupertinoLocalizations.of(
        editableTextState.context,
      );
      return <ContextMenuButtonItem>[
        ContextMenuButtonItem(onPressed: null, label: localizations.noSpellCheckReplacementsLabel),
      ];
    }

    final List<ContextMenuButtonItem> buttonItems = <ContextMenuButtonItem>[];

    // Build suggestion buttons.
    for (final String suggestion in spanAtCursorIndex.suggestions.take(_kMaxSuggestions)) {
      buttonItems.add(
        ContextMenuButtonItem(
          onPressed: () {
            if (!editableTextState.mounted) {
              return;
            }
            _replaceText(editableTextState, suggestion, spanAtCursorIndex.range);
          },
          label: suggestion,
        ),
      );
    }
    return buttonItems;
  }

  static void _replaceText(
    EditableTextState editableTextState,
    String text,
    TextRange replacementRange,
  ) {
    // Replacement cannot be performed if the text is read only or obscured.
    assert(!editableTextState.widget.readOnly && !editableTextState.widget.obscureText);

    final TextEditingValue newValue = editableTextState.textEditingValue
        .replaced(replacementRange, text)
        .copyWith(selection: TextSelection.collapsed(offset: replacementRange.start + text.length));
    editableTextState.userUpdateTextEditingValue(newValue, SelectionChangedCause.toolbar);

    // Schedule a call to bringIntoView() after renderEditable updates.
    SchedulerBinding.instance.addPostFrameCallback((Duration duration) {
      if (editableTextState.mounted) {
        editableTextState.bringIntoView(editableTextState.textEditingValue.selection.extent);
      }
    }, debugLabel: 'SpellCheckSuggestions.bringIntoView');
    editableTextState.hideToolbar();
  }

  /// Builds the toolbar buttons based on the [buttonItems].
  List<Widget> _buildToolbarButtons(BuildContext context) {
    return buttonItems.map((ContextMenuButtonItem buttonItem) {
      return CupertinoTextSelectionToolbarButton.buttonItem(buttonItem: buttonItem);
    }).toList();
  }

  @override
  Widget build(BuildContext context) {
    if (buttonItems.isEmpty) {
      return const SizedBox.shrink();
    }

    final List<Widget> children = _buildToolbarButtons(context);
    return CupertinoTextSelectionToolbar(
      anchorAbove: anchors.primaryAnchor,
      anchorBelow:
          anchors.secondaryAnchor == null ? anchors.primaryAnchor : anchors.secondaryAnchor!,
      children: children,
    );
  }
}
