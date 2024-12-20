// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart' show SelectionChangedCause, SuggestionSpan;

import 'adaptive_text_selection_toolbar.dart';
import 'colors.dart';
import 'material.dart';
import 'spell_check_suggestions_toolbar_layout_delegate.dart';
import 'text_selection_toolbar_text_button.dart';

// The default height of the SpellCheckSuggestionsToolbar, which
// assumes there are the maximum number of spell check suggestions available, 3.
// Size eyeballed on Pixel 4 emulator running Android API 31.
const double _kDefaultToolbarHeight = 193.0;

/// The maximum number of suggestions in the toolbar is 3, plus a delete button.
const int _kMaxSuggestions = 3;

/// The default spell check suggestions toolbar for Android.
///
/// Tries to position itself below the [anchor], but if it doesn't fit, then it
/// readjusts to fit above bottom view insets.
///
/// See also:
///
///  * [CupertinoSpellCheckSuggestionsToolbar], which is similar but builds an
///    iOS-style spell check toolbar.
class SpellCheckSuggestionsToolbar extends StatelessWidget {
  /// Constructs a [SpellCheckSuggestionsToolbar].
  ///
  /// [buttonItems] must not contain more than four items, generally three
  /// suggestions and one delete button.
  const SpellCheckSuggestionsToolbar({super.key, required this.anchor, required this.buttonItems})
    : assert(buttonItems.length <= _kMaxSuggestions + 1);

  /// Constructs a [SpellCheckSuggestionsToolbar] with the default children for
  /// an [EditableText].
  ///
  /// See also:
  ///  * [CupertinoSpellCheckSuggestionsToolbar.editableText], which is similar
  ///    but builds an iOS-style toolbar.
  SpellCheckSuggestionsToolbar.editableText({
    super.key,
    required EditableTextState editableTextState,
  }) : buttonItems = buildButtonItems(editableTextState) ?? <ContextMenuButtonItem>[],
       anchor = getToolbarAnchor(editableTextState.contextMenuAnchors);

  /// {@template flutter.material.SpellCheckSuggestionsToolbar.anchor}
  /// The focal point below which the toolbar attempts to position itself.
  /// {@endtemplate}
  final Offset anchor;

  /// The [ContextMenuButtonItem]s that will be turned into the correct button
  /// widgets and displayed in the spell check suggestions toolbar.
  ///
  /// Must not contain more than four items, typically three suggestions and a
  /// delete button.
  ///
  /// See also:
  ///
  ///  * [AdaptiveTextSelectionToolbar.buttonItems], the list of
  ///    [ContextMenuButtonItem]s that are used to build the buttons of the
  ///    text selection toolbar.
  ///  * [CupertinoSpellCheckSuggestionsToolbar.buttonItems], the list of
  ///    [ContextMenuButtonItem]s used to build the Cupertino style spell check
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

    // Build delete button.
    final ContextMenuButtonItem deleteButton = ContextMenuButtonItem(
      onPressed: () {
        if (!editableTextState.mounted) {
          return;
        }
        _replaceText(editableTextState, '', editableTextState.currentTextEditingValue.composing);
      },
      type: ContextMenuButtonType.delete,
    );
    buttonItems.add(deleteButton);

    return buttonItems;
  }

  static void _replaceText(
    EditableTextState editableTextState,
    String text,
    TextRange replacementRange,
  ) {
    // Replacement cannot be performed if the text is read only or obscured.
    assert(!editableTextState.widget.readOnly && !editableTextState.widget.obscureText);

    final TextEditingValue newValue = editableTextState.textEditingValue.replaced(
      replacementRange,
      text,
    );
    editableTextState.userUpdateTextEditingValue(newValue, SelectionChangedCause.toolbar);

    // Schedule a call to bringIntoView() after renderEditable updates.
    SchedulerBinding.instance.addPostFrameCallback((Duration duration) {
      if (editableTextState.mounted) {
        editableTextState.bringIntoView(editableTextState.textEditingValue.selection.extent);
      }
    }, debugLabel: 'SpellCheckerSuggestionsToolbar.bringIntoView');
    editableTextState.hideToolbar();
  }

  /// Determines the Offset that the toolbar will be anchored to.
  static Offset getToolbarAnchor(TextSelectionToolbarAnchors anchors) {
    // Since this will be positioned below the anchor point, use the secondary
    // anchor by default.
    return anchors.secondaryAnchor == null ? anchors.primaryAnchor : anchors.secondaryAnchor!;
  }

  /// Builds the toolbar buttons based on the [buttonItems].
  List<Widget> _buildToolbarButtons(BuildContext context) {
    return buttonItems.map((ContextMenuButtonItem buttonItem) {
      final TextSelectionToolbarTextButton button = TextSelectionToolbarTextButton(
        padding: const EdgeInsets.fromLTRB(20, 0, 0, 0),
        onPressed: buttonItem.onPressed,
        alignment: Alignment.centerLeft,
        child: Text(
          AdaptiveTextSelectionToolbar.getButtonLabel(context, buttonItem),
          style:
              buttonItem.type == ContextMenuButtonType.delete
                  ? const TextStyle(color: Colors.blue)
                  : null,
        ),
      );

      if (buttonItem.type != ContextMenuButtonType.delete) {
        return button;
      }
      return DecoratedBox(
        decoration: const BoxDecoration(border: Border(top: BorderSide(color: Colors.grey))),
        child: button,
      );
    }).toList();
  }

  @override
  Widget build(BuildContext context) {
    if (buttonItems.isEmpty) {
      return const SizedBox.shrink();
    }

    // Adjust toolbar height if needed.
    final double spellCheckSuggestionsToolbarHeight =
        _kDefaultToolbarHeight - (48.0 * (4 - buttonItems.length));
    // Incorporate the padding distance between the content and toolbar.
    final MediaQueryData mediaQueryData = MediaQuery.of(context);
    final double softKeyboardViewInsetsBottom = mediaQueryData.viewInsets.bottom;
    final double paddingAbove =
        mediaQueryData.padding.top + CupertinoTextSelectionToolbar.kToolbarScreenPadding;
    // Makes up for the Padding.
    final Offset localAdjustment = Offset(
      CupertinoTextSelectionToolbar.kToolbarScreenPadding,
      paddingAbove,
    );

    return Padding(
      padding: EdgeInsets.fromLTRB(
        CupertinoTextSelectionToolbar.kToolbarScreenPadding,
        paddingAbove,
        CupertinoTextSelectionToolbar.kToolbarScreenPadding,
        CupertinoTextSelectionToolbar.kToolbarScreenPadding + softKeyboardViewInsetsBottom,
      ),
      child: CustomSingleChildLayout(
        delegate: SpellCheckSuggestionsToolbarLayoutDelegate(anchor: anchor - localAdjustment),
        child: AnimatedSize(
          // This duration was eyeballed on a Pixel 2 emulator running Android
          // API 28 for the Material TextSelectionToolbar.
          duration: const Duration(milliseconds: 140),
          child: _SpellCheckSuggestionsToolbarContainer(
            height: spellCheckSuggestionsToolbarHeight,
            children: <Widget>[..._buildToolbarButtons(context)],
          ),
        ),
      ),
    );
  }
}

/// The Material-styled toolbar outline for the spell check suggestions
/// toolbar.
class _SpellCheckSuggestionsToolbarContainer extends StatelessWidget {
  const _SpellCheckSuggestionsToolbarContainer({required this.height, required this.children});

  final double height;
  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    return Material(
      // This elevation was eyeballed on a Pixel 4 emulator running Android
      // API 31 for the SpellCheckSuggestionsToolbar.
      elevation: 2.0,
      type: MaterialType.card,
      child: SizedBox(
        // This width was eyeballed on a Pixel 4 emulator running Android
        // API 31 for the SpellCheckSuggestionsToolbar.
        width: 165.0,
        height: height,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: children,
        ),
      ),
    );
  }
}
