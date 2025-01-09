// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'app.dart';
/// @docImport 'context_menu_controller.dart';
/// @docImport 'form.dart';
/// @docImport 'restoration.dart';
/// @docImport 'restoration_properties.dart';
/// @docImport 'selectable_region.dart';
/// @docImport 'text_selection_toolbar_layout_delegate.dart';
library;

import 'dart:async';
import 'dart:math' as math;
import 'dart:ui' as ui hide TextStyle;

import 'package:characters/characters.dart' show CharacterRange, StringCharacters;
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

import 'actions.dart';
import 'app_lifecycle_listener.dart';
import 'autofill.dart';
import 'automatic_keep_alive.dart';
import 'basic.dart';
import 'binding.dart';
import 'constants.dart';
import 'context_menu_button_item.dart';
import 'debug.dart';
import 'default_selection_style.dart';
import 'default_text_editing_shortcuts.dart';
import 'focus_manager.dart';
import 'focus_scope.dart';
import 'focus_traversal.dart';
import 'framework.dart';
import 'localizations.dart';
import 'magnifier.dart';
import 'media_query.dart';
import 'notification_listener.dart';
import 'scroll_configuration.dart';
import 'scroll_controller.dart';
import 'scroll_notification.dart';
import 'scroll_notification_observer.dart';
import 'scroll_physics.dart';
import 'scroll_position.dart';
import 'scrollable.dart';
import 'scrollable_helpers.dart';
import 'shortcuts.dart';
import 'size_changed_layout_notifier.dart';
import 'spell_check.dart';
import 'tap_region.dart';
import 'text.dart';
import 'text_editing_intents.dart';
import 'text_selection.dart';
import 'text_selection_toolbar_anchors.dart';
import 'ticker_provider.dart';
import 'undo_history.dart';
import 'view.dart';
import 'widget_span.dart';

export 'package:flutter/services.dart'
    show
        KeyboardInsertedContent,
        SelectionChangedCause,
        SmartDashesType,
        SmartQuotesType,
        TextEditingValue,
        TextInputType,
        TextSelection;

// Examples can assume:
// late BuildContext context;
// late WidgetTester tester;

/// Signature for the callback that reports when the user changes the selection
/// (including the cursor location).
typedef SelectionChangedCallback =
    void Function(TextSelection selection, SelectionChangedCause? cause);

/// Signature for the callback that reports the app private command results.
typedef AppPrivateCommandCallback = void Function(String action, Map<String, dynamic> data);

/// Signature for a widget builder that builds a context menu for the given
/// [EditableTextState].
///
/// See also:
///
///  * [SelectableRegionContextMenuBuilder], which performs the same role for
///    [SelectableRegion].
typedef EditableTextContextMenuBuilder =
    Widget Function(BuildContext context, EditableTextState editableTextState);

// Signature for a function that determines the target location of the given
// [TextPosition] after applying the given [TextBoundary].
typedef _ApplyTextBoundary = TextPosition Function(TextPosition, bool, TextBoundary);

// The time it takes for the cursor to fade from fully opaque to fully
// transparent and vice versa. A full cursor blink, from transparent to opaque
// to transparent, is twice this duration.
const Duration _kCursorBlinkHalfPeriod = Duration(milliseconds: 500);

// Number of cursor ticks during which the most recently entered character
// is shown in an obscured text field.
const int _kObscureShowLatestCharCursorTicks = 3;

/// The default mime types to be used when allowedMimeTypes is not provided.
///
/// The default value supports inserting images of any supported format.
const List<String> kDefaultContentInsertionMimeTypes = <String>[
  'image/png',
  'image/bmp',
  'image/jpg',
  'image/tiff',
  'image/gif',
  'image/jpeg',
  'image/webp',
];

class _CompositionCallback extends SingleChildRenderObjectWidget {
  const _CompositionCallback({required this.compositeCallback, required this.enabled, super.child});
  final CompositionCallback compositeCallback;
  final bool enabled;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderCompositionCallback(compositeCallback, enabled);
  }

  @override
  void updateRenderObject(BuildContext context, _RenderCompositionCallback renderObject) {
    super.updateRenderObject(context, renderObject);
    // _EditableTextState always uses the same callback.
    assert(renderObject.compositeCallback == compositeCallback);
    renderObject.enabled = enabled;
  }
}

class _RenderCompositionCallback extends RenderProxyBox {
  _RenderCompositionCallback(this.compositeCallback, this._enabled);

  final CompositionCallback compositeCallback;
  VoidCallback? _cancelCallback;

  bool get enabled => _enabled;
  bool _enabled = false;
  set enabled(bool newValue) {
    _enabled = newValue;
    if (!newValue) {
      _cancelCallback?.call();
      _cancelCallback = null;
    } else if (_cancelCallback == null) {
      markNeedsPaint();
    }
  }

  @override
  void paint(PaintingContext context, ui.Offset offset) {
    if (enabled) {
      _cancelCallback ??= context.addCompositionCallback(compositeCallback);
    }
    super.paint(context, offset);
  }
}

/// A controller for an editable text field.
///
/// Whenever the user modifies a text field with an associated
/// [TextEditingController], the text field updates [value] and the controller
/// notifies its listeners. Listeners can then read the [text] and [selection]
/// properties to learn what the user has typed or how the selection has been
/// updated.
///
/// Similarly, if you modify the [text] or [selection] properties, the text
/// field will be notified and will update itself appropriately.
///
/// A [TextEditingController] can also be used to provide an initial value for a
/// text field. If you build a text field with a controller that already has
/// [text], the text field will use that text as its initial value.
///
/// The [value] (as well as [text] and [selection]) of this controller can be
/// updated from within a listener added to this controller. Be aware of
/// infinite loops since the listener will also be notified of the changes made
/// from within itself. Modifying the composing region from within a listener
/// can also have a bad interaction with some input methods. Gboard, for
/// example, will try to restore the composing region of the text if it was
/// modified programmatically, creating an infinite loop of communications
/// between the framework and the input method. Consider using
/// [TextInputFormatter]s instead for as-you-type text modification.
///
/// If both the [text] and [selection] properties need to be changed, set the
/// controller's [value] instead. Setting [text] will clear the selection
/// and composing range.
///
/// Remember to [dispose] of the [TextEditingController] when it is no longer
/// needed. This will ensure we discard any resources used by the object.
///
/// {@tool dartpad}
/// This example creates a [TextField] with a [TextEditingController] whose
/// change listener forces the entered text to be lower case and keeps the
/// cursor at the end of the input.
///
/// ** See code in examples/api/lib/widgets/editable_text/text_editing_controller.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [TextField], which is a Material Design text field that can be controlled
///    with a [TextEditingController].
///  * [EditableText], which is a raw region of editable text that can be
///    controlled with a [TextEditingController].
///  * Learn how to use a [TextEditingController] in one of our [cookbook recipes](https://docs.flutter.dev/cookbook/forms/text-field-changes#2-use-a-texteditingcontroller).
class TextEditingController extends ValueNotifier<TextEditingValue> {
  /// Creates a controller for an editable text field, with no initial selection.
  ///
  /// This constructor treats a null [text] argument as if it were the empty
  /// string.
  ///
  /// The initial selection is `TextSelection.collapsed(offset: -1)`.
  /// This indicates that there is no selection at all ([TextSelection.isValid]
  /// is false in this case). When a text field is built with a controller whose
  /// selection is not valid, the text field will update the selection when it
  /// is focused (the selection will be an empty selection positioned at the
  /// end of the text).
  ///
  /// Consider using [TextEditingController.fromValue] to initialize both the
  /// text and the selection.
  ///
  /// {@tool dartpad}
  /// This example creates a [TextField] with a [TextEditingController] whose
  /// initial selection is empty (collapsed) and positioned at the beginning
  /// of the text (offset is 0).
  ///
  /// ** See code in examples/api/lib/widgets/editable_text/text_editing_controller.1.dart **
  /// {@end-tool}
  TextEditingController({String? text})
    : super(text == null ? TextEditingValue.empty : TextEditingValue(text: text));

  /// Creates a controller for an editable text field from an initial [TextEditingValue].
  ///
  /// This constructor treats a null [value] argument as if it were
  /// [TextEditingValue.empty].
  TextEditingController.fromValue(TextEditingValue? value)
    : assert(
        value == null || !value.composing.isValid || value.isComposingRangeValid,
        'New TextEditingValue $value has an invalid non-empty composing range '
        '${value.composing}. It is recommended to use a valid composing range, '
        'even for readonly text fields.',
      ),
      super(value ?? TextEditingValue.empty);

  /// The current string the user is editing.
  String get text => value.text;

  /// Setting this will notify all the listeners of this [TextEditingController]
  /// that they need to update (it calls [notifyListeners]). For this reason,
  /// this value should only be set between frames, e.g. in response to user
  /// actions, not during the build, layout, or paint phases.
  ///
  /// This property can be set from a listener added to this
  /// [TextEditingController]; **however, one should not also set [selection]
  /// in a separate statement. To change both the [text] and the [selection]
  /// change the controller's [value].** Setting this here will clear
  /// the current selection and composing range, so avoid using it directly
  /// unless that is the desired behavior.
  set text(String newText) {
    value = value.copyWith(
      text: newText,
      selection: const TextSelection.collapsed(offset: -1),
      composing: TextRange.empty,
    );
  }

  @override
  set value(TextEditingValue newValue) {
    assert(
      !newValue.composing.isValid || newValue.isComposingRangeValid,
      'New TextEditingValue $newValue has an invalid non-empty composing range '
      '${newValue.composing}. It is recommended to use a valid composing range, '
      'even for readonly text fields.',
    );
    super.value = newValue;
  }

  /// Builds [TextSpan] from current editing value.
  ///
  /// By default makes text in composing range appear as underlined. Descendants
  /// can override this method to customize appearance of text.
  TextSpan buildTextSpan({
    required BuildContext context,
    TextStyle? style,
    required bool withComposing,
  }) {
    assert(!value.composing.isValid || !withComposing || value.isComposingRangeValid);
    // If the composing range is out of range for the current text, ignore it to
    // preserve the tree integrity, otherwise in release mode a RangeError will
    // be thrown and this EditableText will be built with a broken subtree.
    final bool composingRegionOutOfRange = !value.isComposingRangeValid || !withComposing;

    if (composingRegionOutOfRange) {
      return TextSpan(style: style, text: text);
    }

    final TextStyle composingStyle =
        style?.merge(const TextStyle(decoration: TextDecoration.underline)) ??
        const TextStyle(decoration: TextDecoration.underline);
    return TextSpan(
      style: style,
      children: <TextSpan>[
        TextSpan(text: value.composing.textBefore(value.text)),
        TextSpan(style: composingStyle, text: value.composing.textInside(value.text)),
        TextSpan(text: value.composing.textAfter(value.text)),
      ],
    );
  }

  /// The currently selected range within [text].
  ///
  /// If the selection is collapsed, then this property gives the offset of the
  /// cursor within the text.
  TextSelection get selection => value.selection;

  /// Setting this will notify all the listeners of this [TextEditingController]
  /// that they need to update (it calls [notifyListeners]). For this reason,
  /// this value should only be set between frames, e.g. in response to user
  /// actions, not during the build, layout, or paint phases.
  ///
  /// This property can be set from a listener added to this
  /// [TextEditingController]; however, one should not also set [text]
  /// in a separate statement. To change both the [text] and the [selection]
  /// change the controller's [value].
  ///
  /// If the new selection is outside the composing range, the composing range is
  /// cleared.
  set selection(TextSelection newSelection) {
    if (text.length < newSelection.end || text.length < newSelection.start) {
      throw FlutterError('invalid text selection: $newSelection');
    }
    final TextRange newComposing =
        _isSelectionWithinComposingRange(newSelection) ? value.composing : TextRange.empty;
    value = value.copyWith(selection: newSelection, composing: newComposing);
  }

  /// Set the [value] to empty.
  ///
  /// After calling this function, [text] will be the empty string and the
  /// selection will be collapsed at zero offset.
  ///
  /// Calling this will notify all the listeners of this [TextEditingController]
  /// that they need to update (it calls [notifyListeners]). For this reason,
  /// this method should only be called between frames, e.g. in response to user
  /// actions, not during the build, layout, or paint phases.
  void clear() {
    value = const TextEditingValue(selection: TextSelection.collapsed(offset: 0));
  }

  /// Set the composing region to an empty range.
  ///
  /// The composing region is the range of text that is still being composed.
  /// Calling this function indicates that the user is done composing that
  /// region.
  ///
  /// Calling this will notify all the listeners of this [TextEditingController]
  /// that they need to update (it calls [notifyListeners]). For this reason,
  /// this method should only be called between frames, e.g. in response to user
  /// actions, not during the build, layout, or paint phases.
  void clearComposing() {
    value = value.copyWith(composing: TextRange.empty);
  }

  /// Check that the [selection] is inside of the composing range.
  bool _isSelectionWithinComposingRange(TextSelection selection) {
    return selection.start >= value.composing.start && selection.end <= value.composing.end;
  }
}

/// Toolbar configuration for [EditableText].
///
/// Toolbar is a context menu that will show up when user right click or long
/// press the [EditableText]. It includes several options: cut, copy, paste,
/// and select all.
///
/// [EditableText] and its derived widgets have their own default [ToolbarOptions].
/// Create a custom [ToolbarOptions] if you want explicit control over the toolbar
/// option.
@Deprecated(
  'Use `contextMenuBuilder` instead. '
  'This feature was deprecated after v3.3.0-0.5.pre.',
)
class ToolbarOptions {
  /// Create a toolbar configuration with given options.
  ///
  /// All options default to false if they are not explicitly set.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  const ToolbarOptions({
    this.copy = false,
    this.cut = false,
    this.paste = false,
    this.selectAll = false,
  });

  /// An instance of [ToolbarOptions] with no options enabled.
  static const ToolbarOptions empty = ToolbarOptions();

  /// Whether to show copy option in toolbar.
  ///
  /// Defaults to false.
  final bool copy;

  /// Whether to show cut option in toolbar.
  ///
  /// If [EditableText.readOnly] is set to true, cut will be disabled regardless.
  ///
  /// Defaults to false.
  final bool cut;

  /// Whether to show paste option in toolbar.
  ///
  /// If [EditableText.readOnly] is set to true, paste will be disabled regardless.
  ///
  /// Defaults to false.
  final bool paste;

  /// Whether to show select all option in toolbar.
  ///
  /// Defaults to false.
  final bool selectAll;
}

/// Configures the ability to insert media content through the soft keyboard.
///
/// The configuration provides a handler for any rich content inserted through
/// the system input method, and also provides the ability to limit the mime
/// types of the inserted content.
///
/// See also:
///
/// * [EditableText.contentInsertionConfiguration]
class ContentInsertionConfiguration {
  /// Creates a content insertion configuration with the specified options.
  ///
  /// A handler for inserted content, in the form of [onContentInserted], must
  /// be supplied.
  ///
  /// The allowable mime types of inserted content may also
  /// be provided via [allowedMimeTypes], which cannot be an empty list.
  ContentInsertionConfiguration({
    required this.onContentInserted,
    this.allowedMimeTypes = kDefaultContentInsertionMimeTypes,
  }) : assert(allowedMimeTypes.isNotEmpty);

  /// Called when a user inserts content through the virtual / on-screen keyboard,
  /// currently only used on Android.
  ///
  /// [KeyboardInsertedContent] holds the data representing the inserted content.
  ///
  /// {@tool dartpad}
  ///
  /// This example shows how to access the data for inserted content in your
  /// `TextField`.
  ///
  /// ** See code in examples/api/lib/widgets/editable_text/editable_text.on_content_inserted.0.dart **
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * <https://developer.android.com/guide/topics/text/image-keyboard>
  final ValueChanged<KeyboardInsertedContent> onContentInserted;

  /// {@template flutter.widgets.contentInsertionConfiguration.allowedMimeTypes}
  /// Used when a user inserts image-based content through the device keyboard,
  /// currently only used on Android.
  ///
  /// The passed list of strings will determine which MIME types are allowed to
  /// be inserted via the device keyboard.
  ///
  /// The default mime types are given by [kDefaultContentInsertionMimeTypes].
  /// These are all the mime types that are able to be handled and inserted
  /// from keyboards.
  ///
  /// This field cannot be an empty list.
  ///
  /// {@tool dartpad}
  /// This example shows how to limit image insertion to specific file types.
  ///
  /// ** See code in examples/api/lib/widgets/editable_text/editable_text.on_content_inserted.0.dart **
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * <https://developer.android.com/guide/topics/text/image-keyboard>
  /// {@endtemplate}
  final List<String> allowedMimeTypes;
}

// A time-value pair that represents a key frame in an animation.
class _KeyFrame {
  const _KeyFrame(this.time, this.value);
  // Values extracted from iOS 15.4 UIKit.
  static const List<_KeyFrame> iOSBlinkingCaretKeyFrames = <_KeyFrame>[
    _KeyFrame(0, 1), // 0
    _KeyFrame(0.5, 1), // 1
    _KeyFrame(0.5375, 0.75), // 2
    _KeyFrame(0.575, 0.5), // 3
    _KeyFrame(0.6125, 0.25), // 4
    _KeyFrame(0.65, 0), // 5
    _KeyFrame(0.85, 0), // 6
    _KeyFrame(0.8875, 0.25), // 7
    _KeyFrame(0.925, 0.5), // 8
    _KeyFrame(0.9625, 0.75), // 9
    _KeyFrame(1, 1), // 10
  ];

  // The timing, in seconds, of the specified animation `value`.
  final double time;
  final double value;
}

class _DiscreteKeyFrameSimulation extends Simulation {
  _DiscreteKeyFrameSimulation.iOSBlinkingCaret() : this._(_KeyFrame.iOSBlinkingCaretKeyFrames, 1);
  _DiscreteKeyFrameSimulation._(this._keyFrames, this.maxDuration)
    : assert(_keyFrames.isNotEmpty),
      assert(_keyFrames.last.time <= maxDuration),
      assert(() {
        for (int i = 0; i < _keyFrames.length - 1; i += 1) {
          if (_keyFrames[i].time > _keyFrames[i + 1].time) {
            return false;
          }
        }
        return true;
      }(), 'The key frame sequence must be sorted by time.');

  final double maxDuration;

  final List<_KeyFrame> _keyFrames;

  @override
  double dx(double time) => 0;

  @override
  bool isDone(double time) => time >= maxDuration;

  // The index of the KeyFrame corresponds to the most recent input `time`.
  int _lastKeyFrameIndex = 0;

  @override
  double x(double time) {
    final int length = _keyFrames.length;

    // Perform a linear search in the sorted key frame list, starting from the
    // last key frame found, since the input `time` usually monotonically
    // increases by a small amount.
    int searchIndex;
    final int endIndex;
    if (_keyFrames[_lastKeyFrameIndex].time > time) {
      // The simulation may have restarted. Search within the index range
      // [0, _lastKeyFrameIndex).
      searchIndex = 0;
      endIndex = _lastKeyFrameIndex;
    } else {
      searchIndex = _lastKeyFrameIndex;
      endIndex = length;
    }

    // Find the target key frame. Don't have to check (endIndex - 1): if
    // (endIndex - 2) doesn't work we'll have to pick (endIndex - 1) anyways.
    while (searchIndex < endIndex - 1) {
      assert(_keyFrames[searchIndex].time <= time);
      final _KeyFrame next = _keyFrames[searchIndex + 1];
      if (time < next.time) {
        break;
      }
      searchIndex += 1;
    }

    _lastKeyFrameIndex = searchIndex;
    return _keyFrames[_lastKeyFrameIndex].value;
  }
}

/// A basic text input field.
///
/// This widget interacts with the [TextInput] service to let the user edit the
/// text it contains. It also provides scrolling, selection, and cursor
/// movement.
///
/// The [EditableText] widget is a low-level widget that is intended as a
/// building block for custom widget sets. For a complete user experience,
/// consider using a [TextField] or [CupertinoTextField].
///
/// ## Handling User Input
///
/// Currently the user may change the text this widget contains via keyboard or
/// the text selection menu. When the user inserted or deleted text, you will be
/// notified of the change and get a chance to modify the new text value:
///
/// * The [inputFormatters] will be first applied to the user input.
///
/// * The [controller]'s [TextEditingController.value] will be updated with the
///   formatted result, and the [controller]'s listeners will be notified.
///
/// * The [onChanged] callback, if specified, will be called last.
///
/// ## Input Actions
///
/// A [TextInputAction] can be provided to customize the appearance of the
/// action button on the soft keyboard for Android and iOS. The default action
/// is [TextInputAction.done].
///
/// Many [TextInputAction]s are common between Android and iOS. However, if a
/// [textInputAction] is provided that is not supported by the current
/// platform in debug mode, an error will be thrown when the corresponding
/// EditableText receives focus. For example, providing iOS's "emergencyCall"
/// action when running on an Android device will result in an error when in
/// debug mode. In release mode, incompatible [TextInputAction]s are replaced
/// either with "unspecified" on Android, or "default" on iOS. Appropriate
/// [textInputAction]s can be chosen by checking the current platform and then
/// selecting the appropriate action.
///
/// {@template flutter.widgets.EditableText.lifeCycle}
/// ## Lifecycle
///
/// Upon completion of editing, like pressing the "done" button on the keyboard,
/// two actions take place:
///
///   1st: Editing is finalized. The default behavior of this step includes
///   an invocation of [onChanged]. That default behavior can be overridden.
///   See [onEditingComplete] for details.
///
///   2nd: [onSubmitted] is invoked with the user's input value.
///
/// [onSubmitted] can be used to manually move focus to another input widget
/// when a user finishes with the currently focused input widget.
///
/// When the widget has focus, it will prevent itself from disposing via
/// [AutomaticKeepAliveClientMixin.wantKeepAlive] in order to avoid losing the
/// selection. Removing the focus will allow it to be disposed.
/// {@endtemplate}
///
/// Rather than using this widget directly, consider using [TextField], which
/// is a full-featured, material-design text input field with placeholder text,
/// labels, and [Form] integration.
///
/// ## Text Editing [Intent]s and Their Default [Action]s
///
/// This widget provides default [Action]s for handling common text editing
/// [Intent]s such as deleting, copying and pasting in the text field. These
/// [Action]s can be directly invoked using [Actions.invoke] or the
/// [Actions.maybeInvoke] method. The default text editing keyboard [Shortcuts],
/// typically declared in [DefaultTextEditingShortcuts], also use these
/// [Intent]s and [Action]s to perform the text editing operations they are
/// bound to.
///
/// The default handling of a specific [Intent] can be overridden by placing an
/// [Actions] widget above this widget. See the [Action] class and the
/// [Action.overridable] constructor for more information on how a pre-defined
/// overridable [Action] can be overridden.
///
/// ### Intents for Deleting Text and Their Default Behavior
///
/// | **Intent Class**                 | **Default Behavior when there's selected text**      | **Default Behavior when there is a [caret](https://en.wikipedia.org/wiki/Caret_navigation) (The selection is [TextSelection.collapsed])**  |
/// | :------------------------------- | :--------------------------------------------------- | :----------------------------------------------------------------------- |
/// | [DeleteCharacterIntent]          | Deletes the selected text                            | Deletes the user-perceived character before or after the caret location. |
/// | [DeleteToNextWordBoundaryIntent] | Deletes the selected text and the word before/after the selection's [TextSelection.extent] position | Deletes from the caret location to the previous or the next word boundary |
/// | [DeleteToLineBreakIntent]        | Deletes the selected text, and deletes to the start/end of the line from the selection's [TextSelection.extent] position | Deletes from the caret location to the logical start or end of the current line |
///
/// ### Intents for Moving the [Caret](https://en.wikipedia.org/wiki/Caret_navigation)
///
/// | **Intent Class**                                                                     | **Default Behavior when there's selected text**                  | **Default Behavior when there is a caret ([TextSelection.collapsed])**  |
/// | :----------------------------------------------------------------------------------- | :--------------------------------------------------------------- | :---------------------------------------------------------------------- |
/// | [ExtendSelectionByCharacterIntent](`collapseSelection: true`)                        | Collapses the selection to the logical start/end of the selection | Moves the caret past the user-perceived character before or after the current caret location. |
/// | [ExtendSelectionToNextWordBoundaryIntent](`collapseSelection: true`)                 | Collapses the selection to the word boundary before/after the selection's [TextSelection.extent] position | Moves the caret to the previous/next word boundary. |
/// | [ExtendSelectionToNextWordBoundaryOrCaretLocationIntent](`collapseSelection: true`)  | Collapses the selection to the word boundary before/after the selection's [TextSelection.extent] position, or [TextSelection.base], whichever is closest in the given direction | Moves the caret to the previous/next word boundary. |
/// | [ExtendSelectionToLineBreakIntent](`collapseSelection: true`)                        | Collapses the selection to the start/end of the line at the selection's [TextSelection.extent] position | Moves the caret to the start/end of the current line .|
/// | [ExtendSelectionVerticallyToAdjacentLineIntent](`collapseSelection: true`)           | Collapses the selection to the position closest to the selection's [TextSelection.extent], on the previous/next adjacent line | Moves the caret to the closest position on the previous/next adjacent line. |
/// | [ExtendSelectionVerticallyToAdjacentPageIntent](`collapseSelection: true`)           | Collapses the selection to the position closest to the selection's [TextSelection.extent], on the previous/next adjacent page | Moves the caret to the closest position on the previous/next adjacent page. |
/// | [ExtendSelectionToDocumentBoundaryIntent](`collapseSelection: true`)                 | Collapses the selection to the start/end of the document | Moves the caret to the start/end of the document. |
///
/// #### Intents for Extending the Selection
///
/// | **Intent Class**                                                                     | **Default Behavior when there's selected text**                  | **Default Behavior when there is a caret ([TextSelection.collapsed])**  |
/// | :----------------------------------------------------------------------------------- | :--------------------------------------------------------------- | :---------------------------------------------------------------------- |
/// | [ExtendSelectionByCharacterIntent](`collapseSelection: false`)                       | Moves the selection's [TextSelection.extent] past the user-perceived character before/after it |
/// | [ExtendSelectionToNextWordBoundaryIntent](`collapseSelection: false`)                | Moves the selection's [TextSelection.extent] to the previous/next word boundary |
/// | [ExtendSelectionToNextWordBoundaryOrCaretLocationIntent](`collapseSelection: false`) | Moves the selection's [TextSelection.extent] to the previous/next word boundary, or [TextSelection.base] whichever is closest in the given direction | Moves the selection's [TextSelection.extent] to the previous/next word boundary. |
/// | [ExtendSelectionToLineBreakIntent](`collapseSelection: false`)                       | Moves the selection's [TextSelection.extent] to the start/end of the line |
/// | [ExtendSelectionVerticallyToAdjacentLineIntent](`collapseSelection: false`)          | Moves the selection's [TextSelection.extent] to the closest position on the previous/next adjacent line |
/// | [ExtendSelectionVerticallyToAdjacentPageIntent](`collapseSelection: false`)          | Moves the selection's [TextSelection.extent] to the closest position on the previous/next adjacent page |
/// | [ExtendSelectionToDocumentBoundaryIntent](`collapseSelection: false`)                | Moves the selection's [TextSelection.extent] to the start/end of the document |
/// | [SelectAllTextIntent]  | Selects the entire document |
///
/// ### Other Intents
///
/// | **Intent Class**                        | **Default Behavior**                                 |
/// | :-------------------------------------- | :--------------------------------------------------- |
/// | [DoNothingAndStopPropagationTextIntent] | Does nothing in the input field, and prevents the key event from further propagating in the widget tree. |
/// | [ReplaceTextIntent]                     | Replaces the current [TextEditingValue] in the input field's [TextEditingController], and triggers all related user callbacks and [TextInputFormatter]s. |
/// | [UpdateSelectionIntent]                 | Updates the current selection in the input field's [TextEditingController], and triggers the [onSelectionChanged] callback. |
/// | [CopySelectionTextIntent]               | Copies or cuts the selected text into the clipboard |
/// | [PasteTextIntent]                       | Inserts the current text in the clipboard after the caret location, or replaces the selected text if the selection is not collapsed. |
///
/// ## Text Editing [Shortcuts]
///
/// It's also possible to directly remap keyboard shortcuts to new [Intent]s by
/// inserting a [Shortcuts] widget above this in the widget tree. When using
/// [WidgetsApp], the large set of default text editing keyboard shortcuts are
/// declared near the top of the widget tree in [DefaultTextEditingShortcuts],
/// and any [Shortcuts] widget between it and this [EditableText] will override
/// those defaults.
///
/// {@template flutter.widgets.editableText.shortcutsAndTextInput}
/// ### Interactions Between [Shortcuts] and Text Input
///
/// Shortcuts prevent text input fields from receiving their keystrokes as text
/// input. For example, placing a [Shortcuts] widget in the widget tree above
/// a text input field and creating a shortcut for [LogicalKeyboardKey.keyA]
/// will prevent the field from receiving that key as text input. In other
/// words, typing key "A" into the field will trigger the shortcut and will not
/// insert a letter "a" into the field.
///
/// This happens because of the way that key strokes are handled in Flutter.
/// When a keystroke is received in Flutter's engine, it first gives the
/// framework the opportunity to handle it as a raw key event through
/// [SystemChannels.keyEvent]. This is what [Shortcuts] listens to indirectly
/// through its [FocusNode]. If it is not handled, then it will proceed to try
/// handling it as text input through [SystemChannels.textInput], which is what
/// [EditableTextState] listens to through [TextInputClient].
///
/// This behavior, where a shortcut prevents text input into some field, can be
/// overridden by using another [Shortcuts] widget lower in the widget tree and
/// mapping the desired key stroke(s) to [DoNothingAndStopPropagationIntent].
/// The key event will be reported as unhandled by the framework and will then
/// be sent as text input as usual.
/// {@endtemplate}
///
/// ## Gesture Events Handling
///
/// When [rendererIgnoresPointer] is false (the default), this widget provides
/// rudimentary, platform-agnostic gesture handling for user actions such as
/// tapping, long-pressing, and scrolling.
///
/// To provide more complete gesture handling, including double-click to select
/// a word, drag selection, and platform-specific handling of gestures such as
/// long presses, consider setting [rendererIgnoresPointer] to true and using
/// [TextSelectionGestureDetectorBuilder].
///
/// {@template flutter.widgets.editableText.showCaretOnScreen}
/// ## Keep the caret visible when focused
///
/// When focused, this widget will make attempts to keep the text area and its
/// caret (even when [showCursor] is `false`) visible, on these occasions:
///
///  * When the user focuses this text field and it is not [readOnly].
///  * When the user changes the selection of the text field, or changes the
///    text when the text field is not [readOnly].
///  * When the virtual keyboard pops up.
/// {@endtemplate}
///
/// ## Scrolling Considerations
///
/// If this [EditableText] is not a descendant of [Scaffold] and is being used
/// within a [Scrollable] or nested [Scrollable]s, consider placing a
/// [ScrollNotificationObserver] above the root [Scrollable] that contains this
/// [EditableText] to ensure proper scroll coordination for [EditableText] and
/// its components like [TextSelectionOverlay].
///
/// {@template flutter.widgets.editableText.accessibility}
/// ## Troubleshooting Common Accessibility Issues
///
/// ### Customizing User Input Accessibility Announcements
///
/// To customize user input accessibility announcements triggered by text
/// changes, use [SemanticsService.announce] to make the desired
/// accessibility announcement.
///
/// On iOS, the on-screen keyboard may announce the most recent input
/// incorrectly when a [TextInputFormatter] inserts a thousands separator to
/// a currency value text field. The following example demonstrates how to
/// suppress the default accessibility announcements by always announcing
/// the content of the text field as a US currency value (the `\$` inserts
/// a dollar sign, the `$newText` interpolates the `newText` variable):
///
/// ```dart
/// onChanged: (String newText) {
///   if (newText.isNotEmpty) {
///     SemanticsService.announce('\$$newText', Directionality.of(context));
///   }
/// }
/// ```
///
/// {@endtemplate}
///
/// See also:
///
///  * [TextField], which is a full-featured, material-design text input field
///    with placeholder text, labels, and [Form] integration.
class EditableText extends StatefulWidget {
  /// Creates a basic text input control.
  ///
  /// The [maxLines] property can be set to null to remove the restriction on
  /// the number of lines. By default, it is one, meaning this is a single-line
  /// text field. [maxLines] must be null or greater than zero.
  ///
  /// If [keyboardType] is not set or is null, its value will be inferred from
  /// [autofillHints], if [autofillHints] is not empty. Otherwise it defaults to
  /// [TextInputType.text] if [maxLines] is exactly one, and
  /// [TextInputType.multiline] if [maxLines] is null or greater than one.
  ///
  /// The text cursor is not shown if [showCursor] is false or if [showCursor]
  /// is null (the default) and [readOnly] is true.
  EditableText({
    super.key,
    required this.controller,
    required this.focusNode,
    this.readOnly = false,
    this.obscuringCharacter = '•',
    this.obscureText = false,
    this.autocorrect = true,
    SmartDashesType? smartDashesType,
    SmartQuotesType? smartQuotesType,
    this.enableSuggestions = true,
    required this.style,
    StrutStyle? strutStyle,
    required this.cursorColor,
    required this.backgroundCursorColor,
    this.textAlign = TextAlign.start,
    this.textDirection,
    this.locale,
    @Deprecated(
      'Use textScaler instead. '
      'Use of textScaleFactor was deprecated in preparation for the upcoming nonlinear text scaling support. '
      'This feature was deprecated after v3.12.0-2.0.pre.',
    )
    this.textScaleFactor,
    this.textScaler,
    this.maxLines = 1,
    this.minLines,
    this.expands = false,
    this.forceLine = true,
    this.textHeightBehavior,
    this.textWidthBasis = TextWidthBasis.parent,
    this.autofocus = false,
    bool? showCursor,
    this.showSelectionHandles = false,
    this.selectionColor,
    this.selectionControls,
    TextInputType? keyboardType,
    this.textInputAction,
    this.textCapitalization = TextCapitalization.none,
    this.onChanged,
    this.onEditingComplete,
    this.onSubmitted,
    this.onAppPrivateCommand,
    this.onSelectionChanged,
    this.onSelectionHandleTapped,
    this.groupId = EditableText,
    this.onTapOutside,
    this.onTapUpOutside,
    List<TextInputFormatter>? inputFormatters,
    this.mouseCursor,
    this.rendererIgnoresPointer = false,
    this.cursorWidth = 2.0,
    this.cursorHeight,
    this.cursorRadius,
    this.cursorOpacityAnimates = false,
    this.cursorOffset,
    this.paintCursorAboveText = false,
    this.selectionHeightStyle = ui.BoxHeightStyle.tight,
    this.selectionWidthStyle = ui.BoxWidthStyle.tight,
    this.scrollPadding = const EdgeInsets.all(20.0),
    this.keyboardAppearance = Brightness.light,
    this.dragStartBehavior = DragStartBehavior.start,
    bool? enableInteractiveSelection,
    this.scrollController,
    this.scrollPhysics,
    this.autocorrectionTextRectColor,
    @Deprecated(
      'Use `contextMenuBuilder` instead. '
      'This feature was deprecated after v3.3.0-0.5.pre.',
    )
    ToolbarOptions? toolbarOptions,
    this.autofillHints = const <String>[],
    this.autofillClient,
    this.clipBehavior = Clip.hardEdge,
    this.restorationId,
    this.scrollBehavior,
    @Deprecated(
      'Use `stylusHandwritingEnabled` instead. '
      'This feature was deprecated after v3.27.0-0.2.pre.',
    )
    this.scribbleEnabled = true,
    this.stylusHandwritingEnabled = defaultStylusHandwritingEnabled,
    this.enableIMEPersonalizedLearning = true,
    this.contentInsertionConfiguration,
    this.contextMenuBuilder,
    this.spellCheckConfiguration,
    this.magnifierConfiguration = TextMagnifierConfiguration.disabled,
    this.undoController,
  }) : assert(obscuringCharacter.length == 1),
       smartDashesType =
           smartDashesType ?? (obscureText ? SmartDashesType.disabled : SmartDashesType.enabled),
       smartQuotesType =
           smartQuotesType ?? (obscureText ? SmartQuotesType.disabled : SmartQuotesType.enabled),
       assert(minLines == null || minLines > 0),
       assert(
         (maxLines == null) || (minLines == null) || (maxLines >= minLines),
         "minLines can't be greater than maxLines",
       ),
       assert(
         !expands || (maxLines == null && minLines == null),
         'minLines and maxLines must be null when expands is true.',
       ),
       assert(!obscureText || maxLines == 1, 'Obscured fields cannot be multiline.'),
       enableInteractiveSelection = enableInteractiveSelection ?? (!readOnly || !obscureText),
       toolbarOptions =
           selectionControls is TextSelectionHandleControls && toolbarOptions == null
               ? ToolbarOptions.empty
               : toolbarOptions ??
                   (obscureText
                       ? (readOnly
                           // No point in even offering "Select All" in a read-only obscured
                           // field.
                           ? ToolbarOptions.empty
                           // Writable, but obscured.
                           : const ToolbarOptions(selectAll: true, paste: true))
                       : (readOnly
                           // Read-only, not obscured.
                           ? const ToolbarOptions(selectAll: true, copy: true)
                           // Writable, not obscured.
                           : const ToolbarOptions(
                             copy: true,
                             cut: true,
                             selectAll: true,
                             paste: true,
                           ))),
       assert(
         spellCheckConfiguration == null ||
             spellCheckConfiguration == const SpellCheckConfiguration.disabled() ||
             spellCheckConfiguration.misspelledTextStyle != null,
         'spellCheckConfiguration must specify a misspelledTextStyle if spell check behavior is desired',
       ),
       _strutStyle = strutStyle,
       keyboardType =
           keyboardType ?? _inferKeyboardType(autofillHints: autofillHints, maxLines: maxLines),
       inputFormatters =
           maxLines == 1
               ? <TextInputFormatter>[
                 FilteringTextInputFormatter.singleLineFormatter,
                 ...inputFormatters ?? const Iterable<TextInputFormatter>.empty(),
               ]
               : inputFormatters,
       showCursor = showCursor ?? !readOnly;

  /// Controls the text being edited.
  final TextEditingController controller;

  /// Controls whether this widget has keyboard focus.
  final FocusNode focusNode;

  /// {@template flutter.widgets.editableText.obscuringCharacter}
  /// Character used for obscuring text if [obscureText] is true.
  ///
  /// Must be only a single character.
  ///
  /// Defaults to the character U+2022 BULLET (•).
  /// {@endtemplate}
  final String obscuringCharacter;

  /// {@template flutter.widgets.editableText.obscureText}
  /// Whether to hide the text being edited (e.g., for passwords).
  ///
  /// When this is set to true, all the characters in the text field are
  /// replaced by [obscuringCharacter], and the text in the field cannot be
  /// copied with copy or cut. If [readOnly] is also true, then the text cannot
  /// be selected.
  ///
  /// Defaults to false.
  /// {@endtemplate}
  final bool obscureText;

  /// {@macro dart.ui.textHeightBehavior}
  final TextHeightBehavior? textHeightBehavior;

  /// {@macro flutter.painting.textPainter.textWidthBasis}
  final TextWidthBasis textWidthBasis;

  /// {@template flutter.widgets.editableText.readOnly}
  /// Whether the text can be changed.
  ///
  /// When this is set to true, the text cannot be modified
  /// by any shortcut or keyboard operation. The text is still selectable.
  ///
  /// Defaults to false.
  /// {@endtemplate}
  final bool readOnly;

  /// Whether the text will take the full width regardless of the text width.
  ///
  /// When this is set to false, the width will be based on text width, which
  /// will also be affected by [textWidthBasis].
  ///
  /// Defaults to true.
  ///
  /// See also:
  ///
  ///  * [textWidthBasis], which controls the calculation of text width.
  final bool forceLine;

  /// Configuration of toolbar options.
  ///
  /// By default, all options are enabled. If [readOnly] is true, paste and cut
  /// will be disabled regardless. If [obscureText] is true, cut and copy will
  /// be disabled regardless. If [readOnly] and [obscureText] are both true,
  /// select all will also be disabled.
  final ToolbarOptions toolbarOptions;

  /// Whether to show selection handles.
  ///
  /// When a selection is active, there will be two handles at each side of
  /// boundary, or one handle if the selection is collapsed. The handles can be
  /// dragged to adjust the selection.
  ///
  /// See also:
  ///
  ///  * [showCursor], which controls the visibility of the cursor.
  final bool showSelectionHandles;

  /// {@template flutter.widgets.editableText.showCursor}
  /// Whether to show cursor.
  ///
  /// The cursor refers to the blinking caret when the [EditableText] is focused.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [showSelectionHandles], which controls the visibility of the selection handles.
  final bool showCursor;

  /// {@template flutter.widgets.editableText.autocorrect}
  /// Whether to enable autocorrection.
  ///
  /// Defaults to true.
  /// {@endtemplate}
  final bool autocorrect;

  /// {@macro flutter.services.TextInputConfiguration.smartDashesType}
  final SmartDashesType smartDashesType;

  /// {@macro flutter.services.TextInputConfiguration.smartQuotesType}
  final SmartQuotesType smartQuotesType;

  /// {@macro flutter.services.TextInputConfiguration.enableSuggestions}
  final bool enableSuggestions;

  /// The text style to use for the editable text.
  final TextStyle style;

  /// Controls the undo state of the current editable text.
  ///
  /// If null, this widget will create its own [UndoHistoryController].
  final UndoHistoryController? undoController;

  /// {@template flutter.widgets.editableText.strutStyle}
  /// The strut style used for the vertical layout.
  ///
  /// [StrutStyle] is used to establish a predictable vertical layout.
  /// Since fonts may vary depending on user input and due to font
  /// fallback, [StrutStyle.forceStrutHeight] is enabled by default
  /// to lock all lines to the height of the base [TextStyle], provided by
  /// [style]. This ensures the typed text fits within the allotted space.
  ///
  /// If null, the strut used will inherit values from the [style] and will
  /// have [StrutStyle.forceStrutHeight] set to true. When no [style] is
  /// passed, the theme's [TextStyle] will be used to generate [strutStyle]
  /// instead.
  ///
  /// To disable strut-based vertical alignment and allow dynamic vertical
  /// layout based on the glyphs typed, use [StrutStyle.disabled].
  ///
  /// Flutter's strut is based on [typesetting strut](https://en.wikipedia.org/wiki/Strut_(typesetting))
  /// and CSS's [line-height](https://www.w3.org/TR/CSS2/visudet.html#line-height).
  /// {@endtemplate}
  ///
  /// Within editable text and text fields, [StrutStyle] will not use its standalone
  /// default values, and will instead inherit omitted/null properties from the
  /// [TextStyle] instead. See [StrutStyle.inheritFromTextStyle].
  StrutStyle get strutStyle {
    if (_strutStyle == null) {
      return StrutStyle.fromTextStyle(style, forceStrutHeight: true);
    }
    return _strutStyle.inheritFromTextStyle(style);
  }

  final StrutStyle? _strutStyle;

  /// {@template flutter.widgets.editableText.textAlign}
  /// How the text should be aligned horizontally.
  ///
  /// Defaults to [TextAlign.start].
  /// {@endtemplate}
  final TextAlign textAlign;

  /// {@template flutter.widgets.editableText.textDirection}
  /// The directionality of the text.
  ///
  /// This decides how [textAlign] values like [TextAlign.start] and
  /// [TextAlign.end] are interpreted.
  ///
  /// This is also used to disambiguate how to render bidirectional text. For
  /// example, if the text is an English phrase followed by a Hebrew phrase,
  /// in a [TextDirection.ltr] context the English phrase will be on the left
  /// and the Hebrew phrase to its right, while in a [TextDirection.rtl]
  /// context, the English phrase will be on the right and the Hebrew phrase on
  /// its left.
  ///
  /// Defaults to the ambient [Directionality], if any.
  /// {@endtemplate}
  final TextDirection? textDirection;

  /// {@template flutter.widgets.editableText.textCapitalization}
  /// Configures how the platform keyboard will select an uppercase or
  /// lowercase keyboard.
  ///
  /// Only supports text keyboards, other keyboard types will ignore this
  /// configuration. Capitalization is locale-aware.
  ///
  /// Defaults to [TextCapitalization.none].
  ///
  /// See also:
  ///
  ///  * [TextCapitalization], for a description of each capitalization behavior.
  ///
  /// {@endtemplate}
  final TextCapitalization textCapitalization;

  /// Used to select a font when the same Unicode character can
  /// be rendered differently, depending on the locale.
  ///
  /// It's rarely necessary to set this property. By default its value
  /// is inherited from the enclosing app with `Localizations.localeOf(context)`.
  ///
  /// See [RenderEditable.locale] for more information.
  final Locale? locale;

  /// {@template flutter.widgets.editableText.textScaleFactor}
  /// Deprecated. Will be removed in a future version of Flutter. Use
  /// [textScaler] instead.
  ///
  /// The number of font pixels for each logical pixel.
  ///
  /// For example, if the text scale factor is 1.5, text will be 50% larger than
  /// the specified font size.
  ///
  /// Defaults to the [MediaQueryData.textScaleFactor] obtained from the ambient
  /// [MediaQuery], or 1.0 if there is no [MediaQuery] in scope.
  /// {@endtemplate}
  @Deprecated(
    'Use textScaler instead. '
    'Use of textScaleFactor was deprecated in preparation for the upcoming nonlinear text scaling support. '
    'This feature was deprecated after v3.12.0-2.0.pre.',
  )
  final double? textScaleFactor;

  /// {@macro flutter.painting.textPainter.textScaler}
  final TextScaler? textScaler;

  /// The color to use when painting the cursor.
  final Color cursorColor;

  /// The color to use when painting the autocorrection Rect.
  ///
  /// For [CupertinoTextField]s, the value is set to the ambient
  /// [CupertinoThemeData.primaryColor] with 20% opacity. For [TextField]s, the
  /// value is null on non-iOS platforms and the same color used in [CupertinoTextField]
  /// on iOS.
  ///
  /// Currently the autocorrection Rect only appears on iOS.
  ///
  /// Defaults to null, which disables autocorrection Rect painting.
  final Color? autocorrectionTextRectColor;

  /// The color to use when painting the background cursor aligned with the text
  /// while rendering the floating cursor.
  ///
  /// Typically this would be set to [CupertinoColors.inactiveGray].
  ///
  /// See also:
  ///
  ///  * [FloatingCursorDragState], which explains the floating cursor feature
  ///    in detail.
  final Color backgroundCursorColor;

  /// {@template flutter.widgets.editableText.maxLines}
  /// The maximum number of lines to show at one time, wrapping if necessary.
  ///
  /// This affects the height of the field itself and does not limit the number
  /// of lines that can be entered into the field.
  ///
  /// If this is 1 (the default), the text will not wrap, but will scroll
  /// horizontally instead.
  ///
  /// If this is null, there is no limit to the number of lines, and the text
  /// container will start with enough vertical space for one line and
  /// automatically grow to accommodate additional lines as they are entered, up
  /// to the height of its constraints.
  ///
  /// If this is not null, the value must be greater than zero, and it will lock
  /// the input to the given number of lines and take up enough horizontal space
  /// to accommodate that number of lines. Setting [minLines] as well allows the
  /// input to grow and shrink between the indicated range.
  ///
  /// The full set of behaviors possible with [minLines] and [maxLines] are as
  /// follows. These examples apply equally to [TextField], [TextFormField],
  /// [CupertinoTextField], and [EditableText].
  ///
  /// Input that occupies a single line and scrolls horizontally as needed.
  /// ```dart
  /// const TextField()
  /// ```
  ///
  /// Input whose height grows from one line up to as many lines as needed for
  /// the text that was entered. If a height limit is imposed by its parent, it
  /// will scroll vertically when its height reaches that limit.
  /// ```dart
  /// const TextField(maxLines: null)
  /// ```
  ///
  /// The input's height is large enough for the given number of lines. If
  /// additional lines are entered the input scrolls vertically.
  /// ```dart
  /// const TextField(maxLines: 2)
  /// ```
  ///
  /// Input whose height grows with content between a min and max. An infinite
  /// max is possible with `maxLines: null`.
  /// ```dart
  /// const TextField(minLines: 2, maxLines: 4)
  /// ```
  ///
  /// See also:
  ///
  ///  * [minLines], which sets the minimum number of lines visible.
  /// {@endtemplate}
  ///  * [expands], which determines whether the field should fill the height of
  ///    its parent.
  final int? maxLines;

  /// {@template flutter.widgets.editableText.minLines}
  /// The minimum number of lines to occupy when the content spans fewer lines.
  ///
  /// This affects the height of the field itself and does not limit the number
  /// of lines that can be entered into the field.
  ///
  /// If this is null (default), text container starts with enough vertical space
  /// for one line and grows to accommodate additional lines as they are entered.
  ///
  /// This can be used in combination with [maxLines] for a varying set of behaviors.
  ///
  /// If the value is set, it must be greater than zero. If the value is greater
  /// than 1, [maxLines] should also be set to either null or greater than
  /// this value.
  ///
  /// When [maxLines] is set as well, the height will grow between the indicated
  /// range of lines. When [maxLines] is null, it will grow as high as needed,
  /// starting from [minLines].
  ///
  /// A few examples of behaviors possible with [minLines] and [maxLines] are as follows.
  /// These apply equally to [TextField], [TextFormField], [CupertinoTextField],
  /// and [EditableText].
  ///
  /// Input that always occupies at least 2 lines and has an infinite max.
  /// Expands vertically as needed.
  /// ```dart
  /// TextField(minLines: 2)
  /// ```
  ///
  /// Input whose height starts from 2 lines and grows up to 4 lines at which
  /// point the height limit is reached. If additional lines are entered it will
  /// scroll vertically.
  /// ```dart
  /// const TextField(minLines:2, maxLines: 4)
  /// ```
  ///
  /// Defaults to null.
  ///
  /// See also:
  ///
  ///  * [maxLines], which sets the maximum number of lines visible, and has
  ///    several examples of how minLines and maxLines interact to produce
  ///    various behaviors.
  /// {@endtemplate}
  ///  * [expands], which determines whether the field should fill the height of
  ///    its parent.
  final int? minLines;

  /// {@template flutter.widgets.editableText.expands}
  /// Whether this widget's height will be sized to fill its parent.
  ///
  /// If set to true and wrapped in a parent widget like [Expanded] or
  /// [SizedBox], the input will expand to fill the parent.
  ///
  /// [maxLines] and [minLines] must both be null when this is set to true,
  /// otherwise an error is thrown.
  ///
  /// Defaults to false.
  ///
  /// See the examples in [maxLines] for the complete picture of how [maxLines],
  /// [minLines], and [expands] interact to produce various behaviors.
  ///
  /// Input that matches the height of its parent:
  /// ```dart
  /// const Expanded(
  ///   child: TextField(maxLines: null, expands: true),
  /// )
  /// ```
  /// {@endtemplate}
  final bool expands;

  /// {@template flutter.widgets.editableText.autofocus}
  /// Whether this text field should focus itself if nothing else is already
  /// focused.
  ///
  /// If true, the keyboard will open as soon as this text field obtains focus.
  /// Otherwise, the keyboard is only shown after the user taps the text field.
  ///
  /// Defaults to false.
  /// {@endtemplate}
  // See https://github.com/flutter/flutter/issues/7035 for the rationale for this
  // keyboard behavior.
  final bool autofocus;

  /// The color to use when painting the selection.
  ///
  /// If this property is null, this widget gets the selection color from the
  /// [DefaultSelectionStyle].
  ///
  /// For [CupertinoTextField]s, the value is set to the ambient
  /// [CupertinoThemeData.primaryColor] with 20% opacity. For [TextField]s, the
  /// value is set to the ambient [TextSelectionThemeData.selectionColor].
  final Color? selectionColor;

  /// {@template flutter.widgets.editableText.selectionControls}
  /// Optional delegate for building the text selection handles.
  ///
  /// Historically, this field also controlled the toolbar. This is now handled
  /// by [contextMenuBuilder] instead. However, for backwards compatibility, when
  /// [selectionControls] is set to an object that does not mix in
  /// [TextSelectionHandleControls], [contextMenuBuilder] is ignored and the
  /// [TextSelectionControls.buildToolbar] method is used instead.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [CupertinoTextField], which wraps an [EditableText] and which shows the
  ///    selection toolbar upon user events that are appropriate on the iOS
  ///    platform.
  ///  * [TextField], a Material Design themed wrapper of [EditableText], which
  ///    shows the selection toolbar upon appropriate user events based on the
  ///    user's platform set in [ThemeData.platform].
  final TextSelectionControls? selectionControls;

  /// {@template flutter.widgets.editableText.keyboardType}
  /// The type of keyboard to use for editing the text.
  ///
  /// Defaults to [TextInputType.text] if [maxLines] is one and
  /// [TextInputType.multiline] otherwise.
  /// {@endtemplate}
  final TextInputType keyboardType;

  /// The type of action button to use with the soft keyboard.
  final TextInputAction? textInputAction;

  /// {@template flutter.widgets.editableText.onChanged}
  /// Called when the user initiates a change to the TextField's
  /// value: when they have inserted or deleted text.
  ///
  /// This callback doesn't run when the TextField's text is changed
  /// programmatically, via the TextField's [controller]. Typically it
  /// isn't necessary to be notified of such changes, since they're
  /// initiated by the app itself.
  ///
  /// To be notified of all changes to the TextField's text, cursor,
  /// and selection, one can add a listener to its [controller] with
  /// [TextEditingController.addListener].
  ///
  /// [onChanged] is called before [onSubmitted] when user indicates completion
  /// of editing, such as when pressing the "done" button on the keyboard. That
  /// default behavior can be overridden. See [onEditingComplete] for details.
  ///
  /// {@tool dartpad}
  /// This example shows how onChanged could be used to check the TextField's
  /// current value each time the user inserts or deletes a character.
  ///
  /// ** See code in examples/api/lib/widgets/editable_text/editable_text.on_changed.0.dart **
  /// {@end-tool}
  /// {@endtemplate}
  ///
  /// ## Handling emojis and other complex characters
  /// {@template flutter.widgets.EditableText.onChanged}
  /// It's important to always use
  /// [characters](https://pub.dev/packages/characters) when dealing with user
  /// input text that may contain complex characters. This will ensure that
  /// extended grapheme clusters and surrogate pairs are treated as single
  /// characters, as they appear to the user.
  ///
  /// For example, when finding the length of some user input, use
  /// `string.characters.length`. Do NOT use `string.length` or even
  /// `string.runes.length`. For the complex character "👨‍👩‍👦", this
  /// appears to the user as a single character, and `string.characters.length`
  /// intuitively returns 1. On the other hand, `string.length` returns 8, and
  /// `string.runes.length` returns 5!
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [inputFormatters], which are called before [onChanged]
  ///    runs and can validate and change ("format") the input value.
  ///  * [onEditingComplete], [onSubmitted], [onSelectionChanged]:
  ///    which are more specialized input change notifications.
  ///  * [TextEditingController], which implements the [Listenable] interface
  ///    and notifies its listeners on [TextEditingValue] changes.
  final ValueChanged<String>? onChanged;

  /// {@template flutter.widgets.editableText.onEditingComplete}
  /// Called when the user submits editable content (e.g., user presses the "done"
  /// button on the keyboard).
  ///
  /// The default implementation of [onEditingComplete] executes 2 different
  /// behaviors based on the situation:
  ///
  ///  - When a completion action is pressed, such as "done", "go", "send", or
  ///    "search", the user's content is submitted to the [controller] and then
  ///    focus is given up.
  ///
  ///  - When a non-completion action is pressed, such as "next" or "previous",
  ///    the user's content is submitted to the [controller], but focus is not
  ///    given up because developers may want to immediately move focus to
  ///    another input widget within [onSubmitted].
  ///
  /// Providing [onEditingComplete] prevents the aforementioned default behavior.
  /// {@endtemplate}
  final VoidCallback? onEditingComplete;

  /// {@template flutter.widgets.editableText.onSubmitted}
  /// Called when the user indicates that they are done editing the text in the
  /// field.
  ///
  /// By default, [onSubmitted] is called after [onChanged] when the user
  /// has finalized editing; or, if the default behavior has been overridden,
  /// after [onEditingComplete]. See [onEditingComplete] for details.
  ///
  /// ## Testing
  /// The following is the recommended way to trigger [onSubmitted] in a test:
  ///
  /// ```dart
  /// await tester.testTextInput.receiveAction(TextInputAction.done);
  /// ```
  ///
  /// Sending a `LogicalKeyboardKey.enter` via `tester.sendKeyEvent` will not
  /// trigger [onSubmitted]. This is because on a real device, the engine
  /// translates the enter key to a done action, but `tester.sendKeyEvent` sends
  /// the key to the framework only.
  /// {@endtemplate}
  final ValueChanged<String>? onSubmitted;

  /// {@template flutter.widgets.editableText.onAppPrivateCommand}
  /// This is used to receive a private command from the input method.
  ///
  /// Called when the result of [TextInputClient.performPrivateCommand] is
  /// received.
  ///
  /// This can be used to provide domain-specific features that are only known
  /// between certain input methods and their clients.
  ///
  /// See also:
  ///   * [performPrivateCommand](https://developer.android.com/reference/android/view/inputmethod/InputConnection#performPrivateCommand\(java.lang.String,%20android.os.Bundle\)),
  ///     which is the Android documentation for performPrivateCommand, used to
  ///     send a command from the input method.
  ///   * [sendAppPrivateCommand](https://developer.android.com/reference/android/view/inputmethod/InputMethodManager#sendAppPrivateCommand),
  ///     which is the Android documentation for sendAppPrivateCommand, used to
  ///     send a command to the input method.
  /// {@endtemplate}
  final AppPrivateCommandCallback? onAppPrivateCommand;

  /// {@template flutter.widgets.editableText.onSelectionChanged}
  /// Called when the user changes the selection of text (including the cursor
  /// location).
  /// {@endtemplate}
  final SelectionChangedCallback? onSelectionChanged;

  /// {@macro flutter.widgets.SelectionOverlay.onSelectionHandleTapped}
  final VoidCallback? onSelectionHandleTapped;

  /// {@template flutter.widgets.editableText.groupId}
  /// The group identifier for the [TextFieldTapRegion] of this text field.
  ///
  /// Text fields with the same group identifier share the same tap region.
  /// Defaults to the type of [EditableText].
  ///
  /// See also:
  ///
  ///  * [TextFieldTapRegion], to give a [groupId] to a widget that is to be
  ///    included in a [EditableText]'s tap region that has [groupId] set.
  /// {@endtemplate}
  final Object groupId;

  /// {@template flutter.widgets.editableText.onTapOutside}
  /// Called for each tap down that occurs outside of the [TextFieldTapRegion]
  /// group when the text field is focused.
  ///
  /// If this is null, [EditableTextTapOutsideIntent] will be invoked. In the
  /// default implementation, [FocusNode.unfocus] will be called on the
  /// [focusNode] for this text field when a [PointerDownEvent] is received on
  /// another part of the UI. However, it will not unfocus as a result of mobile
  /// application touch events (which does not include mouse clicks), to conform
  /// with the platform conventions. To change this behavior, a callback may be
  /// set here or [EditableTextTapOutsideIntent] may be overridden.
  ///
  /// When adding additional controls to a text field (for example, a spinner, a
  /// button that copies the selected text, or modifies formatting), it is
  /// helpful if tapping on that control doesn't unfocus the text field. In
  /// order for an external widget to be considered as part of the text field
  /// for the purposes of tapping "outside" of the field, wrap the control in a
  /// [TextFieldTapRegion].
  ///
  /// The [PointerDownEvent] passed to the function is the event that caused the
  /// notification. It is possible that the event may occur outside of the
  /// immediate bounding box defined by the text field, although it will be
  /// within the bounding box of a [TextFieldTapRegion] member.
  /// {@endtemplate}
  ///
  /// {@tool dartpad}
  /// This example shows how to use a `TextFieldTapRegion` to wrap a set of
  /// "spinner" buttons that increment and decrement a value in the [TextField]
  /// without causing the text field to lose keyboard focus.
  ///
  /// This example includes a generic `SpinnerField<T>` class that you can copy
  /// into your own project and customize.
  ///
  /// ** See code in examples/api/lib/widgets/tap_region/text_field_tap_region.0.dart **
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [TapRegion] for how the region group is determined.
  ///  * [onTapUpOutside] which is called for each tap up.
  ///  * [EditableTextTapOutsideIntent] for the intent that is invoked if
  ///  this is null.
  final TapRegionCallback? onTapOutside;

  /// {@template flutter.widgets.editableText.onTapUpOutside}
  /// Called for each tap up that occurs outside of the [TextFieldTapRegion]
  /// group when the text field is focused.
  ///
  /// The [PointerUpEvent] passed to the function is the event that caused the
  /// notification. It is possible that the event may occur outside of the
  /// immediate bounding box defined by the text field, although it will be
  /// within the bounding box of a [TextFieldTapRegion] member.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [TapRegion] for how the region group is determined.
  ///  * [onTapOutside], which is called for each tap down.
  final TapRegionUpCallback? onTapUpOutside;

  /// {@template flutter.widgets.editableText.inputFormatters}
  /// Optional input validation and formatting overrides.
  ///
  /// Formatters are run in the provided order when the user changes the text
  /// this widget contains. When this parameter changes, the new formatters will
  /// not be applied until the next time the user inserts or deletes text.
  /// Similar to the [onChanged] callback, formatters don't run when the text is
  /// changed programmatically via [controller].
  ///
  /// See also:
  ///
  ///  * [TextEditingController], which implements the [Listenable] interface
  ///    and notifies its listeners on [TextEditingValue] changes.
  /// {@endtemplate}
  final List<TextInputFormatter>? inputFormatters;

  /// The cursor for a mouse pointer when it enters or is hovering over the
  /// widget.
  ///
  /// If this property is null, [SystemMouseCursors.text] will be used.
  ///
  /// The [mouseCursor] is the only property of [EditableText] that controls the
  /// appearance of the mouse pointer. All other properties related to "cursor"
  /// stands for the text cursor, which is usually a blinking vertical line at
  /// the editing position.
  final MouseCursor? mouseCursor;

  /// Whether the caller will provide gesture handling (true), or if the
  /// [EditableText] is expected to handle basic gestures (false).
  ///
  /// When this is false, the [EditableText] (or more specifically, the
  /// [RenderEditable]) enables some rudimentary gestures (tap to position the
  /// cursor, long-press to select all, and some scrolling behavior).
  ///
  /// These behaviors are sufficient for debugging purposes but are inadequate
  /// for user-facing applications. To enable platform-specific behaviors, use a
  /// [TextSelectionGestureDetectorBuilder] to wrap the [EditableText], and set
  /// [rendererIgnoresPointer] to true.
  ///
  /// When [rendererIgnoresPointer] is true true, the [RenderEditable] created
  /// by this widget will not handle pointer events.
  ///
  /// This property is false by default.
  ///
  /// See also:
  ///
  ///  * [RenderEditable.ignorePointer], which implements this feature.
  ///  * [TextSelectionGestureDetectorBuilder], which implements platform-specific
  ///    gestures and behaviors.
  final bool rendererIgnoresPointer;

  /// {@template flutter.widgets.editableText.cursorWidth}
  /// How thick the cursor will be.
  ///
  /// Defaults to 2.0.
  ///
  /// The cursor will draw under the text. The cursor width will extend
  /// to the right of the boundary between characters for left-to-right text
  /// and to the left for right-to-left text. This corresponds to extending
  /// downstream relative to the selected position. Negative values may be used
  /// to reverse this behavior.
  /// {@endtemplate}
  final double cursorWidth;

  /// {@template flutter.widgets.editableText.cursorHeight}
  /// How tall the cursor will be.
  ///
  /// If this property is null, [RenderEditable.preferredLineHeight] will be used.
  /// {@endtemplate}
  final double? cursorHeight;

  /// {@template flutter.widgets.editableText.cursorRadius}
  /// How rounded the corners of the cursor should be.
  ///
  /// By default, the cursor has no radius.
  /// {@endtemplate}
  final Radius? cursorRadius;

  /// {@template flutter.widgets.editableText.cursorOpacityAnimates}
  /// Whether the cursor will animate from fully transparent to fully opaque
  /// during each cursor blink.
  ///
  /// By default, the cursor opacity will animate on iOS platforms and will not
  /// animate on Android platforms.
  /// {@endtemplate}
  final bool cursorOpacityAnimates;

  /// {@macro flutter.rendering.RenderEditable.cursorOffset}
  final Offset? cursorOffset;

  /// {@macro flutter.rendering.RenderEditable.paintCursorAboveText}
  final bool paintCursorAboveText;

  /// Controls how tall the selection highlight boxes are computed to be.
  ///
  /// See [ui.BoxHeightStyle] for details on available styles.
  final ui.BoxHeightStyle selectionHeightStyle;

  /// Controls how wide the selection highlight boxes are computed to be.
  ///
  /// See [ui.BoxWidthStyle] for details on available styles.
  final ui.BoxWidthStyle selectionWidthStyle;

  /// The appearance of the keyboard.
  ///
  /// This setting is only honored on iOS devices.
  ///
  /// Defaults to [Brightness.light].
  final Brightness keyboardAppearance;

  /// {@template flutter.widgets.editableText.scrollPadding}
  /// Configures the padding for the edges surrounding a [Scrollable] when the
  /// text field scrolls into view.
  ///
  /// When this widget receives focus and is not completely visible (for example
  /// scrolled partially off the screen or overlapped by the keyboard), then it
  /// will attempt to make itself visible by scrolling a surrounding
  /// [Scrollable], if one is present. This value controls how far from the
  /// edges of a [Scrollable] the TextField will be positioned after the scroll.
  ///
  /// Defaults to EdgeInsets.all(20.0).
  /// {@endtemplate}
  final EdgeInsets scrollPadding;

  /// {@template flutter.widgets.editableText.enableInteractiveSelection}
  /// Whether to enable user interface affordances for changing the
  /// text selection.
  ///
  /// For example, setting this to true will enable features such as
  /// long-pressing the TextField to select text and show the
  /// cut/copy/paste menu, and tapping to move the text caret.
  ///
  /// When this is false, the text selection cannot be adjusted by
  /// the user, text cannot be copied, and the user cannot paste into
  /// the text field from the clipboard.
  ///
  /// Defaults to true.
  /// {@endtemplate}
  final bool enableInteractiveSelection;

  /// Setting this property to true makes the cursor stop blinking or fading
  /// on and off once the cursor appears on focus. This property is useful for
  /// testing purposes.
  ///
  /// It does not affect the necessity to focus the EditableText for the cursor
  /// to appear in the first place.
  ///
  /// Defaults to false, resulting in a typical blinking cursor.
  static bool debugDeterministicCursor = false;

  /// {@macro flutter.widgets.scrollable.dragStartBehavior}
  final DragStartBehavior dragStartBehavior;

  /// {@template flutter.widgets.editableText.scrollController}
  /// The [ScrollController] to use when vertically scrolling the input.
  ///
  /// If null, it will instantiate a new ScrollController.
  ///
  /// See [Scrollable.controller].
  /// {@endtemplate}
  final ScrollController? scrollController;

  /// {@template flutter.widgets.editableText.scrollPhysics}
  /// The [ScrollPhysics] to use when vertically scrolling the input.
  ///
  /// If not specified, it will behave according to the current platform.
  ///
  /// See [Scrollable.physics].
  /// {@endtemplate}
  ///
  /// If an explicit [ScrollBehavior] is provided to [scrollBehavior], the
  /// [ScrollPhysics] provided by that behavior will take precedence after
  /// [scrollPhysics].
  final ScrollPhysics? scrollPhysics;

  /// {@template flutter.widgets.editableText.scribbleEnabled}
  /// Whether iOS 14 Scribble features are enabled for this widget.
  ///
  /// Only available on iPads.
  ///
  /// Defaults to true.
  /// {@endtemplate}
  @Deprecated(
    'Use `stylusHandwritingEnabled` instead. '
    'This feature was deprecated after v3.27.0-0.2.pre.',
  )
  final bool scribbleEnabled;

  /// {@template flutter.widgets.editableText.stylusHandwritingEnabled}
  /// Whether this input supports stylus handwriting, where the user can write
  /// directly on top of a field.
  ///
  /// Currently only the following devices are supported:
  ///
  ///  * iPads running iOS 14 and above using an Apple Pencil.
  ///  * Android devices running API 34 and above and using an active stylus.
  /// {@endtemplate}
  ///
  /// On Android, Scribe gestures are detected outside of [EditableText],
  /// typically by [TextSelectionGestureDetectorBuilder]. This is handled
  /// automatically in [TextField].
  ///
  /// See also:
  ///
  ///   * [ScribbleClient], which can be mixed into an arbitrary widget to
  ///     provide iOS Scribble functionality.
  ///   * [Scribe], which can be used to interact with Android Scribe directly.
  final bool stylusHandwritingEnabled;

  /// {@template flutter.widgets.editableText.selectionEnabled}
  /// Same as [enableInteractiveSelection].
  ///
  /// This getter exists primarily for consistency with
  /// [RenderEditable.selectionEnabled].
  /// {@endtemplate}
  bool get selectionEnabled => enableInteractiveSelection;

  /// {@template flutter.widgets.editableText.autofillHints}
  /// A list of strings that helps the autofill service identify the type of this
  /// text input.
  ///
  /// When set to null, this text input will not send its autofill information
  /// to the platform, preventing it from participating in autofills triggered
  /// by a different [AutofillClient], even if they're in the same
  /// [AutofillScope]. Additionally, on Android and web, setting this to null
  /// will disable autofill for this text field.
  ///
  /// The minimum platform SDK version that supports Autofill is API level 26
  /// for Android, and iOS 10.0 for iOS.
  ///
  /// Defaults to an empty list.
  ///
  /// ### Setting up iOS autofill:
  ///
  /// To provide the best user experience and ensure your app fully supports
  /// password autofill on iOS, follow these steps:
  ///
  /// * Set up your iOS app's
  ///   [associated domains](https://developer.apple.com/documentation/safariservices/supporting_associated_domains_in_your_app).
  /// * Some autofill hints only work with specific [keyboardType]s. For example,
  ///   [AutofillHints.name] requires [TextInputType.name] and [AutofillHints.email]
  ///   works only with [TextInputType.emailAddress]. Make sure the input field has a
  ///   compatible [keyboardType]. Empirically, [TextInputType.name] works well
  ///   with many autofill hints that are predefined on iOS.
  ///
  /// ### Troubleshooting Autofill
  ///
  /// Autofill service providers rely heavily on [autofillHints]. Make sure the
  /// entries in [autofillHints] are supported by the autofill service currently
  /// in use (the name of the service can typically be found in your mobile
  /// device's system settings).
  ///
  /// #### Autofill UI refuses to show up when I tap on the text field
  ///
  /// Check the device's system settings and make sure autofill is turned on,
  /// and there are available credentials stored in the autofill service.
  ///
  /// * iOS password autofill: Go to Settings -> Password, turn on "Autofill
  ///   Passwords", and add new passwords for testing by pressing the top right
  ///   "+" button. Use an arbitrary "website" if you don't have associated
  ///   domains set up for your app. As long as there's at least one password
  ///   stored, you should be able to see a key-shaped icon in the quick type
  ///   bar on the software keyboard, when a password related field is focused.
  ///
  /// * iOS contact information autofill: iOS seems to pull contact info from
  ///   the Apple ID currently associated with the device. Go to Settings ->
  ///   Apple ID (usually the first entry, or "Sign in to your iPhone" if you
  ///   haven't set up one on the device), and fill out the relevant fields. If
  ///   you wish to test more contact info types, try adding them in Contacts ->
  ///   My Card.
  ///
  /// * Android autofill: Go to Settings -> System -> Languages & input ->
  ///   Autofill service. Enable the autofill service of your choice, and make
  ///   sure there are available credentials associated with your app.
  ///
  /// Specifying [InputDecoration.hintText] may also help autofill services
  /// (like Samsung Pass) determine the expected content type of an input field,
  /// although this is typically not required when autofillHints are present.
  ///
  /// #### I called `TextInput.finishAutofillContext` but the autofill save
  /// prompt isn't showing
  ///
  /// * iOS: iOS may not show a prompt or any other visual indication when it
  ///   saves user password. Go to Settings -> Password and check if your new
  ///   password is saved. Neither saving password nor auto-generating strong
  ///   password works without properly setting up associated domains in your
  ///   app. To set up associated domains, follow the instructions in
  ///   <https://developer.apple.com/documentation/safariservices/supporting_associated_domains_in_your_app>.
  ///
  /// {@endtemplate}
  /// {@macro flutter.services.AutofillConfiguration.autofillHints}
  final Iterable<String>? autofillHints;

  /// The [AutofillClient] that controls this input field's autofill behavior.
  ///
  /// When null, this widget's [EditableTextState] will be used as the
  /// [AutofillClient]. This property may override [autofillHints].
  final AutofillClient? autofillClient;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// Restoration ID to save and restore the scroll offset of the
  /// [EditableText].
  ///
  /// If a restoration id is provided, the [EditableText] will persist its
  /// current scroll offset and restore it during state restoration.
  ///
  /// The scroll offset is persisted in a [RestorationBucket] claimed from
  /// the surrounding [RestorationScope] using the provided restoration ID.
  ///
  /// Persisting and restoring the content of the [EditableText] is the
  /// responsibility of the owner of the [controller], who may use a
  /// [RestorableTextEditingController] for that purpose.
  ///
  /// See also:
  ///
  ///  * [RestorationManager], which explains how state restoration works in
  ///    Flutter.
  final String? restorationId;

  /// {@template flutter.widgets.editableText.scrollBehavior}
  /// A [ScrollBehavior] that will be applied to this widget individually.
  ///
  /// Defaults to null, wherein the inherited [ScrollBehavior] is copied and
  /// modified to alter the viewport decoration, like [Scrollbar]s.
  ///
  /// [ScrollBehavior]s also provide [ScrollPhysics]. If an explicit
  /// [ScrollPhysics] is provided in [scrollPhysics], it will take precedence,
  /// followed by [scrollBehavior], and then the inherited ancestor
  /// [ScrollBehavior].
  /// {@endtemplate}
  ///
  /// The [ScrollBehavior] of the inherited [ScrollConfiguration] will be
  /// modified by default to only apply a [Scrollbar] if [maxLines] is greater
  /// than 1.
  final ScrollBehavior? scrollBehavior;

  /// {@macro flutter.services.TextInputConfiguration.enableIMEPersonalizedLearning}
  final bool enableIMEPersonalizedLearning;

  /// {@template flutter.widgets.editableText.contentInsertionConfiguration}
  /// Configuration of handler for media content inserted via the system input
  /// method.
  ///
  /// Defaults to null in which case media content insertion will be disabled,
  /// and the system will display a message informing the user that the text field
  /// does not support inserting media content.
  ///
  /// Set [ContentInsertionConfiguration.onContentInserted] to provide a handler.
  /// Additionally, set [ContentInsertionConfiguration.allowedMimeTypes]
  /// to limit the allowable mime types for inserted content.
  ///
  /// {@tool dartpad}
  ///
  /// This example shows how to access the data for inserted content in your
  /// `TextField`.
  ///
  /// ** See code in examples/api/lib/widgets/editable_text/editable_text.on_content_inserted.0.dart **
  /// {@end-tool}
  ///
  /// If [contentInsertionConfiguration] is not provided, by default
  /// an empty list of mime types will be sent to the Flutter Engine.
  /// A handler function must be provided in order to customize the allowable
  /// mime types for inserted content.
  ///
  /// If rich content is inserted without a handler, the system will display
  /// a message informing the user that the current text input does not support
  /// inserting rich content.
  /// {@endtemplate}
  final ContentInsertionConfiguration? contentInsertionConfiguration;

  /// {@template flutter.widgets.EditableText.contextMenuBuilder}
  /// Builds the text selection toolbar when requested by the user.
  ///
  /// The context menu is built when [EditableTextState.showToolbar] is called,
  /// typically by one of the callbacks installed by the widget created by
  /// [TextSelectionGestureDetectorBuilder.buildGestureDetector]. The widget
  /// returned by [contextMenuBuilder] is passed to a [ContextMenuController].
  ///
  /// If no callback is provided, no context menu will be shown.
  ///
  /// The [EditableTextContextMenuBuilder] signature used by the
  /// [contextMenuBuilder] callback has two parameters, the [BuildContext] of
  /// the [EditableText] and the [EditableTextState] of the [EditableText].
  ///
  /// The [EditableTextState] has two properties that are especially useful when
  /// building the widgets for the context menu:
  ///
  /// * [EditableTextState.contextMenuAnchors] specifies the desired anchor
  ///   position for the context menu.
  ///
  /// * [EditableTextState.contextMenuButtonItems] represents the buttons that
  ///   should typically be built for this widget (e.g. cut, copy, paste).
  ///
  /// The [TextSelectionToolbarLayoutDelegate] class may be particularly useful
  /// in honoring the preferred anchor positions.
  ///
  /// For backwards compatibility, when [EditableText.selectionControls] is set
  /// to an object that does not mix in [TextSelectionHandleControls],
  /// [contextMenuBuilder] is ignored and the
  /// [TextSelectionControls.buildToolbar] method is used instead.
  ///
  /// {@tool dartpad}
  /// This example shows how to customize the menu, in this case by keeping the
  /// default buttons for the platform but modifying their appearance.
  ///
  /// ** See code in examples/api/lib/material/context_menu/editable_text_toolbar_builder.0.dart **
  /// {@end-tool}
  ///
  /// {@tool dartpad}
  /// This example shows how to show a custom button only when an email address
  /// is currently selected.
  ///
  /// ** See code in examples/api/lib/material/context_menu/editable_text_toolbar_builder.1.dart **
  /// {@end-tool}
  ///
  /// See also:
  ///   * [AdaptiveTextSelectionToolbar], which builds the default text selection
  ///     toolbar for the current platform, but allows customization of the
  ///     buttons.
  ///   * [AdaptiveTextSelectionToolbar.getAdaptiveButtons], which builds the
  ///     button Widgets for the current platform given
  ///     [ContextMenuButtonItem]s.
  ///   * [BrowserContextMenu], which allows the browser's context menu on web
  ///     to be disabled and Flutter-rendered context menus to appear.
  /// {@endtemplate}
  final EditableTextContextMenuBuilder? contextMenuBuilder;

  /// {@template flutter.widgets.EditableText.spellCheckConfiguration}
  /// Configuration that details how spell check should be performed.
  ///
  /// Specifies the [SpellCheckService] used to spell check text input and the
  /// [TextStyle] used to style text with misspelled words.
  ///
  /// If the [SpellCheckService] is left null, spell check is disabled by
  /// default unless the [DefaultSpellCheckService] is supported, in which case
  /// it is used. It is currently supported only on Android and iOS.
  ///
  /// If this configuration is left null, then spell check is disabled by default.
  /// {@endtemplate}
  final SpellCheckConfiguration? spellCheckConfiguration;

  /// The configuration for the magnifier to use with selections in this text
  /// field.
  ///
  /// {@macro flutter.widgets.magnifier.intro}
  final TextMagnifierConfiguration magnifierConfiguration;

  /// The default value for [stylusHandwritingEnabled].
  static const bool defaultStylusHandwritingEnabled = true;

  bool get _userSelectionEnabled => enableInteractiveSelection && (!readOnly || !obscureText);

  /// Returns the [ContextMenuButtonItem]s representing the buttons in this
  /// platform's default selection menu for an editable field.
  ///
  /// For example, [EditableText] uses this to generate the default buttons for
  /// its context menu.
  ///
  /// See also:
  ///
  /// * [EditableTextState.contextMenuButtonItems], which gives the
  ///   [ContextMenuButtonItem]s for a specific EditableText.
  /// * [SelectableRegion.getSelectableButtonItems], which performs a similar
  ///   role but for content that is selectable but not editable.
  /// * [AdaptiveTextSelectionToolbar], which builds the toolbar itself, and can
  ///   take a list of [ContextMenuButtonItem]s with
  ///   [AdaptiveTextSelectionToolbar.buttonItems].
  /// * [AdaptiveTextSelectionToolbar.getAdaptiveButtons], which builds the button
  ///   Widgets for the current platform given [ContextMenuButtonItem]s.
  static List<ContextMenuButtonItem> getEditableButtonItems({
    required final ClipboardStatus? clipboardStatus,
    required final VoidCallback? onCopy,
    required final VoidCallback? onCut,
    required final VoidCallback? onPaste,
    required final VoidCallback? onSelectAll,
    required final VoidCallback? onLookUp,
    required final VoidCallback? onSearchWeb,
    required final VoidCallback? onShare,
    required final VoidCallback? onLiveTextInput,
  }) {
    final List<ContextMenuButtonItem> resultButtonItem = <ContextMenuButtonItem>[];

    // Configure button items with clipboard.
    if (onPaste == null || clipboardStatus != ClipboardStatus.unknown) {
      // If the paste button is enabled, don't render anything until the state
      // of the clipboard is known, since it's used to determine if paste is
      // shown.

      // On Android, the share button is before the select all button.
      final bool showShareBeforeSelectAll = defaultTargetPlatform == TargetPlatform.android;

      resultButtonItem.addAll(<ContextMenuButtonItem>[
        if (onCut != null) ContextMenuButtonItem(onPressed: onCut, type: ContextMenuButtonType.cut),
        if (onCopy != null)
          ContextMenuButtonItem(onPressed: onCopy, type: ContextMenuButtonType.copy),
        if (onPaste != null)
          ContextMenuButtonItem(onPressed: onPaste, type: ContextMenuButtonType.paste),
        if (onShare != null && showShareBeforeSelectAll)
          ContextMenuButtonItem(onPressed: onShare, type: ContextMenuButtonType.share),
        if (onSelectAll != null)
          ContextMenuButtonItem(onPressed: onSelectAll, type: ContextMenuButtonType.selectAll),
        if (onLookUp != null)
          ContextMenuButtonItem(onPressed: onLookUp, type: ContextMenuButtonType.lookUp),
        if (onSearchWeb != null)
          ContextMenuButtonItem(onPressed: onSearchWeb, type: ContextMenuButtonType.searchWeb),
        if (onShare != null && !showShareBeforeSelectAll)
          ContextMenuButtonItem(onPressed: onShare, type: ContextMenuButtonType.share),
      ]);
    }

    // Config button items with Live Text.
    if (onLiveTextInput != null) {
      resultButtonItem.add(
        ContextMenuButtonItem(
          onPressed: onLiveTextInput,
          type: ContextMenuButtonType.liveTextInput,
        ),
      );
    }

    return resultButtonItem;
  }

  // Infer the keyboard type of an `EditableText` if it's not specified.
  static TextInputType _inferKeyboardType({
    required Iterable<String>? autofillHints,
    required int? maxLines,
  }) {
    if (autofillHints == null || autofillHints.isEmpty) {
      return maxLines == 1 ? TextInputType.text : TextInputType.multiline;
    }

    final String effectiveHint = autofillHints.first;

    // On iOS oftentimes specifying a text content type is not enough to qualify
    // the input field for autofill. The keyboard type also needs to be compatible
    // with the content type. To get autofill to work by default on EditableText,
    // the keyboard type inference on iOS is done differently from other platforms.
    //
    // The entries with "autofill not working" comments are the iOS text content
    // types that should work with the specified keyboard type but won't trigger
    // (even within a native app). Tested on iOS 13.5.
    if (!kIsWeb) {
      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          const Map<String, TextInputType> iOSKeyboardType = <String, TextInputType>{
            AutofillHints.addressCity: TextInputType.name,
            AutofillHints.addressCityAndState: TextInputType.name, // Autofill not working.
            AutofillHints.addressState: TextInputType.name,
            AutofillHints.countryName: TextInputType.name,
            AutofillHints.creditCardNumber: TextInputType.number, // Couldn't test.
            AutofillHints.email: TextInputType.emailAddress,
            AutofillHints.familyName: TextInputType.name,
            AutofillHints.fullStreetAddress: TextInputType.name,
            AutofillHints.givenName: TextInputType.name,
            AutofillHints.jobTitle: TextInputType.name, // Autofill not working.
            AutofillHints.location: TextInputType.name, // Autofill not working.
            AutofillHints.middleName: TextInputType.name, // Autofill not working.
            AutofillHints.name: TextInputType.name,
            AutofillHints.namePrefix: TextInputType.name, // Autofill not working.
            AutofillHints.nameSuffix: TextInputType.name, // Autofill not working.
            AutofillHints.newPassword: TextInputType.text,
            AutofillHints.newUsername: TextInputType.text,
            AutofillHints.nickname: TextInputType.name, // Autofill not working.
            AutofillHints.oneTimeCode: TextInputType.number,
            AutofillHints.organizationName: TextInputType.text, // Autofill not working.
            AutofillHints.password: TextInputType.text,
            AutofillHints.postalCode: TextInputType.name,
            AutofillHints.streetAddressLine1: TextInputType.name,
            AutofillHints.streetAddressLine2: TextInputType.name, // Autofill not working.
            AutofillHints.sublocality: TextInputType.name, // Autofill not working.
            AutofillHints.telephoneNumber: TextInputType.name,
            AutofillHints.url: TextInputType.url, // Autofill not working.
            AutofillHints.username: TextInputType.text,
          };

          final TextInputType? keyboardType = iOSKeyboardType[effectiveHint];
          if (keyboardType != null) {
            return keyboardType;
          }
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          break;
      }
    }

    if (maxLines != 1) {
      return TextInputType.multiline;
    }

    const Map<String, TextInputType> inferKeyboardType = <String, TextInputType>{
      AutofillHints.addressCity: TextInputType.streetAddress,
      AutofillHints.addressCityAndState: TextInputType.streetAddress,
      AutofillHints.addressState: TextInputType.streetAddress,
      AutofillHints.birthday: TextInputType.datetime,
      AutofillHints.birthdayDay: TextInputType.datetime,
      AutofillHints.birthdayMonth: TextInputType.datetime,
      AutofillHints.birthdayYear: TextInputType.datetime,
      AutofillHints.countryCode: TextInputType.number,
      AutofillHints.countryName: TextInputType.text,
      AutofillHints.creditCardExpirationDate: TextInputType.datetime,
      AutofillHints.creditCardExpirationDay: TextInputType.datetime,
      AutofillHints.creditCardExpirationMonth: TextInputType.datetime,
      AutofillHints.creditCardExpirationYear: TextInputType.datetime,
      AutofillHints.creditCardFamilyName: TextInputType.name,
      AutofillHints.creditCardGivenName: TextInputType.name,
      AutofillHints.creditCardMiddleName: TextInputType.name,
      AutofillHints.creditCardName: TextInputType.name,
      AutofillHints.creditCardNumber: TextInputType.number,
      AutofillHints.creditCardSecurityCode: TextInputType.number,
      AutofillHints.creditCardType: TextInputType.text,
      AutofillHints.email: TextInputType.emailAddress,
      AutofillHints.familyName: TextInputType.name,
      AutofillHints.fullStreetAddress: TextInputType.streetAddress,
      AutofillHints.gender: TextInputType.text,
      AutofillHints.givenName: TextInputType.name,
      AutofillHints.impp: TextInputType.url,
      AutofillHints.jobTitle: TextInputType.text,
      AutofillHints.language: TextInputType.text,
      AutofillHints.location: TextInputType.streetAddress,
      AutofillHints.middleInitial: TextInputType.name,
      AutofillHints.middleName: TextInputType.name,
      AutofillHints.name: TextInputType.name,
      AutofillHints.namePrefix: TextInputType.name,
      AutofillHints.nameSuffix: TextInputType.name,
      AutofillHints.newPassword: TextInputType.text,
      AutofillHints.newUsername: TextInputType.text,
      AutofillHints.nickname: TextInputType.text,
      AutofillHints.oneTimeCode: TextInputType.text,
      AutofillHints.organizationName: TextInputType.text,
      AutofillHints.password: TextInputType.text,
      AutofillHints.photo: TextInputType.text,
      AutofillHints.postalAddress: TextInputType.streetAddress,
      AutofillHints.postalAddressExtended: TextInputType.streetAddress,
      AutofillHints.postalAddressExtendedPostalCode: TextInputType.number,
      AutofillHints.postalCode: TextInputType.number,
      AutofillHints.streetAddressLevel1: TextInputType.streetAddress,
      AutofillHints.streetAddressLevel2: TextInputType.streetAddress,
      AutofillHints.streetAddressLevel3: TextInputType.streetAddress,
      AutofillHints.streetAddressLevel4: TextInputType.streetAddress,
      AutofillHints.streetAddressLine1: TextInputType.streetAddress,
      AutofillHints.streetAddressLine2: TextInputType.streetAddress,
      AutofillHints.streetAddressLine3: TextInputType.streetAddress,
      AutofillHints.sublocality: TextInputType.streetAddress,
      AutofillHints.telephoneNumber: TextInputType.phone,
      AutofillHints.telephoneNumberAreaCode: TextInputType.phone,
      AutofillHints.telephoneNumberCountryCode: TextInputType.phone,
      AutofillHints.telephoneNumberDevice: TextInputType.phone,
      AutofillHints.telephoneNumberExtension: TextInputType.phone,
      AutofillHints.telephoneNumberLocal: TextInputType.phone,
      AutofillHints.telephoneNumberLocalPrefix: TextInputType.phone,
      AutofillHints.telephoneNumberLocalSuffix: TextInputType.phone,
      AutofillHints.telephoneNumberNational: TextInputType.phone,
      AutofillHints.transactionAmount: TextInputType.numberWithOptions(decimal: true),
      AutofillHints.transactionCurrency: TextInputType.text,
      AutofillHints.url: TextInputType.url,
      AutofillHints.username: TextInputType.text,
    };

    return inferKeyboardType[effectiveHint] ?? TextInputType.text;
  }

  @override
  EditableTextState createState() => EditableTextState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<TextEditingController>('controller', controller));
    properties.add(DiagnosticsProperty<FocusNode>('focusNode', focusNode));
    properties.add(DiagnosticsProperty<bool>('obscureText', obscureText, defaultValue: false));
    properties.add(DiagnosticsProperty<bool>('readOnly', readOnly, defaultValue: false));
    properties.add(DiagnosticsProperty<bool>('autocorrect', autocorrect, defaultValue: true));
    properties.add(
      EnumProperty<SmartDashesType>(
        'smartDashesType',
        smartDashesType,
        defaultValue: obscureText ? SmartDashesType.disabled : SmartDashesType.enabled,
      ),
    );
    properties.add(
      EnumProperty<SmartQuotesType>(
        'smartQuotesType',
        smartQuotesType,
        defaultValue: obscureText ? SmartQuotesType.disabled : SmartQuotesType.enabled,
      ),
    );
    properties.add(
      DiagnosticsProperty<bool>('enableSuggestions', enableSuggestions, defaultValue: true),
    );
    style.debugFillProperties(properties);
    properties.add(EnumProperty<TextAlign>('textAlign', textAlign, defaultValue: null));
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
    properties.add(DiagnosticsProperty<Locale>('locale', locale, defaultValue: null));
    properties.add(DiagnosticsProperty<TextScaler>('textScaler', textScaler, defaultValue: null));
    properties.add(IntProperty('maxLines', maxLines, defaultValue: 1));
    properties.add(IntProperty('minLines', minLines, defaultValue: null));
    properties.add(DiagnosticsProperty<bool>('expands', expands, defaultValue: false));
    properties.add(DiagnosticsProperty<bool>('autofocus', autofocus, defaultValue: false));
    properties.add(
      DiagnosticsProperty<TextInputType>('keyboardType', keyboardType, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<ScrollController>(
        'scrollController',
        scrollController,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<ScrollPhysics>('scrollPhysics', scrollPhysics, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<Iterable<String>>('autofillHints', autofillHints, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TextHeightBehavior>(
        'textHeightBehavior',
        textHeightBehavior,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<bool>('scribbleEnabled', scribbleEnabled, defaultValue: true),
    );
    properties.add(
      DiagnosticsProperty<bool>(
        'stylusHandwritingEnabled',
        stylusHandwritingEnabled,
        defaultValue: defaultStylusHandwritingEnabled,
      ),
    );
    properties.add(
      DiagnosticsProperty<bool>(
        'enableIMEPersonalizedLearning',
        enableIMEPersonalizedLearning,
        defaultValue: true,
      ),
    );
    properties.add(
      DiagnosticsProperty<bool>(
        'enableInteractiveSelection',
        enableInteractiveSelection,
        defaultValue: true,
      ),
    );
    properties.add(
      DiagnosticsProperty<UndoHistoryController>(
        'undoController',
        undoController,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<SpellCheckConfiguration>(
        'spellCheckConfiguration',
        spellCheckConfiguration,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<List<String>>(
        'contentCommitMimeTypes',
        contentInsertionConfiguration?.allowedMimeTypes ?? const <String>[],
        defaultValue:
            contentInsertionConfiguration == null
                ? const <String>[]
                : kDefaultContentInsertionMimeTypes,
      ),
    );
  }
}

/// State for an [EditableText].
class EditableTextState extends State<EditableText>
    with
        AutomaticKeepAliveClientMixin<EditableText>,
        WidgetsBindingObserver,
        TickerProviderStateMixin<EditableText>,
        TextSelectionDelegate,
        TextInputClient
    implements AutofillClient {
  Timer? _cursorTimer;
  AnimationController get _cursorBlinkOpacityController {
    return _backingCursorBlinkOpacityController ??= AnimationController(vsync: this)
      ..addListener(_onCursorColorTick);
  }

  AnimationController? _backingCursorBlinkOpacityController;
  late final Simulation _iosBlinkCursorSimulation = _DiscreteKeyFrameSimulation.iOSBlinkingCaret();

  final ValueNotifier<bool> _cursorVisibilityNotifier = ValueNotifier<bool>(true);
  final GlobalKey _editableKey = GlobalKey();

  /// Detects whether the clipboard can paste.
  final ClipboardStatusNotifier clipboardStatus =
      kIsWeb
          // Web browsers will show a permission dialog when Clipboard.hasStrings is
          // called. In an EditableText, this will happen before the paste button is
          // clicked, often before the context menu is even shown. To avoid this
          // poor user experience, always show the paste button on web.
          ? _WebClipboardStatusNotifier()
          : ClipboardStatusNotifier();

  /// Detects whether the Live Text input is enabled.
  ///
  /// See also:
  ///  * [LiveText], where the availability of Live Text input can be obtained.
  final LiveTextInputStatusNotifier? _liveTextInputStatus =
      kIsWeb ? null : LiveTextInputStatusNotifier();

  TextInputConnection? _textInputConnection;
  bool get _hasInputConnection => _textInputConnection?.attached ?? false;

  TextSelectionOverlay? _selectionOverlay;
  ScrollNotificationObserverState? _scrollNotificationObserver;
  ({TextEditingValue value, Rect selectionBounds})? _dataWhenToolbarShowScheduled;
  bool _listeningToScrollNotificationObserver = false;

  bool get _webContextMenuEnabled => kIsWeb && BrowserContextMenu.enabled;

  final GlobalKey _scrollableKey = GlobalKey();
  ScrollController? _internalScrollController;
  ScrollController get _scrollController =>
      widget.scrollController ?? (_internalScrollController ??= ScrollController());

  final LayerLink _toolbarLayerLink = LayerLink();
  final LayerLink _startHandleLayerLink = LayerLink();
  final LayerLink _endHandleLayerLink = LayerLink();

  bool _didAutoFocus = false;

  AutofillGroupState? _currentAutofillScope;
  @override
  AutofillScope? get currentAutofillScope => _currentAutofillScope;

  AutofillClient get _effectiveAutofillClient => widget.autofillClient ?? this;

  late SpellCheckConfiguration _spellCheckConfiguration;
  late TextStyle _style;

  /// Configuration that determines how spell check will be performed.
  ///
  /// If possible, this configuration will contain a default for the
  /// [SpellCheckService] if it is not otherwise specified.
  ///
  /// See also:
  ///  * [DefaultSpellCheckService], the spell check service used by default.
  @visibleForTesting
  SpellCheckConfiguration get spellCheckConfiguration => _spellCheckConfiguration;

  /// Whether or not spell check is enabled.
  ///
  /// Spell check is enabled when a [SpellCheckConfiguration] has been specified
  /// for the widget.
  bool get spellCheckEnabled => _spellCheckConfiguration.spellCheckEnabled;

  /// The most up-to-date spell check results for text input.
  ///
  /// These results will be updated via calls to spell check through a
  /// [SpellCheckService] and used by this widget to build the [TextSpan] tree
  /// for text input and menus for replacement suggestions of misspelled words.
  SpellCheckResults? spellCheckResults;

  bool get _spellCheckResultsReceived =>
      spellCheckEnabled &&
      spellCheckResults != null &&
      spellCheckResults!.suggestionSpans.isNotEmpty;

  /// The text processing service used to retrieve the native text processing actions.
  final ProcessTextService _processTextService = DefaultProcessTextService();

  /// The list of native text processing actions provided by the engine.
  final List<ProcessTextAction> _processTextActions = <ProcessTextAction>[];

  /// Whether to create an input connection with the platform for text editing
  /// or not.
  ///
  /// Read-only input fields do not need a connection with the platform since
  /// there's no need for text editing capabilities (e.g. virtual keyboard).
  ///
  /// On the web, we always need a connection because we want some browser
  /// functionalities to continue to work on read-only input fields like:
  ///
  /// - Relevant context menu.
  /// - cmd/ctrl+c shortcut to copy.
  /// - cmd/ctrl+a to select all.
  /// - Changing the selection using a physical keyboard.
  bool get _shouldCreateInputConnection => kIsWeb || !widget.readOnly;

  // The time it takes for the floating cursor to snap to the text aligned
  // cursor position after the user has finished placing it.
  static const Duration _floatingCursorResetTime = Duration(milliseconds: 125);

  AnimationController? _floatingCursorResetController;

  Orientation? _lastOrientation;

  bool get _stylusHandwritingEnabled {
    // During the deprecation period, respect scribbleEnabled being explicitly
    // set.
    if (!widget.scribbleEnabled) {
      return widget.scribbleEnabled;
    }
    return widget.stylusHandwritingEnabled;
  }

  late final AppLifecycleListener _appLifecycleListener;
  bool _justResumed = false;

  @override
  bool get wantKeepAlive => widget.focusNode.hasFocus;

  Color get _cursorColor {
    final double effectiveOpacity = math.min(
      widget.cursorColor.alpha / 255.0,
      _cursorBlinkOpacityController.value,
    );
    return widget.cursorColor.withOpacity(effectiveOpacity);
  }

  @override
  bool get cutEnabled {
    if (widget.selectionControls is! TextSelectionHandleControls) {
      return widget.toolbarOptions.cut && !widget.readOnly && !widget.obscureText;
    }
    return !widget.readOnly && !widget.obscureText && !textEditingValue.selection.isCollapsed;
  }

  @override
  bool get copyEnabled {
    if (widget.selectionControls is! TextSelectionHandleControls) {
      return widget.toolbarOptions.copy && !widget.obscureText;
    }
    return !widget.obscureText && !textEditingValue.selection.isCollapsed;
  }

  @override
  bool get pasteEnabled {
    if (widget.selectionControls is! TextSelectionHandleControls) {
      return widget.toolbarOptions.paste && !widget.readOnly;
    }
    return !widget.readOnly && (clipboardStatus.value == ClipboardStatus.pasteable);
  }

  @override
  bool get selectAllEnabled {
    if (widget.selectionControls is! TextSelectionHandleControls) {
      return widget.toolbarOptions.selectAll &&
          (!widget.readOnly || !widget.obscureText) &&
          widget.enableInteractiveSelection;
    }

    if (!widget.enableInteractiveSelection || (widget.readOnly && widget.obscureText)) {
      return false;
    }

    switch (defaultTargetPlatform) {
      case TargetPlatform.macOS:
        return false;
      case TargetPlatform.iOS:
        return textEditingValue.text.isNotEmpty && textEditingValue.selection.isCollapsed;
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        return textEditingValue.text.isNotEmpty &&
            !(textEditingValue.selection.start == 0 &&
                textEditingValue.selection.end == textEditingValue.text.length);
    }
  }

  @override
  bool get lookUpEnabled {
    if (defaultTargetPlatform != TargetPlatform.iOS) {
      return false;
    }
    return !widget.obscureText &&
        !textEditingValue.selection.isCollapsed &&
        textEditingValue.selection.textInside(textEditingValue.text).trim() != '';
  }

  @override
  bool get searchWebEnabled {
    if (defaultTargetPlatform != TargetPlatform.iOS) {
      return false;
    }

    return !widget.obscureText &&
        !textEditingValue.selection.isCollapsed &&
        textEditingValue.selection.textInside(textEditingValue.text).trim() != '';
  }

  @override
  bool get shareEnabled {
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.iOS:
        return !widget.obscureText &&
            !textEditingValue.selection.isCollapsed &&
            textEditingValue.selection.textInside(textEditingValue.text).trim() != '';
      case TargetPlatform.macOS:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        return false;
    }
  }

  @override
  bool get liveTextInputEnabled {
    return _liveTextInputStatus?.value == LiveTextInputStatus.enabled &&
        !widget.obscureText &&
        !widget.readOnly &&
        textEditingValue.selection.isCollapsed;
  }

  void _onChangedClipboardStatus() {
    setState(() {
      // Inform the widget that the value of clipboardStatus has changed.
    });
  }

  void _onChangedLiveTextInputStatus() {
    setState(() {
      // Inform the widget that the value of liveTextInputStatus has changed.
    });
  }

  TextEditingValue get _textEditingValueforTextLayoutMetrics {
    final Widget? editableWidget = _editableKey.currentContext?.widget;
    if (editableWidget is! _Editable) {
      throw StateError('_Editable must be mounted.');
    }
    return editableWidget.value;
  }

  /// Copy current selection to [Clipboard].
  @override
  void copySelection(SelectionChangedCause cause) {
    final TextSelection selection = textEditingValue.selection;
    if (selection.isCollapsed || widget.obscureText) {
      return;
    }
    final String text = textEditingValue.text;
    Clipboard.setData(ClipboardData(text: selection.textInside(text)));
    if (cause == SelectionChangedCause.toolbar) {
      bringIntoView(textEditingValue.selection.extent);
      hideToolbar(false);

      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          break;
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
          // Collapse the selection and hide the toolbar and handles.
          userUpdateTextEditingValue(
            TextEditingValue(
              text: textEditingValue.text,
              selection: TextSelection.collapsed(offset: textEditingValue.selection.end),
            ),
            SelectionChangedCause.toolbar,
          );
      }
    }
    clipboardStatus.update();
  }

  /// Cut current selection to [Clipboard].
  @override
  void cutSelection(SelectionChangedCause cause) {
    if (widget.readOnly || widget.obscureText) {
      return;
    }
    final TextSelection selection = textEditingValue.selection;
    final String text = textEditingValue.text;
    if (selection.isCollapsed) {
      return;
    }
    Clipboard.setData(ClipboardData(text: selection.textInside(text)));
    _replaceText(ReplaceTextIntent(textEditingValue, '', selection, cause));
    if (cause == SelectionChangedCause.toolbar) {
      // Schedule a call to bringIntoView() after renderEditable updates.
      SchedulerBinding.instance.addPostFrameCallback((_) {
        if (mounted) {
          bringIntoView(textEditingValue.selection.extent);
        }
      }, debugLabel: 'EditableText.bringSelectionIntoView');
      hideToolbar();
    }
    clipboardStatus.update();
  }

  bool get _allowPaste {
    return !widget.readOnly && textEditingValue.selection.isValid;
  }

  /// Paste text from [Clipboard].
  @override
  Future<void> pasteText(SelectionChangedCause cause) async {
    if (!_allowPaste) {
      return;
    }
    // Snapshot the input before using `await`.
    // See https://github.com/flutter/flutter/issues/11427
    final ClipboardData? data = await Clipboard.getData(Clipboard.kTextPlain);
    if (data == null) {
      return;
    }
    _pasteText(cause, data.text!);
  }

  void _pasteText(SelectionChangedCause cause, String text) {
    if (!_allowPaste) {
      return;
    }

    // After the paste, the cursor should be collapsed and located after the
    // pasted content.
    final TextSelection selection = textEditingValue.selection;
    final int lastSelectionIndex = math.max(selection.baseOffset, selection.extentOffset);
    final TextEditingValue collapsedTextEditingValue = textEditingValue.copyWith(
      selection: TextSelection.collapsed(offset: lastSelectionIndex),
    );

    userUpdateTextEditingValue(collapsedTextEditingValue.replaced(selection, text), cause);
    if (cause == SelectionChangedCause.toolbar) {
      // Schedule a call to bringIntoView() after renderEditable updates.
      SchedulerBinding.instance.addPostFrameCallback((_) {
        if (mounted) {
          bringIntoView(textEditingValue.selection.extent);
        }
      }, debugLabel: 'EditableText.bringSelectionIntoView');
      hideToolbar();
    }
  }

  /// Select the entire text value.
  @override
  void selectAll(SelectionChangedCause cause) {
    if (widget.readOnly && widget.obscureText) {
      // If we can't modify it, and we can't copy it, there's no point in
      // selecting it.
      return;
    }
    userUpdateTextEditingValue(
      textEditingValue.copyWith(
        selection: TextSelection(baseOffset: 0, extentOffset: textEditingValue.text.length),
      ),
      cause,
    );

    if (cause == SelectionChangedCause.toolbar) {
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
          break;
        case TargetPlatform.macOS:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          hideToolbar();
      }
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          bringIntoView(textEditingValue.selection.extent);
        case TargetPlatform.macOS:
        case TargetPlatform.iOS:
          break;
      }
    }
  }

  /// Look up the current selection,
  /// as in the "Look Up" edit menu button on iOS.
  ///
  /// Currently this is only implemented for iOS.
  ///
  /// Throws an error if the selection is empty or collapsed.
  Future<void> lookUpSelection(SelectionChangedCause cause) async {
    assert(!widget.obscureText);

    final String text = textEditingValue.selection.textInside(textEditingValue.text);
    if (widget.obscureText || text.isEmpty) {
      return;
    }
    await SystemChannels.platform.invokeMethod('LookUp.invoke', text);
  }

  /// Launch a web search on the current selection,
  /// as in the "Search Web" edit menu button on iOS.
  ///
  /// Currently this is only implemented for iOS.
  ///
  /// When 'obscureText' is true or the selection is empty,
  /// this function will not do anything
  Future<void> searchWebForSelection(SelectionChangedCause cause) async {
    assert(!widget.obscureText);
    if (widget.obscureText) {
      return;
    }

    final String text = textEditingValue.selection.textInside(textEditingValue.text);
    if (text.isNotEmpty) {
      await SystemChannels.platform.invokeMethod('SearchWeb.invoke', text);
    }
  }

  /// Launch the share interface for the current selection,
  /// as in the "Share..." edit menu button on iOS.
  ///
  /// Currently this is only implemented for iOS and Android.
  ///
  /// When 'obscureText' is true or the selection is empty,
  /// this function will not do anything
  Future<void> shareSelection(SelectionChangedCause cause) async {
    assert(!widget.obscureText);
    if (widget.obscureText) {
      return;
    }

    final String text = textEditingValue.selection.textInside(textEditingValue.text);
    if (text.isNotEmpty) {
      await SystemChannels.platform.invokeMethod('Share.invoke', text);
    }
  }

  void _startLiveTextInput(SelectionChangedCause cause) {
    if (!liveTextInputEnabled) {
      return;
    }
    if (_hasInputConnection) {
      LiveText.startLiveTextInput();
    }
    if (cause == SelectionChangedCause.toolbar) {
      hideToolbar();
    }
  }

  /// Finds specified [SuggestionSpan] that matches the provided index using
  /// binary search.
  ///
  /// See also:
  ///
  ///  * [SpellCheckSuggestionsToolbar], the Material style spell check
  ///    suggestions toolbar that uses this method to render the correct
  ///    suggestions in the toolbar for a misspelled word.
  SuggestionSpan? findSuggestionSpanAtCursorIndex(int cursorIndex) {
    if (!_spellCheckResultsReceived ||
        spellCheckResults!.suggestionSpans.last.range.end < cursorIndex) {
      // No spell check results have been received or the cursor index is out
      // of range that suggestionSpans covers.
      return null;
    }

    final List<SuggestionSpan> suggestionSpans = spellCheckResults!.suggestionSpans;
    int leftIndex = 0;
    int rightIndex = suggestionSpans.length - 1;
    int midIndex = 0;

    while (leftIndex <= rightIndex) {
      midIndex = ((leftIndex + rightIndex) / 2).floor();
      final int currentSpanStart = suggestionSpans[midIndex].range.start;
      final int currentSpanEnd = suggestionSpans[midIndex].range.end;

      if (cursorIndex <= currentSpanEnd && cursorIndex >= currentSpanStart) {
        return suggestionSpans[midIndex];
      } else if (cursorIndex <= currentSpanStart) {
        rightIndex = midIndex - 1;
      } else {
        leftIndex = midIndex + 1;
      }
    }
    return null;
  }

  /// Infers the [SpellCheckConfiguration] used to perform spell check.
  ///
  /// If spell check is enabled, this will try to infer a value for
  /// the [SpellCheckService] if left unspecified.
  static SpellCheckConfiguration _inferSpellCheckConfiguration(
    SpellCheckConfiguration? configuration,
  ) {
    final SpellCheckService? spellCheckService = configuration?.spellCheckService;
    final bool spellCheckAutomaticallyDisabled =
        configuration == null || configuration == const SpellCheckConfiguration.disabled();
    final bool spellCheckServiceIsConfigured =
        spellCheckService != null ||
        WidgetsBinding.instance.platformDispatcher.nativeSpellCheckServiceDefined;
    if (spellCheckAutomaticallyDisabled || !spellCheckServiceIsConfigured) {
      // Only enable spell check if a non-disabled configuration is provided
      // and if that configuration does not specify a spell check service,
      // a native spell checker must be supported.
      assert(() {
        if (!spellCheckAutomaticallyDisabled && !spellCheckServiceIsConfigured) {
          FlutterError.reportError(
            FlutterErrorDetails(
              exception: FlutterError(
                'Spell check was enabled with spellCheckConfiguration, but the '
                'current platform does not have a supported spell check '
                'service, and none was provided. Consider disabling spell '
                'check for this platform or passing a SpellCheckConfiguration '
                'with a specified spell check service.',
              ),
              library: 'widget library',
              stack: StackTrace.current,
            ),
          );
        }
        return true;
      }());
      return const SpellCheckConfiguration.disabled();
    }

    return configuration.copyWith(
      spellCheckService: spellCheckService ?? DefaultSpellCheckService(),
    );
  }

  /// Returns the [ContextMenuButtonItem]s for the given [ToolbarOptions].
  @Deprecated(
    'Use `contextMenuBuilder` instead of `toolbarOptions`. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  List<ContextMenuButtonItem>? buttonItemsForToolbarOptions([TargetPlatform? targetPlatform]) {
    final ToolbarOptions toolbarOptions = widget.toolbarOptions;
    if (toolbarOptions == ToolbarOptions.empty) {
      return null;
    }
    return <ContextMenuButtonItem>[
      if (toolbarOptions.cut && cutEnabled)
        ContextMenuButtonItem(
          onPressed: () {
            cutSelection(SelectionChangedCause.toolbar);
          },
          type: ContextMenuButtonType.cut,
        ),
      if (toolbarOptions.copy && copyEnabled)
        ContextMenuButtonItem(
          onPressed: () {
            copySelection(SelectionChangedCause.toolbar);
          },
          type: ContextMenuButtonType.copy,
        ),
      if (toolbarOptions.paste && pasteEnabled)
        ContextMenuButtonItem(
          onPressed: () {
            pasteText(SelectionChangedCause.toolbar);
          },
          type: ContextMenuButtonType.paste,
        ),
      if (toolbarOptions.selectAll && selectAllEnabled)
        ContextMenuButtonItem(
          onPressed: () {
            selectAll(SelectionChangedCause.toolbar);
          },
          type: ContextMenuButtonType.selectAll,
        ),
    ];
  }

  /// Gets the line heights at the start and end of the selection for the given
  /// [EditableTextState].
  ///
  /// See also:
  ///
  /// * [TextSelectionToolbarAnchors.getSelectionRect], which depends on this
  ///   information.
  ({double startGlyphHeight, double endGlyphHeight}) getGlyphHeights() {
    final TextSelection selection = textEditingValue.selection;

    // Only calculate handle rects if the text in the previous frame
    // is the same as the text in the current frame. This is done because
    // widget.renderObject contains the renderEditable from the previous frame.
    // If the text changed between the current and previous frames then
    // widget.renderObject.getRectForComposingRange might fail. In cases where
    // the current frame is different from the previous we fall back to
    // renderObject.preferredLineHeight.
    final InlineSpan span = renderEditable.text!;
    final String prevText = span.toPlainText();
    final String currText = textEditingValue.text;
    if (prevText != currText || !selection.isValid || selection.isCollapsed) {
      return (
        startGlyphHeight: renderEditable.preferredLineHeight,
        endGlyphHeight: renderEditable.preferredLineHeight,
      );
    }

    final String selectedGraphemes = selection.textInside(currText);
    final int firstSelectedGraphemeExtent = selectedGraphemes.characters.first.length;
    final Rect? startCharacterRect = renderEditable.getRectForComposingRange(
      TextRange(start: selection.start, end: selection.start + firstSelectedGraphemeExtent),
    );
    final int lastSelectedGraphemeExtent = selectedGraphemes.characters.last.length;
    final Rect? endCharacterRect = renderEditable.getRectForComposingRange(
      TextRange(start: selection.end - lastSelectedGraphemeExtent, end: selection.end),
    );
    return (
      startGlyphHeight: startCharacterRect?.height ?? renderEditable.preferredLineHeight,
      endGlyphHeight: endCharacterRect?.height ?? renderEditable.preferredLineHeight,
    );
  }

  /// {@template flutter.widgets.EditableText.getAnchors}
  /// Returns the anchor points for the default context menu.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [contextMenuButtonItems], which provides the [ContextMenuButtonItem]s
  ///    for the default context menu buttons.
  TextSelectionToolbarAnchors get contextMenuAnchors {
    if (renderEditable.lastSecondaryTapDownPosition != null) {
      return TextSelectionToolbarAnchors(
        primaryAnchor: renderEditable.lastSecondaryTapDownPosition!,
      );
    }

    final (startGlyphHeight: double startGlyphHeight, endGlyphHeight: double endGlyphHeight) =
        getGlyphHeights();
    final TextSelection selection = textEditingValue.selection;
    final List<TextSelectionPoint> points = renderEditable.getEndpointsForSelection(selection);
    return TextSelectionToolbarAnchors.fromSelection(
      renderBox: renderEditable,
      startGlyphHeight: startGlyphHeight,
      endGlyphHeight: endGlyphHeight,
      selectionEndpoints: points,
    );
  }

  /// Returns the [ContextMenuButtonItem]s representing the buttons in this
  /// platform's default selection menu for [EditableText].
  ///
  /// See also:
  ///
  /// * [EditableText.getEditableButtonItems], which performs a similar role,
  ///   but for any editable field, not just specifically EditableText.
  /// * [SelectableRegionState.contextMenuButtonItems], which performs a similar
  ///   role but for content that is selectable but not editable.
  /// * [contextMenuAnchors], which provides the anchor points for the default
  ///   context menu.
  /// * [AdaptiveTextSelectionToolbar], which builds the toolbar itself, and can
  ///   take a list of [ContextMenuButtonItem]s with
  ///   [AdaptiveTextSelectionToolbar.buttonItems].
  /// * [AdaptiveTextSelectionToolbar.getAdaptiveButtons], which builds the
  ///   button Widgets for the current platform given [ContextMenuButtonItem]s.
  List<ContextMenuButtonItem> get contextMenuButtonItems {
    return buttonItemsForToolbarOptions() ??
          EditableText.getEditableButtonItems(
            clipboardStatus: clipboardStatus.value,
            onCopy: copyEnabled ? () => copySelection(SelectionChangedCause.toolbar) : null,
            onCut: cutEnabled ? () => cutSelection(SelectionChangedCause.toolbar) : null,
            onPaste: pasteEnabled ? () => pasteText(SelectionChangedCause.toolbar) : null,
            onSelectAll: selectAllEnabled ? () => selectAll(SelectionChangedCause.toolbar) : null,
            onLookUp: lookUpEnabled ? () => lookUpSelection(SelectionChangedCause.toolbar) : null,
            onSearchWeb:
                searchWebEnabled
                    ? () => searchWebForSelection(SelectionChangedCause.toolbar)
                    : null,
            onShare: shareEnabled ? () => shareSelection(SelectionChangedCause.toolbar) : null,
            onLiveTextInput:
                liveTextInputEnabled
                    ? () => _startLiveTextInput(SelectionChangedCause.toolbar)
                    : null,
          )
      ..addAll(_textProcessingActionButtonItems);
  }

  List<ContextMenuButtonItem> get _textProcessingActionButtonItems {
    final List<ContextMenuButtonItem> buttonItems = <ContextMenuButtonItem>[];
    final TextSelection selection = textEditingValue.selection;
    if (widget.obscureText || !selection.isValid || selection.isCollapsed) {
      return buttonItems;
    }

    for (final ProcessTextAction action in _processTextActions) {
      buttonItems.add(
        ContextMenuButtonItem(
          label: action.label,
          onPressed: () async {
            final String selectedText = selection.textInside(textEditingValue.text);
            if (selectedText.isNotEmpty) {
              final String? processedText = await _processTextService.processTextAction(
                action.id,
                selectedText,
                widget.readOnly,
              );
              // If an activity does not return a modified version, just hide the toolbar.
              // Otherwise use the result to replace the selected text.
              if (processedText != null && _allowPaste) {
                _pasteText(SelectionChangedCause.toolbar, processedText);
              } else {
                hideToolbar();
              }
            }
          },
        ),
      );
    }
    return buttonItems;
  }

  // State lifecycle:

  @protected
  @override
  void initState() {
    super.initState();
    _liveTextInputStatus?.addListener(_onChangedLiveTextInputStatus);
    clipboardStatus.addListener(_onChangedClipboardStatus);
    widget.controller.addListener(_didChangeTextEditingValue);
    widget.focusNode.addListener(_handleFocusChanged);
    _cursorVisibilityNotifier.value = widget.showCursor;
    _spellCheckConfiguration = _inferSpellCheckConfiguration(widget.spellCheckConfiguration);
    _appLifecycleListener = AppLifecycleListener(onResume: () => _justResumed = true);
    _initProcessTextActions();
  }

  /// Query the engine to initialize the list of text processing actions to show
  /// in the text selection toolbar.
  Future<void> _initProcessTextActions() async {
    _processTextActions.clear();
    _processTextActions.addAll(await _processTextService.queryTextActions());
  }

  // Whether `TickerMode.of(context)` is true and animations (like blinking the
  // cursor) are supposed to run.
  bool _tickersEnabled = true;

  @protected
  @override
  void didChangeDependencies() {
    super.didChangeDependencies();

    _style =
        MediaQuery.boldTextOf(context)
            ? widget.style.merge(const TextStyle(fontWeight: FontWeight.bold))
            : widget.style;

    final AutofillGroupState? newAutofillGroup = AutofillGroup.maybeOf(context);
    if (currentAutofillScope != newAutofillGroup) {
      _currentAutofillScope?.unregister(autofillId);
      _currentAutofillScope = newAutofillGroup;
      _currentAutofillScope?.register(_effectiveAutofillClient);
    }

    if (!_didAutoFocus && widget.autofocus) {
      _didAutoFocus = true;
      SchedulerBinding.instance.addPostFrameCallback((_) {
        if (mounted && renderEditable.hasSize) {
          _flagInternalFocus();
          FocusScope.of(context).autofocus(widget.focusNode);
        }
      }, debugLabel: 'EditableText.autofocus');
    }

    // Restart or stop the blinking cursor when TickerMode changes.
    final bool newTickerEnabled = TickerMode.of(context);
    if (_tickersEnabled != newTickerEnabled) {
      _tickersEnabled = newTickerEnabled;
      if (_showBlinkingCursor) {
        _startCursorBlink();
      } else if (!_tickersEnabled && _cursorTimer != null) {
        _stopCursorBlink();
      }
    }

    // Check for changes in viewId.
    if (_hasInputConnection) {
      final int newViewId = View.of(context).viewId;
      if (newViewId != _viewId) {
        _textInputConnection!.updateConfig(_effectiveAutofillClient.textInputConfiguration);
      }
    }

    if (defaultTargetPlatform != TargetPlatform.iOS &&
        defaultTargetPlatform != TargetPlatform.android) {
      return;
    }

    // Hide the text selection toolbar on mobile when orientation changes.
    final Orientation orientation = MediaQuery.orientationOf(context);
    if (_lastOrientation == null) {
      _lastOrientation = orientation;
      return;
    }
    if (orientation != _lastOrientation) {
      _lastOrientation = orientation;
      if (defaultTargetPlatform == TargetPlatform.iOS) {
        hideToolbar(false);
      }
      if (defaultTargetPlatform == TargetPlatform.android) {
        hideToolbar();
      }
    }

    if (_listeningToScrollNotificationObserver) {
      // Only update subscription when we have previously subscribed to the
      // scroll notification observer. We only subscribe to the scroll
      // notification observer when the context menu is shown on platforms that
      // support _platformSupportsFadeOnScroll.
      _scrollNotificationObserver?.removeListener(_handleContextMenuOnParentScroll);
      _scrollNotificationObserver = ScrollNotificationObserver.maybeOf(context);
      _scrollNotificationObserver?.addListener(_handleContextMenuOnParentScroll);
    }
  }

  @protected
  @override
  void didUpdateWidget(EditableText oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.controller != oldWidget.controller) {
      oldWidget.controller.removeListener(_didChangeTextEditingValue);
      widget.controller.addListener(_didChangeTextEditingValue);
      _updateRemoteEditingValueIfNeeded();
    }

    if (_selectionOverlay != null &&
        (widget.contextMenuBuilder != oldWidget.contextMenuBuilder ||
            widget.selectionControls != oldWidget.selectionControls ||
            widget.onSelectionHandleTapped != oldWidget.onSelectionHandleTapped ||
            widget.dragStartBehavior != oldWidget.dragStartBehavior ||
            widget.magnifierConfiguration != oldWidget.magnifierConfiguration)) {
      final bool shouldShowToolbar = _selectionOverlay!.toolbarIsVisible;
      final bool shouldShowHandles = _selectionOverlay!.handlesVisible;
      _selectionOverlay!.dispose();
      _selectionOverlay = _createSelectionOverlay();
      if (shouldShowToolbar || shouldShowHandles) {
        SchedulerBinding.instance.addPostFrameCallback((Duration _) {
          if (shouldShowToolbar) {
            _selectionOverlay!.showToolbar();
          }
          if (shouldShowHandles) {
            _selectionOverlay!.showHandles();
          }
        });
      }
    } else if (widget.controller.selection != oldWidget.controller.selection) {
      _selectionOverlay?.update(_value);
    }
    _selectionOverlay?.handlesVisible = widget.showSelectionHandles;

    if (widget.autofillClient != oldWidget.autofillClient) {
      _currentAutofillScope?.unregister(oldWidget.autofillClient?.autofillId ?? autofillId);
      _currentAutofillScope?.register(_effectiveAutofillClient);
    }

    if (widget.focusNode != oldWidget.focusNode) {
      oldWidget.focusNode.removeListener(_handleFocusChanged);
      widget.focusNode.addListener(_handleFocusChanged);
      updateKeepAlive();
    }

    if (!_shouldCreateInputConnection) {
      _closeInputConnectionIfNeeded();
    } else if (oldWidget.readOnly && _hasFocus) {
      // _openInputConnection must be called after layout information is available.
      // See https://github.com/flutter/flutter/issues/126312
      SchedulerBinding.instance.addPostFrameCallback((Duration _) {
        _openInputConnection();
      }, debugLabel: 'EditableText.openInputConnection');
    }

    if (kIsWeb && _hasInputConnection) {
      if (oldWidget.readOnly != widget.readOnly) {
        _textInputConnection!.updateConfig(_effectiveAutofillClient.textInputConfiguration);
      }
    }

    if (_hasInputConnection) {
      if (oldWidget.obscureText != widget.obscureText) {
        _textInputConnection!.updateConfig(_effectiveAutofillClient.textInputConfiguration);
      }
    }

    if (widget.style != oldWidget.style) {
      // The _textInputConnection will pick up the new style when it attaches in
      // _openInputConnection.
      _style =
          MediaQuery.boldTextOf(context)
              ? widget.style.merge(const TextStyle(fontWeight: FontWeight.bold))
              : widget.style;
      if (_hasInputConnection) {
        _textInputConnection!.setStyle(
          fontFamily: _style.fontFamily,
          fontSize: _style.fontSize,
          fontWeight: _style.fontWeight,
          textDirection: _textDirection,
          textAlign: widget.textAlign,
        );
      }
    }

    if (widget.showCursor != oldWidget.showCursor) {
      _startOrStopCursorTimerIfNeeded();
    }
    final bool canPaste =
        widget.selectionControls is TextSelectionHandleControls
            ? pasteEnabled
            : widget.selectionControls?.canPaste(this) ?? false;
    if (widget.selectionEnabled && pasteEnabled && canPaste) {
      clipboardStatus.update();
    }
  }

  void _disposeScrollNotificationObserver() {
    _listeningToScrollNotificationObserver = false;
    if (_scrollNotificationObserver != null) {
      _scrollNotificationObserver!.removeListener(_handleContextMenuOnParentScroll);
      _scrollNotificationObserver = null;
    }
  }

  @protected
  @override
  void dispose() {
    _internalScrollController?.dispose();
    _currentAutofillScope?.unregister(autofillId);
    widget.controller.removeListener(_didChangeTextEditingValue);
    _floatingCursorResetController?.dispose();
    _floatingCursorResetController = null;
    _closeInputConnectionIfNeeded();
    assert(!_hasInputConnection);
    _cursorTimer?.cancel();
    _cursorTimer = null;
    _backingCursorBlinkOpacityController?.dispose();
    _backingCursorBlinkOpacityController = null;
    _selectionOverlay?.dispose();
    _selectionOverlay = null;
    widget.focusNode.removeListener(_handleFocusChanged);
    WidgetsBinding.instance.removeObserver(this);
    _liveTextInputStatus?.removeListener(_onChangedLiveTextInputStatus);
    _liveTextInputStatus?.dispose();
    clipboardStatus.removeListener(_onChangedClipboardStatus);
    clipboardStatus.dispose();
    _cursorVisibilityNotifier.dispose();
    _appLifecycleListener.dispose();
    FocusManager.instance.removeListener(_unflagInternalFocus);
    _disposeScrollNotificationObserver();
    super.dispose();
    assert(_batchEditDepth <= 0, 'unfinished batch edits: $_batchEditDepth');
  }

  // TextInputClient implementation:

  /// The last known [TextEditingValue] of the platform text input plugin.
  ///
  /// This value is updated when the platform text input plugin sends a new
  /// update via [updateEditingValue], or when [EditableText] calls
  /// [TextInputConnection.setEditingState] to overwrite the platform text input
  /// plugin's [TextEditingValue].
  ///
  /// Used in [_updateRemoteEditingValueIfNeeded] to determine whether the
  /// remote value is outdated and needs updating.
  TextEditingValue? _lastKnownRemoteTextEditingValue;

  @override
  TextEditingValue get currentTextEditingValue => _value;

  @override
  void updateEditingValue(TextEditingValue value) {
    // This method handles text editing state updates from the platform text
    // input plugin. The [EditableText] may not have the focus or an open input
    // connection, as autofill can update a disconnected [EditableText].

    // Since we still have to support keyboard select, this is the best place
    // to disable text updating.
    if (!_shouldCreateInputConnection) {
      return;
    }

    if (_checkNeedsAdjustAffinity(value)) {
      value = value.copyWith(
        selection: value.selection.copyWith(affinity: _value.selection.affinity),
      );
    }

    if (widget.readOnly) {
      // In the read-only case, we only care about selection changes, and reject
      // everything else.
      value = _value.copyWith(selection: value.selection);
    }
    _lastKnownRemoteTextEditingValue = value;

    if (value == _value) {
      // This is possible, for example, when the numeric keyboard is input,
      // the engine will notify twice for the same value.
      // Track at https://github.com/flutter/flutter/issues/65811
      return;
    }

    if (value.text == _value.text && value.composing == _value.composing) {
      // `selection` is the only change.
      SelectionChangedCause cause;
      if (_textInputConnection?.scribbleInProgress ?? false) {
        cause = SelectionChangedCause.scribble;
      } else if (_pointOffsetOrigin != null) {
        // For floating cursor selection when force pressing the space bar.
        cause = SelectionChangedCause.forcePress;
      } else {
        cause = SelectionChangedCause.keyboard;
      }
      _handleSelectionChanged(value.selection, cause);
    } else {
      if (value.text != _value.text) {
        // Hide the toolbar if the text was changed, but only hide the toolbar
        // overlay; the selection handle's visibility will be handled
        // by `_handleSelectionChanged`. https://github.com/flutter/flutter/issues/108673
        hideToolbar(false);
      }
      _currentPromptRectRange = null;

      final bool revealObscuredInput =
          _hasInputConnection &&
          widget.obscureText &&
          WidgetsBinding.instance.platformDispatcher.brieflyShowPassword &&
          value.text.length == _value.text.length + 1;

      _obscureShowCharTicksPending = revealObscuredInput ? _kObscureShowLatestCharCursorTicks : 0;
      _obscureLatestCharIndex = revealObscuredInput ? _value.selection.baseOffset : null;
      _formatAndSetValue(value, SelectionChangedCause.keyboard);
    }

    if (_showBlinkingCursor && _cursorTimer != null) {
      // To keep the cursor from blinking while typing, restart the timer here.
      _stopCursorBlink(resetCharTicks: false);
      _startCursorBlink();
    }

    // Wherever the value is changed by the user, schedule a showCaretOnScreen
    // to make sure the user can see the changes they just made. Programmatic
    // changes to `textEditingValue` do not trigger the behavior even if the
    // text field is focused.
    _scheduleShowCaretOnScreen(withAnimation: true);
  }

  bool _checkNeedsAdjustAffinity(TextEditingValue value) {
    // Trust the engine affinity if the text changes or selection changes.
    return value.text == _value.text &&
        value.selection.isCollapsed == _value.selection.isCollapsed &&
        value.selection.start == _value.selection.start &&
        value.selection.affinity != _value.selection.affinity;
  }

  @override
  void performAction(TextInputAction action) {
    switch (action) {
      case TextInputAction.newline:
        // If this is a multiline EditableText, do nothing for a "newline"
        // action; The newline is already inserted. Otherwise, finalize
        // editing.
        if (!_isMultiline) {
          _finalizeEditing(action, shouldUnfocus: true);
        }
      case TextInputAction.done:
      case TextInputAction.go:
      case TextInputAction.next:
      case TextInputAction.previous:
      case TextInputAction.search:
      case TextInputAction.send:
        _finalizeEditing(action, shouldUnfocus: true);
      case TextInputAction.continueAction:
      case TextInputAction.emergencyCall:
      case TextInputAction.join:
      case TextInputAction.none:
      case TextInputAction.route:
      case TextInputAction.unspecified:
        // Finalize editing, but don't give up focus because this keyboard
        // action does not imply the user is done inputting information.
        _finalizeEditing(action, shouldUnfocus: false);
    }
  }

  @override
  void performPrivateCommand(String action, Map<String, dynamic> data) {
    widget.onAppPrivateCommand?.call(action, data);
  }

  @override
  void insertContent(KeyboardInsertedContent content) {
    assert(
      widget.contentInsertionConfiguration?.allowedMimeTypes.contains(content.mimeType) ?? false,
    );
    widget.contentInsertionConfiguration?.onContentInserted.call(content);
  }

  // The original position of the caret on FloatingCursorDragState.start.
  Offset? _startCaretCenter;

  // The most recent text position as determined by the location of the floating
  // cursor.
  TextPosition? _lastTextPosition;

  // The offset of the floating cursor as determined from the start call.
  Offset? _pointOffsetOrigin;

  // The most recent position of the floating cursor.
  Offset? _lastBoundedOffset;

  // Because the center of the cursor is preferredLineHeight / 2 below the touch
  // origin, but the touch origin is used to determine which line the cursor is
  // on, we need this offset to correctly render and move the cursor.
  Offset get _floatingCursorOffset => Offset(0, renderEditable.preferredLineHeight / 2);

  @override
  void updateFloatingCursor(RawFloatingCursorPoint point) {
    _floatingCursorResetController ??= AnimationController(vsync: this)
      ..addListener(_onFloatingCursorResetTick);
    switch (point.state) {
      case FloatingCursorDragState.Start:
        if (_floatingCursorResetController!.isAnimating) {
          _floatingCursorResetController!.stop();
          _onFloatingCursorResetTick();
        }
        // Stop cursor blinking and making it visible.
        _stopCursorBlink(resetCharTicks: false);
        _cursorBlinkOpacityController.value = 1.0;
        // We want to send in points that are centered around a (0,0) origin, so
        // we cache the position.
        _pointOffsetOrigin = point.offset;

        final Offset startCaretCenter;
        final TextPosition currentTextPosition;
        final bool shouldResetOrigin;
        // Only non-null when starting a floating cursor via long press.
        if (point.startLocation != null) {
          shouldResetOrigin = false;
          (startCaretCenter, currentTextPosition) = point.startLocation!;
        } else {
          shouldResetOrigin = true;
          currentTextPosition = TextPosition(
            offset: renderEditable.selection!.baseOffset,
            affinity: renderEditable.selection!.affinity,
          );
          startCaretCenter = renderEditable.getLocalRectForCaret(currentTextPosition).center;
        }

        _startCaretCenter = startCaretCenter;
        _lastBoundedOffset = renderEditable.calculateBoundedFloatingCursorOffset(
          _startCaretCenter! - _floatingCursorOffset,
          shouldResetOrigin: shouldResetOrigin,
        );
        _lastTextPosition = currentTextPosition;
        renderEditable.setFloatingCursor(point.state, _lastBoundedOffset!, _lastTextPosition!);
      case FloatingCursorDragState.Update:
        final Offset centeredPoint = point.offset! - _pointOffsetOrigin!;
        final Offset rawCursorOffset = _startCaretCenter! + centeredPoint - _floatingCursorOffset;

        _lastBoundedOffset = renderEditable.calculateBoundedFloatingCursorOffset(rawCursorOffset);
        _lastTextPosition = renderEditable.getPositionForPoint(
          renderEditable.localToGlobal(_lastBoundedOffset! + _floatingCursorOffset),
        );
        renderEditable.setFloatingCursor(point.state, _lastBoundedOffset!, _lastTextPosition!);
      case FloatingCursorDragState.End:
        // Resume cursor blinking.
        _startCursorBlink();
        // We skip animation if no update has happened.
        if (_lastTextPosition != null && _lastBoundedOffset != null) {
          _floatingCursorResetController!.value = 0.0;
          _floatingCursorResetController!.animateTo(
            1.0,
            duration: _floatingCursorResetTime,
            curve: Curves.decelerate,
          );
        }
    }
  }

  void _onFloatingCursorResetTick() {
    final Offset finalPosition =
        renderEditable.getLocalRectForCaret(_lastTextPosition!).centerLeft - _floatingCursorOffset;
    if (_floatingCursorResetController!.isCompleted) {
      renderEditable.setFloatingCursor(
        FloatingCursorDragState.End,
        finalPosition,
        _lastTextPosition!,
      );
      // During a floating cursor's move gesture (1 finger), a cursor is
      // animated only visually, without actually updating the selection.
      // Only after move gesture is complete, this function will be called
      // to actually update the selection to the new cursor location with
      // zero selection length.

      // However, During a floating cursor's selection gesture (2 fingers), the
      // selection is constantly updated by the engine throughout the gesture.
      // Thus when the gesture is complete, we should not update the selection
      // to the cursor location with zero selection length, because that would
      // overwrite the selection made by floating cursor selection.

      // Here we use `isCollapsed` to distinguish between floating cursor's
      // move gesture (1 finger) vs selection gesture (2 fingers), as
      // the engine does not provide information other than notifying a
      // new selection during with selection gesture (2 fingers).
      if (renderEditable.selection!.isCollapsed) {
        // The cause is technically the force cursor, but the cause is listed as tap as the desired functionality is the same.
        _handleSelectionChanged(
          TextSelection.fromPosition(_lastTextPosition!),
          SelectionChangedCause.forcePress,
        );
      }
      _startCaretCenter = null;
      _lastTextPosition = null;
      _pointOffsetOrigin = null;
      _lastBoundedOffset = null;
    } else {
      final double lerpValue = _floatingCursorResetController!.value;
      final double lerpX = ui.lerpDouble(_lastBoundedOffset!.dx, finalPosition.dx, lerpValue)!;
      final double lerpY = ui.lerpDouble(_lastBoundedOffset!.dy, finalPosition.dy, lerpValue)!;

      renderEditable.setFloatingCursor(
        FloatingCursorDragState.Update,
        Offset(lerpX, lerpY),
        _lastTextPosition!,
        resetLerpValue: lerpValue,
      );
    }
  }

  @pragma('vm:notify-debugger-on-exception')
  void _finalizeEditing(TextInputAction action, {required bool shouldUnfocus}) {
    // Take any actions necessary now that the user has completed editing.
    if (widget.onEditingComplete != null) {
      try {
        widget.onEditingComplete!();
      } catch (exception, stack) {
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            library: 'widgets',
            context: ErrorDescription('while calling onEditingComplete for $action'),
          ),
        );
      }
    } else {
      // Default behavior if the developer did not provide an
      // onEditingComplete callback: Finalize editing and remove focus, or move
      // it to the next/previous field, depending on the action.
      widget.controller.clearComposing();
      if (shouldUnfocus) {
        switch (action) {
          case TextInputAction.none:
          case TextInputAction.unspecified:
          case TextInputAction.done:
          case TextInputAction.go:
          case TextInputAction.search:
          case TextInputAction.send:
          case TextInputAction.continueAction:
          case TextInputAction.join:
          case TextInputAction.route:
          case TextInputAction.emergencyCall:
          case TextInputAction.newline:
            widget.focusNode.unfocus();
          case TextInputAction.next:
            widget.focusNode.nextFocus();
          case TextInputAction.previous:
            widget.focusNode.previousFocus();
        }
      }
    }

    final ValueChanged<String>? onSubmitted = widget.onSubmitted;
    if (onSubmitted == null) {
      return;
    }

    // Invoke optional callback with the user's submitted content.
    try {
      onSubmitted(_value.text);
    } catch (exception, stack) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: exception,
          stack: stack,
          library: 'widgets',
          context: ErrorDescription('while calling onSubmitted for $action'),
        ),
      );
    }

    // If `shouldUnfocus` is true, the text field should no longer be focused
    // after the microtask queue is drained. But in case the developer cancelled
    // the focus change in the `onSubmitted` callback by focusing this input
    // field again, reset the soft keyboard.
    // See https://github.com/flutter/flutter/issues/84240.
    //
    // `_restartConnectionIfNeeded` creates a new TextInputConnection to replace
    // the current one. This on iOS switches to a new input view and on Android
    // restarts the input method, and in both cases the soft keyboard will be
    // reset.
    if (shouldUnfocus) {
      _scheduleRestartConnection();
    }
  }

  int _batchEditDepth = 0;

  /// Begins a new batch edit, within which new updates made to the text editing
  /// value will not be sent to the platform text input plugin.
  ///
  /// Batch edits nest. When the outermost batch edit finishes, [endBatchEdit]
  /// will attempt to send [currentTextEditingValue] to the text input plugin if
  /// it detected a change.
  void beginBatchEdit() {
    _batchEditDepth += 1;
  }

  /// Ends the current batch edit started by the last call to [beginBatchEdit],
  /// and send [currentTextEditingValue] to the text input plugin if needed.
  ///
  /// Throws an error in debug mode if this [EditableText] is not in a batch
  /// edit.
  void endBatchEdit() {
    _batchEditDepth -= 1;
    assert(
      _batchEditDepth >= 0,
      'Unbalanced call to endBatchEdit: beginBatchEdit must be called first.',
    );
    _updateRemoteEditingValueIfNeeded();
  }

  void _updateRemoteEditingValueIfNeeded() {
    if (_batchEditDepth > 0 || !_hasInputConnection) {
      return;
    }
    final TextEditingValue localValue = _value;
    if (localValue == _lastKnownRemoteTextEditingValue) {
      return;
    }
    _textInputConnection!.setEditingState(localValue);
    _lastKnownRemoteTextEditingValue = localValue;
  }

  TextEditingValue get _value => widget.controller.value;
  set _value(TextEditingValue value) {
    widget.controller.value = value;
  }

  bool get _hasFocus => widget.focusNode.hasFocus;
  bool get _isMultiline => widget.maxLines != 1;

  // Finds the closest scroll offset to the current scroll offset that fully
  // reveals the given caret rect. If the given rect's main axis extent is too
  // large to be fully revealed in `renderEditable`, it will be centered along
  // the main axis.
  //
  // If this is a multiline EditableText (which means the Editable can only
  // scroll vertically), the given rect's height will first be extended to match
  // `renderEditable.preferredLineHeight`, before the target scroll offset is
  // calculated.
  RevealedOffset _getOffsetToRevealCaret(Rect rect) {
    if (!_scrollController.position.allowImplicitScrolling) {
      return RevealedOffset(offset: _scrollController.offset, rect: rect);
    }

    final Size editableSize = renderEditable.size;
    final double additionalOffset;
    final Offset unitOffset;

    if (!_isMultiline) {
      additionalOffset =
          rect.width >= editableSize.width
              // Center `rect` if it's oversized.
              ? editableSize.width / 2 - rect.center.dx
              // Valid additional offsets range from (rect.right - size.width)
              // to (rect.left). Pick the closest one if out of range.
              : clampDouble(0.0, rect.right - editableSize.width, rect.left);
      unitOffset = const Offset(1, 0);
    } else {
      // The caret is vertically centered within the line. Expand the caret's
      // height so that it spans the line because we're going to ensure that the
      // entire expanded caret is scrolled into view.
      final Rect expandedRect = Rect.fromCenter(
        center: rect.center,
        width: rect.width,
        height: math.max(rect.height, renderEditable.preferredLineHeight),
      );

      additionalOffset =
          expandedRect.height >= editableSize.height
              ? editableSize.height / 2 - expandedRect.center.dy
              : clampDouble(0.0, expandedRect.bottom - editableSize.height, expandedRect.top);
      unitOffset = const Offset(0, 1);
    }

    // No overscrolling when encountering tall fonts/scripts that extend past
    // the ascent.
    final double targetOffset = clampDouble(
      additionalOffset + _scrollController.offset,
      _scrollController.position.minScrollExtent,
      _scrollController.position.maxScrollExtent,
    );

    final double offsetDelta = _scrollController.offset - targetOffset;
    return RevealedOffset(rect: rect.shift(unitOffset * offsetDelta), offset: targetOffset);
  }

  /// Whether to send the autofill information to the autofill service. True by
  /// default.
  bool get _needsAutofill =>
      _effectiveAutofillClient.textInputConfiguration.autofillConfiguration.enabled;

  // Must be called after layout.
  // See https://github.com/flutter/flutter/issues/126312
  void _openInputConnection() {
    if (!_shouldCreateInputConnection) {
      return;
    }
    if (!_hasInputConnection) {
      final TextEditingValue localValue = _value;

      // When _needsAutofill == true && currentAutofillScope == null, autofill
      // is allowed but saving the user input from the text field is
      // discouraged.
      //
      // In case the autofillScope changes from a non-null value to null, or
      // _needsAutofill changes to false from true, the platform needs to be
      // notified to exclude this field from the autofill context. So we need to
      // provide the autofillId.
      _textInputConnection =
          _needsAutofill && currentAutofillScope != null
              ? currentAutofillScope!.attach(this, _effectiveAutofillClient.textInputConfiguration)
              : TextInput.attach(this, _effectiveAutofillClient.textInputConfiguration);
      _updateSizeAndTransform();
      _schedulePeriodicPostFrameCallbacks();
      _textInputConnection!
        ..setStyle(
          fontFamily: _style.fontFamily,
          fontSize: _style.fontSize,
          fontWeight: _style.fontWeight,
          textDirection: _textDirection,
          textAlign: widget.textAlign,
        )
        ..setEditingState(localValue)
        ..show();
      if (_needsAutofill) {
        // Request autofill AFTER the size and the transform have been sent to
        // the platform text input plugin.
        _textInputConnection!.requestAutofill();
      }
      _lastKnownRemoteTextEditingValue = localValue;
    } else {
      _textInputConnection!.show();
    }
  }

  void _closeInputConnectionIfNeeded() {
    if (_hasInputConnection) {
      _textInputConnection!.close();
      _textInputConnection = null;
      _lastKnownRemoteTextEditingValue = null;
      _scribbleCacheKey = null;
      removeTextPlaceholder();
    }
  }

  void _openOrCloseInputConnectionIfNeeded() {
    if (_hasFocus && widget.focusNode.consumeKeyboardToken()) {
      _openInputConnection();
    } else if (!_hasFocus) {
      _closeInputConnectionIfNeeded();
      widget.controller.clearComposing();
    }
  }

  bool _restartConnectionScheduled = false;
  void _scheduleRestartConnection() {
    if (_restartConnectionScheduled) {
      return;
    }
    _restartConnectionScheduled = true;
    scheduleMicrotask(_restartConnectionIfNeeded);
  }

  // Discards the current [TextInputConnection] and establishes a new one.
  //
  // This method is rarely needed. This is currently used to reset the input
  // type when the "submit" text input action is triggered and the developer
  // puts the focus back to this input field..
  void _restartConnectionIfNeeded() {
    _restartConnectionScheduled = false;
    if (!_hasInputConnection || !_shouldCreateInputConnection) {
      return;
    }
    _textInputConnection!.close();
    _textInputConnection = null;
    _lastKnownRemoteTextEditingValue = null;

    final AutofillScope? currentAutofillScope = _needsAutofill ? this.currentAutofillScope : null;
    final TextInputConnection newConnection =
        currentAutofillScope?.attach(this, textInputConfiguration) ??
        TextInput.attach(this, _effectiveAutofillClient.textInputConfiguration);
    _textInputConnection = newConnection;

    newConnection
      ..show()
      ..setStyle(
        fontFamily: _style.fontFamily,
        fontSize: _style.fontSize,
        fontWeight: _style.fontWeight,
        textDirection: _textDirection,
        textAlign: widget.textAlign,
      )
      ..setEditingState(_value);
    _lastKnownRemoteTextEditingValue = _value;
  }

  @override
  void didChangeInputControl(TextInputControl? oldControl, TextInputControl? newControl) {
    if (_hasFocus && _hasInputConnection) {
      oldControl?.hide();
      newControl?.show();
    }
  }

  @override
  void connectionClosed() {
    if (_hasInputConnection) {
      _textInputConnection!.connectionClosedReceived();
      _textInputConnection = null;
      _lastKnownRemoteTextEditingValue = null;
      widget.focusNode.unfocus();
    }
  }

  // Indicates that a call to _handleFocusChanged originated within
  // EditableText, allowing it to distinguish between internal and external
  // focus changes.
  bool _nextFocusChangeIsInternal = false;

  // Sets _nextFocusChangeIsInternal to true only until any subsequent focus
  // change happens.
  void _flagInternalFocus() {
    _nextFocusChangeIsInternal = true;
    FocusManager.instance.addListener(_unflagInternalFocus);
  }

  void _unflagInternalFocus() {
    _nextFocusChangeIsInternal = false;
    FocusManager.instance.removeListener(_unflagInternalFocus);
  }

  /// Express interest in interacting with the keyboard.
  ///
  /// If this control is already attached to the keyboard, this function will
  /// request that the keyboard become visible. Otherwise, this function will
  /// ask the focus system that it become focused. If successful in acquiring
  /// focus, the control will then attach to the keyboard and request that the
  /// keyboard become visible.
  void requestKeyboard() {
    if (_hasFocus) {
      _openInputConnection();
    } else {
      _flagInternalFocus();
      widget.focusNode
          .requestFocus(); // This eventually calls _openInputConnection also, see _handleFocusChanged.
    }
  }

  void _updateOrDisposeSelectionOverlayIfNeeded() {
    if (_selectionOverlay != null) {
      if (_hasFocus) {
        _selectionOverlay!.update(_value);
      } else {
        _selectionOverlay!.dispose();
        _selectionOverlay = null;
      }
    }
  }

  final bool _platformSupportsFadeOnScroll = switch (defaultTargetPlatform) {
    TargetPlatform.android || TargetPlatform.iOS => true,
    TargetPlatform.fuchsia ||
    TargetPlatform.linux ||
    TargetPlatform.macOS ||
    TargetPlatform.windows => false,
  };

  bool _isInternalScrollableNotification(BuildContext? notificationContext) {
    final ScrollableState? scrollableState =
        notificationContext?.findAncestorStateOfType<ScrollableState>();
    return _scrollableKey.currentContext == scrollableState?.context;
  }

  bool _scrollableNotificationIsFromSameSubtree(BuildContext? notificationContext) {
    if (notificationContext == null) {
      return false;
    }
    BuildContext? currentContext = context;
    // The notification context of a ScrollNotification points to the RawGestureDetector
    // of the Scrollable. We get the ScrollableState associated with this notification
    // by looking up the tree.
    final ScrollableState? notificationScrollableState =
        notificationContext.findAncestorStateOfType<ScrollableState>();
    if (notificationScrollableState == null) {
      return false;
    }
    while (currentContext != null) {
      final ScrollableState? scrollableState =
          currentContext.findAncestorStateOfType<ScrollableState>();
      if (scrollableState == notificationScrollableState) {
        return true;
      }
      currentContext = scrollableState?.context;
    }
    return false;
  }

  void _handleContextMenuOnParentScroll(ScrollNotification notification) {
    // Do some preliminary checks to avoid expensive subtree traversal.
    if (notification is! ScrollStartNotification && notification is! ScrollEndNotification) {
      return;
    }
    switch (notification) {
      case ScrollStartNotification() when _dataWhenToolbarShowScheduled != null:
      case ScrollEndNotification() when _dataWhenToolbarShowScheduled == null:
        break;
      case ScrollEndNotification() when _dataWhenToolbarShowScheduled!.value != _value:
        _dataWhenToolbarShowScheduled = null;
        _disposeScrollNotificationObserver();
      case ScrollNotification(:final BuildContext? context)
          when !_isInternalScrollableNotification(context) &&
              _scrollableNotificationIsFromSameSubtree(context):
        _handleContextMenuOnScroll(notification);
    }
  }

  Rect _calculateDeviceRect() {
    final Size screenSize = MediaQuery.sizeOf(context);
    final ui.FlutterView view = View.of(context);
    final double obscuredVertical =
        (view.padding.top + view.padding.bottom + view.viewInsets.bottom) / view.devicePixelRatio;
    final double obscuredHorizontal =
        (view.padding.left + view.padding.right) / view.devicePixelRatio;
    final Size visibleScreenSize = Size(
      screenSize.width - obscuredHorizontal,
      screenSize.height - obscuredVertical,
    );
    return Rect.fromLTWH(
      view.padding.left / view.devicePixelRatio,
      view.padding.top / view.devicePixelRatio,
      visibleScreenSize.width,
      visibleScreenSize.height,
    );
  }

  bool _showToolbarOnScreenScheduled = false;
  void _handleContextMenuOnScroll(ScrollNotification notification) {
    if (_webContextMenuEnabled) {
      return;
    }
    if (!_platformSupportsFadeOnScroll) {
      _selectionOverlay?.updateForScroll();
      return;
    }
    // When the scroll begins and the toolbar is visible, hide it
    // until scrolling ends.
    //
    // The selection and renderEditable need to be visible within the current
    // viewport for the toolbar to show when scrolling ends. If they are not
    // then the toolbar is shown when they are scrolled back into view, unless
    // invalidated by a change in TextEditingValue.
    if (notification is ScrollStartNotification) {
      if (_dataWhenToolbarShowScheduled != null) {
        return;
      }
      final bool toolbarIsVisible =
          _selectionOverlay != null &&
          _selectionOverlay!.toolbarIsVisible &&
          !_selectionOverlay!.spellCheckToolbarIsVisible;
      if (!toolbarIsVisible) {
        return;
      }
      final List<TextBox> selectionBoxes = renderEditable.getBoxesForSelection(_value.selection);
      final Rect selectionBounds =
          _value.selection.isCollapsed || selectionBoxes.isEmpty
              ? renderEditable.getLocalRectForCaret(_value.selection.extent)
              : selectionBoxes
                  .map((TextBox box) => box.toRect())
                  .reduce((Rect result, Rect rect) => result.expandToInclude(rect));
      _dataWhenToolbarShowScheduled = (value: _value, selectionBounds: selectionBounds);
      _selectionOverlay?.hideToolbar();
    } else if (notification is ScrollEndNotification) {
      if (_dataWhenToolbarShowScheduled == null) {
        return;
      }
      if (_dataWhenToolbarShowScheduled!.value != _value) {
        // Value has changed so we should invalidate any toolbar scheduling.
        _dataWhenToolbarShowScheduled = null;
        _disposeScrollNotificationObserver();
        return;
      }

      if (_showToolbarOnScreenScheduled) {
        return;
      }
      _showToolbarOnScreenScheduled = true;
      SchedulerBinding.instance.addPostFrameCallback((Duration _) {
        _showToolbarOnScreenScheduled = false;
        if (!mounted) {
          return;
        }
        final Rect deviceRect = _calculateDeviceRect();
        final bool selectionVisibleInEditable =
            renderEditable.selectionStartInViewport.value ||
            renderEditable.selectionEndInViewport.value;
        final Rect selectionBounds = MatrixUtils.transformRect(
          renderEditable.getTransformTo(null),
          _dataWhenToolbarShowScheduled!.selectionBounds,
        );
        final bool selectionOverlapsWithDeviceRect =
            !selectionBounds.hasNaN && deviceRect.overlaps(selectionBounds);

        if (selectionVisibleInEditable &&
            selectionOverlapsWithDeviceRect &&
            _selectionInViewport(_dataWhenToolbarShowScheduled!.selectionBounds)) {
          showToolbar();
          _dataWhenToolbarShowScheduled = null;
        }
      }, debugLabel: 'EditableText.scheduleToolbar');
    }
  }

  bool _selectionInViewport(Rect selectionBounds) {
    RenderAbstractViewport? closestViewport = RenderAbstractViewport.maybeOf(renderEditable);
    while (closestViewport != null) {
      final Rect selectionBoundsLocalToViewport = MatrixUtils.transformRect(
        renderEditable.getTransformTo(closestViewport),
        selectionBounds,
      );
      if (selectionBoundsLocalToViewport.hasNaN ||
          closestViewport.paintBounds.hasNaN ||
          !closestViewport.paintBounds.overlaps(selectionBoundsLocalToViewport)) {
        return false;
      }
      closestViewport = RenderAbstractViewport.maybeOf(closestViewport.parent);
    }
    return true;
  }

  TextSelectionOverlay _createSelectionOverlay() {
    final EditableTextContextMenuBuilder? contextMenuBuilder = widget.contextMenuBuilder;
    final TextSelectionOverlay selectionOverlay = TextSelectionOverlay(
      clipboardStatus: clipboardStatus,
      context: context,
      value: _value,
      debugRequiredFor: widget,
      toolbarLayerLink: _toolbarLayerLink,
      startHandleLayerLink: _startHandleLayerLink,
      endHandleLayerLink: _endHandleLayerLink,
      renderObject: renderEditable,
      selectionControls: widget.selectionControls,
      selectionDelegate: this,
      dragStartBehavior: widget.dragStartBehavior,
      onSelectionHandleTapped: widget.onSelectionHandleTapped,
      contextMenuBuilder:
          contextMenuBuilder == null || _webContextMenuEnabled
              ? null
              : (BuildContext context) {
                return contextMenuBuilder(context, this);
              },
      magnifierConfiguration: widget.magnifierConfiguration,
    );

    return selectionOverlay;
  }

  @pragma('vm:notify-debugger-on-exception')
  void _handleSelectionChanged(TextSelection selection, SelectionChangedCause? cause) {
    // We return early if the selection is not valid. This can happen when the
    // text of [EditableText] is updated at the same time as the selection is
    // changed by a gesture event.
    final String text = widget.controller.value.text;
    if (text.length < selection.end || text.length < selection.start) {
      return;
    }

    widget.controller.selection = selection;

    // This will show the keyboard for all selection changes on the
    // EditableText except for those triggered by a keyboard input.
    // Typically EditableText shouldn't take user keyboard input if
    // it's not focused already. If the EditableText is being
    // autofilled it shouldn't request focus.
    switch (cause) {
      case null:
      case SelectionChangedCause.doubleTap:
      case SelectionChangedCause.drag:
      case SelectionChangedCause.forcePress:
      case SelectionChangedCause.longPress:
      case SelectionChangedCause.scribble:
      case SelectionChangedCause.tap:
      case SelectionChangedCause.toolbar:
        requestKeyboard();
      case SelectionChangedCause.keyboard:
    }
    if (widget.selectionControls == null && widget.contextMenuBuilder == null) {
      _selectionOverlay?.dispose();
      _selectionOverlay = null;
    } else {
      if (_selectionOverlay == null) {
        _selectionOverlay = _createSelectionOverlay();
      } else {
        _selectionOverlay!.update(_value);
      }
      _selectionOverlay!.handlesVisible = widget.showSelectionHandles;
      _selectionOverlay!.showHandles();
    }
    // TODO(chunhtai): we should make sure selection actually changed before
    // we call the onSelectionChanged.
    // https://github.com/flutter/flutter/issues/76349.
    try {
      widget.onSelectionChanged?.call(selection, cause);
    } catch (exception, stack) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: exception,
          stack: stack,
          library: 'widgets',
          context: ErrorDescription('while calling onSelectionChanged for $cause'),
        ),
      );
    }

    // To keep the cursor from blinking while it moves, restart the timer here.
    if (_showBlinkingCursor && _cursorTimer != null) {
      _stopCursorBlink(resetCharTicks: false);
      _startCursorBlink();
    }
  }

  // Animation configuration for scrolling the caret back on screen.
  static const Duration _caretAnimationDuration = Duration(milliseconds: 100);
  static const Curve _caretAnimationCurve = Curves.fastOutSlowIn;

  bool _showCaretOnScreenScheduled = false;

  void _scheduleShowCaretOnScreen({required bool withAnimation}) {
    if (_showCaretOnScreenScheduled) {
      return;
    }
    _showCaretOnScreenScheduled = true;
    SchedulerBinding.instance.addPostFrameCallback((Duration _) {
      _showCaretOnScreenScheduled = false;
      // Since we are in a post frame callback, check currentContext in case
      // RenderEditable has been disposed (in which case it will be null).
      final RenderEditable? renderEditable =
          _editableKey.currentContext?.findRenderObject() as RenderEditable?;
      if (renderEditable == null ||
          !(renderEditable.selection?.isValid ?? false) ||
          !_scrollController.hasClients) {
        return;
      }

      final double lineHeight = renderEditable.preferredLineHeight;

      // Enlarge the target rect by scrollPadding to ensure that caret is not
      // positioned directly at the edge after scrolling.
      double bottomSpacing = widget.scrollPadding.bottom;
      if (_selectionOverlay?.selectionControls != null) {
        final double handleHeight =
            _selectionOverlay!.selectionControls!.getHandleSize(lineHeight).height;
        final double interactiveHandleHeight = math.max(handleHeight, kMinInteractiveDimension);
        final Offset anchor = _selectionOverlay!.selectionControls!.getHandleAnchor(
          TextSelectionHandleType.collapsed,
          lineHeight,
        );
        final double handleCenter = handleHeight / 2 - anchor.dy;
        bottomSpacing = math.max(handleCenter + interactiveHandleHeight / 2, bottomSpacing);
      }

      final EdgeInsets caretPadding = widget.scrollPadding.copyWith(bottom: bottomSpacing);

      final Rect caretRect = renderEditable.getLocalRectForCaret(renderEditable.selection!.extent);
      final RevealedOffset targetOffset = _getOffsetToRevealCaret(caretRect);

      final Rect rectToReveal;
      final TextSelection selection = textEditingValue.selection;
      if (selection.isCollapsed) {
        rectToReveal = targetOffset.rect;
      } else {
        final List<TextBox> selectionBoxes = renderEditable.getBoxesForSelection(selection);
        // selectionBoxes may be empty if, for example, the selection does not
        // encompass a full character, like if it only contained part of an
        // extended grapheme cluster.
        if (selectionBoxes.isEmpty) {
          rectToReveal = targetOffset.rect;
        } else {
          rectToReveal =
              selection.baseOffset < selection.extentOffset
                  ? selectionBoxes.last.toRect()
                  : selectionBoxes.first.toRect();
        }
      }

      if (withAnimation) {
        _scrollController.animateTo(
          targetOffset.offset,
          duration: _caretAnimationDuration,
          curve: _caretAnimationCurve,
        );
        renderEditable.showOnScreen(
          rect: caretPadding.inflateRect(rectToReveal),
          duration: _caretAnimationDuration,
          curve: _caretAnimationCurve,
        );
      } else {
        _scrollController.jumpTo(targetOffset.offset);
        renderEditable.showOnScreen(rect: caretPadding.inflateRect(rectToReveal));
      }
    }, debugLabel: 'EditableText.showCaret');
  }

  late double _lastBottomViewInset;

  @override
  void didChangeMetrics() {
    if (!mounted) {
      return;
    }
    final ui.FlutterView view = View.of(context);
    if (_lastBottomViewInset != view.viewInsets.bottom) {
      SchedulerBinding.instance.addPostFrameCallback((Duration _) {
        _selectionOverlay?.updateForScroll();
      }, debugLabel: 'EditableText.updateForScroll');
      if (_lastBottomViewInset < view.viewInsets.bottom) {
        // Because the metrics change signal from engine will come here every frame
        // (on both iOS and Android). So we don't need to show caret with animation.
        _scheduleShowCaretOnScreen(withAnimation: false);
      }
    }
    _lastBottomViewInset = view.viewInsets.bottom;
  }

  Future<void> _performSpellCheck(final String text) async {
    try {
      final Locale? localeForSpellChecking = widget.locale ?? Localizations.maybeLocaleOf(context);

      assert(
        localeForSpellChecking != null,
        'Locale must be specified in widget or Localization widget must be in scope',
      );

      final List<SuggestionSpan>? suggestions = await _spellCheckConfiguration.spellCheckService!
          .fetchSpellCheckSuggestions(localeForSpellChecking!, text);

      if (suggestions == null) {
        // The request to fetch spell check suggestions was canceled due to ongoing request.
        return;
      }

      spellCheckResults = SpellCheckResults(text, suggestions);
      renderEditable.text = buildTextSpan();
    } catch (exception, stack) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: exception,
          stack: stack,
          library: 'widgets',
          context: ErrorDescription('while performing spell check'),
        ),
      );
    }
  }

  @pragma('vm:notify-debugger-on-exception')
  void _formatAndSetValue(
    TextEditingValue value,
    SelectionChangedCause? cause, {
    bool userInteraction = false,
  }) {
    final TextEditingValue oldValue = _value;
    final bool textChanged = oldValue.text != value.text;
    final bool textCommitted = !oldValue.composing.isCollapsed && value.composing.isCollapsed;
    final bool selectionChanged = oldValue.selection != value.selection;

    if (textChanged || textCommitted) {
      // Only apply input formatters if the text has changed (including uncommitted
      // text in the composing region), or when the user committed the composing
      // text.
      // Gboard is very persistent in restoring the composing region. Applying
      // input formatters on composing-region-only changes (except clearing the
      // current composing region) is very infinite-loop-prone: the formatters
      // will keep trying to modify the composing region while Gboard will keep
      // trying to restore the original composing region.
      try {
        value =
            widget.inputFormatters?.fold<TextEditingValue>(
              value,
              (TextEditingValue newValue, TextInputFormatter formatter) =>
                  formatter.formatEditUpdate(_value, newValue),
            ) ??
            value;

        if (spellCheckEnabled && value.text.isNotEmpty && _value.text != value.text) {
          _performSpellCheck(value.text);
        }
      } catch (exception, stack) {
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            library: 'widgets',
            context: ErrorDescription('while applying input formatters'),
          ),
        );
      }
    }

    final TextSelection oldTextSelection = textEditingValue.selection;

    // Put all optional user callback invocations in a batch edit to prevent
    // sending multiple `TextInput.updateEditingValue` messages.
    beginBatchEdit();
    _value = value;
    // Changes made by the keyboard can sometimes be "out of band" for listening
    // components, so always send those events, even if we didn't think it
    // changed. Also, the user long pressing should always send a selection change
    // as well.
    if (selectionChanged ||
        (userInteraction &&
            (cause == SelectionChangedCause.longPress ||
                cause == SelectionChangedCause.keyboard))) {
      _handleSelectionChanged(_value.selection, cause);
      _bringIntoViewBySelectionState(oldTextSelection, value.selection, cause);
    }
    final String currentText = _value.text;
    if (oldValue.text != currentText) {
      try {
        widget.onChanged?.call(currentText);
      } catch (exception, stack) {
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            library: 'widgets',
            context: ErrorDescription('while calling onChanged'),
          ),
        );
      }
    }
    endBatchEdit();
  }

  void _bringIntoViewBySelectionState(
    TextSelection oldSelection,
    TextSelection newSelection,
    SelectionChangedCause? cause,
  ) {
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        if (cause == SelectionChangedCause.longPress || cause == SelectionChangedCause.drag) {
          bringIntoView(newSelection.extent);
        }
      case TargetPlatform.linux:
      case TargetPlatform.windows:
      case TargetPlatform.fuchsia:
      case TargetPlatform.android:
        if (cause == SelectionChangedCause.drag) {
          if (oldSelection.baseOffset != newSelection.baseOffset) {
            bringIntoView(newSelection.base);
          } else if (oldSelection.extentOffset != newSelection.extentOffset) {
            bringIntoView(newSelection.extent);
          }
        }
    }
  }

  void _onCursorColorTick() {
    final double effectiveOpacity = math.min(
      widget.cursorColor.alpha / 255.0,
      _cursorBlinkOpacityController.value,
    );
    renderEditable.cursorColor = widget.cursorColor.withOpacity(effectiveOpacity);
    _cursorVisibilityNotifier.value =
        widget.showCursor &&
        (EditableText.debugDeterministicCursor || _cursorBlinkOpacityController.value > 0);
  }

  bool get _showBlinkingCursor =>
      _hasFocus &&
      _value.selection.isCollapsed &&
      widget.showCursor &&
      _tickersEnabled &&
      !renderEditable.floatingCursorOn;

  /// Whether the blinking cursor is actually visible at this precise moment
  /// (it's hidden half the time, since it blinks).
  @visibleForTesting
  bool get cursorCurrentlyVisible => _cursorBlinkOpacityController.value > 0;

  /// The cursor blink interval (the amount of time the cursor is in the "on"
  /// state or the "off" state). A complete cursor blink period is twice this
  /// value (half on, half off).
  @visibleForTesting
  Duration get cursorBlinkInterval => _kCursorBlinkHalfPeriod;

  /// The current status of the text selection handles.
  @visibleForTesting
  TextSelectionOverlay? get selectionOverlay => _selectionOverlay;

  int _obscureShowCharTicksPending = 0;
  int? _obscureLatestCharIndex;

  void _startCursorBlink() {
    assert(
      !(_cursorTimer?.isActive ?? false) ||
          !(_backingCursorBlinkOpacityController?.isAnimating ?? false),
    );
    if (!widget.showCursor) {
      return;
    }
    if (!_tickersEnabled) {
      return;
    }
    _cursorTimer?.cancel();
    _cursorBlinkOpacityController.value = 1.0;
    if (EditableText.debugDeterministicCursor) {
      return;
    }
    if (widget.cursorOpacityAnimates) {
      _cursorBlinkOpacityController
          .animateWith(_iosBlinkCursorSimulation)
          .whenComplete(_onCursorTick);
    } else {
      _cursorTimer = Timer.periodic(_kCursorBlinkHalfPeriod, (Timer timer) {
        _onCursorTick();
      });
    }
  }

  void _onCursorTick() {
    if (_obscureShowCharTicksPending > 0) {
      _obscureShowCharTicksPending =
          WidgetsBinding.instance.platformDispatcher.brieflyShowPassword
              ? _obscureShowCharTicksPending - 1
              : 0;
      if (_obscureShowCharTicksPending == 0) {
        setState(() {});
      }
    }

    if (widget.cursorOpacityAnimates) {
      _cursorTimer?.cancel();
      // Schedule this as an async task to avoid blocking tester.pumpAndSettle
      // indefinitely.
      _cursorTimer = Timer(
        Duration.zero,
        () => _cursorBlinkOpacityController
            .animateWith(_iosBlinkCursorSimulation)
            .whenComplete(_onCursorTick),
      );
    } else {
      if (!(_cursorTimer?.isActive ?? false) && _tickersEnabled) {
        _cursorTimer = Timer.periodic(_kCursorBlinkHalfPeriod, (Timer timer) {
          _onCursorTick();
        });
      }
      _cursorBlinkOpacityController.value = _cursorBlinkOpacityController.value == 0 ? 1 : 0;
    }
  }

  void _stopCursorBlink({bool resetCharTicks = true}) {
    // If the cursor is animating, stop the animation, and we always
    // want the cursor to be visible when the floating cursor is enabled.
    _cursorBlinkOpacityController.value = renderEditable.floatingCursorOn ? 1.0 : 0.0;
    _cursorTimer?.cancel();
    _cursorTimer = null;
    if (resetCharTicks) {
      _obscureShowCharTicksPending = 0;
    }
  }

  void _startOrStopCursorTimerIfNeeded() {
    if (!_showBlinkingCursor) {
      _stopCursorBlink();
    } else if (_cursorTimer == null) {
      _startCursorBlink();
    }
  }

  void _didChangeTextEditingValue() {
    if (_hasFocus && !_value.selection.isValid) {
      // If this field is focused and the selection is invalid, place the cursor at
      // the end. Does not rely on _handleFocusChanged because it makes selection
      // handles visible on Android.
      // Unregister as a listener to the text controller while making the change.
      widget.controller.removeListener(_didChangeTextEditingValue);
      widget.controller.selection = _adjustedSelectionWhenFocused()!;
      widget.controller.addListener(_didChangeTextEditingValue);
    }
    _updateRemoteEditingValueIfNeeded();
    _startOrStopCursorTimerIfNeeded();
    _updateOrDisposeSelectionOverlayIfNeeded();
    // TODO(abarth): Teach RenderEditable about ValueNotifier<TextEditingValue>
    // to avoid this setState().
    setState(() {
      /* We use widget.controller.value in build(). */
    });
    _verticalSelectionUpdateAction.stopCurrentVerticalRunIfSelectionChanges();
  }

  void _handleFocusChanged() {
    _openOrCloseInputConnectionIfNeeded();
    _startOrStopCursorTimerIfNeeded();
    _updateOrDisposeSelectionOverlayIfNeeded();
    if (_hasFocus) {
      // Listen for changing viewInsets, which indicates keyboard showing up.
      WidgetsBinding.instance.addObserver(this);
      _lastBottomViewInset = View.of(context).viewInsets.bottom;
      if (!widget.readOnly) {
        _scheduleShowCaretOnScreen(withAnimation: true);
      }
      final TextSelection? updatedSelection = _adjustedSelectionWhenFocused();
      if (updatedSelection != null) {
        _handleSelectionChanged(updatedSelection, null);
      }
    } else {
      WidgetsBinding.instance.removeObserver(this);
      setState(() {
        _currentPromptRectRange = null;
      });
    }
    updateKeepAlive();
  }

  TextSelection? _adjustedSelectionWhenFocused() {
    TextSelection? selection;
    final bool isDesktop = switch (defaultTargetPlatform) {
      TargetPlatform.android || TargetPlatform.iOS || TargetPlatform.fuchsia => false,
      TargetPlatform.macOS || TargetPlatform.linux || TargetPlatform.windows => true,
    };
    final bool shouldSelectAll =
        widget.selectionEnabled &&
        (kIsWeb || isDesktop) &&
        !_isMultiline &&
        !_nextFocusChangeIsInternal &&
        !_justResumed;
    _justResumed = false;
    if (shouldSelectAll) {
      // On native web and desktop platforms, single line <input> tags
      // select all when receiving focus.
      selection = TextSelection(baseOffset: 0, extentOffset: _value.text.length);
    } else if (!_value.selection.isValid) {
      // Place cursor at the end if the selection is invalid when we receive focus.
      selection = TextSelection.collapsed(offset: _value.text.length);
    }
    return selection;
  }

  void _compositeCallback(Layer layer) {
    // The callback can be invoked when the layer is detached.
    // The input connection can be closed by the platform in which case this
    // widget doesn't rebuild.
    if (!renderEditable.attached || !_hasInputConnection) {
      return;
    }
    assert(mounted);
    assert((context as Element).debugIsActive);
    _updateSizeAndTransform();
  }

  // Must be called after layout.
  // See https://github.com/flutter/flutter/issues/126312
  void _updateSizeAndTransform() {
    final Size size = renderEditable.size;
    final Matrix4 transform = renderEditable.getTransformTo(null);
    _textInputConnection!.setEditableSizeAndTransform(size, transform);
  }

  void _schedulePeriodicPostFrameCallbacks([Duration? duration]) {
    if (!_hasInputConnection) {
      return;
    }
    _updateSelectionRects();
    _updateComposingRectIfNeeded();
    _updateCaretRectIfNeeded();
    SchedulerBinding.instance.addPostFrameCallback(
      _schedulePeriodicPostFrameCallbacks,
      debugLabel: 'EditableText.postFrameCallbacks',
    );
  }

  _ScribbleCacheKey? _scribbleCacheKey;

  void _updateSelectionRects({bool force = false}) {
    if (!_stylusHandwritingEnabled || defaultTargetPlatform != TargetPlatform.iOS) {
      return;
    }

    final ScrollDirection scrollDirection = _scrollController.position.userScrollDirection;
    if (scrollDirection != ScrollDirection.idle) {
      return;
    }

    final InlineSpan inlineSpan = renderEditable.text!;
    final TextScaler effectiveTextScaler = switch ((widget.textScaler, widget.textScaleFactor)) {
      (final TextScaler textScaler, _) => textScaler,
      (null, final double textScaleFactor) => TextScaler.linear(textScaleFactor),
      (null, null) => MediaQuery.textScalerOf(context),
    };

    final _ScribbleCacheKey newCacheKey = _ScribbleCacheKey(
      inlineSpan: inlineSpan,
      textAlign: widget.textAlign,
      textDirection: _textDirection,
      textScaler: effectiveTextScaler,
      textHeightBehavior: widget.textHeightBehavior ?? DefaultTextHeightBehavior.maybeOf(context),
      locale: widget.locale,
      structStyle: widget.strutStyle,
      placeholder: _placeholderLocation,
      size: renderEditable.size,
    );

    final RenderComparison comparison =
        force
            ? RenderComparison.layout
            : _scribbleCacheKey?.compare(newCacheKey) ?? RenderComparison.layout;
    if (comparison.index < RenderComparison.layout.index) {
      return;
    }
    _scribbleCacheKey = newCacheKey;

    final List<SelectionRect> rects = <SelectionRect>[];
    int graphemeStart = 0;
    // Can't use _value.text here: the controller value could change between
    // frames.
    final String plainText = inlineSpan.toPlainText(includeSemanticsLabels: false);
    final CharacterRange characterRange = CharacterRange(plainText);
    while (characterRange.moveNext()) {
      final int graphemeEnd = graphemeStart + characterRange.current.length;
      final List<TextBox> boxes = renderEditable.getBoxesForSelection(
        TextSelection(baseOffset: graphemeStart, extentOffset: graphemeEnd),
      );

      final TextBox? box = boxes.isEmpty ? null : boxes.first;
      if (box != null) {
        final Rect paintBounds = renderEditable.paintBounds;
        // Stop early when characters are already below the bottom edge of the
        // RenderEditable, regardless of its clipBehavior.
        if (paintBounds.bottom <= box.top) {
          break;
        }
        // Include any TextBox which intersects with the RenderEditable.
        if (paintBounds.left <= box.right &&
            box.left <= paintBounds.right &&
            paintBounds.top <= box.bottom) {
          // At least some part of the letter is visible within the text field.
          rects.add(
            SelectionRect(position: graphemeStart, bounds: box.toRect(), direction: box.direction),
          );
        }
      }
      graphemeStart = graphemeEnd;
    }
    _textInputConnection!.setSelectionRects(rects);
  }

  // Sends the current composing rect to the embedder's text input plugin.
  //
  // In cases where the composing rect hasn't been updated in the embedder due
  // to the lag of asynchronous messages over the channel, the position of the
  // current caret rect is used instead.
  //
  // See: [_updateCaretRectIfNeeded]
  void _updateComposingRectIfNeeded() {
    final TextRange composingRange = _value.composing;
    assert(mounted);
    Rect? composingRect = renderEditable.getRectForComposingRange(composingRange);
    // Send the caret location instead if there's no marked text yet.
    if (composingRect == null) {
      final int offset = composingRange.isValid ? composingRange.start : 0;
      composingRect = renderEditable.getLocalRectForCaret(TextPosition(offset: offset));
    }
    _textInputConnection!.setComposingRect(composingRect);
  }

  // Sends the current caret rect to the embedder's text input plugin.
  //
  // The position of the caret rect is updated periodically such that if the
  // user initiates composing input, the current cursor rect can be used for
  // the first character until the composing rect can be sent.
  //
  // On selection changes, the start of the selection is used. This ensures
  // that regardless of the direction the selection was created, the cursor is
  // set to the position where next text input occurs. This position is used to
  // position the IME's candidate selection menu.
  //
  // See: [_updateComposingRectIfNeeded]
  void _updateCaretRectIfNeeded() {
    final TextSelection? selection = renderEditable.selection;
    if (selection == null || !selection.isValid) {
      return;
    }
    final TextPosition currentTextPosition = TextPosition(offset: selection.start);
    final Rect caretRect = renderEditable.getLocalRectForCaret(currentTextPosition);
    _textInputConnection!.setCaretRect(caretRect);
  }

  TextDirection get _textDirection => widget.textDirection ?? Directionality.of(context);

  /// The renderer for this widget's descendant.
  ///
  /// This property is typically used to notify the renderer of input gestures
  /// when [RenderEditable.ignorePointer] is true.
  late final RenderEditable renderEditable =
      _editableKey.currentContext!.findRenderObject()! as RenderEditable;

  @override
  TextEditingValue get textEditingValue => _value;

  double get _devicePixelRatio => MediaQuery.devicePixelRatioOf(context);

  @override
  void userUpdateTextEditingValue(TextEditingValue value, SelectionChangedCause? cause) {
    // Compare the current TextEditingValue with the pre-format new
    // TextEditingValue value, in case the formatter would reject the change.
    final bool shouldShowCaret =
        widget.readOnly ? _value.selection != value.selection : _value != value;
    if (shouldShowCaret) {
      _scheduleShowCaretOnScreen(withAnimation: true);
    }

    // Even if the value doesn't change, it may be necessary to focus and build
    // the selection overlay. For example, this happens when right clicking an
    // unfocused field that previously had a selection in the same spot.
    if (value == textEditingValue) {
      if (!widget.focusNode.hasFocus) {
        _flagInternalFocus();
        widget.focusNode.requestFocus();
        _selectionOverlay ??= _createSelectionOverlay();
      }
      return;
    }

    _formatAndSetValue(value, cause, userInteraction: true);
  }

  @override
  void bringIntoView(TextPosition position) {
    final Rect localRect = renderEditable.getLocalRectForCaret(position);
    final RevealedOffset targetOffset = _getOffsetToRevealCaret(localRect);

    _scrollController.jumpTo(targetOffset.offset);
    renderEditable.showOnScreen(rect: targetOffset.rect);
  }

  /// Shows the selection toolbar at the location of the current cursor.
  ///
  /// Returns `false` if a toolbar couldn't be shown, such as when the toolbar
  /// is already shown, or when no text selection currently exists.
  @override
  bool showToolbar() {
    // Web is using native dom elements to enable clipboard functionality of the
    // context menu: copy, paste, select, cut. It might also provide additional
    // functionality depending on the browser (such as translate). Due to this,
    // we should not show a Flutter toolbar for the editable text elements
    // unless the browser's context menu is explicitly disabled.
    if (_webContextMenuEnabled) {
      return false;
    }

    if (_selectionOverlay == null) {
      return false;
    }
    if (_selectionOverlay!.toolbarIsVisible) {
      return false;
    }
    _liveTextInputStatus?.update();
    clipboardStatus.update();
    _selectionOverlay!.showToolbar();
    // Listen to parent scroll events when the toolbar is visible so it can be
    // hidden during a scroll on supported platforms.
    if (_platformSupportsFadeOnScroll) {
      _listeningToScrollNotificationObserver = true;
      _scrollNotificationObserver?.removeListener(_handleContextMenuOnParentScroll);
      _scrollNotificationObserver = ScrollNotificationObserver.maybeOf(context);
      _scrollNotificationObserver?.addListener(_handleContextMenuOnParentScroll);
    }
    return true;
  }

  @override
  void hideToolbar([bool hideHandles = true]) {
    // Stop listening to parent scroll events when toolbar is hidden.
    _disposeScrollNotificationObserver();
    if (hideHandles) {
      // Hide the handles and the toolbar.
      _selectionOverlay?.hide();
    } else if (_selectionOverlay?.toolbarIsVisible ?? false) {
      // Hide only the toolbar but not the handles.
      _selectionOverlay?.hideToolbar();
    }
  }

  /// Toggles the visibility of the toolbar.
  void toggleToolbar([bool hideHandles = true]) {
    final TextSelectionOverlay selectionOverlay = _selectionOverlay ??= _createSelectionOverlay();
    if (selectionOverlay.toolbarIsVisible) {
      hideToolbar(hideHandles);
    } else {
      showToolbar();
    }
  }

  /// Shows toolbar with spell check suggestions of misspelled words that are
  /// available for click-and-replace.
  bool showSpellCheckSuggestionsToolbar() {
    // Spell check suggestions toolbars are intended to be shown on non-web
    // platforms. Additionally, the Cupertino style toolbar can't be drawn on
    // the web with the HTML renderer due to
    // https://github.com/flutter/flutter/issues/123560.
    if (!spellCheckEnabled ||
        _webContextMenuEnabled ||
        widget.readOnly ||
        _selectionOverlay == null ||
        !_spellCheckResultsReceived ||
        findSuggestionSpanAtCursorIndex(textEditingValue.selection.extentOffset) == null) {
      // Only attempt to show the spell check suggestions toolbar if there
      // is a toolbar specified and spell check suggestions available to show.
      return false;
    }

    assert(
      _spellCheckConfiguration.spellCheckSuggestionsToolbarBuilder != null,
      'spellCheckSuggestionsToolbarBuilder must be defined in '
      'SpellCheckConfiguration to show a toolbar with spell check '
      'suggestions',
    );

    _selectionOverlay!.showSpellCheckSuggestionsToolbar((BuildContext context) {
      return _spellCheckConfiguration.spellCheckSuggestionsToolbarBuilder!(context, this);
    });
    return true;
  }

  /// Shows the magnifier at the position given by `positionToShow`,
  /// if there is no magnifier visible.
  ///
  /// Updates the magnifier to the position given by `positionToShow`,
  /// if there is a magnifier visible.
  ///
  /// Does nothing if a magnifier couldn't be shown, such as when the selection
  /// overlay does not currently exist.
  void showMagnifier(Offset positionToShow) {
    if (_selectionOverlay == null) {
      return;
    }

    if (_selectionOverlay!.magnifierIsVisible) {
      _selectionOverlay!.updateMagnifier(positionToShow);
    } else {
      _selectionOverlay!.showMagnifier(positionToShow);
    }
  }

  /// Hides the magnifier if it is visible.
  void hideMagnifier() {
    if (_selectionOverlay == null) {
      return;
    }

    if (_selectionOverlay!.magnifierIsVisible) {
      _selectionOverlay!.hideMagnifier();
    }
  }

  // Tracks the location a [_ScribblePlaceholder] should be rendered in the
  // text.
  //
  // A value of -1 indicates there should be no placeholder, otherwise the
  // value should be between 0 and the length of the text, inclusive.
  int _placeholderLocation = -1;

  @override
  void insertTextPlaceholder(Size size) {
    if (!_stylusHandwritingEnabled) {
      return;
    }

    if (!widget.controller.selection.isValid) {
      return;
    }

    setState(() {
      _placeholderLocation = _value.text.length - widget.controller.selection.end;
    });
  }

  @override
  void removeTextPlaceholder() {
    if (!_stylusHandwritingEnabled || _placeholderLocation == -1) {
      return;
    }

    setState(() {
      _placeholderLocation = -1;
    });
  }

  @override
  void performSelector(String selectorName) {
    final Intent? intent = intentForMacOSSelector(selectorName);

    if (intent != null) {
      final BuildContext? primaryContext = primaryFocus?.context;
      if (primaryContext != null) {
        Actions.invoke(primaryContext, intent);
      }
    }
  }

  @override
  String get autofillId => 'EditableText-$hashCode';

  int? _viewId;

  @override
  TextInputConfiguration get textInputConfiguration {
    final List<String>? autofillHints = widget.autofillHints?.toList(growable: false);
    final AutofillConfiguration autofillConfiguration =
        autofillHints != null
            ? AutofillConfiguration(
              uniqueIdentifier: autofillId,
              autofillHints: autofillHints,
              currentEditingValue: currentTextEditingValue,
            )
            : AutofillConfiguration.disabled;

    _viewId = View.of(context).viewId;
    return TextInputConfiguration(
      viewId: _viewId,
      inputType: widget.keyboardType,
      readOnly: widget.readOnly,
      obscureText: widget.obscureText,
      autocorrect: widget.autocorrect,
      smartDashesType: widget.smartDashesType,
      smartQuotesType: widget.smartQuotesType,
      enableSuggestions: widget.enableSuggestions,
      enableInteractiveSelection: widget._userSelectionEnabled,
      inputAction:
          widget.textInputAction ??
          (widget.keyboardType == TextInputType.multiline
              ? TextInputAction.newline
              : TextInputAction.done),
      textCapitalization: widget.textCapitalization,
      keyboardAppearance: widget.keyboardAppearance,
      autofillConfiguration: autofillConfiguration,
      enableIMEPersonalizedLearning: widget.enableIMEPersonalizedLearning,
      allowedMimeTypes:
          widget.contentInsertionConfiguration == null
              ? const <String>[]
              : widget.contentInsertionConfiguration!.allowedMimeTypes,
    );
  }

  @override
  void autofill(TextEditingValue value) => updateEditingValue(value);

  // null if no promptRect should be shown.
  TextRange? _currentPromptRectRange;

  @override
  void showAutocorrectionPromptRect(int start, int end) {
    setState(() {
      _currentPromptRectRange = TextRange(start: start, end: end);
    });
  }

  VoidCallback? _semanticsOnCopy(TextSelectionControls? controls) {
    return widget.selectionEnabled &&
            _hasFocus &&
            (widget.selectionControls is TextSelectionHandleControls
                ? copyEnabled
                : copyEnabled && (widget.selectionControls?.canCopy(this) ?? false))
        ? () {
          controls?.handleCopy(this);
          copySelection(SelectionChangedCause.toolbar);
        }
        : null;
  }

  VoidCallback? _semanticsOnCut(TextSelectionControls? controls) {
    return widget.selectionEnabled &&
            _hasFocus &&
            (widget.selectionControls is TextSelectionHandleControls
                ? cutEnabled
                : cutEnabled && (widget.selectionControls?.canCut(this) ?? false))
        ? () {
          controls?.handleCut(this);
          cutSelection(SelectionChangedCause.toolbar);
        }
        : null;
  }

  VoidCallback? _semanticsOnPaste(TextSelectionControls? controls) {
    return widget.selectionEnabled &&
            _hasFocus &&
            (widget.selectionControls is TextSelectionHandleControls
                ? pasteEnabled
                : pasteEnabled && (widget.selectionControls?.canPaste(this) ?? false)) &&
            (clipboardStatus.value == ClipboardStatus.pasteable)
        ? () {
          controls?.handlePaste(this);
          pasteText(SelectionChangedCause.toolbar);
        }
        : null;
  }

  // Returns the closest boundary location to `extent` but not including `extent`
  // itself (unless already at the start/end of the text), in the direction
  // specified by `forward`.
  TextPosition _moveBeyondTextBoundary(
    TextPosition extent,
    bool forward,
    TextBoundary textBoundary,
  ) {
    assert(extent.offset >= 0);
    final int newOffset =
        forward
            ? textBoundary.getTrailingTextBoundaryAt(extent.offset) ?? _value.text.length
            // if x is a boundary defined by `textBoundary`, most textBoundaries (except
            // LineBreaker) guarantees `x == textBoundary.getLeadingTextBoundaryAt(x)`.
            // Use x - 1 here to make sure we don't get stuck at the fixed point x.
            : textBoundary.getLeadingTextBoundaryAt(extent.offset - 1) ?? 0;
    return TextPosition(offset: newOffset);
  }

  // Returns the closest boundary location to `extent`, including `extent`
  // itself, in the direction specified by `forward`.
  //
  // This method returns a fixed point of itself: applying `_toTextBoundary`
  // again on the returned TextPosition gives the same TextPosition. It's used
  // exclusively for handling line boundaries, since performing "move to line
  // start" more than once usually doesn't move you to the previous line.
  TextPosition _moveToTextBoundary(TextPosition extent, bool forward, TextBoundary textBoundary) {
    assert(extent.offset >= 0);
    final int caretOffset;
    switch (extent.affinity) {
      case TextAffinity.upstream:
        if (extent.offset < 1 && !forward) {
          assert(extent.offset == 0);
          return const TextPosition(offset: 0);
        }
        // When the text affinity is upstream, the caret is associated with the
        // grapheme before the code unit at `extent.offset`.
        // TODO(LongCatIsLooong): don't assume extent.offset is at a grapheme
        // boundary, and do this instead:
        // final int graphemeStart = CharacterRange.at(string, extent.offset).stringBeforeLength - 1;
        caretOffset = math.max(0, extent.offset - 1);
      case TextAffinity.downstream:
        caretOffset = extent.offset;
    }
    // The line boundary range does not include some control characters
    // (most notably, Line Feed), in which case there's
    // `x ∉ getTextBoundaryAt(x)`. In case `caretOffset` points to one such
    // control character, we define that these control characters themselves are
    // still part of the previous line, but also exclude them from the
    // line boundary range since they're non-printing. IOW, no additional
    // processing needed since the LineBoundary class does exactly that.
    return forward
        ? TextPosition(
          offset: textBoundary.getTrailingTextBoundaryAt(caretOffset) ?? _value.text.length,
          affinity: TextAffinity.upstream,
        )
        : TextPosition(offset: textBoundary.getLeadingTextBoundaryAt(caretOffset) ?? 0);
  }

  // --------------------------- Text Editing Actions ---------------------------

  TextBoundary _characterBoundary() =>
      widget.obscureText ? _CodePointBoundary(_value.text) : CharacterBoundary(_value.text);
  TextBoundary _nextWordBoundary() =>
      widget.obscureText ? _documentBoundary() : renderEditable.wordBoundaries.moveByWordBoundary;
  TextBoundary _linebreak() =>
      widget.obscureText ? _documentBoundary() : LineBoundary(renderEditable);
  TextBoundary _paragraphBoundary() => ParagraphBoundary(_value.text);
  TextBoundary _documentBoundary() => DocumentBoundary(_value.text);

  Action<T> _makeOverridable<T extends Intent>(Action<T> defaultAction) {
    return Action<T>.overridable(context: context, defaultAction: defaultAction);
  }

  /// Transpose the characters immediately before and after the current
  /// collapsed selection.
  ///
  /// When the cursor is at the end of the text, transposes the last two
  /// characters, if they exist.
  ///
  /// When the cursor is at the start of the text, does nothing.
  void _transposeCharacters(TransposeCharactersIntent intent) {
    if (_value.text.characters.length <= 1 ||
        !_value.selection.isCollapsed ||
        _value.selection.baseOffset == 0) {
      return;
    }

    final String text = _value.text;
    final TextSelection selection = _value.selection;
    final bool atEnd = selection.baseOffset == text.length;
    final CharacterRange transposing = CharacterRange.at(text, selection.baseOffset);
    if (atEnd) {
      transposing.moveBack(2);
    } else {
      transposing
        ..moveBack()
        ..expandNext();
    }
    assert(transposing.currentCharacters.length == 2);

    userUpdateTextEditingValue(
      TextEditingValue(
        text:
            transposing.stringBefore +
            transposing.currentCharacters.last +
            transposing.currentCharacters.first +
            transposing.stringAfter,
        selection: TextSelection.collapsed(
          offset: transposing.stringBeforeLength + transposing.current.length,
        ),
      ),
      SelectionChangedCause.keyboard,
    );
  }

  late final Action<TransposeCharactersIntent> _transposeCharactersAction =
      CallbackAction<TransposeCharactersIntent>(onInvoke: _transposeCharacters);

  void _replaceText(ReplaceTextIntent intent) {
    final TextEditingValue oldValue = _value;
    final TextEditingValue newValue = intent.currentTextEditingValue.replaced(
      intent.replacementRange,
      intent.replacementText,
    );
    userUpdateTextEditingValue(newValue, intent.cause);

    // If there's no change in text and selection (e.g. when selecting and
    // pasting identical text), the widget won't be rebuilt on value update.
    // Handle this by calling _didChangeTextEditingValue() so caret and scroll
    // updates can happen.
    if (newValue == oldValue) {
      _didChangeTextEditingValue();
    }
  }

  late final Action<ReplaceTextIntent> _replaceTextAction = CallbackAction<ReplaceTextIntent>(
    onInvoke: _replaceText,
  );

  // Scrolls either to the beginning or end of the document depending on the
  // intent's `forward` parameter.
  void _scrollToDocumentBoundary(ScrollToDocumentBoundaryIntent intent) {
    if (intent.forward) {
      bringIntoView(TextPosition(offset: _value.text.length));
    } else {
      bringIntoView(const TextPosition(offset: 0));
    }
  }

  /// Handles [ScrollIntent] by scrolling the [Scrollable] inside of
  /// [EditableText].
  void _scroll(ScrollIntent intent) {
    if (intent.type != ScrollIncrementType.page) {
      return;
    }

    final ScrollPosition position = _scrollController.position;
    if (widget.maxLines == 1) {
      _scrollController.jumpTo(position.maxScrollExtent);
      return;
    }

    // If the field isn't scrollable, do nothing. For example, when the lines of
    // text is less than maxLines, the field has nothing to scroll.
    if (position.maxScrollExtent == 0.0 && position.minScrollExtent == 0.0) {
      return;
    }

    final ScrollableState? state = _scrollableKey.currentState as ScrollableState?;
    final double increment = ScrollAction.getDirectionalIncrement(state!, intent);
    final double destination = clampDouble(
      position.pixels + increment,
      position.minScrollExtent,
      position.maxScrollExtent,
    );
    if (destination == position.pixels) {
      return;
    }
    _scrollController.jumpTo(destination);
  }

  /// Extend the selection down by page if the `forward` parameter is true, or
  /// up by page otherwise.
  void _extendSelectionByPage(ExtendSelectionByPageIntent intent) {
    if (widget.maxLines == 1) {
      return;
    }

    final TextSelection nextSelection;
    final Rect extentRect = renderEditable.getLocalRectForCaret(_value.selection.extent);
    final ScrollableState? state = _scrollableKey.currentState as ScrollableState?;
    final double increment = ScrollAction.getDirectionalIncrement(
      state!,
      ScrollIntent(
        direction: intent.forward ? AxisDirection.down : AxisDirection.up,
        type: ScrollIncrementType.page,
      ),
    );
    final ScrollPosition position = _scrollController.position;
    if (intent.forward) {
      if (_value.selection.extentOffset >= _value.text.length) {
        return;
      }
      final Offset nextExtentOffset = Offset(extentRect.left, extentRect.top + increment);
      final double height = position.maxScrollExtent + renderEditable.size.height;
      final TextPosition nextExtent =
          nextExtentOffset.dy + position.pixels >= height
              ? TextPosition(offset: _value.text.length)
              : renderEditable.getPositionForPoint(renderEditable.localToGlobal(nextExtentOffset));
      nextSelection = _value.selection.copyWith(extentOffset: nextExtent.offset);
    } else {
      if (_value.selection.extentOffset <= 0) {
        return;
      }
      final Offset nextExtentOffset = Offset(extentRect.left, extentRect.top + increment);
      final TextPosition nextExtent =
          nextExtentOffset.dy + position.pixels <= 0
              ? const TextPosition(offset: 0)
              : renderEditable.getPositionForPoint(renderEditable.localToGlobal(nextExtentOffset));
      nextSelection = _value.selection.copyWith(extentOffset: nextExtent.offset);
    }

    bringIntoView(nextSelection.extent);
    userUpdateTextEditingValue(
      _value.copyWith(selection: nextSelection),
      SelectionChangedCause.keyboard,
    );
  }

  void _updateSelection(UpdateSelectionIntent intent) {
    assert(
      intent.newSelection.start <= intent.currentTextEditingValue.text.length,
      'invalid selection: ${intent.newSelection}: it must not exceed the current text length ${intent.currentTextEditingValue.text.length}',
    );
    assert(
      intent.newSelection.end <= intent.currentTextEditingValue.text.length,
      'invalid selection: ${intent.newSelection}: it must not exceed the current text length ${intent.currentTextEditingValue.text.length}',
    );

    bringIntoView(intent.newSelection.extent);
    userUpdateTextEditingValue(
      intent.currentTextEditingValue.copyWith(selection: intent.newSelection),
      intent.cause,
    );
  }

  late final Action<UpdateSelectionIntent> _updateSelectionAction =
      CallbackAction<UpdateSelectionIntent>(onInvoke: _updateSelection);

  late final _UpdateTextSelectionVerticallyAction<DirectionalCaretMovementIntent>
  _verticalSelectionUpdateAction =
      _UpdateTextSelectionVerticallyAction<DirectionalCaretMovementIntent>(this);

  Object? _hideToolbarIfVisible(DismissIntent intent) {
    if (_selectionOverlay?.toolbarIsVisible ?? false) {
      hideToolbar(false);
      return null;
    }
    return Actions.invoke(context, intent);
  }

  /// The default behavior used if [EditableText.onTapOutside] is null.
  ///
  /// The `event` argument is the [PointerDownEvent] that caused the notification.
  void _defaultOnTapOutside(BuildContext context, PointerDownEvent event) {
    Actions.invoke(
      context,
      EditableTextTapOutsideIntent(focusNode: widget.focusNode, pointerDownEvent: event),
    );
  }

  late final Map<Type, Action<Intent>> _actions = <Type, Action<Intent>>{
    DoNothingAndStopPropagationTextIntent: DoNothingAction(consumesKey: false),
    ReplaceTextIntent: _replaceTextAction,
    UpdateSelectionIntent: _updateSelectionAction,
    DirectionalFocusIntent: DirectionalFocusAction.forTextField(),
    DismissIntent: CallbackAction<DismissIntent>(onInvoke: _hideToolbarIfVisible),

    // Delete
    DeleteCharacterIntent: _makeOverridable(
      _DeleteTextAction<DeleteCharacterIntent>(this, _characterBoundary, _moveBeyondTextBoundary),
    ),
    DeleteToNextWordBoundaryIntent: _makeOverridable(
      _DeleteTextAction<DeleteToNextWordBoundaryIntent>(
        this,
        _nextWordBoundary,
        _moveBeyondTextBoundary,
      ),
    ),
    DeleteToLineBreakIntent: _makeOverridable(
      _DeleteTextAction<DeleteToLineBreakIntent>(this, _linebreak, _moveToTextBoundary),
    ),

    // Extend/Move Selection
    ExtendSelectionByCharacterIntent: _makeOverridable(
      _UpdateTextSelectionAction<ExtendSelectionByCharacterIntent>(
        this,
        _characterBoundary,
        _moveBeyondTextBoundary,
        ignoreNonCollapsedSelection: false,
      ),
    ),
    ExtendSelectionByPageIntent: _makeOverridable(
      CallbackAction<ExtendSelectionByPageIntent>(onInvoke: _extendSelectionByPage),
    ),
    ExtendSelectionToNextWordBoundaryIntent: _makeOverridable(
      _UpdateTextSelectionAction<ExtendSelectionToNextWordBoundaryIntent>(
        this,
        _nextWordBoundary,
        _moveBeyondTextBoundary,
        ignoreNonCollapsedSelection: true,
      ),
    ),
    ExtendSelectionToNextParagraphBoundaryIntent: _makeOverridable(
      _UpdateTextSelectionAction<ExtendSelectionToNextParagraphBoundaryIntent>(
        this,
        _paragraphBoundary,
        _moveBeyondTextBoundary,
        ignoreNonCollapsedSelection: true,
      ),
    ),
    ExtendSelectionToLineBreakIntent: _makeOverridable(
      _UpdateTextSelectionAction<ExtendSelectionToLineBreakIntent>(
        this,
        _linebreak,
        _moveToTextBoundary,
        ignoreNonCollapsedSelection: true,
      ),
    ),
    ExtendSelectionVerticallyToAdjacentLineIntent: _makeOverridable(_verticalSelectionUpdateAction),
    ExtendSelectionVerticallyToAdjacentPageIntent: _makeOverridable(_verticalSelectionUpdateAction),
    ExtendSelectionToNextParagraphBoundaryOrCaretLocationIntent: _makeOverridable(
      _UpdateTextSelectionAction<ExtendSelectionToNextParagraphBoundaryOrCaretLocationIntent>(
        this,
        _paragraphBoundary,
        _moveBeyondTextBoundary,
        ignoreNonCollapsedSelection: true,
      ),
    ),
    ExtendSelectionToDocumentBoundaryIntent: _makeOverridable(
      _UpdateTextSelectionAction<ExtendSelectionToDocumentBoundaryIntent>(
        this,
        _documentBoundary,
        _moveBeyondTextBoundary,
        ignoreNonCollapsedSelection: true,
      ),
    ),
    ExtendSelectionToNextWordBoundaryOrCaretLocationIntent: _makeOverridable(
      _UpdateTextSelectionAction<ExtendSelectionToNextWordBoundaryOrCaretLocationIntent>(
        this,
        _nextWordBoundary,
        _moveBeyondTextBoundary,
        ignoreNonCollapsedSelection: true,
      ),
    ),
    ScrollToDocumentBoundaryIntent: _makeOverridable(
      CallbackAction<ScrollToDocumentBoundaryIntent>(onInvoke: _scrollToDocumentBoundary),
    ),
    ScrollIntent: CallbackAction<ScrollIntent>(onInvoke: _scroll),

    // Expand Selection
    ExpandSelectionToLineBreakIntent: _makeOverridable(
      _UpdateTextSelectionAction<ExpandSelectionToLineBreakIntent>(
        this,
        _linebreak,
        _moveToTextBoundary,
        ignoreNonCollapsedSelection: true,
        isExpand: true,
      ),
    ),
    ExpandSelectionToDocumentBoundaryIntent: _makeOverridable(
      _UpdateTextSelectionAction<ExpandSelectionToDocumentBoundaryIntent>(
        this,
        _documentBoundary,
        _moveToTextBoundary,
        ignoreNonCollapsedSelection: true,
        isExpand: true,
        extentAtIndex: true,
      ),
    ),

    // Copy Paste
    SelectAllTextIntent: _makeOverridable(_SelectAllAction(this)),
    CopySelectionTextIntent: _makeOverridable(_CopySelectionAction(this)),
    PasteTextIntent: _makeOverridable(
      CallbackAction<PasteTextIntent>(
        onInvoke: (PasteTextIntent intent) => pasteText(intent.cause),
      ),
    ),

    TransposeCharactersIntent: _makeOverridable(_transposeCharactersAction),
    EditableTextTapOutsideIntent: _makeOverridable(_EditableTextTapOutsideAction()),
  };

  @protected
  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMediaQuery(context));
    super.build(context); // See AutomaticKeepAliveClientMixin.

    final TextSelectionControls? controls = widget.selectionControls;
    final TextScaler effectiveTextScaler = switch ((widget.textScaler, widget.textScaleFactor)) {
      (final TextScaler textScaler, _) => textScaler,
      (null, final double textScaleFactor) => TextScaler.linear(textScaleFactor),
      (null, null) => MediaQuery.textScalerOf(context),
    };

    return _CompositionCallback(
      compositeCallback: _compositeCallback,
      enabled: _hasInputConnection,
      child: Actions(
        actions: _actions,
        child: Builder(
          builder: (BuildContext context) {
            return TextFieldTapRegion(
              groupId: widget.groupId,
              onTapOutside:
                  _hasFocus
                      ? widget.onTapOutside ??
                          (PointerDownEvent event) => _defaultOnTapOutside(context, event)
                      : null,
              onTapUpOutside: widget.onTapUpOutside,
              debugLabel: kReleaseMode ? null : 'EditableText',
              child: MouseRegion(
                cursor: widget.mouseCursor ?? SystemMouseCursors.text,
                child: UndoHistory<TextEditingValue>(
                  value: widget.controller,
                  onTriggered: (TextEditingValue value) {
                    userUpdateTextEditingValue(value, SelectionChangedCause.keyboard);
                  },
                  shouldChangeUndoStack: (TextEditingValue? oldValue, TextEditingValue newValue) {
                    if (!newValue.selection.isValid) {
                      return false;
                    }

                    if (oldValue == null) {
                      return true;
                    }

                    switch (defaultTargetPlatform) {
                      case TargetPlatform.iOS:
                      case TargetPlatform.macOS:
                      case TargetPlatform.fuchsia:
                      case TargetPlatform.linux:
                      case TargetPlatform.windows:
                        // Composing text is not counted in history coalescing.
                        if (!widget.controller.value.composing.isCollapsed) {
                          return false;
                        }
                      case TargetPlatform.android:
                        // Gboard on Android puts non-CJK words in composing regions. Coalesce
                        // composing text in order to allow the saving of partial words in that
                        // case.
                        break;
                    }

                    return oldValue.text != newValue.text ||
                        oldValue.composing != newValue.composing;
                  },
                  undoStackModifier: (TextEditingValue value) {
                    // On Android we should discard the composing region when pushing
                    // a new entry to the undo stack. This prevents the TextInputPlugin
                    // from restarting the input on every undo/redo when the composing
                    // region is changed by the framework.
                    return defaultTargetPlatform == TargetPlatform.android
                        ? value.copyWith(composing: TextRange.empty)
                        : value;
                  },
                  focusNode: widget.focusNode,
                  controller: widget.undoController,
                  child: Focus(
                    focusNode: widget.focusNode,
                    includeSemantics: false,
                    debugLabel: kReleaseMode ? null : 'EditableText',
                    child: NotificationListener<ScrollNotification>(
                      onNotification: (ScrollNotification notification) {
                        _handleContextMenuOnScroll(notification);
                        _scribbleCacheKey = null;
                        return false;
                      },
                      child: Scrollable(
                        key: _scrollableKey,
                        excludeFromSemantics: true,
                        axisDirection: _isMultiline ? AxisDirection.down : AxisDirection.right,
                        controller: _scrollController,
                        physics: widget.scrollPhysics,
                        dragStartBehavior: widget.dragStartBehavior,
                        restorationId: widget.restorationId,
                        // If a ScrollBehavior is not provided, only apply scrollbars when
                        // multiline. The overscroll indicator should not be applied in
                        // either case, glowing or stretching.
                        scrollBehavior:
                            widget.scrollBehavior ??
                            ScrollConfiguration.of(
                              context,
                            ).copyWith(scrollbars: _isMultiline, overscroll: false),
                        viewportBuilder: (BuildContext context, ViewportOffset offset) {
                          return CompositedTransformTarget(
                            link: _toolbarLayerLink,
                            child: Semantics(
                              onCopy: _semanticsOnCopy(controls),
                              onCut: _semanticsOnCut(controls),
                              onPaste: _semanticsOnPaste(controls),
                              child: _ScribbleFocusable(
                                editableKey: _editableKey,
                                enabled: _stylusHandwritingEnabled,
                                focusNode: widget.focusNode,
                                updateSelectionRects: () {
                                  _openInputConnection();
                                  _updateSelectionRects(force: true);
                                },
                                child: SizeChangedLayoutNotifier(
                                  child: _Editable(
                                    key: _editableKey,
                                    startHandleLayerLink: _startHandleLayerLink,
                                    endHandleLayerLink: _endHandleLayerLink,
                                    inlineSpan: buildTextSpan(),
                                    value: _value,
                                    cursorColor: _cursorColor,
                                    backgroundCursorColor: widget.backgroundCursorColor,
                                    showCursor: _cursorVisibilityNotifier,
                                    forceLine: widget.forceLine,
                                    readOnly: widget.readOnly,
                                    hasFocus: _hasFocus,
                                    maxLines: widget.maxLines,
                                    minLines: widget.minLines,
                                    expands: widget.expands,
                                    strutStyle: widget.strutStyle,
                                    selectionColor:
                                        _selectionOverlay?.spellCheckToolbarIsVisible ?? false
                                            ? _spellCheckConfiguration.misspelledSelectionColor ??
                                                widget.selectionColor
                                            : widget.selectionColor,
                                    textScaler: effectiveTextScaler,
                                    textAlign: widget.textAlign,
                                    textDirection: _textDirection,
                                    locale: widget.locale,
                                    textHeightBehavior:
                                        widget.textHeightBehavior ??
                                        DefaultTextHeightBehavior.maybeOf(context),
                                    textWidthBasis: widget.textWidthBasis,
                                    obscuringCharacter: widget.obscuringCharacter,
                                    obscureText: widget.obscureText,
                                    offset: offset,
                                    rendererIgnoresPointer: widget.rendererIgnoresPointer,
                                    cursorWidth: widget.cursorWidth,
                                    cursorHeight: widget.cursorHeight,
                                    cursorRadius: widget.cursorRadius,
                                    cursorOffset: widget.cursorOffset ?? Offset.zero,
                                    selectionHeightStyle: widget.selectionHeightStyle,
                                    selectionWidthStyle: widget.selectionWidthStyle,
                                    paintCursorAboveText: widget.paintCursorAboveText,
                                    enableInteractiveSelection: widget._userSelectionEnabled,
                                    textSelectionDelegate: this,
                                    devicePixelRatio: _devicePixelRatio,
                                    promptRectRange: _currentPromptRectRange,
                                    promptRectColor: widget.autocorrectionTextRectColor,
                                    clipBehavior: widget.clipBehavior,
                                  ),
                                ),
                              ),
                            ),
                          );
                        },
                      ),
                    ),
                  ),
                ),
              ),
            );
          },
        ),
      ),
    );
  }

  /// Builds [TextSpan] from current editing value.
  ///
  /// By default makes text in composing range appear as underlined.
  /// Descendants can override this method to customize appearance of text.
  TextSpan buildTextSpan() {
    if (widget.obscureText) {
      String text = _value.text;
      text = widget.obscuringCharacter * text.length;
      // Reveal the latest character in an obscured field only on mobile.
      const Set<TargetPlatform> mobilePlatforms = <TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.fuchsia,
        TargetPlatform.iOS,
      };
      final bool brieflyShowPassword =
          WidgetsBinding.instance.platformDispatcher.brieflyShowPassword &&
          mobilePlatforms.contains(defaultTargetPlatform);
      if (brieflyShowPassword) {
        final int? o = _obscureShowCharTicksPending > 0 ? _obscureLatestCharIndex : null;
        if (o != null && o >= 0 && o < text.length) {
          text = text.replaceRange(o, o + 1, _value.text.substring(o, o + 1));
        }
      }
      return TextSpan(style: _style, text: text);
    }
    if (_placeholderLocation >= 0 && _placeholderLocation <= _value.text.length) {
      final List<_ScribblePlaceholder> placeholders = <_ScribblePlaceholder>[];
      final int placeholderLocation = _value.text.length - _placeholderLocation;
      if (_isMultiline) {
        // The zero size placeholder here allows the line to break and keep the caret on the first line.
        placeholders.add(const _ScribblePlaceholder(child: SizedBox.shrink(), size: Size.zero));
        placeholders.add(
          _ScribblePlaceholder(
            child: const SizedBox.shrink(),
            size: Size(renderEditable.size.width, 0.0),
          ),
        );
      } else {
        placeholders.add(
          const _ScribblePlaceholder(child: SizedBox.shrink(), size: Size(100.0, 0.0)),
        );
      }
      return TextSpan(
        style: _style,
        children: <InlineSpan>[
          TextSpan(text: _value.text.substring(0, placeholderLocation)),
          ...placeholders,
          TextSpan(text: _value.text.substring(placeholderLocation)),
        ],
      );
    }
    final bool withComposing = !widget.readOnly && _hasFocus;
    if (_spellCheckResultsReceived) {
      // If the composing range is out of range for the current text, ignore it to
      // preserve the tree integrity, otherwise in release mode a RangeError will
      // be thrown and this EditableText will be built with a broken subtree.
      assert(!_value.composing.isValid || !withComposing || _value.isComposingRangeValid);

      final bool composingRegionOutOfRange = !_value.isComposingRangeValid || !withComposing;

      return buildTextSpanWithSpellCheckSuggestions(
        _value,
        composingRegionOutOfRange,
        _style,
        _spellCheckConfiguration.misspelledTextStyle!,
        spellCheckResults!,
      );
    }

    // Read only mode should not paint text composing.
    return widget.controller.buildTextSpan(
      context: context,
      style: _style,
      withComposing: withComposing,
    );
  }
}

class _Editable extends MultiChildRenderObjectWidget {
  _Editable({
    super.key,
    required this.inlineSpan,
    required this.value,
    required this.startHandleLayerLink,
    required this.endHandleLayerLink,
    this.cursorColor,
    this.backgroundCursorColor,
    required this.showCursor,
    required this.forceLine,
    required this.readOnly,
    this.textHeightBehavior,
    required this.textWidthBasis,
    required this.hasFocus,
    required this.maxLines,
    this.minLines,
    required this.expands,
    this.strutStyle,
    this.selectionColor,
    required this.textScaler,
    required this.textAlign,
    required this.textDirection,
    this.locale,
    required this.obscuringCharacter,
    required this.obscureText,
    required this.offset,
    this.rendererIgnoresPointer = false,
    required this.cursorWidth,
    this.cursorHeight,
    this.cursorRadius,
    required this.cursorOffset,
    required this.paintCursorAboveText,
    this.selectionHeightStyle = ui.BoxHeightStyle.tight,
    this.selectionWidthStyle = ui.BoxWidthStyle.tight,
    this.enableInteractiveSelection = true,
    required this.textSelectionDelegate,
    required this.devicePixelRatio,
    this.promptRectRange,
    this.promptRectColor,
    required this.clipBehavior,
  }) : super(children: WidgetSpan.extractFromInlineSpan(inlineSpan, textScaler));

  final InlineSpan inlineSpan;
  final TextEditingValue value;
  final Color? cursorColor;
  final LayerLink startHandleLayerLink;
  final LayerLink endHandleLayerLink;
  final Color? backgroundCursorColor;
  final ValueNotifier<bool> showCursor;
  final bool forceLine;
  final bool readOnly;
  final bool hasFocus;
  final int? maxLines;
  final int? minLines;
  final bool expands;
  final StrutStyle? strutStyle;
  final Color? selectionColor;
  final TextScaler textScaler;
  final TextAlign textAlign;
  final TextDirection textDirection;
  final Locale? locale;
  final String obscuringCharacter;
  final bool obscureText;
  final TextHeightBehavior? textHeightBehavior;
  final TextWidthBasis textWidthBasis;
  final ViewportOffset offset;
  final bool rendererIgnoresPointer;
  final double cursorWidth;
  final double? cursorHeight;
  final Radius? cursorRadius;
  final Offset cursorOffset;
  final bool paintCursorAboveText;
  final ui.BoxHeightStyle selectionHeightStyle;
  final ui.BoxWidthStyle selectionWidthStyle;
  final bool enableInteractiveSelection;
  final TextSelectionDelegate textSelectionDelegate;
  final double devicePixelRatio;
  final TextRange? promptRectRange;
  final Color? promptRectColor;
  final Clip clipBehavior;

  @override
  RenderEditable createRenderObject(BuildContext context) {
    return RenderEditable(
      text: inlineSpan,
      cursorColor: cursorColor,
      startHandleLayerLink: startHandleLayerLink,
      endHandleLayerLink: endHandleLayerLink,
      backgroundCursorColor: backgroundCursorColor,
      showCursor: showCursor,
      forceLine: forceLine,
      readOnly: readOnly,
      hasFocus: hasFocus,
      maxLines: maxLines,
      minLines: minLines,
      expands: expands,
      strutStyle: strutStyle,
      selectionColor: selectionColor,
      textScaler: textScaler,
      textAlign: textAlign,
      textDirection: textDirection,
      locale: locale ?? Localizations.maybeLocaleOf(context),
      selection: value.selection,
      offset: offset,
      ignorePointer: rendererIgnoresPointer,
      obscuringCharacter: obscuringCharacter,
      obscureText: obscureText,
      textHeightBehavior: textHeightBehavior,
      textWidthBasis: textWidthBasis,
      cursorWidth: cursorWidth,
      cursorHeight: cursorHeight,
      cursorRadius: cursorRadius,
      cursorOffset: cursorOffset,
      paintCursorAboveText: paintCursorAboveText,
      selectionHeightStyle: selectionHeightStyle,
      selectionWidthStyle: selectionWidthStyle,
      enableInteractiveSelection: enableInteractiveSelection,
      textSelectionDelegate: textSelectionDelegate,
      devicePixelRatio: devicePixelRatio,
      promptRectRange: promptRectRange,
      promptRectColor: promptRectColor,
      clipBehavior: clipBehavior,
    );
  }

  @override
  void updateRenderObject(BuildContext context, RenderEditable renderObject) {
    renderObject
      ..text = inlineSpan
      ..cursorColor = cursorColor
      ..startHandleLayerLink = startHandleLayerLink
      ..endHandleLayerLink = endHandleLayerLink
      ..backgroundCursorColor = backgroundCursorColor
      ..showCursor = showCursor
      ..forceLine = forceLine
      ..readOnly = readOnly
      ..hasFocus = hasFocus
      ..maxLines = maxLines
      ..minLines = minLines
      ..expands = expands
      ..strutStyle = strutStyle
      ..selectionColor = selectionColor
      ..textScaler = textScaler
      ..textAlign = textAlign
      ..textDirection = textDirection
      ..locale = locale ?? Localizations.maybeLocaleOf(context)
      ..selection = value.selection
      ..offset = offset
      ..ignorePointer = rendererIgnoresPointer
      ..textHeightBehavior = textHeightBehavior
      ..textWidthBasis = textWidthBasis
      ..obscuringCharacter = obscuringCharacter
      ..obscureText = obscureText
      ..cursorWidth = cursorWidth
      ..cursorHeight = cursorHeight
      ..cursorRadius = cursorRadius
      ..cursorOffset = cursorOffset
      ..selectionHeightStyle = selectionHeightStyle
      ..selectionWidthStyle = selectionWidthStyle
      ..enableInteractiveSelection = enableInteractiveSelection
      ..textSelectionDelegate = textSelectionDelegate
      ..devicePixelRatio = devicePixelRatio
      ..paintCursorAboveText = paintCursorAboveText
      ..promptRectColor = promptRectColor
      ..clipBehavior = clipBehavior
      ..setPromptRectRange(promptRectRange);
  }
}

@immutable
class _ScribbleCacheKey {
  const _ScribbleCacheKey({
    required this.inlineSpan,
    required this.textAlign,
    required this.textDirection,
    required this.textScaler,
    required this.textHeightBehavior,
    required this.locale,
    required this.structStyle,
    required this.placeholder,
    required this.size,
  });

  final TextAlign textAlign;
  final TextDirection textDirection;
  final TextScaler textScaler;
  final TextHeightBehavior? textHeightBehavior;
  final Locale? locale;
  final StrutStyle structStyle;
  final int placeholder;
  final Size size;
  final InlineSpan inlineSpan;

  RenderComparison compare(_ScribbleCacheKey other) {
    if (identical(other, this)) {
      return RenderComparison.identical;
    }
    final bool needsLayout =
        textAlign != other.textAlign ||
        textDirection != other.textDirection ||
        textScaler != other.textScaler ||
        (textHeightBehavior ?? const TextHeightBehavior()) !=
            (other.textHeightBehavior ?? const TextHeightBehavior()) ||
        locale != other.locale ||
        structStyle != other.structStyle ||
        placeholder != other.placeholder ||
        size != other.size;
    return needsLayout ? RenderComparison.layout : inlineSpan.compareTo(other.inlineSpan);
  }
}

class _ScribbleFocusable extends StatefulWidget {
  const _ScribbleFocusable({
    required this.child,
    required this.focusNode,
    required this.editableKey,
    required this.updateSelectionRects,
    required this.enabled,
  });

  final Widget child;
  final FocusNode focusNode;
  final GlobalKey editableKey;
  final VoidCallback updateSelectionRects;
  final bool enabled;

  @override
  _ScribbleFocusableState createState() => _ScribbleFocusableState();
}

class _ScribbleFocusableState extends State<_ScribbleFocusable> implements ScribbleClient {
  _ScribbleFocusableState() : _elementIdentifier = (_nextElementIdentifier++).toString();

  @override
  void initState() {
    super.initState();
    if (widget.enabled) {
      TextInput.registerScribbleElement(elementIdentifier, this);
    }
  }

  @override
  void didUpdateWidget(_ScribbleFocusable oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (!oldWidget.enabled && widget.enabled) {
      TextInput.registerScribbleElement(elementIdentifier, this);
    }

    if (oldWidget.enabled && !widget.enabled) {
      TextInput.unregisterScribbleElement(elementIdentifier);
    }
  }

  @override
  void dispose() {
    TextInput.unregisterScribbleElement(elementIdentifier);
    super.dispose();
  }

  RenderEditable? get renderEditable =>
      widget.editableKey.currentContext?.findRenderObject() as RenderEditable?;

  static int _nextElementIdentifier = 1;
  final String _elementIdentifier;

  @override
  String get elementIdentifier => _elementIdentifier;

  @override
  void onScribbleFocus(Offset offset) {
    widget.focusNode.requestFocus();
    renderEditable?.selectPositionAt(from: offset, cause: SelectionChangedCause.scribble);
    widget.updateSelectionRects();
  }

  @override
  bool isInScribbleRect(Rect rect) {
    final Rect calculatedBounds = bounds;
    if (renderEditable?.readOnly ?? false) {
      return false;
    }
    if (calculatedBounds == Rect.zero) {
      return false;
    }
    if (!calculatedBounds.overlaps(rect)) {
      return false;
    }
    final Rect intersection = calculatedBounds.intersect(rect);
    final HitTestResult result = HitTestResult();
    WidgetsBinding.instance.hitTestInView(result, intersection.center, View.of(context).viewId);
    return result.path.any((HitTestEntry entry) => entry.target == renderEditable);
  }

  @override
  Rect get bounds {
    final RenderBox? box = context.findRenderObject() as RenderBox?;
    if (box == null || !mounted || !box.attached) {
      return Rect.zero;
    }
    final Matrix4 transform = box.getTransformTo(null);
    return MatrixUtils.transformRect(
      transform,
      Rect.fromLTWH(0, 0, box.size.width, box.size.height),
    );
  }

  @override
  Widget build(BuildContext context) {
    return widget.child;
  }
}

class _ScribblePlaceholder extends WidgetSpan {
  const _ScribblePlaceholder({required super.child, required this.size});

  /// The size of the span, used in place of adding a placeholder size to the [TextPainter].
  final Size size;

  @override
  void build(
    ui.ParagraphBuilder builder, {
    TextScaler textScaler = TextScaler.noScaling,
    List<PlaceholderDimensions>? dimensions,
  }) {
    assert(debugAssertIsValid());
    final bool hasStyle = style != null;
    if (hasStyle) {
      builder.pushStyle(style!.getTextStyle(textScaler: textScaler));
    }
    builder.addPlaceholder(size.width, size.height, alignment);
    if (hasStyle) {
      builder.pop();
    }
  }
}

/// A text boundary that uses code points as logical boundaries.
///
/// A code point represents a single character. This may be smaller than what is
/// represented by a user-perceived character, or grapheme. For example, a
/// single grapheme (in this case a Unicode extended grapheme cluster) like
/// "👨‍👩‍👦" consists of five code points: the man emoji, a zero
/// width joiner, the woman emoji, another zero width joiner, and the boy emoji.
/// The [String] has a length of eight because each emoji consists of two code
/// units.
///
/// Code units are the units by which Dart's String class is measured, which is
/// encoded in UTF-16.
///
/// See also:
///
///  * [String.runes], which deals with code points like this class.
///  * [Characters], which deals with graphemes.
///  * [CharacterBoundary], which is a [TextBoundary] like this class, but whose
///    boundaries are graphemes instead of code points.
class _CodePointBoundary extends TextBoundary {
  const _CodePointBoundary(this._text);

  final String _text;

  // Returns true if the given position falls in the center of a surrogate pair.
  bool _breaksSurrogatePair(int position) {
    assert(position > 0 && position < _text.length && _text.length > 1);
    return TextPainter.isHighSurrogate(_text.codeUnitAt(position - 1)) &&
        TextPainter.isLowSurrogate(_text.codeUnitAt(position));
  }

  @override
  int? getLeadingTextBoundaryAt(int position) {
    if (_text.isEmpty || position < 0) {
      return null;
    }
    if (position == 0) {
      return 0;
    }
    if (position >= _text.length) {
      return _text.length;
    }
    if (_text.length <= 1) {
      return position;
    }

    return _breaksSurrogatePair(position) ? position - 1 : position;
  }

  @override
  int? getTrailingTextBoundaryAt(int position) {
    if (_text.isEmpty || position >= _text.length) {
      return null;
    }
    if (position < 0) {
      return 0;
    }
    if (position == _text.length - 1) {
      return _text.length;
    }
    if (_text.length <= 1) {
      return position;
    }

    return _breaksSurrogatePair(position + 1) ? position + 2 : position + 1;
  }
}

// -------------------------------  Text Actions -------------------------------
class _DeleteTextAction<T extends DirectionalTextEditingIntent> extends ContextAction<T> {
  _DeleteTextAction(this.state, this.getTextBoundary, this._applyTextBoundary);

  final EditableTextState state;
  final TextBoundary Function() getTextBoundary;
  final _ApplyTextBoundary _applyTextBoundary;

  void _hideToolbarIfTextChanged(ReplaceTextIntent intent) {
    if (state._selectionOverlay == null || !state.selectionOverlay!.toolbarIsVisible) {
      return;
    }
    final TextEditingValue oldValue = intent.currentTextEditingValue;
    final TextEditingValue newValue = intent.currentTextEditingValue.replaced(
      intent.replacementRange,
      intent.replacementText,
    );
    if (oldValue.text != newValue.text) {
      // Hide the toolbar if the text was changed, but only hide the toolbar
      // overlay; the selection handle's visibility will be handled
      // by `_handleSelectionChanged`.
      state.hideToolbar(false);
    }
  }

  @override
  Object? invoke(T intent, [BuildContext? context]) {
    final TextSelection selection = state._value.selection;
    if (!selection.isValid) {
      return null;
    }
    assert(selection.isValid);
    // Expands the selection to ensure the range covers full graphemes.
    final TextBoundary atomicBoundary = state._characterBoundary();
    if (!selection.isCollapsed) {
      // Expands the selection to ensure the range covers full graphemes.
      final TextRange range = TextRange(
        start: atomicBoundary.getLeadingTextBoundaryAt(selection.start) ?? state._value.text.length,
        end: atomicBoundary.getTrailingTextBoundaryAt(selection.end - 1) ?? 0,
      );
      final ReplaceTextIntent replaceTextIntent = ReplaceTextIntent(
        state._value,
        '',
        range,
        SelectionChangedCause.keyboard,
      );
      _hideToolbarIfTextChanged(replaceTextIntent);
      return Actions.invoke(context!, replaceTextIntent);
    }

    final int target = _applyTextBoundary(selection.base, intent.forward, getTextBoundary()).offset;

    final TextRange rangeToDelete = TextSelection(
      baseOffset:
          intent.forward
              ? atomicBoundary.getLeadingTextBoundaryAt(selection.baseOffset) ??
                  state._value.text.length
              : atomicBoundary.getTrailingTextBoundaryAt(selection.baseOffset - 1) ?? 0,
      extentOffset: target,
    );
    final ReplaceTextIntent replaceTextIntent = ReplaceTextIntent(
      state._value,
      '',
      rangeToDelete,
      SelectionChangedCause.keyboard,
    );
    _hideToolbarIfTextChanged(replaceTextIntent);
    return Actions.invoke(context!, replaceTextIntent);
  }

  @override
  bool get isActionEnabled => !state.widget.readOnly && state._value.selection.isValid;
}

class _UpdateTextSelectionAction<T extends DirectionalCaretMovementIntent>
    extends ContextAction<T> {
  _UpdateTextSelectionAction(
    this.state,
    this.getTextBoundary,
    this.applyTextBoundary, {
    required this.ignoreNonCollapsedSelection,
    this.isExpand = false,
    this.extentAtIndex = false,
  });

  final EditableTextState state;
  final bool ignoreNonCollapsedSelection;
  final bool isExpand;
  final bool extentAtIndex;
  final TextBoundary Function() getTextBoundary;
  final _ApplyTextBoundary applyTextBoundary;

  static const int NEWLINE_CODE_UNIT = 10;

  // Returns true iff the given position is at a wordwrap boundary in the
  // upstream position.
  bool _isAtWordwrapUpstream(TextPosition position) {
    final TextPosition end = TextPosition(
      offset: state.renderEditable.getLineAtOffset(position).end,
      affinity: TextAffinity.upstream,
    );
    return end == position &&
        end.offset != state.textEditingValue.text.length &&
        state.textEditingValue.text.codeUnitAt(position.offset) != NEWLINE_CODE_UNIT;
  }

  // Returns true if the given position at a wordwrap boundary in the
  // downstream position.
  bool _isAtWordwrapDownstream(TextPosition position) {
    final TextPosition start = TextPosition(
      offset: state.renderEditable.getLineAtOffset(position).start,
    );
    return start == position &&
        start.offset != 0 &&
        state.textEditingValue.text.codeUnitAt(position.offset - 1) != NEWLINE_CODE_UNIT;
  }

  @override
  Object? invoke(T intent, [BuildContext? context]) {
    final TextSelection selection = state._value.selection;
    assert(selection.isValid);

    final bool collapseSelection = intent.collapseSelection || !state.widget.selectionEnabled;
    if (!selection.isCollapsed && !ignoreNonCollapsedSelection && collapseSelection) {
      return Actions.invoke(
        context!,
        UpdateSelectionIntent(
          state._value,
          TextSelection.collapsed(offset: intent.forward ? selection.end : selection.start),
          SelectionChangedCause.keyboard,
        ),
      );
    }

    TextPosition extent = selection.extent;
    // If continuesAtWrap is true extent and is at the relevant wordwrap, then
    // move it just to the other side of the wordwrap.
    if (intent.continuesAtWrap) {
      if (intent.forward && _isAtWordwrapUpstream(extent)) {
        extent = TextPosition(offset: extent.offset);
      } else if (!intent.forward && _isAtWordwrapDownstream(extent)) {
        extent = TextPosition(offset: extent.offset, affinity: TextAffinity.upstream);
      }
    }

    final bool shouldTargetBase =
        isExpand &&
        (intent.forward
            ? selection.baseOffset > selection.extentOffset
            : selection.baseOffset < selection.extentOffset);
    final TextPosition newExtent = applyTextBoundary(
      shouldTargetBase ? selection.base : extent,
      intent.forward,
      getTextBoundary(),
    );
    final TextSelection newSelection =
        collapseSelection || (!isExpand && newExtent.offset == selection.baseOffset)
            ? TextSelection.fromPosition(newExtent)
            : isExpand
            ? selection.expandTo(newExtent, extentAtIndex || selection.isCollapsed)
            : selection.extendTo(newExtent);

    final bool shouldCollapseToBase =
        intent.collapseAtReversal &&
        (selection.baseOffset - selection.extentOffset) *
                (selection.baseOffset - newSelection.extentOffset) <
            0;
    final TextSelection newRange =
        shouldCollapseToBase ? TextSelection.fromPosition(selection.base) : newSelection;
    return Actions.invoke(
      context!,
      UpdateSelectionIntent(state._value, newRange, SelectionChangedCause.keyboard),
    );
  }

  @override
  bool get isActionEnabled => state._value.selection.isValid;
}

class _UpdateTextSelectionVerticallyAction<T extends DirectionalCaretMovementIntent>
    extends ContextAction<T> {
  _UpdateTextSelectionVerticallyAction(this.state);

  final EditableTextState state;

  VerticalCaretMovementRun? _verticalMovementRun;
  TextSelection? _runSelection;

  void stopCurrentVerticalRunIfSelectionChanges() {
    final TextSelection? runSelection = _runSelection;
    if (runSelection == null) {
      assert(_verticalMovementRun == null);
      return;
    }
    _runSelection = state._value.selection;
    final TextSelection currentSelection = state.widget.controller.selection;
    final bool continueCurrentRun =
        currentSelection.isValid &&
        currentSelection.isCollapsed &&
        currentSelection.baseOffset == runSelection.baseOffset &&
        currentSelection.extentOffset == runSelection.extentOffset;
    if (!continueCurrentRun) {
      _verticalMovementRun = null;
      _runSelection = null;
    }
  }

  @override
  void invoke(T intent, [BuildContext? context]) {
    assert(state._value.selection.isValid);

    final bool collapseSelection = intent.collapseSelection || !state.widget.selectionEnabled;
    final TextEditingValue value = state._textEditingValueforTextLayoutMetrics;
    if (!value.selection.isValid) {
      return;
    }

    if (_verticalMovementRun?.isValid == false) {
      _verticalMovementRun = null;
      _runSelection = null;
    }

    final VerticalCaretMovementRun currentRun =
        _verticalMovementRun ??
        state.renderEditable.startVerticalCaretMovement(state.renderEditable.selection!.extent);

    final bool shouldMove =
        intent is ExtendSelectionVerticallyToAdjacentPageIntent
            ? currentRun.moveByOffset(
              (intent.forward ? 1.0 : -1.0) * state.renderEditable.size.height,
            )
            : intent.forward
            ? currentRun.moveNext()
            : currentRun.movePrevious();
    final TextPosition newExtent =
        shouldMove
            ? currentRun.current
            : intent.forward
            ? TextPosition(offset: value.text.length)
            : const TextPosition(offset: 0);
    final TextSelection newSelection =
        collapseSelection
            ? TextSelection.fromPosition(newExtent)
            : value.selection.extendTo(newExtent);

    Actions.invoke(
      context!,
      UpdateSelectionIntent(value, newSelection, SelectionChangedCause.keyboard),
    );
    if (state._value.selection == newSelection) {
      _verticalMovementRun = currentRun;
      _runSelection = newSelection;
    }
  }

  @override
  bool get isActionEnabled => state._value.selection.isValid;
}

class _SelectAllAction extends ContextAction<SelectAllTextIntent> {
  _SelectAllAction(this.state);

  final EditableTextState state;

  @override
  Object? invoke(SelectAllTextIntent intent, [BuildContext? context]) {
    return Actions.invoke(
      context!,
      UpdateSelectionIntent(
        state._value,
        TextSelection(baseOffset: 0, extentOffset: state._value.text.length),
        intent.cause,
      ),
    );
  }

  @override
  bool get isActionEnabled => state.widget.selectionEnabled;
}

class _CopySelectionAction extends ContextAction<CopySelectionTextIntent> {
  _CopySelectionAction(this.state);

  final EditableTextState state;

  @override
  void invoke(CopySelectionTextIntent intent, [BuildContext? context]) {
    if (intent.collapseSelection) {
      state.cutSelection(intent.cause);
    } else {
      state.copySelection(intent.cause);
    }
  }

  @override
  bool get isActionEnabled => state._value.selection.isValid && !state._value.selection.isCollapsed;
}

/// A [ClipboardStatusNotifier] whose [value] is hardcoded to
/// [ClipboardStatus.pasteable].
///
/// Useful to avoid showing a permission dialog on web, which happens when
/// [Clipboard.hasStrings] is called.
class _WebClipboardStatusNotifier extends ClipboardStatusNotifier {
  @override
  ClipboardStatus value = ClipboardStatus.pasteable;

  @override
  Future<void> update() {
    return Future<void>.value();
  }
}

class _EditableTextTapOutsideAction extends ContextAction<EditableTextTapOutsideIntent> {
  _EditableTextTapOutsideAction();

  @override
  void invoke(EditableTextTapOutsideIntent intent, [BuildContext? context]) {
    // The focus dropping behavior is only present on desktop platforms.
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.iOS:
      case TargetPlatform.fuchsia:
        // On mobile platforms, we don't unfocus on touch events unless they're
        // in the web browser, but we do unfocus for all other kinds of events.
        switch (intent.pointerDownEvent.kind) {
          case ui.PointerDeviceKind.touch:
            if (kIsWeb) {
              intent.focusNode.unfocus();
            }
          case ui.PointerDeviceKind.mouse:
          case ui.PointerDeviceKind.stylus:
          case ui.PointerDeviceKind.invertedStylus:
          case ui.PointerDeviceKind.unknown:
            intent.focusNode.unfocus();
          case ui.PointerDeviceKind.trackpad:
            throw UnimplementedError('Unexpected pointer down event for trackpad');
        }
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        intent.focusNode.unfocus();
    }
  }
}
