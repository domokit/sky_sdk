// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:async';
import 'dart:math' show max, min;

import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

import 'actions.dart';
import 'basic.dart';
import 'constants.dart';
import 'editable_text.dart';
import 'focus_manager.dart';
import 'framework.dart';
import 'inherited_notifier.dart';
import 'layout_builder.dart';
import 'overlay.dart';
import 'shortcuts.dart';
import 'tap_region.dart';
import 'value_listenable_builder.dart';

// Examples can assume:
// late BuildContext context;

/// The type of the [RawAutocomplete] callback which computes the list of
/// optional completions for the widget's field, based on the text the user has
/// entered so far.
///
/// See also:
///
///   * [RawAutocomplete.optionsBuilder], which is of this type.
typedef AutocompleteOptionsBuilder<T extends Object> =
    FutureOr<Iterable<T>> Function(TextEditingValue textEditingValue);

/// The type of the callback used by the [RawAutocomplete] widget to indicate
/// that the user has selected an option.
///
/// See also:
///
///   * [RawAutocomplete.onSelected], which is of this type.
typedef AutocompleteOnSelected<T extends Object> = void Function(T option);

/// The type of the [RawAutocomplete] callback which returns a [Widget] that
/// displays the specified [options] and calls [onSelected] if the user
/// selects an option.
///
/// The returned widget from this callback will be wrapped in an
/// [AutocompleteHighlightedOption] inherited widget. This will allow
/// this callback to determine which option is currently highlighted for
/// keyboard navigation.
///
/// See also:
///
///   * [RawAutocomplete.optionsViewBuilder], which is of this type.
typedef AutocompleteOptionsViewBuilder<T extends Object> =
    Widget Function(
      BuildContext context,
      AutocompleteOnSelected<T> onSelected,
      Iterable<T> options,
    );

/// The type of the Autocomplete callback which returns the widget that
/// contains the input [TextField] or [TextFormField].
///
/// See also:
///
///   * [RawAutocomplete.fieldViewBuilder], which is of this type.
typedef AutocompleteFieldViewBuilder =
    Widget Function(
      BuildContext context,
      TextEditingController textEditingController,
      FocusNode focusNode,
      VoidCallback onFieldSubmitted,
    );

/// The type of the [RawAutocomplete] callback that converts an option value to
/// a string which can be displayed in the widget's options menu.
///
/// See also:
///
///   * [RawAutocomplete.displayStringForOption], which is of this type.
typedef AutocompleteOptionToString<T extends Object> = String Function(T option);

/// A direction in which to open the options-view overlay.
///
/// See also:
///
///  * [RawAutocomplete.optionsViewOpenDirection], which is of this type.
///  * [RawAutocomplete.optionsViewBuilder] to specify how to build the
///    selectable-options widget.
///  * [RawAutocomplete.fieldViewBuilder] to optionally specify how to build the
///    corresponding field widget.
enum OptionsViewOpenDirection {
  /// Open upward.
  ///
  /// The bottom edge of the options view will align with the top edge
  /// of the text field built by [RawAutocomplete.fieldViewBuilder].
  up,

  /// Open downward.
  ///
  /// The top edge of the options view will align with the bottom edge
  /// of the text field built by [RawAutocomplete.fieldViewBuilder].
  down,
}

// TODO(justinmc): Mention AutocompleteCupertino when it is implemented.
/// {@template flutter.widgets.RawAutocomplete.RawAutocomplete}
/// A widget for helping the user make a selection by entering some text and
/// choosing from among a list of options.
///
/// The user's text input is received in a field built with the
/// [fieldViewBuilder] parameter. The options to be displayed are determined
/// using [optionsBuilder] and rendered with [optionsViewBuilder].
/// {@endtemplate}
///
/// This is a core framework widget with very basic UI.
///
/// {@tool dartpad}
/// This example shows how to create a very basic autocomplete widget using the
/// [fieldViewBuilder] and [optionsViewBuilder] parameters.
///
/// ** See code in examples/api/lib/widgets/autocomplete/raw_autocomplete.0.dart **
/// {@end-tool}
///
/// The type parameter T represents the type of the options. Most commonly this
/// is a String, as in the example above. However, it's also possible to use
/// another type with a `toString` method, or a custom [displayStringForOption].
/// Options will be compared using `==`, so it may be beneficial to override
/// [Object.==] and [Object.hashCode] for custom types.
///
/// {@tool dartpad}
/// This example is similar to the previous example, but it uses a custom T data
/// type instead of directly using String.
///
/// ** See code in examples/api/lib/widgets/autocomplete/raw_autocomplete.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows the use of RawAutocomplete in a form.
///
/// ** See code in examples/api/lib/widgets/autocomplete/raw_autocomplete.2.dart **
/// {@end-tool}
///
/// See also:
///
///  * [Autocomplete], which is a Material-styled implementation that is based
/// on RawAutocomplete.
class RawAutocomplete<T extends Object> extends StatefulWidget {
  /// Create an instance of RawAutocomplete.
  ///
  /// [displayStringForOption], [optionsBuilder] and [optionsViewBuilder] must
  /// not be null.
  const RawAutocomplete({
    super.key,
    required this.optionsViewBuilder,
    required this.optionsBuilder,
    this.optionsViewOpenDirection = OptionsViewOpenDirection.down,
    this.displayStringForOption = defaultStringForOption,
    this.fieldViewBuilder,
    this.focusNode,
    this.onSelected,
    this.textEditingController,
    this.initialValue,
  }) : assert(
         fieldViewBuilder != null ||
             (key != null && focusNode != null && textEditingController != null),
         'Pass in a fieldViewBuilder, or otherwise create a separate field and pass in the FocusNode, TextEditingController, and a key. Use the key with RawAutocomplete.onFieldSubmitted.',
       ),
       assert((focusNode == null) == (textEditingController == null)),
       assert(
         !(textEditingController != null && initialValue != null),
         'textEditingController and initialValue cannot be simultaneously defined.',
       );

  /// {@template flutter.widgets.RawAutocomplete.fieldViewBuilder}
  /// Builds the field whose input is used to get the options.
  ///
  /// Pass the provided [TextEditingController] to the field built here so that
  /// RawAutocomplete can listen for changes.
  /// {@endtemplate}
  ///
  /// If this parameter is null, then a [SizedBox.shrink] is built instead.
  /// For how that pattern can be useful, see [textEditingController].
  final AutocompleteFieldViewBuilder? fieldViewBuilder;

  /// The [FocusNode] that is used for the text field.
  ///
  /// {@template flutter.widgets.RawAutocomplete.split}
  /// The main purpose of this parameter is to allow the use of a separate text
  /// field located in another part of the widget tree instead of the text
  /// field built by [fieldViewBuilder]. For example, it may be desirable to
  /// place the text field in the AppBar and the options below in the main body.
  ///
  /// When following this pattern, [fieldViewBuilder] can be omitted,
  /// so that a text field is not drawn where it would normally be.
  /// A separate text field can be created elsewhere, and a
  /// FocusNode and TextEditingController can be passed both to that text field
  /// and to RawAutocomplete.
  ///
  /// {@tool dartpad}
  /// This examples shows how to create an autocomplete widget with the text
  /// field in the AppBar and the results in the main body of the app.
  ///
  /// ** See code in examples/api/lib/widgets/autocomplete/raw_autocomplete.focus_node.0.dart **
  /// {@end-tool}
  /// {@endtemplate}
  ///
  /// If this parameter is not null, then [textEditingController] must also be
  /// not null.
  final FocusNode? focusNode;

  /// {@template flutter.widgets.RawAutocomplete.optionsViewBuilder}
  /// Builds the selectable options widgets from a list of options objects.
  ///
  /// The options are displayed floating below or above the field inside of an
  /// [Overlay], not at the same place in the widget tree as [RawAutocomplete].
  /// To control whether it opens upward or downward, use
  /// [optionsViewOpenDirection].
  ///
  /// In order to track which item is highlighted by keyboard navigation, the
  /// resulting options will be wrapped in an inherited
  /// [AutocompleteHighlightedOption] widget.
  /// Inside this callback, the index of the highlighted option can be obtained
  /// from [AutocompleteHighlightedOption.of] to display the highlighted option
  /// with a visual highlight to indicate it will be the option selected from
  /// the keyboard.
  ///
  /// {@endtemplate}
  final AutocompleteOptionsViewBuilder<T> optionsViewBuilder;

  /// {@template flutter.widgets.RawAutocomplete.optionsViewOpenDirection}
  /// The direction in which to open the options-view overlay.
  ///
  /// Defaults to [OptionsViewOpenDirection.down].
  /// {@endtemplate}
  final OptionsViewOpenDirection optionsViewOpenDirection;

  /// {@template flutter.widgets.RawAutocomplete.displayStringForOption}
  /// Returns the string to display in the field when the option is selected.
  ///
  /// This is useful when using a custom T type and the string to display is
  /// different than the string to search by.
  ///
  /// If not provided, will use `option.toString()`.
  /// {@endtemplate}
  final AutocompleteOptionToString<T> displayStringForOption;

  /// {@template flutter.widgets.RawAutocomplete.onSelected}
  /// Called when an option is selected by the user.
  /// {@endtemplate}
  final AutocompleteOnSelected<T>? onSelected;

  /// {@template flutter.widgets.RawAutocomplete.optionsBuilder}
  /// A function that returns the current selectable options objects given the
  /// current TextEditingValue.
  /// {@endtemplate}
  final AutocompleteOptionsBuilder<T> optionsBuilder;

  /// The [TextEditingController] that is used for the text field.
  ///
  /// {@macro flutter.widgets.RawAutocomplete.split}
  ///
  /// If this parameter is not null, then [focusNode] must also be not null.
  final TextEditingController? textEditingController;

  /// {@template flutter.widgets.RawAutocomplete.initialValue}
  /// The initial value to use for the text field.
  /// {@endtemplate}
  ///
  /// Setting the initial value does not notify [textEditingController]'s
  /// listeners, and thus will not cause the options UI to appear.
  ///
  /// This parameter is ignored if [textEditingController] is defined.
  final TextEditingValue? initialValue;

  /// Calls [AutocompleteFieldViewBuilder]'s onFieldSubmitted callback for the
  /// RawAutocomplete widget indicated by the given [GlobalKey].
  ///
  /// This is not typically used unless a custom field is implemented instead of
  /// using [fieldViewBuilder]. In the typical case, the onFieldSubmitted
  /// callback is passed via the [AutocompleteFieldViewBuilder] signature. When
  /// not using fieldViewBuilder, the same callback can be called by using this
  /// static method.
  ///
  /// See also:
  ///
  ///  * [focusNode] and [textEditingController], which contain a code example
  ///    showing how to create a separate field outside of fieldViewBuilder.
  static void onFieldSubmitted<T extends Object>(GlobalKey key) {
    final _RawAutocompleteState<T> rawAutocomplete = key.currentState! as _RawAutocompleteState<T>;
    rawAutocomplete._onFieldSubmitted();
  }

  /// The default way to convert an option to a string in
  /// [displayStringForOption].
  ///
  /// Uses the `toString` method of the given `option`.
  static String defaultStringForOption(Object? option) {
    return option.toString();
  }

  @override
  State<RawAutocomplete<T>> createState() => _RawAutocompleteState<T>();
}

class _RawAutocompleteState<T extends Object> extends State<RawAutocomplete<T>> {
  final GlobalKey _fieldKey = GlobalKey();
  final LayerLink _optionsLayerLink = LayerLink();

  /// The box constraints that the field was last built with.
  final ValueNotifier<BoxConstraints?> _fieldBoxConstraints = ValueNotifier<BoxConstraints?>(null);

  final OverlayPortalController _optionsViewController = OverlayPortalController(
    debugLabel: '_RawAutocompleteState',
  );

  TextEditingController? _internalTextEditingController;
  TextEditingController get _textEditingController {
    return widget.textEditingController ??
        (_internalTextEditingController ??= TextEditingController()..addListener(_onChangedField));
  }

  FocusNode? _internalFocusNode;
  FocusNode get _focusNode {
    return widget.focusNode ??
        (_internalFocusNode ??= FocusNode()..addListener(_updateOptionsViewVisibility));
  }

  late final Map<Type, CallbackAction<Intent>> _actionMap = <Type, CallbackAction<Intent>>{
    AutocompletePreviousOptionIntent: _AutocompleteCallbackAction<AutocompletePreviousOptionIntent>(
      onInvoke: _highlightPreviousOption,
      isEnabledCallback: () => _canShowOptionsView,
    ),
    AutocompleteNextOptionIntent: _AutocompleteCallbackAction<AutocompleteNextOptionIntent>(
      onInvoke: _highlightNextOption,
      isEnabledCallback: () => _canShowOptionsView,
    ),
    DismissIntent: CallbackAction<DismissIntent>(onInvoke: _hideOptions),
  };

  Iterable<T> _options = Iterable<T>.empty();
  T? _selection;
  // Set the initial value to null so when this widget gets focused for the first
  // time it will try to run the options view builder.
  String? _lastFieldText;
  final ValueNotifier<int> _highlightedOptionIndex = ValueNotifier<int>(0);

  static const Map<ShortcutActivator, Intent> _shortcuts = <ShortcutActivator, Intent>{
    SingleActivator(LogicalKeyboardKey.arrowUp): AutocompletePreviousOptionIntent(),
    SingleActivator(LogicalKeyboardKey.arrowDown): AutocompleteNextOptionIntent(),
  };

  bool get _canShowOptionsView => _focusNode.hasFocus && _selection == null && _options.isNotEmpty;

  void _updateOptionsViewVisibility() {
    if (_canShowOptionsView) {
      _optionsViewController.show();
    } else {
      _optionsViewController.hide();
    }
  }

  // Assigning an ID to every call of _onChangedField is necessary to avoid a
  // situation where _options is updated by an older call when multiple
  // _onChangedField calls are running simultaneously.
  int _onChangedCallId = 0;
  // Called when _textEditingController changes.
  Future<void> _onChangedField() async {
    final TextEditingValue value = _textEditingController.value;

    // Makes sure that options change only when content of the field changes.
    bool shouldUpdateOptions = false;
    if (value.text != _lastFieldText) {
      shouldUpdateOptions = true;
      _onChangedCallId += 1;
    }
    _lastFieldText = value.text;
    final int callId = _onChangedCallId;
    final Iterable<T> options = await widget.optionsBuilder(value);

    // Makes sure that previous call results do not replace new ones.
    if (callId != _onChangedCallId || !shouldUpdateOptions) {
      return;
    }
    _options = options;
    _updateHighlight(_highlightedOptionIndex.value);
    final T? selection = _selection;
    if (selection != null && value.text != widget.displayStringForOption(selection)) {
      _selection = null;
    }

    _updateOptionsViewVisibility();
  }

  // Called from fieldViewBuilder when the user submits the field.
  void _onFieldSubmitted() {
    if (_optionsViewController.isShowing) {
      _select(_options.elementAt(_highlightedOptionIndex.value));
    }
  }

  // Select the given option and update the widget.
  void _select(T nextSelection) {
    if (nextSelection == _selection) {
      return;
    }
    _selection = nextSelection;
    final String selectionString = widget.displayStringForOption(nextSelection);
    _textEditingController.value = TextEditingValue(
      selection: TextSelection.collapsed(offset: selectionString.length),
      text: selectionString,
    );
    widget.onSelected?.call(nextSelection);
    _updateOptionsViewVisibility();
  }

  void _updateHighlight(int newIndex) {
    _highlightedOptionIndex.value = _options.isEmpty ? 0 : newIndex % _options.length;
  }

  void _highlightPreviousOption(AutocompletePreviousOptionIntent intent) {
    assert(_canShowOptionsView);
    _updateOptionsViewVisibility();
    assert(_optionsViewController.isShowing);
    _updateHighlight(_highlightedOptionIndex.value - 1);
  }

  void _highlightNextOption(AutocompleteNextOptionIntent intent) {
    assert(_canShowOptionsView);
    _updateOptionsViewVisibility();
    assert(_optionsViewController.isShowing);
    _updateHighlight(_highlightedOptionIndex.value + 1);
  }

  Object? _hideOptions(DismissIntent intent) {
    if (_optionsViewController.isShowing) {
      _optionsViewController.hide();
      return null;
    } else {
      return Actions.invoke(context, intent);
    }
  }

  Widget _buildOptionsView(BuildContext context) {
    return ValueListenableBuilder<BoxConstraints?>(
      valueListenable: _fieldBoxConstraints,
      builder: (BuildContext context, BoxConstraints? constraints, Widget? child) {
        return _RawAutocompleteOptions(
          fieldKey: _fieldKey,
          optionsLayerLink: _optionsLayerLink,
          optionsViewOpenDirection: widget.optionsViewOpenDirection,
          overlayContext: context,
          textDirection: Directionality.maybeOf(context),
          highlightIndexNotifier: _highlightedOptionIndex,
          fieldConstraints: _fieldBoxConstraints.value!,
          builder: (BuildContext context) {
            return widget.optionsViewBuilder(context, _select, _options);
          },
        );
      },
    );
  }

  @override
  void initState() {
    super.initState();
    final TextEditingController initialController =
        widget.textEditingController ??
        (_internalTextEditingController = TextEditingController.fromValue(widget.initialValue));
    initialController.addListener(_onChangedField);
    widget.focusNode?.addListener(_updateOptionsViewVisibility);
  }

  @override
  void didUpdateWidget(RawAutocomplete<T> oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (!identical(oldWidget.textEditingController, widget.textEditingController)) {
      oldWidget.textEditingController?.removeListener(_onChangedField);
      if (oldWidget.textEditingController == null) {
        _internalTextEditingController?.dispose();
        _internalTextEditingController = null;
      }
      widget.textEditingController?.addListener(_onChangedField);
    }
    if (!identical(oldWidget.focusNode, widget.focusNode)) {
      oldWidget.focusNode?.removeListener(_updateOptionsViewVisibility);
      if (oldWidget.focusNode == null) {
        _internalFocusNode?.dispose();
        _internalFocusNode = null;
      }
      widget.focusNode?.addListener(_updateOptionsViewVisibility);
    }
  }

  @override
  void dispose() {
    widget.textEditingController?.removeListener(_onChangedField);
    _internalTextEditingController?.dispose();
    widget.focusNode?.removeListener(_updateOptionsViewVisibility);
    _internalFocusNode?.dispose();
    _highlightedOptionIndex.dispose();
    _fieldBoxConstraints.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final Widget fieldView =
        widget.fieldViewBuilder?.call(
          context,
          _textEditingController,
          _focusNode,
          _onFieldSubmitted,
        ) ??
        const SizedBox.shrink();
    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        // TODO(victorsanni): Also track the width of the field box so that the
        // options view maintains the same width as the field if its width
        // changes but its constraints remain unchanged.
        _fieldBoxConstraints.value = constraints;
        return OverlayPortal.targetsRootOverlay(
          key: _fieldKey,
          controller: _optionsViewController,
          overlayChildBuilder: _buildOptionsView,
          child: TextFieldTapRegion(
            child: Shortcuts(
              shortcuts: _shortcuts,
              child: Actions(
                actions: _actionMap,
                child: CompositedTransformTarget(link: _optionsLayerLink, child: fieldView),
              ),
            ),
          ),
        );
      },
    );
  }
}

class _RawAutocompleteOptions extends StatefulWidget {
  const _RawAutocompleteOptions({
    required this.fieldKey,
    required this.optionsLayerLink,
    required this.optionsViewOpenDirection,
    required this.overlayContext,
    required this.textDirection,
    required this.highlightIndexNotifier,
    required this.builder,
    required this.fieldConstraints,
  });

  final WidgetBuilder builder;
  final GlobalKey fieldKey;

  final LayerLink optionsLayerLink;
  final OptionsViewOpenDirection optionsViewOpenDirection;
  final BuildContext overlayContext;
  final TextDirection? textDirection;
  final ValueNotifier<int> highlightIndexNotifier;
  final BoxConstraints fieldConstraints;

  @override
  State<_RawAutocompleteOptions> createState() => _RawAutocompleteOptionsState();
}

class _RawAutocompleteOptionsState extends State<_RawAutocompleteOptions> {
  VoidCallback? removeCompositionCallback;
  Offset fieldOffset = Offset.zero;

  // Get the field offset if the field's position changes when its layer tree
  // is composited, which occurs for example if the field is in a scroll view.
  Offset _getFieldOffset() {
    final RenderBox? fieldRenderBox =
        widget.fieldKey.currentContext?.findRenderObject() as RenderBox?;
    final RenderBox? overlay =
        Overlay.of(widget.overlayContext).context.findRenderObject() as RenderBox?;
    return fieldRenderBox?.localToGlobal(Offset.zero, ancestor: overlay) ?? Offset.zero;
  }

  void _onLeaderComposition(Layer leaderLayer) {
    SchedulerBinding.instance.addPostFrameCallback((Duration duration) {
      if (!mounted) {
        return;
      }
      final Offset nextFieldOffset = _getFieldOffset();
      if (nextFieldOffset != fieldOffset) {
        setState(() {
          fieldOffset = nextFieldOffset;
        });
      }
    });
  }

  @override
  void initState() {
    super.initState();
    removeCompositionCallback = widget.optionsLayerLink.leader?.addCompositionCallback(
      _onLeaderComposition,
    );
  }

  @override
  void didUpdateWidget(_RawAutocompleteOptions oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.optionsLayerLink.leader != oldWidget.optionsLayerLink.leader) {
      removeCompositionCallback?.call();
      removeCompositionCallback = widget.optionsLayerLink.leader?.addCompositionCallback(
        _onLeaderComposition,
      );
    }
  }

  @override
  void dispose() {
    removeCompositionCallback?.call();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return CompositedTransformFollower(
      link: widget.optionsLayerLink,
      followerAnchor: switch (widget.optionsViewOpenDirection) {
        OptionsViewOpenDirection.up => Alignment.bottomLeft,
        OptionsViewOpenDirection.down => Alignment.topLeft,
      },
      // When the field goes offscreen, don't show the options.
      showWhenUnlinked: false,
      child: CustomSingleChildLayout(
        delegate: _RawAutocompleteOptionsLayoutDelegate(
          layerLink: widget.optionsLayerLink,
          fieldOffset: fieldOffset,
          optionsViewOpenDirection: widget.optionsViewOpenDirection,
          textDirection: Directionality.of(context),
          fieldConstraints: widget.fieldConstraints,
        ),
        child: TextFieldTapRegion(
          child: AutocompleteHighlightedOption(
            highlightIndexNotifier: widget.highlightIndexNotifier,
            // optionsViewBuilder must be able to look up
            // AutocompleteHighlightedOption in its context.
            child: Builder(builder: widget.builder),
          ),
        ),
      ),
    );
  }
}

/// Positions the options view.
class _RawAutocompleteOptionsLayoutDelegate extends SingleChildLayoutDelegate {
  _RawAutocompleteOptionsLayoutDelegate({
    required this.layerLink,
    required this.fieldOffset,
    required this.optionsViewOpenDirection,
    required this.textDirection,
    required this.fieldConstraints,
  }) : assert(layerLink.leaderSize != null);

  /// Links the options in [RawAutocomplete.optionsViewBuilder] to the field in
  /// [RawAutocomplete.fieldViewBuilder].
  final LayerLink layerLink;

  /// The position of the field in [RawAutocomplete.fieldViewBuilder].
  final Offset fieldOffset;

  /// A direction in which to open the options view overlay.
  final OptionsViewOpenDirection optionsViewOpenDirection;

  /// The [TextDirection] of this part of the widget tree.
  final TextDirection textDirection;

  /// The [BoxConstraints] for the field in [RawAutocomplete.fieldViewBuilder].
  final BoxConstraints fieldConstraints;

  // A big enough height for about one item in the default
  // Autocomplete.optionsViewBuilder. The assumption is that the user likely
  // wants the list of options to move to stay on the screen rather than get any
  // smaller than this. Allows Autocomplete to work when it has very little
  // screen height available (as in b/317115348) by positioning itself on top of
  // the field, while in other cases to size itself based on the height under
  // the field.
  static const double _kMinUsableHeight = kMinInteractiveDimension;

  // Limits the child to the space above/below the field, with a minimum, and
  // with the same maxWidth constraint as the field has.
  @override
  BoxConstraints getConstraintsForChild(BoxConstraints constraints) {
    final Size fieldSize = layerLink.leaderSize!;
    return BoxConstraints(
      // The field width may be zero if this is a split RawAutocomplete with no
      // field of its own. In that case, don't change the constraints width.
      maxWidth: fieldSize.width == 0.0 ? constraints.maxWidth : fieldSize.width,
      maxHeight: max(_kMinUsableHeight, switch (optionsViewOpenDirection) {
        OptionsViewOpenDirection.down => constraints.maxHeight - fieldOffset.dy - fieldSize.height,
        OptionsViewOpenDirection.up => fieldOffset.dy,
      }),
    );
  }

  // Positions the child above/below the field and aligned with the left/right
  // side based on text direction.
  @override
  Offset getPositionForChild(Size size, Size childSize) {
    final Size fieldSize = layerLink.leaderSize!;
    final double dx = switch (textDirection) {
      TextDirection.ltr => 0.0,
      TextDirection.rtl => fieldSize.width - childSize.width,
    };
    final double dy = switch (optionsViewOpenDirection) {
      OptionsViewOpenDirection.down => min(
        fieldSize.height,
        size.height - childSize.height - fieldOffset.dy,
      ),
      OptionsViewOpenDirection.up => size.height - min(childSize.height, fieldOffset.dy),
    };
    return Offset(dx, dy);
  }

  @override
  bool shouldRelayout(_RawAutocompleteOptionsLayoutDelegate oldDelegate) {
    if (!fieldOffset.isFinite || !layerLink.leaderSize!.isFinite) {
      return false;
    }
    return layerLink != oldDelegate.layerLink ||
        fieldOffset != oldDelegate.fieldOffset ||
        optionsViewOpenDirection != oldDelegate.optionsViewOpenDirection ||
        textDirection != oldDelegate.textDirection ||
        fieldConstraints != oldDelegate.fieldConstraints;
  }
}

class _AutocompleteCallbackAction<T extends Intent> extends CallbackAction<T> {
  _AutocompleteCallbackAction({required super.onInvoke, required this.isEnabledCallback});

  // The enabled state determines whether the action will consume the
  // key shortcut or let it continue on to the underlying text field.
  // They should only be enabled when the options are showing so shortcuts
  // can be used to navigate them.
  final bool Function() isEnabledCallback;

  @override
  bool isEnabled(covariant T intent) => isEnabledCallback();

  @override
  bool consumesKey(covariant T intent) => isEnabled(intent);
}

/// An [Intent] to highlight the previous option in the autocomplete list.
class AutocompletePreviousOptionIntent extends Intent {
  /// Creates an instance of AutocompletePreviousOptionIntent.
  const AutocompletePreviousOptionIntent();
}

/// An [Intent] to highlight the next option in the autocomplete list.
class AutocompleteNextOptionIntent extends Intent {
  /// Creates an instance of AutocompleteNextOptionIntent.
  const AutocompleteNextOptionIntent();
}

/// An inherited widget used to indicate which autocomplete option should be
/// highlighted for keyboard navigation.
///
/// The `RawAutoComplete` widget will wrap the options view generated by the
/// `optionsViewBuilder` with this widget to provide the highlighted option's
/// index to the builder.
///
/// In the builder callback the index of the highlighted option can be obtained
/// by using the static [of] method:
///
/// ```dart
/// int highlightedIndex = AutocompleteHighlightedOption.of(context);
/// ```
///
/// which can then be used to tell which option should be given a visual
/// indication that will be the option selected with the keyboard.
class AutocompleteHighlightedOption extends InheritedNotifier<ValueNotifier<int>> {
  /// Create an instance of AutocompleteHighlightedOption inherited widget.
  const AutocompleteHighlightedOption({
    super.key,
    required ValueNotifier<int> highlightIndexNotifier,
    required super.child,
  }) : super(notifier: highlightIndexNotifier);

  /// Returns the index of the highlighted option from the closest
  /// [AutocompleteHighlightedOption] ancestor.
  ///
  /// If there is no ancestor, it returns 0.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// int highlightedIndex = AutocompleteHighlightedOption.of(context);
  /// ```
  static int of(BuildContext context) {
    return context
            .dependOnInheritedWidgetOfExactType<AutocompleteHighlightedOption>()
            ?.notifier
            ?.value ??
        0;
  }
}
