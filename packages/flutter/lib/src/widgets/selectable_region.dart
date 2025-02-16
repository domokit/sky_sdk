// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter_test/flutter_test.dart';
///
/// @docImport 'editable_text.dart';
/// @docImport 'text.dart';
library;

import 'dart:async';
import 'dart:math';

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:vector_math/vector_math_64.dart';

import 'actions.dart';
import 'basic.dart';
import 'context_menu_button_item.dart';
import 'debug.dart';
import 'focus_manager.dart';
import 'focus_scope.dart';
import 'framework.dart';
import 'gesture_detector.dart';
import 'magnifier.dart';
import 'media_query.dart';
import 'overlay.dart';
import 'platform_selectable_region_context_menu.dart';
import 'selection_container.dart';
import 'text_editing_intents.dart';
import 'text_selection.dart';
import 'text_selection_toolbar_anchors.dart';

// Examples can assume:
// late GlobalKey key;

const Set<PointerDeviceKind> _kLongPressSelectionDevices = <PointerDeviceKind>{
  PointerDeviceKind.touch,
  PointerDeviceKind.stylus,
  PointerDeviceKind.invertedStylus,
};

// In practice some selectables like widgetspan shift several pixels. So when
// the vertical position diff is within the threshold, compare the horizontal
// position to make the compareScreenOrder function more robust.
const double _kSelectableVerticalComparingThreshold = 3.0;

/// A widget that introduces an area for user selections.
///
/// Flutter widgets are not selectable by default. Wrapping a widget subtree
/// with a [SelectableRegion] widget enables selection within that subtree (for
/// example, [Text] widgets automatically look for selectable regions to enable
/// selection). The wrapped subtree can be selected by users using mouse or
/// touch gestures, e.g. users can select widgets by holding the mouse
/// left-click and dragging across widgets, or they can use long press gestures
/// to select words on touch devices.
///
/// A [SelectableRegion] widget requires configuration; in particular specific
/// [selectionControls] must be provided.
///
/// The [SelectionArea] widget from the [material] library configures a
/// [SelectableRegion] in a platform-specific manner (e.g. using a Material
/// toolbar on Android, a Cupertino toolbar on iOS), and it may therefore be
/// simpler to use that widget rather than using [SelectableRegion] directly.
///
/// ## An overview of the selection system.
///
/// Every [Selectable] under the [SelectableRegion] can be selected. They form a
/// selection tree structure to handle the selection.
///
/// The [SelectableRegion] is a wrapper over [SelectionContainer]. It listens to
/// user gestures and sends corresponding [SelectionEvent]s to the
/// [SelectionContainer] it creates.
///
/// A [SelectionContainer] is a single [Selectable] that handles
/// [SelectionEvent]s on behalf of child [Selectable]s in the subtree. It
/// creates a [SelectionRegistrarScope] with its [SelectionContainer.delegate]
/// to collect child [Selectable]s and sends the [SelectionEvent]s it receives
/// from the parent [SelectionRegistrar] to the appropriate child [Selectable]s.
/// It creates an abstraction for the parent [SelectionRegistrar] as if it is
/// interacting with a single [Selectable].
///
/// The [SelectionContainer] created by [SelectableRegion] is the root node of a
/// selection tree. Each non-leaf node in the tree is a [SelectionContainer],
/// and the leaf node is a leaf widget whose render object implements
/// [Selectable]. They are connected through [SelectionRegistrarScope]s created
/// by [SelectionContainer]s.
///
/// Both [SelectionContainer]s and the leaf [Selectable]s need to register
/// themselves to the [SelectionRegistrar] from the
/// [SelectionContainer.maybeOf] if they want to participate in the
/// selection.
///
/// An example selection tree will look like:
///
/// {@tool snippet}
///
/// ```dart
/// MaterialApp(
///   home: SelectableRegion(
///     selectionControls: materialTextSelectionControls,
///     child: Scaffold(
///       appBar: AppBar(title: const Text('Flutter Code Sample')),
///       body: ListView(
///         children: const <Widget>[
///           Text('Item 0', style: TextStyle(fontSize: 50.0)),
///           Text('Item 1', style: TextStyle(fontSize: 50.0)),
///         ],
///       ),
///     ),
///   ),
/// )
/// ```
/// {@end-tool}
///
///
///               SelectionContainer
///               (SelectableRegion)
///                  /         \
///                 /           \
///                /             \
///           Selectable          \
///      ("Flutter Code Sample")   \
///                                 \
///                          SelectionContainer
///                              (ListView)
///                              /       \
///                             /         \
///                            /           \
///                     Selectable        Selectable
///                     ("Item 0")         ("Item 1")
///
///
/// ## Making a widget selectable
///
/// Some leaf widgets, such as [Text], have all of the selection logic wired up
/// automatically and can be selected as long as they are under a
/// [SelectableRegion].
///
/// To make a custom selectable widget, its render object needs to mix in
/// [Selectable] and implement the required APIs to handle [SelectionEvent]s
/// as well as paint appropriate selection highlights.
///
/// The render object also needs to register itself to a [SelectionRegistrar].
/// For the most cases, one can use [SelectionRegistrant] to auto-register
/// itself with the register returned from [SelectionContainer.maybeOf] as
/// seen in the example below.
///
/// {@tool dartpad}
/// This sample demonstrates how to create an adapter widget that makes any
/// child widget selectable.
///
/// ** See code in examples/api/lib/material/selectable_region/selectable_region.0.dart **
/// {@end-tool}
///
/// ## Complex layout
///
/// By default, the screen order is used as the selection order. If a group of
/// [Selectable]s needs to select differently, consider wrapping them with a
/// [SelectionContainer] to customize its selection behavior.
///
/// {@tool dartpad}
/// This sample demonstrates how to create a [SelectionContainer] that only
/// allows selecting everything or nothing with no partial selection.
///
/// ** See code in examples/api/lib/material/selection_container/selection_container.0.dart **
/// {@end-tool}
///
/// In the case where a group of widgets should be excluded from selection under
/// a [SelectableRegion], consider wrapping that group of widgets using
/// [SelectionContainer.disabled].
///
/// {@tool dartpad}
/// This sample demonstrates how to disable selection for a Text in a Column.
///
/// ** See code in examples/api/lib/material/selection_container/selection_container_disabled.0.dart **
/// {@end-tool}
///
/// To create a separate selection system from its parent selection area,
/// wrap part of the subtree with another [SelectableRegion]. The selection of the
/// child selection area can not extend past its subtree, and the selection of
/// the parent selection area can not extend inside the child selection area.
///
/// ## Selection status
///
/// A [SelectableRegion]s [SelectableRegionSelectionStatus] is used to indicate whether
/// the [SelectableRegion] is actively changing the selection, or has finalized it. For
/// example, during a mouse click + drag, the [SelectableRegionSelectionStatus] will be
/// set to [SelectableRegionSelectionStatus.changing], and when the mouse click is released
/// the status will be set to [SelectableRegionSelectionStatus.finalized].
///
/// The default value of [SelectableRegion]s selection status
/// is [SelectableRegionSelectionStatus.finalized].
///
/// To access the [SelectableRegionSelectionStatus] of a parent [SelectableRegion]
/// use [SelectableRegionSelectionStatusScope.maybeOf] and retrieve the value from
/// the [ValueListenable].
///
/// One can also listen for changes to the [SelectableRegionSelectionStatus] by
/// adding a listener to the [ValueListenable] retrieved from [SelectableRegionSelectionStatusScope.maybeOf]
/// through [ValueListenable.addListener]. In Stateful widgets this is typically
/// done in [State.didChangeDependencies]. Remove the listener when no longer
/// needed, typically in your Stateful widgets [State.dispose] method through
/// [ValueListenable.removeListener].
///
/// ## Tests
///
/// In a test, a region can be selected either by faking drag events (e.g. using
/// [WidgetTester.dragFrom]) or by sending intents to a widget inside the region
/// that has been given a [GlobalKey], e.g.:
///
/// ```dart
/// Actions.invoke(key.currentContext!, const SelectAllTextIntent(SelectionChangedCause.keyboard));
/// ```
///
/// See also:
///
///  * [SelectionArea], which creates a [SelectableRegion] with
///    platform-adaptive selection controls.
///  * [SelectableText], which enables selection on a single run of text.
///  * [SelectionHandler], which contains APIs to handle selection events from the
///    [SelectableRegion].
///  * [Selectable], which provides API to participate in the selection system.
///  * [SelectionRegistrar], which [Selectable] needs to subscribe to receive
///    selection events.
///  * [SelectionContainer], which collects selectable widgets in the subtree
///    and provides api to dispatch selection event to the collected widget.
///  * [SelectionListener], which enables accessing the [SelectionDetails] of
///    the selectable subtree it wraps.
class SelectableRegion extends StatefulWidget {
  /// Create a new [SelectableRegion] widget.
  ///
  /// The [selectionControls] are used for building the selection handles and
  /// toolbar for mobile devices.
  const SelectableRegion({
    super.key,
    this.contextMenuBuilder,
    this.focusNode,
    this.magnifierConfiguration = TextMagnifierConfiguration.disabled,
    this.onSelectionChanged,
    required this.selectionControls,
    required this.child,
  });

  /// The configuration for the magnifier used with selections in this region.
  ///
  /// By default, [SelectableRegion]'s [TextMagnifierConfiguration] is disabled.
  /// For a version of [SelectableRegion] that adapts automatically to the
  /// current platform, consider [SelectionArea].
  ///
  /// {@macro flutter.widgets.magnifier.intro}
  final TextMagnifierConfiguration magnifierConfiguration;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// The child widget this selection area applies to.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// {@macro flutter.widgets.EditableText.contextMenuBuilder}
  final SelectableRegionContextMenuBuilder? contextMenuBuilder;

  /// The delegate to build the selection handles and toolbar for mobile
  /// devices.
  ///
  /// The [emptyTextSelectionControls] global variable provides a default
  /// [TextSelectionControls] implementation with no controls.
  final TextSelectionControls selectionControls;

  /// Called when the selected content changes.
  final ValueChanged<SelectedContent?>? onSelectionChanged;

  /// Returns the [ContextMenuButtonItem]s representing the buttons in this
  /// platform's default selection menu.
  ///
  /// For example, [SelectableRegion] uses this to generate the default buttons
  /// for its context menu.
  ///
  /// See also:
  ///
  /// * [SelectableRegionState.contextMenuButtonItems], which gives the
  ///   [ContextMenuButtonItem]s for a specific SelectableRegion.
  /// * [EditableText.getEditableButtonItems], which performs a similar role but
  ///   for content that is both selectable and editable.
  /// * [AdaptiveTextSelectionToolbar], which builds the toolbar itself, and can
  ///   take a list of [ContextMenuButtonItem]s with
  ///   [AdaptiveTextSelectionToolbar.buttonItems].
  /// * [AdaptiveTextSelectionToolbar.getAdaptiveButtons], which builds the button
  ///   Widgets for the current platform given [ContextMenuButtonItem]s.
  static List<ContextMenuButtonItem> getSelectableButtonItems({
    required final SelectionGeometry selectionGeometry,
    required final VoidCallback onCopy,
    required final VoidCallback onSelectAll,
    required final VoidCallback? onShare,
  }) {
    final bool canCopy = selectionGeometry.status == SelectionStatus.uncollapsed;
    final bool canSelectAll = selectionGeometry.hasContent;
    final bool platformCanShare = switch (defaultTargetPlatform) {
      TargetPlatform.android => selectionGeometry.status == SelectionStatus.uncollapsed,
      TargetPlatform.macOS ||
      TargetPlatform.fuchsia ||
      TargetPlatform.linux ||
      TargetPlatform.windows => false,
      // TODO(bleroux): the share button should be shown on iOS but the share
      // functionality requires some changes on the engine side because, on iPad,
      // it needs an anchor for the popup.
      // See: https://github.com/flutter/flutter/issues/141775.
      TargetPlatform.iOS => false,
    };
    final bool canShare = onShare != null && platformCanShare;

    // On Android, the share button is before the select all button.
    final bool showShareBeforeSelectAll = defaultTargetPlatform == TargetPlatform.android;

    // Determine which buttons will appear so that the order and total number is
    // known. A button's position in the menu can slightly affect its
    // appearance.
    return <ContextMenuButtonItem>[
      if (canCopy) ContextMenuButtonItem(onPressed: onCopy, type: ContextMenuButtonType.copy),
      if (canShare && showShareBeforeSelectAll)
        ContextMenuButtonItem(onPressed: onShare, type: ContextMenuButtonType.share),
      if (canSelectAll)
        ContextMenuButtonItem(onPressed: onSelectAll, type: ContextMenuButtonType.selectAll),
      if (canShare && !showShareBeforeSelectAll)
        ContextMenuButtonItem(onPressed: onShare, type: ContextMenuButtonType.share),
    ];
  }

  @override
  State<StatefulWidget> createState() => SelectableRegionState();
}

/// State for a [SelectableRegion].
class SelectableRegionState extends State<SelectableRegion>
    with TextSelectionDelegate
    implements SelectionRegistrar {
  late final Map<Type, Action<Intent>> _actions = <Type, Action<Intent>>{
    SelectAllTextIntent: _makeOverridable(_SelectAllAction(this)),
    CopySelectionTextIntent: _makeOverridable(_CopySelectionAction(this)),
    ExtendSelectionToNextWordBoundaryOrCaretLocationIntent: _makeOverridable(
      _GranularlyExtendSelectionAction<ExtendSelectionToNextWordBoundaryOrCaretLocationIntent>(
        this,
        granularity: TextGranularity.word,
      ),
    ),
    ExpandSelectionToDocumentBoundaryIntent: _makeOverridable(
      _GranularlyExtendSelectionAction<ExpandSelectionToDocumentBoundaryIntent>(
        this,
        granularity: TextGranularity.document,
      ),
    ),
    ExpandSelectionToLineBreakIntent: _makeOverridable(
      _GranularlyExtendSelectionAction<ExpandSelectionToLineBreakIntent>(
        this,
        granularity: TextGranularity.line,
      ),
    ),
    ExtendSelectionByCharacterIntent: _makeOverridable(
      _GranularlyExtendCaretSelectionAction<ExtendSelectionByCharacterIntent>(
        this,
        granularity: TextGranularity.character,
      ),
    ),
    ExtendSelectionToNextWordBoundaryIntent: _makeOverridable(
      _GranularlyExtendCaretSelectionAction<ExtendSelectionToNextWordBoundaryIntent>(
        this,
        granularity: TextGranularity.word,
      ),
    ),
    ExtendSelectionToLineBreakIntent: _makeOverridable(
      _GranularlyExtendCaretSelectionAction<ExtendSelectionToLineBreakIntent>(
        this,
        granularity: TextGranularity.line,
      ),
    ),
    ExtendSelectionVerticallyToAdjacentLineIntent: _makeOverridable(
      _DirectionallyExtendCaretSelectionAction<ExtendSelectionVerticallyToAdjacentLineIntent>(this),
    ),
    ExtendSelectionToDocumentBoundaryIntent: _makeOverridable(
      _GranularlyExtendCaretSelectionAction<ExtendSelectionToDocumentBoundaryIntent>(
        this,
        granularity: TextGranularity.document,
      ),
    ),
  };

  final Map<Type, GestureRecognizerFactory> _gestureRecognizers =
      <Type, GestureRecognizerFactory>{};
  SelectionOverlay? _selectionOverlay;
  final LayerLink _startHandleLayerLink = LayerLink();
  final LayerLink _endHandleLayerLink = LayerLink();
  final LayerLink _toolbarLayerLink = LayerLink();
  final StaticSelectionContainerDelegate _selectionDelegate = StaticSelectionContainerDelegate();
  // there should only ever be one selectable, which is the SelectionContainer.
  Selectable? _selectable;

  bool get _hasSelectionOverlayGeometry =>
      _selectionDelegate.value.startSelectionPoint != null ||
      _selectionDelegate.value.endSelectionPoint != null;

  Orientation? _lastOrientation;
  SelectedContent? _lastSelectedContent;

  /// The [SelectionOverlay] that is currently visible on the screen.
  ///
  /// Can be null if there is no visible [SelectionOverlay].
  @visibleForTesting
  SelectionOverlay? get selectionOverlay => _selectionOverlay;

  /// The text processing service used to retrieve the native text processing actions.
  final ProcessTextService _processTextService = DefaultProcessTextService();

  /// The list of native text processing actions provided by the engine.
  final List<ProcessTextAction> _processTextActions = <ProcessTextAction>[];

  // The focus node to use if the widget didn't supply one.
  FocusNode? _localFocusNode;
  FocusNode get _focusNode =>
      widget.focusNode ?? (_localFocusNode ??= FocusNode(debugLabel: 'SelectableRegion'));

  /// Notifies its listeners when the selection state in this [SelectableRegion] changes.
  final _SelectableRegionSelectionStatusNotifier _selectionStatusNotifier =
      _SelectableRegionSelectionStatusNotifier._();

  @protected
  @override
  void initState() {
    super.initState();
    _focusNode.addListener(_handleFocusChanged);
    _initMouseGestureRecognizer();
    _initTouchGestureRecognizer();
    // Right clicks.
    _gestureRecognizers[TapGestureRecognizer] =
        GestureRecognizerFactoryWithHandlers<TapGestureRecognizer>(
          () => TapGestureRecognizer(debugOwner: this),
          (TapGestureRecognizer instance) {
            instance.onSecondaryTapDown = _handleRightClickDown;
          },
        );
    _initProcessTextActions();
  }

  /// Query the engine to initialize the list of text processing actions to show
  /// in the text selection toolbar.
  Future<void> _initProcessTextActions() async {
    _processTextActions.clear();
    _processTextActions.addAll(await _processTextService.queryTextActions());
  }

  @protected
  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.iOS:
        break;
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
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
      hideToolbar(defaultTargetPlatform == TargetPlatform.android);
    }
  }

  @protected
  @override
  void didUpdateWidget(SelectableRegion oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.focusNode != oldWidget.focusNode) {
      if (oldWidget.focusNode == null && widget.focusNode != null) {
        _localFocusNode?.removeListener(_handleFocusChanged);
        _localFocusNode?.dispose();
        _localFocusNode = null;
      } else if (widget.focusNode == null && oldWidget.focusNode != null) {
        oldWidget.focusNode!.removeListener(_handleFocusChanged);
      }
      _focusNode.addListener(_handleFocusChanged);
      if (_focusNode.hasFocus != oldWidget.focusNode?.hasFocus) {
        _handleFocusChanged();
      }
    }
  }

  Action<T> _makeOverridable<T extends Intent>(Action<T> defaultAction) {
    return Action<T>.overridable(context: context, defaultAction: defaultAction);
  }

  void _handleFocusChanged() {
    if (!_focusNode.hasFocus) {
      if (kIsWeb) {
        PlatformSelectableRegionContextMenu.detach(_selectionDelegate);
      }
      if (SchedulerBinding.instance.lifecycleState == AppLifecycleState.resumed) {
        // We should only clear the selection when this SelectableRegion loses
        // focus while the application is currently running. It is possible
        // that the application is not currently running, for example on desktop
        // platforms, clicking on a different window switches the focus to
        // the new window causing the Flutter application to go inactive. In this
        // case we want to retain the selection so it remains when we return to
        // the Flutter application.
        clearSelection();
        _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
        _finalizeSelectableRegionStatus();
      }
    }
    if (kIsWeb) {
      PlatformSelectableRegionContextMenu.attach(_selectionDelegate);
    }
  }

  void _updateSelectionStatus() {
    final SelectionGeometry geometry = _selectionDelegate.value;
    final TextSelection selection = switch (geometry.status) {
      SelectionStatus.uncollapsed ||
      SelectionStatus.collapsed => const TextSelection(baseOffset: 0, extentOffset: 1),
      SelectionStatus.none => const TextSelection.collapsed(offset: 1),
    };
    textEditingValue = TextEditingValue(text: '__', selection: selection);
    if (_hasSelectionOverlayGeometry) {
      _updateSelectionOverlay();
    } else {
      _selectionOverlay?.dispose();
      _selectionOverlay = null;
    }
  }

  // gestures.

  /// Whether the Shift key was pressed when the most recent [PointerDownEvent]
  /// was tracked by the [BaseTapAndDragGestureRecognizer].
  bool _isShiftPressed = false;

  // The position of the most recent secondary tap down event on this
  // SelectableRegion.
  Offset? _lastSecondaryTapDownPosition;

  // The device kind for the pointer of the most recent tap down event on this
  // SelectableRegion.
  PointerDeviceKind? _lastPointerDeviceKind;

  static bool _isPrecisePointerDevice(PointerDeviceKind pointerDeviceKind) {
    switch (pointerDeviceKind) {
      case PointerDeviceKind.mouse:
        return true;
      case PointerDeviceKind.trackpad:
      case PointerDeviceKind.stylus:
      case PointerDeviceKind.invertedStylus:
      case PointerDeviceKind.touch:
      case PointerDeviceKind.unknown:
        return false;
    }
  }

  void _finalizeSelectableRegionStatus() {
    if (_selectionStatusNotifier.value != SelectableRegionSelectionStatus.changing) {
      // Don't finalize the selection again if it is not currently changing.
      return;
    }
    _selectionStatusNotifier.value = SelectableRegionSelectionStatus.finalized;
  }

  // Converts the details.consecutiveTapCount from a TapAndDrag*Details object,
  // which can grow to be infinitely large, to a value between 1 and the supported
  // max consecutive tap count. The value that the raw count is converted to is
  // based on the default observed behavior on the native platforms.
  //
  // This method should be used in all instances when details.consecutiveTapCount
  // would be used.
  int _getEffectiveConsecutiveTapCount(int rawCount) {
    int maxConsecutiveTap = 3;
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
        if (_lastPointerDeviceKind != null && _lastPointerDeviceKind != PointerDeviceKind.mouse) {
          // When the pointer device kind is not precise like a mouse, native
          // Android resets the tap count at 2. For example, this is so the
          // selection can collapse on the third tap.
          maxConsecutiveTap = 2;
        }
        // From observation, these platforms reset their tap count to 0 when
        // the number of consecutive taps exceeds the max consecutive tap supported.
        // For example on native Android, when going past a triple click,
        // on the fourth click the selection is moved to the precise click
        // position, on the fifth click the word at the position is selected, and
        // on the sixth click the paragraph at the position is selected.
        return rawCount <= maxConsecutiveTap
            ? rawCount
            : (rawCount % maxConsecutiveTap == 0
                ? maxConsecutiveTap
                : rawCount % maxConsecutiveTap);
      case TargetPlatform.linux:
        // From observation, these platforms reset their tap count to 0 when
        // the number of consecutive taps exceeds the max consecutive tap supported.
        // For example on Debian Linux with GTK, when going past a triple click,
        // on the fourth click the selection is moved to the precise click
        // position, on the fifth click the word at the position is selected, and
        // on the sixth click the paragraph at the position is selected.
        return rawCount <= maxConsecutiveTap
            ? rawCount
            : (rawCount % maxConsecutiveTap == 0
                ? maxConsecutiveTap
                : rawCount % maxConsecutiveTap);
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        // From observation, these platforms hold their tap count at the max
        // consecutive tap supported. For example on macOS, when going past a triple
        // click, the selection should be retained at the paragraph that was first
        // selected on triple click.
        return min(rawCount, maxConsecutiveTap);
    }
  }

  void _initMouseGestureRecognizer() {
    _gestureRecognizers[TapAndPanGestureRecognizer] =
        GestureRecognizerFactoryWithHandlers<TapAndPanGestureRecognizer>(
          () => TapAndPanGestureRecognizer(
            debugOwner: this,
            supportedDevices: <PointerDeviceKind>{PointerDeviceKind.mouse},
          ),
          (TapAndPanGestureRecognizer instance) {
            instance
              ..onTapTrackStart = _onTapTrackStart
              ..onTapTrackReset = _onTapTrackReset
              ..onTapDown = _startNewMouseSelectionGesture
              ..onTapUp = _handleMouseTapUp
              ..onDragStart = _handleMouseDragStart
              ..onDragUpdate = _handleMouseDragUpdate
              ..onDragEnd = _handleMouseDragEnd
              ..onCancel = clearSelection
              ..dragStartBehavior = DragStartBehavior.down;
          },
        );
  }

  void _onTapTrackStart() {
    _isShiftPressed =
        HardwareKeyboard.instance.logicalKeysPressed.intersection(<LogicalKeyboardKey>{
          LogicalKeyboardKey.shiftLeft,
          LogicalKeyboardKey.shiftRight,
        }).isNotEmpty;
  }

  void _onTapTrackReset() {
    _isShiftPressed = false;
  }

  void _initTouchGestureRecognizer() {
    // A [TapAndHorizontalDragGestureRecognizer] is used on non-precise pointer devices
    // like PointerDeviceKind.touch so [SelectableRegion] gestures do not conflict with
    // ancestor Scrollable gestures in common scenarios like a vertically scrolling list view.
    _gestureRecognizers[TapAndHorizontalDragGestureRecognizer] =
        GestureRecognizerFactoryWithHandlers<TapAndHorizontalDragGestureRecognizer>(
          () => TapAndHorizontalDragGestureRecognizer(
            debugOwner: this,
            supportedDevices:
                PointerDeviceKind.values.where((PointerDeviceKind device) {
                  return device != PointerDeviceKind.mouse;
                }).toSet(),
          ),
          (TapAndHorizontalDragGestureRecognizer instance) {
            instance
              // iOS does not provide a device specific touch slop
              // unlike Android (~8.0), so the touch slop for a [Scrollable]
              // always default to kTouchSlop which is 18.0. When
              // [SelectableRegion] is the child of a horizontal
              // scrollable that means the [SelectableRegion] will
              // always win the gesture arena when competing with
              // the ancestor scrollable because they both have
              // the same touch slop threshold and the child receives
              // the [PointerEvent] first. To avoid this conflict
              // and ensure a smooth scrolling experience, on
              // iOS the [TapAndHorizontalDragGestureRecognizer]
              // will wait for all other gestures to lose before
              // declaring victory.
              ..eagerVictoryOnDrag = defaultTargetPlatform != TargetPlatform.iOS
              ..onTapDown = _startNewMouseSelectionGesture
              ..onTapUp = _handleMouseTapUp
              ..onDragStart = _handleMouseDragStart
              ..onDragUpdate = _handleMouseDragUpdate
              ..onDragEnd = _handleMouseDragEnd
              ..onCancel = clearSelection
              ..dragStartBehavior = DragStartBehavior.down;
          },
        );
    _gestureRecognizers[LongPressGestureRecognizer] =
        GestureRecognizerFactoryWithHandlers<LongPressGestureRecognizer>(
          () => LongPressGestureRecognizer(
            debugOwner: this,
            supportedDevices: _kLongPressSelectionDevices,
          ),
          (LongPressGestureRecognizer instance) {
            instance
              ..onLongPressStart = _handleTouchLongPressStart
              ..onLongPressMoveUpdate = _handleTouchLongPressMoveUpdate
              ..onLongPressEnd = _handleTouchLongPressEnd;
          },
        );
  }

  Offset? _doubleTapOffset;
  void _startNewMouseSelectionGesture(TapDragDownDetails details) {
    _lastPointerDeviceKind = details.kind;
    switch (_getEffectiveConsecutiveTapCount(details.consecutiveTapCount)) {
      case 1:
        _focusNode.requestFocus();
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
            // On mobile platforms the selection is set on tap up for the first
            // tap.
            break;
          case TargetPlatform.macOS:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            hideToolbar();
            // It is impossible to extend the selection when the shift key is
            // pressed and the start of the selection has not been initialized.
            // In this case we fallback on collapsing the selection to first
            // initialize the selection.
            final bool isShiftPressedValid =
                _isShiftPressed && _selectionDelegate.value.startSelectionPoint != null;
            if (isShiftPressedValid) {
              _selectEndTo(offset: details.globalPosition);
              _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
              break;
            }
            clearSelection();
            _collapseSelectionAt(offset: details.globalPosition);
            _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
        }
      case 2:
        switch (defaultTargetPlatform) {
          case TargetPlatform.iOS:
            if (kIsWeb && details.kind != null && !_isPrecisePointerDevice(details.kind!)) {
              // Double tap on iOS web triggers when a drag begins after the double tap.
              _doubleTapOffset = details.globalPosition;
              break;
            }
            _selectWordAt(offset: details.globalPosition);
            _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
            if (details.kind != null && !_isPrecisePointerDevice(details.kind!)) {
              _showHandles();
            }
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.macOS:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            _selectWordAt(offset: details.globalPosition);
            _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
        }
      case 3:
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
            if (details.kind != null && _isPrecisePointerDevice(details.kind!)) {
              // Triple tap on static text is only supported on mobile
              // platforms using a precise pointer device.
              _selectParagraphAt(offset: details.globalPosition);
              _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
            }
          case TargetPlatform.macOS:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            _selectParagraphAt(offset: details.globalPosition);
            _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
        }
    }
    _updateSelectedContentIfNeeded();
  }

  void _handleMouseDragStart(TapDragStartDetails details) {
    switch (_getEffectiveConsecutiveTapCount(details.consecutiveTapCount)) {
      case 1:
        if (details.kind != null && !_isPrecisePointerDevice(details.kind!)) {
          // Drag to select is only enabled with a precise pointer device.
          return;
        }
        _selectStartTo(offset: details.globalPosition);
        _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
    }
    _updateSelectedContentIfNeeded();
  }

  void _handleMouseDragUpdate(TapDragUpdateDetails details) {
    switch (_getEffectiveConsecutiveTapCount(details.consecutiveTapCount)) {
      case 1:
        if (details.kind != null && !_isPrecisePointerDevice(details.kind!)) {
          // Drag to select is only enabled with a precise pointer device.
          return;
        }
        _selectEndTo(offset: details.globalPosition, continuous: true);
        _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
      case 2:
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
            // Double tap + drag is only supported on Android when using a precise
            // pointer device or when not on the web.
            if (!kIsWeb || details.kind != null && _isPrecisePointerDevice(details.kind!)) {
              _selectEndTo(
                offset: details.globalPosition,
                continuous: true,
                textGranularity: TextGranularity.word,
              );
              _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
            }
          case TargetPlatform.iOS:
            if (kIsWeb &&
                details.kind != null &&
                !_isPrecisePointerDevice(details.kind!) &&
                _doubleTapOffset != null) {
              // On iOS web a double tap does not select the word at the position,
              // until the drag has begun.
              _selectWordAt(offset: _doubleTapOffset!);
              _doubleTapOffset = null;
            }
            _selectEndTo(
              offset: details.globalPosition,
              continuous: true,
              textGranularity: TextGranularity.word,
            );
            _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
            if (details.kind != null && !_isPrecisePointerDevice(details.kind!)) {
              _showHandles();
            }
          case TargetPlatform.macOS:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            _selectEndTo(
              offset: details.globalPosition,
              continuous: true,
              textGranularity: TextGranularity.word,
            );
            _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
        }
      case 3:
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
            // Triple tap + drag is only supported on mobile devices when using
            // a precise pointer device.
            if (details.kind != null && _isPrecisePointerDevice(details.kind!)) {
              _selectEndTo(
                offset: details.globalPosition,
                continuous: true,
                textGranularity: TextGranularity.paragraph,
              );
              _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
            }
          case TargetPlatform.macOS:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            _selectEndTo(
              offset: details.globalPosition,
              continuous: true,
              textGranularity: TextGranularity.paragraph,
            );
            _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
        }
    }
    _updateSelectedContentIfNeeded();
  }

  void _handleMouseDragEnd(TapDragEndDetails details) {
    assert(_lastPointerDeviceKind != null);
    final bool isPointerPrecise = _isPrecisePointerDevice(_lastPointerDeviceKind!);
    // On mobile platforms like android, fuchsia, and iOS, a drag gesture will
    // only show the selection overlay when the drag has finished and the pointer
    // device kind is not precise, for example at the end of a double tap + drag
    // to select on native iOS.
    final bool shouldShowSelectionOverlayOnMobile = !isPointerPrecise;
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
        if (shouldShowSelectionOverlayOnMobile) {
          _showHandles();
          _showToolbar();
        }
      case TargetPlatform.iOS:
        if (shouldShowSelectionOverlayOnMobile) {
          _showToolbar();
        }
      case TargetPlatform.macOS:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        // The selection overlay is not shown on desktop platforms after a drag.
        break;
    }
    _finalizeSelection();
    _updateSelectedContentIfNeeded();
    _finalizeSelectableRegionStatus();
  }

  void _handleMouseTapUp(TapDragUpDetails details) {
    if (defaultTargetPlatform == TargetPlatform.iOS &&
        _positionIsOnActiveSelection(globalPosition: details.globalPosition)) {
      // On iOS when the tap occurs on the previous selection, instead of
      // moving the selection, the context menu will be toggled.
      final bool toolbarIsVisible = _selectionOverlay?.toolbarIsVisible ?? false;
      if (toolbarIsVisible) {
        hideToolbar(false);
      } else {
        _showToolbar();
      }
      return;
    }
    switch (_getEffectiveConsecutiveTapCount(details.consecutiveTapCount)) {
      case 1:
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
            hideToolbar();
            _collapseSelectionAt(offset: details.globalPosition);
            _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
          case TargetPlatform.macOS:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
          // On desktop platforms the selection is set on tap down.
        }
      case 2:
        final bool isPointerPrecise = _isPrecisePointerDevice(details.kind);
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
            if (!isPointerPrecise) {
              // On Android, a double tap will only show the selection overlay after
              // the following tap up when the pointer device kind is not precise.
              _showHandles();
              _showToolbar();
            }
          case TargetPlatform.iOS:
            if (!isPointerPrecise) {
              if (kIsWeb) {
                // Double tap on iOS web only triggers when a drag begins after the double tap.
                break;
              }
              // On iOS, a double tap will only show the selection toolbar after
              // the following tap up when the pointer device kind is not precise.
              _showToolbar();
            }
          case TargetPlatform.macOS:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            // The selection overlay is not shown on desktop platforms
            // on a double click.
            break;
        }
    }
    _finalizeSelectableRegionStatus();
    _updateSelectedContentIfNeeded();
  }

  void _updateSelectedContentIfNeeded() {
    if (widget.onSelectionChanged == null) {
      return;
    }
    final SelectedContent? content = _selectable?.getSelectedContent();
    if (_lastSelectedContent?.plainText != content?.plainText) {
      _lastSelectedContent = content;
      widget.onSelectionChanged!.call(_lastSelectedContent);
    }
  }

  void _handleTouchLongPressStart(LongPressStartDetails details) {
    HapticFeedback.selectionClick();
    _focusNode.requestFocus();
    _selectWordAt(offset: details.globalPosition);
    _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
    // Platforms besides Android will show the text selection handles when
    // the long press is initiated. Android shows the text selection handles when
    // the long press has ended, usually after a pointer up event is received.
    if (defaultTargetPlatform != TargetPlatform.android) {
      _showHandles();
    }
    _updateSelectedContentIfNeeded();
  }

  void _handleTouchLongPressMoveUpdate(LongPressMoveUpdateDetails details) {
    _selectEndTo(offset: details.globalPosition, textGranularity: TextGranularity.word);
    _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
    _updateSelectedContentIfNeeded();
  }

  void _handleTouchLongPressEnd(LongPressEndDetails details) {
    _finalizeSelection();
    _updateSelectedContentIfNeeded();
    _finalizeSelectableRegionStatus();
    _showToolbar();
    if (defaultTargetPlatform == TargetPlatform.android) {
      _showHandles();
    }
  }

  bool _positionIsOnActiveSelection({required Offset globalPosition}) {
    for (final Rect selectionRect in _selectionDelegate.value.selectionRects) {
      final Matrix4 transform = _selectable!.getTransformTo(null);
      final Rect globalRect = MatrixUtils.transformRect(transform, selectionRect);
      if (globalRect.contains(globalPosition)) {
        return true;
      }
    }
    return false;
  }

  void _handleRightClickDown(TapDownDetails details) {
    final Offset? previousSecondaryTapDownPosition = _lastSecondaryTapDownPosition;
    final bool toolbarIsVisible = _selectionOverlay?.toolbarIsVisible ?? false;
    _lastSecondaryTapDownPosition = details.globalPosition;
    _focusNode.requestFocus();
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.windows:
        // If _lastSecondaryTapDownPosition is within the current selection then
        // keep the current selection, if not then collapse it.
        final bool lastSecondaryTapDownPositionWasOnActiveSelection = _positionIsOnActiveSelection(
          globalPosition: details.globalPosition,
        );
        if (lastSecondaryTapDownPositionWasOnActiveSelection) {
          // Restore _lastSecondaryTapDownPosition since it may be cleared if a user
          // accesses contextMenuAnchors.
          _lastSecondaryTapDownPosition = details.globalPosition;
          _showHandles();
          _showToolbar(location: _lastSecondaryTapDownPosition);
          _updateSelectedContentIfNeeded();
          return;
        }
        _collapseSelectionAt(offset: _lastSecondaryTapDownPosition!);
      case TargetPlatform.iOS:
        _selectWordAt(offset: _lastSecondaryTapDownPosition!);
      case TargetPlatform.macOS:
        if (previousSecondaryTapDownPosition == _lastSecondaryTapDownPosition && toolbarIsVisible) {
          hideToolbar();
          return;
        }
        _selectWordAt(offset: _lastSecondaryTapDownPosition!);
      case TargetPlatform.linux:
        if (toolbarIsVisible) {
          hideToolbar();
          return;
        }
        // If _lastSecondaryTapDownPosition is within the current selection then
        // keep the current selection, if not then collapse it.
        final bool lastSecondaryTapDownPositionWasOnActiveSelection = _positionIsOnActiveSelection(
          globalPosition: details.globalPosition,
        );
        if (!lastSecondaryTapDownPositionWasOnActiveSelection) {
          _collapseSelectionAt(offset: _lastSecondaryTapDownPosition!);
        }
    }
    _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
    _finalizeSelectableRegionStatus();
    // Restore _lastSecondaryTapDownPosition since it may be cleared if a user
    // accesses contextMenuAnchors.
    _lastSecondaryTapDownPosition = details.globalPosition;
    _showHandles();
    _showToolbar(location: _lastSecondaryTapDownPosition);
    _updateSelectedContentIfNeeded();
  }

  // Selection update helper methods.

  Offset? _selectionEndPosition;
  bool get _userDraggingSelectionEnd => _selectionEndPosition != null;
  bool _scheduledSelectionEndEdgeUpdate = false;

  /// Sends end [SelectionEdgeUpdateEvent] to the selectable subtree.
  ///
  /// If the selectable subtree returns a [SelectionResult.pending], this method
  /// continues to send [SelectionEdgeUpdateEvent]s every frame until the result
  /// is not pending or users end their gestures.
  void _triggerSelectionEndEdgeUpdate({TextGranularity? textGranularity}) {
    // This method can be called when the drag is not in progress. This can
    // happen if the child scrollable returns SelectionResult.pending, and
    // the selection area scheduled a selection update for the next frame, but
    // the drag is lifted before the scheduled selection update is run.
    if (_scheduledSelectionEndEdgeUpdate || !_userDraggingSelectionEnd) {
      return;
    }
    if (_selectable?.dispatchSelectionEvent(
          SelectionEdgeUpdateEvent.forEnd(
            globalPosition: _selectionEndPosition!,
            granularity: textGranularity,
          ),
        ) ==
        SelectionResult.pending) {
      _scheduledSelectionEndEdgeUpdate = true;
      SchedulerBinding.instance.addPostFrameCallback((Duration timeStamp) {
        if (!_scheduledSelectionEndEdgeUpdate) {
          return;
        }
        _scheduledSelectionEndEdgeUpdate = false;
        _triggerSelectionEndEdgeUpdate(textGranularity: textGranularity);
      }, debugLabel: 'SelectableRegion.endEdgeUpdate');
      return;
    }
  }

  void _onAnyDragEnd(DragEndDetails details) {
    if (widget.selectionControls is! TextSelectionHandleControls) {
      _selectionOverlay!.hideMagnifier();
      _selectionOverlay!.showToolbar();
    } else {
      _selectionOverlay!.hideMagnifier();
      _selectionOverlay!.showToolbar(
        context: context,
        contextMenuBuilder: (BuildContext context) {
          return widget.contextMenuBuilder!(context, this);
        },
      );
    }
    _finalizeSelection();
    _updateSelectedContentIfNeeded();
    _finalizeSelectableRegionStatus();
  }

  void _stopSelectionEndEdgeUpdate() {
    _scheduledSelectionEndEdgeUpdate = false;
    _selectionEndPosition = null;
  }

  Offset? _selectionStartPosition;
  bool get _userDraggingSelectionStart => _selectionStartPosition != null;
  bool _scheduledSelectionStartEdgeUpdate = false;

  /// Sends start [SelectionEdgeUpdateEvent] to the selectable subtree.
  ///
  /// If the selectable subtree returns a [SelectionResult.pending], this method
  /// continues to send [SelectionEdgeUpdateEvent]s every frame until the result
  /// is not pending or users end their gestures.
  void _triggerSelectionStartEdgeUpdate({TextGranularity? textGranularity}) {
    // This method can be called when the drag is not in progress. This can
    // happen if the child scrollable returns SelectionResult.pending, and
    // the selection area scheduled a selection update for the next frame, but
    // the drag is lifted before the scheduled selection update is run.
    if (_scheduledSelectionStartEdgeUpdate || !_userDraggingSelectionStart) {
      return;
    }
    if (_selectable?.dispatchSelectionEvent(
          SelectionEdgeUpdateEvent.forStart(
            globalPosition: _selectionStartPosition!,
            granularity: textGranularity,
          ),
        ) ==
        SelectionResult.pending) {
      _scheduledSelectionStartEdgeUpdate = true;
      SchedulerBinding.instance.addPostFrameCallback((Duration timeStamp) {
        if (!_scheduledSelectionStartEdgeUpdate) {
          return;
        }
        _scheduledSelectionStartEdgeUpdate = false;
        _triggerSelectionStartEdgeUpdate(textGranularity: textGranularity);
      }, debugLabel: 'SelectableRegion.startEdgeUpdate');
      return;
    }
  }

  void _stopSelectionStartEdgeUpdate() {
    _scheduledSelectionStartEdgeUpdate = false;
    _selectionEndPosition = null;
  }

  // SelectionOverlay helper methods.

  late Offset _selectionStartHandleDragPosition;
  late Offset _selectionEndHandleDragPosition;

  void _handleSelectionStartHandleDragStart(DragStartDetails details) {
    assert(_selectionDelegate.value.startSelectionPoint != null);

    final Offset localPosition = _selectionDelegate.value.startSelectionPoint!.localPosition;
    final Matrix4 globalTransform = _selectable!.getTransformTo(null);
    _selectionStartHandleDragPosition = MatrixUtils.transformPoint(globalTransform, localPosition);

    _selectionOverlay!.showMagnifier(
      _buildInfoForMagnifier(details.globalPosition, _selectionDelegate.value.startSelectionPoint!),
    );
    _updateSelectedContentIfNeeded();
  }

  void _handleSelectionStartHandleDragUpdate(DragUpdateDetails details) {
    _selectionStartHandleDragPosition = _selectionStartHandleDragPosition + details.delta;
    // The value corresponds to the paint origin of the selection handle.
    // Offset it to the center of the line to make it feel more natural.
    _selectionStartPosition =
        _selectionStartHandleDragPosition -
        Offset(0, _selectionDelegate.value.startSelectionPoint!.lineHeight / 2);
    _triggerSelectionStartEdgeUpdate();

    _selectionOverlay!.updateMagnifier(
      _buildInfoForMagnifier(details.globalPosition, _selectionDelegate.value.startSelectionPoint!),
    );
    _updateSelectedContentIfNeeded();
    _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
  }

  void _handleSelectionEndHandleDragStart(DragStartDetails details) {
    assert(_selectionDelegate.value.endSelectionPoint != null);
    final Offset localPosition = _selectionDelegate.value.endSelectionPoint!.localPosition;
    final Matrix4 globalTransform = _selectable!.getTransformTo(null);
    _selectionEndHandleDragPosition = MatrixUtils.transformPoint(globalTransform, localPosition);

    _selectionOverlay!.showMagnifier(
      _buildInfoForMagnifier(details.globalPosition, _selectionDelegate.value.endSelectionPoint!),
    );
    _updateSelectedContentIfNeeded();
  }

  void _handleSelectionEndHandleDragUpdate(DragUpdateDetails details) {
    _selectionEndHandleDragPosition = _selectionEndHandleDragPosition + details.delta;
    // The value corresponds to the paint origin of the selection handle.
    // Offset it to the center of the line to make it feel more natural.
    _selectionEndPosition =
        _selectionEndHandleDragPosition -
        Offset(0, _selectionDelegate.value.endSelectionPoint!.lineHeight / 2);
    _triggerSelectionEndEdgeUpdate();

    _selectionOverlay!.updateMagnifier(
      _buildInfoForMagnifier(details.globalPosition, _selectionDelegate.value.endSelectionPoint!),
    );
    _updateSelectedContentIfNeeded();
    _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
  }

  MagnifierInfo _buildInfoForMagnifier(
    Offset globalGesturePosition,
    SelectionPoint selectionPoint,
  ) {
    final Vector3 globalTransform = _selectable!.getTransformTo(null).getTranslation();
    final Offset globalTransformAsOffset = Offset(globalTransform.x, globalTransform.y);
    final Offset globalSelectionPointPosition =
        selectionPoint.localPosition + globalTransformAsOffset;
    final Rect caretRect = Rect.fromLTWH(
      globalSelectionPointPosition.dx,
      globalSelectionPointPosition.dy - selectionPoint.lineHeight,
      0,
      selectionPoint.lineHeight,
    );

    return MagnifierInfo(
      globalGesturePosition: globalGesturePosition,
      caretRect: caretRect,
      fieldBounds: globalTransformAsOffset & _selectable!.size,
      currentLineBoundaries: globalTransformAsOffset & _selectable!.size,
    );
  }

  void _createSelectionOverlay() {
    assert(_hasSelectionOverlayGeometry);
    if (_selectionOverlay != null) {
      return;
    }
    final SelectionPoint? start = _selectionDelegate.value.startSelectionPoint;
    final SelectionPoint? end = _selectionDelegate.value.endSelectionPoint;
    _selectionOverlay = SelectionOverlay(
      context: context,
      debugRequiredFor: widget,
      startHandleType: start?.handleType ?? TextSelectionHandleType.collapsed,
      lineHeightAtStart: start?.lineHeight ?? end!.lineHeight,
      onStartHandleDragStart: _handleSelectionStartHandleDragStart,
      onStartHandleDragUpdate: _handleSelectionStartHandleDragUpdate,
      onStartHandleDragEnd: _onAnyDragEnd,
      endHandleType: end?.handleType ?? TextSelectionHandleType.collapsed,
      lineHeightAtEnd: end?.lineHeight ?? start!.lineHeight,
      onEndHandleDragStart: _handleSelectionEndHandleDragStart,
      onEndHandleDragUpdate: _handleSelectionEndHandleDragUpdate,
      onEndHandleDragEnd: _onAnyDragEnd,
      selectionEndpoints: selectionEndpoints,
      selectionControls: widget.selectionControls,
      selectionDelegate: this,
      clipboardStatus: null,
      startHandleLayerLink: _startHandleLayerLink,
      endHandleLayerLink: _endHandleLayerLink,
      toolbarLayerLink: _toolbarLayerLink,
      magnifierConfiguration: widget.magnifierConfiguration,
    );
  }

  void _updateSelectionOverlay() {
    if (_selectionOverlay == null) {
      return;
    }
    assert(_hasSelectionOverlayGeometry);
    final SelectionPoint? start = _selectionDelegate.value.startSelectionPoint;
    final SelectionPoint? end = _selectionDelegate.value.endSelectionPoint;
    _selectionOverlay!
      ..startHandleType = start?.handleType ?? TextSelectionHandleType.left
      ..lineHeightAtStart = start?.lineHeight ?? end!.lineHeight
      ..endHandleType = end?.handleType ?? TextSelectionHandleType.right
      ..lineHeightAtEnd = end?.lineHeight ?? start!.lineHeight
      ..selectionEndpoints = selectionEndpoints;
  }

  /// Shows the selection handles.
  ///
  /// Returns true if the handles are shown, false if the handles can't be
  /// shown.
  bool _showHandles() {
    if (_selectionOverlay != null) {
      _selectionOverlay!.showHandles();
      return true;
    }

    if (!_hasSelectionOverlayGeometry) {
      return false;
    }

    _createSelectionOverlay();
    _selectionOverlay!.showHandles();
    return true;
  }

  /// Shows the text selection toolbar.
  ///
  /// If the parameter `location` is set, the toolbar will be shown at the
  /// location. Otherwise, the toolbar location will be calculated based on the
  /// handles' locations. The `location` is in the coordinates system of the
  /// [Overlay].
  ///
  /// Returns true if the toolbar is shown, false if the toolbar can't be shown.
  bool _showToolbar({Offset? location}) {
    if (!_hasSelectionOverlayGeometry && _selectionOverlay == null) {
      return false;
    }

    // Web is using native dom elements to enable clipboard functionality of the
    // context menu: copy, paste, select, cut. It might also provide additional
    // functionality depending on the browser (such as translate). Due to this,
    // we should not show a Flutter toolbar for the editable text elements
    // unless the browser's context menu is explicitly disabled.
    if (kIsWeb && BrowserContextMenu.enabled) {
      return false;
    }

    if (_selectionOverlay == null) {
      _createSelectionOverlay();
    }

    _selectionOverlay!.toolbarLocation = location;
    if (widget.selectionControls is! TextSelectionHandleControls) {
      _selectionOverlay!.showToolbar();
      return true;
    }

    _selectionOverlay!.hideToolbar();

    _selectionOverlay!.showToolbar(
      context: context,
      contextMenuBuilder: (BuildContext context) {
        return widget.contextMenuBuilder!(context, this);
      },
    );
    return true;
  }

  /// Sets or updates selection end edge to the `offset` location.
  ///
  /// A selection always contains a select start edge and selection end edge.
  /// They can be created by calling both [_selectStartTo] and [_selectEndTo], or
  /// use other selection APIs, such as [_selectWordAt] or [selectAll].
  ///
  /// This method sets or updates the selection end edge by sending
  /// [SelectionEdgeUpdateEvent]s to the child [Selectable]s.
  ///
  /// If `continuous` is set to true and the update causes scrolling, the
  /// method will continue sending the same [SelectionEdgeUpdateEvent]s to the
  /// child [Selectable]s every frame until the scrolling finishes or a
  /// [_finalizeSelection] is called.
  ///
  /// The `continuous` argument defaults to false.
  ///
  /// The `offset` is in global coordinates.
  ///
  /// Provide the `textGranularity` if the selection should not move by the default
  /// [TextGranularity.character]. Only [TextGranularity.character] and
  /// [TextGranularity.word] are currently supported.
  ///
  /// See also:
  ///  * [_selectStartTo], which sets or updates selection start edge.
  ///  * [_finalizeSelection], which stops the `continuous` updates.
  ///  * [clearSelection], which clears the ongoing selection.
  ///  * [_selectWordAt], which selects a whole word at the location.
  ///  * [_selectParagraphAt], which selects an entire paragraph at the location.
  ///  * [_collapseSelectionAt], which collapses the selection at the location.
  ///  * [selectAll], which selects the entire content.
  void _selectEndTo({
    required Offset offset,
    bool continuous = false,
    TextGranularity? textGranularity,
  }) {
    if (!continuous) {
      _selectable?.dispatchSelectionEvent(
        SelectionEdgeUpdateEvent.forEnd(globalPosition: offset, granularity: textGranularity),
      );
      return;
    }
    if (_selectionEndPosition != offset) {
      _selectionEndPosition = offset;
      _triggerSelectionEndEdgeUpdate(textGranularity: textGranularity);
    }
  }

  /// Sets or updates selection start edge to the `offset` location.
  ///
  /// A selection always contains a select start edge and selection end edge.
  /// They can be created by calling both [_selectStartTo] and [_selectEndTo], or
  /// use other selection APIs, such as [_selectWordAt] or [selectAll].
  ///
  /// This method sets or updates the selection start edge by sending
  /// [SelectionEdgeUpdateEvent]s to the child [Selectable]s.
  ///
  /// If `continuous` is set to true and the update causes scrolling, the
  /// method will continue sending the same [SelectionEdgeUpdateEvent]s to the
  /// child [Selectable]s every frame until the scrolling finishes or a
  /// [_finalizeSelection] is called.
  ///
  /// The `continuous` argument defaults to false.
  ///
  /// The `offset` is in global coordinates.
  ///
  /// Provide the `textGranularity` if the selection should not move by the default
  /// [TextGranularity.character]. Only [TextGranularity.character] and
  /// [TextGranularity.word] are currently supported.
  ///
  /// See also:
  ///  * [_selectEndTo], which sets or updates selection end edge.
  ///  * [_finalizeSelection], which stops the `continuous` updates.
  ///  * [clearSelection], which clears the ongoing selection.
  ///  * [_selectWordAt], which selects a whole word at the location.
  ///  * [_selectParagraphAt], which selects an entire paragraph at the location.
  ///  * [_collapseSelectionAt], which collapses the selection at the location.
  ///  * [selectAll], which selects the entire content.
  void _selectStartTo({
    required Offset offset,
    bool continuous = false,
    TextGranularity? textGranularity,
  }) {
    if (!continuous) {
      _selectable?.dispatchSelectionEvent(
        SelectionEdgeUpdateEvent.forStart(globalPosition: offset, granularity: textGranularity),
      );
      return;
    }
    if (_selectionStartPosition != offset) {
      _selectionStartPosition = offset;
      _triggerSelectionStartEdgeUpdate(textGranularity: textGranularity);
    }
  }

  /// Collapses the selection at the given `offset` location.
  ///
  /// The `offset` is in global coordinates.
  ///
  /// See also:
  ///  * [_selectStartTo], which sets or updates selection start edge.
  ///  * [_selectEndTo], which sets or updates selection end edge.
  ///  * [_finalizeSelection], which stops the `continuous` updates.
  ///  * [clearSelection], which clears the ongoing selection.
  ///  * [_selectWordAt], which selects a whole word at the location.
  ///  * [_selectParagraphAt], which selects an entire paragraph at the location.
  ///  * [selectAll], which selects the entire content.
  void _collapseSelectionAt({required Offset offset}) {
    // There may be other selection ongoing.
    _finalizeSelection();
    _selectStartTo(offset: offset);
    _selectEndTo(offset: offset);
  }

  /// Selects a whole word at the `offset` location.
  ///
  /// The `offset` is in global coordinates.
  ///
  /// If the whole word is already in the current selection, selection won't
  /// change. One call [clearSelection] first if the selection needs to be
  /// updated even if the word is already covered by the current selection.
  ///
  /// One can also use [_selectEndTo] or [_selectStartTo] to adjust the selection
  /// edges after calling this method.
  ///
  /// See also:
  ///  * [_selectStartTo], which sets or updates selection start edge.
  ///  * [_selectEndTo], which sets or updates selection end edge.
  ///  * [_finalizeSelection], which stops the `continuous` updates.
  ///  * [clearSelection], which clears the ongoing selection.
  ///  * [_collapseSelectionAt], which collapses the selection at the location.
  ///  * [_selectParagraphAt], which selects an entire paragraph at the location.
  ///  * [selectAll], which selects the entire content.
  void _selectWordAt({required Offset offset}) {
    // There may be other selection ongoing.
    _finalizeSelection();
    _selectable?.dispatchSelectionEvent(SelectWordSelectionEvent(globalPosition: offset));
  }

  /// Selects the entire paragraph at the `offset` location.
  ///
  /// The `offset` is in global coordinates.
  ///
  /// If the paragraph is already in the current selection, selection won't
  /// change. One call [clearSelection] first if the selection needs to be
  /// updated even if the paragraph is already covered by the current selection.
  ///
  /// One can also use [_selectEndTo] or [_selectStartTo] to adjust the selection
  /// edges after calling this method.
  ///
  /// See also:
  ///  * [_selectStartTo], which sets or updates selection start edge.
  ///  * [_selectEndTo], which sets or updates selection end edge.
  ///  * [_finalizeSelection], which stops the `continuous` updates.
  ///  * [clearSelection], which clear the ongoing selection.
  ///  * [_selectWordAt], which selects a whole word at the location.
  ///  * [selectAll], which selects the entire content.
  void _selectParagraphAt({required Offset offset}) {
    // There may be other selection ongoing.
    _finalizeSelection();
    _selectable?.dispatchSelectionEvent(SelectParagraphSelectionEvent(globalPosition: offset));
  }

  /// Stops any ongoing selection updates.
  ///
  /// This method is different from [clearSelection] that it does not remove
  /// the current selection. It only stops the continuous updates.
  ///
  /// A continuous update can happen as result of calling [_selectStartTo] or
  /// [_selectEndTo] with `continuous` sets to true which causes a [Selectable]
  /// to scroll. Calling this method will stop the update as well as the
  /// scrolling.
  void _finalizeSelection() {
    _stopSelectionEndEdgeUpdate();
    _stopSelectionStartEdgeUpdate();
  }

  /// Removes the ongoing selection for this [SelectableRegion].
  void clearSelection() {
    _finalizeSelection();
    _directionalHorizontalBaseline = null;
    _adjustingSelectionEnd = null;
    _selectable?.dispatchSelectionEvent(const ClearSelectionEvent());
    _updateSelectedContentIfNeeded();
  }

  Future<void> _copy() async {
    final SelectedContent? data = _selectable?.getSelectedContent();
    if (data == null) {
      return;
    }
    await Clipboard.setData(ClipboardData(text: data.plainText));
  }

  Future<void> _share() async {
    final SelectedContent? data = _selectable?.getSelectedContent();
    if (data == null) {
      return;
    }
    await SystemChannels.platform.invokeMethod('Share.invoke', data.plainText);
  }

  /// {@macro flutter.widgets.EditableText.getAnchors}
  ///
  /// See also:
  ///
  ///  * [contextMenuButtonItems], which provides the [ContextMenuButtonItem]s
  ///    for the default context menu buttons.
  TextSelectionToolbarAnchors get contextMenuAnchors {
    if (_lastSecondaryTapDownPosition != null) {
      final TextSelectionToolbarAnchors anchors = TextSelectionToolbarAnchors(
        primaryAnchor: _lastSecondaryTapDownPosition!,
      );
      // Clear the state of _lastSecondaryTapDownPosition after use since a user may
      // access contextMenuAnchors and receive invalid anchors for their context menu.
      _lastSecondaryTapDownPosition = null;
      return anchors;
    }
    final RenderBox renderBox = context.findRenderObject()! as RenderBox;
    return TextSelectionToolbarAnchors.fromSelection(
      renderBox: renderBox,
      startGlyphHeight: startGlyphHeight,
      endGlyphHeight: endGlyphHeight,
      selectionEndpoints: selectionEndpoints,
    );
  }

  bool? _adjustingSelectionEnd;
  bool _determineIsAdjustingSelectionEnd(bool forward) {
    if (_adjustingSelectionEnd != null) {
      return _adjustingSelectionEnd!;
    }
    final bool isReversed;
    final SelectionPoint start = _selectionDelegate.value.startSelectionPoint!;
    final SelectionPoint end = _selectionDelegate.value.endSelectionPoint!;
    if (start.localPosition.dy > end.localPosition.dy) {
      isReversed = true;
    } else if (start.localPosition.dy < end.localPosition.dy) {
      isReversed = false;
    } else {
      isReversed = start.localPosition.dx > end.localPosition.dx;
    }
    // Always move the selection edge that increases the selection range.
    return _adjustingSelectionEnd = forward != isReversed;
  }

  void _granularlyExtendSelection(TextGranularity granularity, bool forward) {
    _directionalHorizontalBaseline = null;
    if (!_selectionDelegate.value.hasSelection) {
      return;
    }
    _selectable?.dispatchSelectionEvent(
      GranularlyExtendSelectionEvent(
        forward: forward,
        isEnd: _determineIsAdjustingSelectionEnd(forward),
        granularity: granularity,
      ),
    );
    _updateSelectedContentIfNeeded();
    _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
    _finalizeSelectableRegionStatus();
  }

  double? _directionalHorizontalBaseline;

  void _directionallyExtendSelection(bool forward) {
    if (!_selectionDelegate.value.hasSelection) {
      return;
    }
    final bool adjustingSelectionExtend = _determineIsAdjustingSelectionEnd(forward);
    final SelectionPoint baseLinePoint =
        adjustingSelectionExtend
            ? _selectionDelegate.value.endSelectionPoint!
            : _selectionDelegate.value.startSelectionPoint!;
    _directionalHorizontalBaseline ??= baseLinePoint.localPosition.dx;
    final Offset globalSelectionPointOffset = MatrixUtils.transformPoint(
      context.findRenderObject()!.getTransformTo(null),
      Offset(_directionalHorizontalBaseline!, 0),
    );
    _selectable?.dispatchSelectionEvent(
      DirectionallyExtendSelectionEvent(
        isEnd: _adjustingSelectionEnd!,
        direction:
            forward ? SelectionExtendDirection.nextLine : SelectionExtendDirection.previousLine,
        dx: globalSelectionPointOffset.dx,
      ),
    );
    _updateSelectedContentIfNeeded();
    _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
    _finalizeSelectableRegionStatus();
  }

  // [TextSelectionDelegate] overrides.

  /// Returns the [ContextMenuButtonItem]s representing the buttons in this
  /// platform's default selection menu.
  ///
  /// See also:
  ///
  /// * [SelectableRegion.getSelectableButtonItems], which performs a similar role,
  ///   but for any selectable text, not just specifically SelectableRegion.
  /// * [EditableTextState.contextMenuButtonItems], which performs a similar role
  ///   but for content that is not just selectable but also editable.
  /// * [contextMenuAnchors], which provides the anchor points for the default
  ///   context menu.
  /// * [AdaptiveTextSelectionToolbar], which builds the toolbar itself, and can
  ///   take a list of [ContextMenuButtonItem]s with
  ///   [AdaptiveTextSelectionToolbar.buttonItems].
  /// * [AdaptiveTextSelectionToolbar.getAdaptiveButtons], which builds the
  ///   button Widgets for the current platform given [ContextMenuButtonItem]s.
  List<ContextMenuButtonItem> get contextMenuButtonItems {
    return SelectableRegion.getSelectableButtonItems(
      selectionGeometry: _selectionDelegate.value,
      onCopy: () {
        _copy();

        // On Android copy should clear the selection.
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
            clearSelection();
            _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
            _finalizeSelectableRegionStatus();
          case TargetPlatform.iOS:
            hideToolbar(false);
          case TargetPlatform.linux:
          case TargetPlatform.macOS:
          case TargetPlatform.windows:
            hideToolbar();
        }
      },
      onSelectAll: () {
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.iOS:
          case TargetPlatform.fuchsia:
            selectAll(SelectionChangedCause.toolbar);
          case TargetPlatform.linux:
          case TargetPlatform.macOS:
          case TargetPlatform.windows:
            selectAll();
            hideToolbar();
        }
      },
      onShare: () {
        _share();

        // On Android, share should clear the selection.
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
            clearSelection();
            _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
            _finalizeSelectableRegionStatus();
          case TargetPlatform.iOS:
            hideToolbar(false);
          case TargetPlatform.linux:
          case TargetPlatform.macOS:
          case TargetPlatform.windows:
            hideToolbar();
        }
      },
    )..addAll(_textProcessingActionButtonItems);
  }

  List<ContextMenuButtonItem> get _textProcessingActionButtonItems {
    final List<ContextMenuButtonItem> buttonItems = <ContextMenuButtonItem>[];
    final SelectedContent? data = _selectable?.getSelectedContent();
    if (data == null) {
      return buttonItems;
    }

    for (final ProcessTextAction action in _processTextActions) {
      buttonItems.add(
        ContextMenuButtonItem(
          label: action.label,
          onPressed: () async {
            final String selectedText = data.plainText;
            if (selectedText.isNotEmpty) {
              await _processTextService.processTextAction(action.id, selectedText, true);
              hideToolbar();
            }
          },
        ),
      );
    }
    return buttonItems;
  }

  /// The line height at the start of the current selection.
  double get startGlyphHeight {
    return _selectionDelegate.value.startSelectionPoint!.lineHeight;
  }

  /// The line height at the end of the current selection.
  double get endGlyphHeight {
    return _selectionDelegate.value.endSelectionPoint!.lineHeight;
  }

  /// Returns the local coordinates of the endpoints of the current selection.
  List<TextSelectionPoint> get selectionEndpoints {
    final SelectionPoint? start = _selectionDelegate.value.startSelectionPoint;
    final SelectionPoint? end = _selectionDelegate.value.endSelectionPoint;
    late List<TextSelectionPoint> points;
    final Offset startLocalPosition = start?.localPosition ?? end!.localPosition;
    final Offset endLocalPosition = end?.localPosition ?? start!.localPosition;
    if (startLocalPosition.dy > endLocalPosition.dy) {
      points = <TextSelectionPoint>[
        TextSelectionPoint(endLocalPosition, TextDirection.ltr),
        TextSelectionPoint(startLocalPosition, TextDirection.ltr),
      ];
    } else {
      points = <TextSelectionPoint>[
        TextSelectionPoint(startLocalPosition, TextDirection.ltr),
        TextSelectionPoint(endLocalPosition, TextDirection.ltr),
      ];
    }
    return points;
  }

  // [TextSelectionDelegate] overrides.
  // TODO(justinmc): After deprecations have been removed, remove
  // TextSelectionDelegate from this class.
  // https://github.com/flutter/flutter/issues/111213

  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  @override
  bool get cutEnabled => false;

  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  @override
  bool get pasteEnabled => false;

  @override
  void hideToolbar([bool hideHandles = true]) {
    _selectionOverlay?.hideToolbar();
    if (hideHandles) {
      _selectionOverlay?.hideHandles();
    }
  }

  @override
  void selectAll([SelectionChangedCause? cause]) {
    clearSelection();
    _selectable?.dispatchSelectionEvent(const SelectAllSelectionEvent());
    if (cause == SelectionChangedCause.toolbar) {
      _showToolbar();
      _showHandles();
    }
    _updateSelectedContentIfNeeded();
    _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
    _finalizeSelectableRegionStatus();
  }

  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  @override
  void copySelection(SelectionChangedCause cause) {
    _copy();
    clearSelection();
    _selectionStatusNotifier.value = SelectableRegionSelectionStatus.changing;
    _finalizeSelectableRegionStatus();
  }

  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  @override
  TextEditingValue textEditingValue = const TextEditingValue(text: '_');

  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  @override
  void bringIntoView(TextPosition position) {
    /* SelectableRegion must be in view at this point. */
  }

  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  @override
  void cutSelection(SelectionChangedCause cause) {
    assert(false);
  }

  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  @override
  void userUpdateTextEditingValue(TextEditingValue value, SelectionChangedCause cause) {
    /* SelectableRegion maintains its own state */
  }

  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  @override
  Future<void> pasteText(SelectionChangedCause cause) async {
    assert(false);
  }

  // [SelectionRegistrar] override.

  @override
  void add(Selectable selectable) {
    assert(_selectable == null);
    _selectable = selectable;
    _selectable!.addListener(_updateSelectionStatus);
    _selectable!.pushHandleLayers(_startHandleLayerLink, _endHandleLayerLink);
  }

  @override
  void remove(Selectable selectable) {
    assert(_selectable == selectable);
    _selectable!.removeListener(_updateSelectionStatus);
    _selectable!.pushHandleLayers(null, null);
    _selectable = null;
  }

  @protected
  @override
  void dispose() {
    _selectable?.removeListener(_updateSelectionStatus);
    _selectable?.pushHandleLayers(null, null);
    _selectionDelegate.dispose();
    _selectionStatusNotifier.dispose();
    // In case dispose was triggered before gesture end, remove the magnifier
    // so it doesn't remain stuck in the overlay forever.
    _selectionOverlay?.hideMagnifier();
    _selectionOverlay?.dispose();
    _selectionOverlay = null;
    widget.focusNode?.removeListener(_handleFocusChanged);
    _localFocusNode?.removeListener(_handleFocusChanged);
    _localFocusNode?.dispose();
    super.dispose();
  }

  @protected
  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasOverlay(context));
    Widget result = SelectableRegionSelectionStatusScope._(
      selectionStatusNotifier: _selectionStatusNotifier,
      child: SelectionContainer(registrar: this, delegate: _selectionDelegate, child: widget.child),
    );
    if (kIsWeb) {
      result = PlatformSelectableRegionContextMenu(child: result);
    }
    return CompositedTransformTarget(
      link: _toolbarLayerLink,
      child: RawGestureDetector(
        gestures: _gestureRecognizers,
        behavior: HitTestBehavior.translucent,
        excludeFromSemantics: true,
        child: Actions(
          actions: _actions,
          child: Focus.withExternalFocusNode(
            includeSemantics: false,
            focusNode: _focusNode,
            child: result,
          ),
        ),
      ),
    );
  }
}

/// An action that does not override any [Action.overridable] in the subtree.
///
/// If this action is invoked by an [Action.overridable], it will immediately
/// invoke the [Action.overridable] and do nothing else. Otherwise, it will call
/// [invokeAction].
abstract class _NonOverrideAction<T extends Intent> extends ContextAction<T> {
  Object? invokeAction(T intent, [BuildContext? context]);

  @override
  Object? invoke(T intent, [BuildContext? context]) {
    if (callingAction != null) {
      return callingAction!.invoke(intent);
    }
    return invokeAction(intent, context);
  }
}

class _SelectAllAction extends _NonOverrideAction<SelectAllTextIntent> {
  _SelectAllAction(this.state);

  final SelectableRegionState state;

  @override
  void invokeAction(SelectAllTextIntent intent, [BuildContext? context]) {
    state.selectAll(SelectionChangedCause.keyboard);
  }
}

class _CopySelectionAction extends _NonOverrideAction<CopySelectionTextIntent> {
  _CopySelectionAction(this.state);

  final SelectableRegionState state;

  @override
  void invokeAction(CopySelectionTextIntent intent, [BuildContext? context]) {
    state._copy();
  }
}

class _GranularlyExtendSelectionAction<T extends DirectionalTextEditingIntent>
    extends _NonOverrideAction<T> {
  _GranularlyExtendSelectionAction(this.state, {required this.granularity});

  final SelectableRegionState state;
  final TextGranularity granularity;

  @override
  void invokeAction(T intent, [BuildContext? context]) {
    state._granularlyExtendSelection(granularity, intent.forward);
  }
}

class _GranularlyExtendCaretSelectionAction<T extends DirectionalCaretMovementIntent>
    extends _NonOverrideAction<T> {
  _GranularlyExtendCaretSelectionAction(this.state, {required this.granularity});

  final SelectableRegionState state;
  final TextGranularity granularity;

  @override
  void invokeAction(T intent, [BuildContext? context]) {
    if (intent.collapseSelection) {
      // Selectable region never collapses selection.
      return;
    }
    state._granularlyExtendSelection(granularity, intent.forward);
  }
}

class _DirectionallyExtendCaretSelectionAction<T extends DirectionalCaretMovementIntent>
    extends _NonOverrideAction<T> {
  _DirectionallyExtendCaretSelectionAction(this.state);

  final SelectableRegionState state;

  @override
  void invokeAction(T intent, [BuildContext? context]) {
    if (intent.collapseSelection) {
      // Selectable region never collapses selection.
      return;
    }
    state._directionallyExtendSelection(intent.forward);
  }
}

/// A delegate that manages updating multiple [Selectable] children where the
/// [Selectable]s do not change or move around frequently.
///
/// This delegate keeps track of the [Selectable]s that received start or end
/// [SelectionEvent]s and the global locations of those events to accurately
/// synthesize [SelectionEvent]s for children [Selectable]s when needed.
///
/// When a new [SelectionEdgeUpdateEvent] is dispatched to a [Selectable], this
/// delegate checks whether the [Selectable] has already received a selection
/// update for each edge that currently exists, and synthesizes an event for the
/// edges that have not yet received an update. This synthesized event is dispatched
/// before dispatching the new event.
///
/// For example, if we have an existing start edge for this delegate and a [Selectable]
/// child receives an end [SelectionEdgeUpdateEvent] and the child hasn't received a start
/// [SelectionEdgeUpdateEvent], we synthesize a start [SelectionEdgeUpdateEvent] for the
/// child [Selectable] and dispatch it before dispatching the original end [SelectionEdgeUpdateEvent].
///
/// See also:
///
///  * [MultiSelectableSelectionContainerDelegate], for the class that provides
///  the main implementation details of this [SelectionContainerDelegate].
class StaticSelectionContainerDelegate extends MultiSelectableSelectionContainerDelegate {
  /// The set of [Selectable]s that have received start events.
  final Set<Selectable> _hasReceivedStartEvent = <Selectable>{};

  /// The set of [Selectable]s that have received end events.
  final Set<Selectable> _hasReceivedEndEvent = <Selectable>{};

  /// The global position of the last selection start edge update.
  Offset? _lastStartEdgeUpdateGlobalPosition;

  /// The global position of the last selection end edge update.
  Offset? _lastEndEdgeUpdateGlobalPosition;

  /// Tracks whether a selection edge update event for a given [Selectable] was received.
  ///
  /// When `forEnd` is true, the [Selectable] will be registered as having received
  /// an end event. When false, the [Selectable] is registered as having received
  /// a start event.
  ///
  /// When `forEnd` is null, the [Selectable] will be registered as having received both
  /// start and end events.
  ///
  /// Call this method when a [SelectionEvent] is dispatched to a child selectable managed
  /// by this delegate.
  ///
  /// Subclasses should call [clearInternalSelectionStateForSelectable] to clean up any state
  /// added by this method, for example when removing a [Selectable] from this delegate.
  @protected
  void didReceiveSelectionEventFor({required Selectable selectable, bool? forEnd}) {
    switch (forEnd) {
      case true:
        _hasReceivedEndEvent.add(selectable);
      case false:
        _hasReceivedStartEvent.add(selectable);
      case null:
        _hasReceivedStartEvent.add(selectable);
        _hasReceivedEndEvent.add(selectable);
    }
  }

  /// Updates the internal selection state after a [SelectionEvent] that
  /// selects a boundary such as: [SelectWordSelectionEvent],
  /// [SelectParagraphSelectionEvent], and [SelectAllSelectionEvent].
  ///
  /// Call this method after determining the new selection as a result of
  /// a [SelectionEvent] that selects a boundary. The [currentSelectionStartIndex]
  /// and [currentSelectionEndIndex] should be set to valid values at the time
  /// this method is called.
  ///
  /// Subclasses should call [clearInternalSelectionStateForSelectable] to clean up any state
  /// added by this method, for example when removing a [Selectable] from this delegate.
  @protected
  void didReceiveSelectionBoundaryEvents() {
    if (currentSelectionStartIndex == -1 || currentSelectionEndIndex == -1) {
      return;
    }
    final int start = min(currentSelectionStartIndex, currentSelectionEndIndex);
    final int end = max(currentSelectionStartIndex, currentSelectionEndIndex);
    for (int index = start; index <= end; index += 1) {
      didReceiveSelectionEventFor(selectable: selectables[index]);
    }
    _updateLastSelectionEdgeLocationsFromGeometries();
  }

  /// Updates the last selection edge location of the edge specified by `forEnd`
  /// to the provided `globalSelectionEdgeLocation`.
  @protected
  void updateLastSelectionEdgeLocation({
    required Offset globalSelectionEdgeLocation,
    required bool forEnd,
  }) {
    if (forEnd) {
      _lastEndEdgeUpdateGlobalPosition = globalSelectionEdgeLocation;
    } else {
      _lastStartEdgeUpdateGlobalPosition = globalSelectionEdgeLocation;
    }
  }

  /// Updates the last selection edge locations of both start and end selection
  /// edges based on their [SelectionGeometry].
  void _updateLastSelectionEdgeLocationsFromGeometries() {
    if (currentSelectionStartIndex != -1 &&
        selectables[currentSelectionStartIndex].value.hasSelection) {
      final Selectable start = selectables[currentSelectionStartIndex];
      final Offset localStartEdge =
          start.value.startSelectionPoint!.localPosition +
          Offset(0, -start.value.startSelectionPoint!.lineHeight / 2);
      updateLastSelectionEdgeLocation(
        globalSelectionEdgeLocation: MatrixUtils.transformPoint(
          start.getTransformTo(null),
          localStartEdge,
        ),
        forEnd: false,
      );
    }
    if (currentSelectionEndIndex != -1 &&
        selectables[currentSelectionEndIndex].value.hasSelection) {
      final Selectable end = selectables[currentSelectionEndIndex];
      final Offset localEndEdge =
          end.value.endSelectionPoint!.localPosition +
          Offset(0, -end.value.endSelectionPoint!.lineHeight / 2);
      updateLastSelectionEdgeLocation(
        globalSelectionEdgeLocation: MatrixUtils.transformPoint(
          end.getTransformTo(null),
          localEndEdge,
        ),
        forEnd: true,
      );
    }
  }

  /// Clears the internal selection state.
  ///
  /// This indicates that no [Selectable] child under this delegate
  /// has received start or end events, and resets any tracked global
  /// locations for start and end [SelectionEdgeUpdateEvent]s.
  @protected
  void clearInternalSelectionState() {
    selectables.forEach(clearInternalSelectionStateForSelectable);
    _lastStartEdgeUpdateGlobalPosition = null;
    _lastEndEdgeUpdateGlobalPosition = null;
  }

  /// Clears the internal selection state for a given [Selectable].
  ///
  /// This indicates that the given `selectable` has neither received a
  /// start or end [SelectionEdgeUpdateEvent]s.
  ///
  /// Subclasses should call this method to clean up state added in
  /// [didReceiveSelectionEventFor] and [didReceiveSelectionBoundaryEvents].
  @protected
  void clearInternalSelectionStateForSelectable(Selectable selectable) {
    _hasReceivedStartEvent.remove(selectable);
    _hasReceivedEndEvent.remove(selectable);
  }

  @override
  void remove(Selectable selectable) {
    clearInternalSelectionStateForSelectable(selectable);
    super.remove(selectable);
  }

  @override
  SelectionResult handleSelectAll(SelectAllSelectionEvent event) {
    final SelectionResult result = super.handleSelectAll(event);
    didReceiveSelectionBoundaryEvents();
    return result;
  }

  @override
  SelectionResult handleSelectWord(SelectWordSelectionEvent event) {
    final SelectionResult result = super.handleSelectWord(event);
    didReceiveSelectionBoundaryEvents();
    return result;
  }

  @override
  SelectionResult handleSelectParagraph(SelectParagraphSelectionEvent event) {
    final SelectionResult result = super.handleSelectParagraph(event);
    didReceiveSelectionBoundaryEvents();
    return result;
  }

  @override
  SelectionResult handleClearSelection(ClearSelectionEvent event) {
    final SelectionResult result = super.handleClearSelection(event);
    clearInternalSelectionState();
    return result;
  }

  @override
  SelectionResult handleSelectionEdgeUpdate(SelectionEdgeUpdateEvent event) {
    updateLastSelectionEdgeLocation(
      globalSelectionEdgeLocation: event.globalPosition,
      forEnd: event.type == SelectionEventType.endEdgeUpdate,
    );
    return super.handleSelectionEdgeUpdate(event);
  }

  @override
  void dispose() {
    clearInternalSelectionState();
    super.dispose();
  }

  @override
  SelectionResult dispatchSelectionEventToChild(Selectable selectable, SelectionEvent event) {
    switch (event.type) {
      case SelectionEventType.startEdgeUpdate:
        didReceiveSelectionEventFor(selectable: selectable, forEnd: false);
        ensureChildUpdated(selectable);
      case SelectionEventType.endEdgeUpdate:
        didReceiveSelectionEventFor(selectable: selectable, forEnd: true);
        ensureChildUpdated(selectable);
      case SelectionEventType.clear:
        clearInternalSelectionStateForSelectable(selectable);
      case SelectionEventType.selectAll:
      case SelectionEventType.selectWord:
      case SelectionEventType.selectParagraph:
        break;
      case SelectionEventType.granularlyExtendSelection:
      case SelectionEventType.directionallyExtendSelection:
        didReceiveSelectionEventFor(selectable: selectable);
        ensureChildUpdated(selectable);
    }
    return super.dispatchSelectionEventToChild(selectable, event);
  }

  /// Ensures the `selectable` child has received the most up to date selection events.
  ///
  /// This method is called when:
  ///   1. A new [Selectable] is added to the delegate, and its screen location
  ///   falls into the previous selection.
  ///   2. Before a [SelectionEvent] of type
  ///   [SelectionEventType.startEdgeUpdate], [SelectionEventType.endEdgeUpdate],
  ///   [SelectionEventType.granularlyExtendSelection], or
  ///   [SelectionEventType.directionallyExtendSelection] is dispatched
  ///   to a [Selectable] child.
  @override
  void ensureChildUpdated(Selectable selectable) {
    if (_lastEndEdgeUpdateGlobalPosition != null && _hasReceivedEndEvent.add(selectable)) {
      final SelectionEdgeUpdateEvent synthesizedEvent = SelectionEdgeUpdateEvent.forEnd(
        globalPosition: _lastEndEdgeUpdateGlobalPosition!,
      );
      if (currentSelectionEndIndex == -1) {
        handleSelectionEdgeUpdate(synthesizedEvent);
      }
      selectable.dispatchSelectionEvent(synthesizedEvent);
    }
    if (_lastStartEdgeUpdateGlobalPosition != null && _hasReceivedStartEvent.add(selectable)) {
      final SelectionEdgeUpdateEvent synthesizedEvent = SelectionEdgeUpdateEvent.forStart(
        globalPosition: _lastStartEdgeUpdateGlobalPosition!,
      );
      if (currentSelectionStartIndex == -1) {
        handleSelectionEdgeUpdate(synthesizedEvent);
      }
      selectable.dispatchSelectionEvent(synthesizedEvent);
    }
  }

  @override
  void didChangeSelectables() {
    if (_lastEndEdgeUpdateGlobalPosition != null) {
      handleSelectionEdgeUpdate(
        SelectionEdgeUpdateEvent.forEnd(globalPosition: _lastEndEdgeUpdateGlobalPosition!),
      );
    }
    if (_lastStartEdgeUpdateGlobalPosition != null) {
      handleSelectionEdgeUpdate(
        SelectionEdgeUpdateEvent.forStart(globalPosition: _lastStartEdgeUpdateGlobalPosition!),
      );
    }
    final Set<Selectable> selectableSet = selectables.toSet();
    _hasReceivedEndEvent.removeWhere(
      (Selectable selectable) => !selectableSet.contains(selectable),
    );
    _hasReceivedStartEvent.removeWhere(
      (Selectable selectable) => !selectableSet.contains(selectable),
    );
    super.didChangeSelectables();
  }
}

/// A delegate that handles events and updates for multiple [Selectable]
/// children.
///
/// Updates are optimized by tracking which [Selectable]s reside on the edges of
/// a selection. Subclasses should implement [ensureChildUpdated] to describe
/// how a [Selectable] should behave when added to a selection.
abstract class MultiSelectableSelectionContainerDelegate extends SelectionContainerDelegate
    with ChangeNotifier {
  /// Creates an instance of [MultiSelectableSelectionContainerDelegate].
  MultiSelectableSelectionContainerDelegate() {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  /// Gets the list of [Selectable]s this delegate is managing.
  List<Selectable> selectables = <Selectable>[];

  /// The number of additional pixels added to the selection handle drawable
  /// area.
  ///
  /// Selection handles that are outside of the drawable area will be hidden.
  /// That logic prevents handles that get scrolled off the viewport from being
  /// drawn on the screen.
  ///
  /// The drawable area = current rectangle of [SelectionContainer] +
  /// _kSelectionHandleDrawableAreaPadding on each side.
  ///
  /// This was an eyeballed value to create smooth user experiences.
  static const double _kSelectionHandleDrawableAreaPadding = 5.0;

  /// The current [Selectable] that contains the selection end edge.
  @protected
  int currentSelectionEndIndex = -1;

  /// The current [Selectable] that contains the selection start edge.
  @protected
  int currentSelectionStartIndex = -1;

  LayerLink? _startHandleLayer;
  Selectable? _startHandleLayerOwner;
  LayerLink? _endHandleLayer;
  Selectable? _endHandleLayerOwner;

  bool _isHandlingSelectionEvent = false;
  bool _scheduledSelectableUpdate = false;
  bool _selectionInProgress = false;
  Set<Selectable> _additions = <Selectable>{};

  bool _extendSelectionInProgress = false;

  @override
  void add(Selectable selectable) {
    assert(!selectables.contains(selectable));
    _additions.add(selectable);
    _scheduleSelectableUpdate();
  }

  @override
  void remove(Selectable selectable) {
    if (_additions.remove(selectable)) {
      // The same selectable was added in the same frame and is not yet
      // incorporated into the selectables.
      //
      // Removing such selectable doesn't require selection geometry update.
      return;
    }
    _removeSelectable(selectable);
    _scheduleSelectableUpdate();
  }

  /// Notifies this delegate that layout of the container has changed.
  void layoutDidChange() {
    _updateSelectionGeometry();
  }

  void _scheduleSelectableUpdate() {
    if (!_scheduledSelectableUpdate) {
      _scheduledSelectableUpdate = true;
      void runScheduledTask([Duration? duration]) {
        if (!_scheduledSelectableUpdate) {
          return;
        }
        _scheduledSelectableUpdate = false;
        _updateSelectables();
      }

      if (SchedulerBinding.instance.schedulerPhase == SchedulerPhase.postFrameCallbacks) {
        // A new task can be scheduled as a result of running the scheduled task
        // from another MultiSelectableSelectionContainerDelegate. This can
        // happen if nesting two SelectionContainers. The selectable can be
        // safely updated in the same frame in this case.
        scheduleMicrotask(runScheduledTask);
      } else {
        SchedulerBinding.instance.addPostFrameCallback(
          runScheduledTask,
          debugLabel: 'SelectionContainer.runScheduledTask',
        );
      }
    }
  }

  void _updateSelectables() {
    // Remove offScreen selectable.
    if (_additions.isNotEmpty) {
      _flushAdditions();
    }
    didChangeSelectables();
  }

  void _flushAdditions() {
    final List<Selectable> mergingSelectables = _additions.toList()..sort(compareOrder);
    final List<Selectable> existingSelectables = selectables;
    selectables = <Selectable>[];
    int mergingIndex = 0;
    int existingIndex = 0;
    int selectionStartIndex = currentSelectionStartIndex;
    int selectionEndIndex = currentSelectionEndIndex;
    // Merge two sorted lists.
    while (mergingIndex < mergingSelectables.length || existingIndex < existingSelectables.length) {
      if (mergingIndex >= mergingSelectables.length ||
          (existingIndex < existingSelectables.length &&
              compareOrder(existingSelectables[existingIndex], mergingSelectables[mergingIndex]) <
                  0)) {
        if (existingIndex == currentSelectionStartIndex) {
          selectionStartIndex = selectables.length;
        }
        if (existingIndex == currentSelectionEndIndex) {
          selectionEndIndex = selectables.length;
        }
        selectables.add(existingSelectables[existingIndex]);
        existingIndex += 1;
        continue;
      }

      // If the merging selectable falls in the selection range, their selection
      // needs to be updated.
      final Selectable mergingSelectable = mergingSelectables[mergingIndex];
      if (existingIndex < max(currentSelectionStartIndex, currentSelectionEndIndex) &&
          existingIndex > min(currentSelectionStartIndex, currentSelectionEndIndex)) {
        ensureChildUpdated(mergingSelectable);
      }
      mergingSelectable.addListener(_handleSelectableGeometryChange);
      selectables.add(mergingSelectable);
      mergingIndex += 1;
    }
    assert(
      mergingIndex == mergingSelectables.length &&
          existingIndex == existingSelectables.length &&
          selectables.length == existingIndex + mergingIndex,
    );
    assert(selectionStartIndex >= -1 || selectionStartIndex < selectables.length);
    assert(selectionEndIndex >= -1 || selectionEndIndex < selectables.length);
    // selection indices should not be set to -1 unless they originally were.
    assert((currentSelectionStartIndex == -1) == (selectionStartIndex == -1));
    assert((currentSelectionEndIndex == -1) == (selectionEndIndex == -1));
    currentSelectionEndIndex = selectionEndIndex;
    currentSelectionStartIndex = selectionStartIndex;
    _additions = <Selectable>{};
  }

  void _removeSelectable(Selectable selectable) {
    assert(selectables.contains(selectable), 'The selectable is not in this registrar.');
    final int index = selectables.indexOf(selectable);
    selectables.removeAt(index);
    if (index <= currentSelectionEndIndex) {
      currentSelectionEndIndex -= 1;
    }
    if (index <= currentSelectionStartIndex) {
      currentSelectionStartIndex -= 1;
    }
    selectable.removeListener(_handleSelectableGeometryChange);
  }

  /// Called when this delegate finishes updating the [Selectable]s.
  @protected
  @mustCallSuper
  void didChangeSelectables() {
    _updateSelectionGeometry();
  }

  @override
  SelectionGeometry get value => _selectionGeometry;
  SelectionGeometry _selectionGeometry = const SelectionGeometry(
    hasContent: false,
    status: SelectionStatus.none,
  );

  /// Updates the [value] in this class and notifies listeners if necessary.
  void _updateSelectionGeometry() {
    final SelectionGeometry newValue = getSelectionGeometry();
    if (_selectionGeometry != newValue) {
      _selectionGeometry = newValue;
      notifyListeners();
    }
    _updateHandleLayersAndOwners();
  }

  static Rect _getBoundingBox(Selectable selectable) {
    Rect result = selectable.boundingBoxes.first;
    for (int index = 1; index < selectable.boundingBoxes.length; index += 1) {
      result = result.expandToInclude(selectable.boundingBoxes[index]);
    }
    return result;
  }

  /// The compare function this delegate used for determining the selection
  /// order of the selectables.
  ///
  /// Defaults to screen order.
  @protected
  Comparator<Selectable> get compareOrder => _compareScreenOrder;

  static int _compareScreenOrder(Selectable a, Selectable b) {
    final Rect rectA = MatrixUtils.transformRect(a.getTransformTo(null), _getBoundingBox(a));
    final Rect rectB = MatrixUtils.transformRect(b.getTransformTo(null), _getBoundingBox(b));
    final int result = _compareVertically(rectA, rectB);
    if (result != 0) {
      return result;
    }
    return _compareHorizontally(rectA, rectB);
  }

  /// Compares two rectangles in the screen order solely by their vertical
  /// positions.
  ///
  /// Returns positive if a is lower, negative if a is higher, 0 if their
  /// order can't be determine solely by their vertical position.
  static int _compareVertically(Rect a, Rect b) {
    // The rectangles overlap so defer to horizontal comparison.
    if ((a.top - b.top < _kSelectableVerticalComparingThreshold &&
            a.bottom - b.bottom > -_kSelectableVerticalComparingThreshold) ||
        (b.top - a.top < _kSelectableVerticalComparingThreshold &&
            b.bottom - a.bottom > -_kSelectableVerticalComparingThreshold)) {
      return 0;
    }
    if ((a.top - b.top).abs() > _kSelectableVerticalComparingThreshold) {
      return a.top > b.top ? 1 : -1;
    }
    return a.bottom > b.bottom ? 1 : -1;
  }

  /// Compares two rectangles in the screen order by their horizontal positions
  /// assuming one of the rectangles enclose the other rect vertically.
  ///
  /// Returns positive if a is lower, negative if a is higher.
  static int _compareHorizontally(Rect a, Rect b) {
    // a encloses b.
    if (a.left - b.left < precisionErrorTolerance && a.right - b.right > -precisionErrorTolerance) {
      return -1;
    }
    // b encloses a.
    if (b.left - a.left < precisionErrorTolerance && b.right - a.right > -precisionErrorTolerance) {
      return 1;
    }
    if ((a.left - b.left).abs() > precisionErrorTolerance) {
      return a.left > b.left ? 1 : -1;
    }
    return a.right > b.right ? 1 : -1;
  }

  void _handleSelectableGeometryChange() {
    // Geometries of selectable children may change multiple times when handling
    // selection events. Ignore these updates since the selection geometry of
    // this delegate will be updated after handling the selection events.
    if (_isHandlingSelectionEvent) {
      return;
    }
    _updateSelectionGeometry();
  }

  /// Gets the combined [SelectionGeometry] for child [Selectable]s.
  @protected
  SelectionGeometry getSelectionGeometry() {
    if (currentSelectionEndIndex == -1 || currentSelectionStartIndex == -1 || selectables.isEmpty) {
      // There is no valid selection.
      return SelectionGeometry(status: SelectionStatus.none, hasContent: selectables.isNotEmpty);
    }

    if (!_extendSelectionInProgress) {
      currentSelectionStartIndex = _adjustSelectionIndexBasedOnSelectionGeometry(
        currentSelectionStartIndex,
        currentSelectionEndIndex,
      );
      currentSelectionEndIndex = _adjustSelectionIndexBasedOnSelectionGeometry(
        currentSelectionEndIndex,
        currentSelectionStartIndex,
      );
    }

    // Need to find the non-null start selection point.
    SelectionGeometry startGeometry = selectables[currentSelectionStartIndex].value;
    final bool forwardSelection = currentSelectionEndIndex >= currentSelectionStartIndex;
    int startIndexWalker = currentSelectionStartIndex;
    while (startIndexWalker != currentSelectionEndIndex &&
        startGeometry.startSelectionPoint == null) {
      startIndexWalker += forwardSelection ? 1 : -1;
      startGeometry = selectables[startIndexWalker].value;
    }

    SelectionPoint? startPoint;
    if (startGeometry.startSelectionPoint != null) {
      final Matrix4 startTransform = getTransformFrom(selectables[startIndexWalker]);
      final Offset start = MatrixUtils.transformPoint(
        startTransform,
        startGeometry.startSelectionPoint!.localPosition,
      );
      // It can be NaN if it is detached or off-screen.
      if (start.isFinite) {
        startPoint = SelectionPoint(
          localPosition: start,
          lineHeight: startGeometry.startSelectionPoint!.lineHeight,
          handleType: startGeometry.startSelectionPoint!.handleType,
        );
      }
    }

    // Need to find the non-null end selection point.
    SelectionGeometry endGeometry = selectables[currentSelectionEndIndex].value;
    int endIndexWalker = currentSelectionEndIndex;
    while (endIndexWalker != currentSelectionStartIndex && endGeometry.endSelectionPoint == null) {
      endIndexWalker += forwardSelection ? -1 : 1;
      endGeometry = selectables[endIndexWalker].value;
    }
    SelectionPoint? endPoint;
    if (endGeometry.endSelectionPoint != null) {
      final Matrix4 endTransform = getTransformFrom(selectables[endIndexWalker]);
      final Offset end = MatrixUtils.transformPoint(
        endTransform,
        endGeometry.endSelectionPoint!.localPosition,
      );
      // It can be NaN if it is detached or off-screen.
      if (end.isFinite) {
        endPoint = SelectionPoint(
          localPosition: end,
          lineHeight: endGeometry.endSelectionPoint!.lineHeight,
          handleType: endGeometry.endSelectionPoint!.handleType,
        );
      }
    }

    // Need to collect selection rects from selectables ranging from the
    // currentSelectionStartIndex to the currentSelectionEndIndex.
    final List<Rect> selectionRects = <Rect>[];
    final Rect? drawableArea =
        hasSize ? Rect.fromLTWH(0, 0, containerSize.width, containerSize.height) : null;
    for (int index = currentSelectionStartIndex; index <= currentSelectionEndIndex; index++) {
      final List<Rect> currSelectableSelectionRects = selectables[index].value.selectionRects;
      final List<Rect> selectionRectsWithinDrawableArea =
          currSelectableSelectionRects
              .map((Rect selectionRect) {
                final Matrix4 transform = getTransformFrom(selectables[index]);
                final Rect localRect = MatrixUtils.transformRect(transform, selectionRect);
                return drawableArea?.intersect(localRect) ?? localRect;
              })
              .where((Rect selectionRect) {
                return selectionRect.isFinite && !selectionRect.isEmpty;
              })
              .toList();
      selectionRects.addAll(selectionRectsWithinDrawableArea);
    }

    return SelectionGeometry(
      startSelectionPoint: startPoint,
      endSelectionPoint: endPoint,
      selectionRects: selectionRects,
      status: startGeometry != endGeometry ? SelectionStatus.uncollapsed : startGeometry.status,
      // Would have at least one selectable child.
      hasContent: true,
    );
  }

  // The currentSelectionStartIndex or currentSelectionEndIndex may not be
  // the current index that contains selection edges. This can happen if the
  // selection edge is in between two selectables. One of the selectable will
  // have its selection collapsed at the index 0 or contentLength depends on
  // whether the selection is reversed or not. The current selection index can
  // be point to either one.
  //
  // This method adjusts the index to point to selectable with valid selection.
  int _adjustSelectionIndexBasedOnSelectionGeometry(int currentIndex, int towardIndex) {
    final bool forward = towardIndex > currentIndex;
    while (currentIndex != towardIndex &&
        selectables[currentIndex].value.status != SelectionStatus.uncollapsed) {
      currentIndex += forward ? 1 : -1;
    }
    return currentIndex;
  }

  @override
  void pushHandleLayers(LayerLink? startHandle, LayerLink? endHandle) {
    if (_startHandleLayer == startHandle && _endHandleLayer == endHandle) {
      return;
    }
    _startHandleLayer = startHandle;
    _endHandleLayer = endHandle;
    _updateHandleLayersAndOwners();
  }

  /// Pushes both handle layers to the selectables that contain selection edges.
  ///
  /// This method needs to be called every time the selectables that contain the
  /// selection edges change, i.e. [currentSelectionStartIndex] or
  /// [currentSelectionEndIndex] changes. Otherwise, the handle may be painted
  /// in the wrong place.
  void _updateHandleLayersAndOwners() {
    LayerLink? effectiveStartHandle = _startHandleLayer;
    LayerLink? effectiveEndHandle = _endHandleLayer;
    if (effectiveStartHandle != null || effectiveEndHandle != null) {
      final Rect? drawableArea =
          hasSize
              ? Rect.fromLTWH(
                0,
                0,
                containerSize.width,
                containerSize.height,
              ).inflate(_kSelectionHandleDrawableAreaPadding)
              : null;
      final bool hideStartHandle =
          value.startSelectionPoint == null ||
          drawableArea == null ||
          !drawableArea.contains(value.startSelectionPoint!.localPosition);
      final bool hideEndHandle =
          value.endSelectionPoint == null ||
          drawableArea == null ||
          !drawableArea.contains(value.endSelectionPoint!.localPosition);
      effectiveStartHandle = hideStartHandle ? null : _startHandleLayer;
      effectiveEndHandle = hideEndHandle ? null : _endHandleLayer;
    }
    if (currentSelectionStartIndex == -1 || currentSelectionEndIndex == -1) {
      // No valid selection.
      if (_startHandleLayerOwner != null) {
        _startHandleLayerOwner!.pushHandleLayers(null, null);
        _startHandleLayerOwner = null;
      }
      if (_endHandleLayerOwner != null) {
        _endHandleLayerOwner!.pushHandleLayers(null, null);
        _endHandleLayerOwner = null;
      }
      return;
    }

    if (selectables[currentSelectionStartIndex] != _startHandleLayerOwner) {
      _startHandleLayerOwner?.pushHandleLayers(null, null);
    }
    if (selectables[currentSelectionEndIndex] != _endHandleLayerOwner) {
      _endHandleLayerOwner?.pushHandleLayers(null, null);
    }

    _startHandleLayerOwner = selectables[currentSelectionStartIndex];

    if (currentSelectionStartIndex == currentSelectionEndIndex) {
      // Selection edges is on the same selectable.
      _endHandleLayerOwner = _startHandleLayerOwner;
      _startHandleLayerOwner!.pushHandleLayers(effectiveStartHandle, effectiveEndHandle);
      return;
    }

    _startHandleLayerOwner!.pushHandleLayers(effectiveStartHandle, null);
    _endHandleLayerOwner = selectables[currentSelectionEndIndex];
    _endHandleLayerOwner!.pushHandleLayers(null, effectiveEndHandle);
  }

  /// Copies the selected contents of all [Selectable]s.
  @override
  SelectedContent? getSelectedContent() {
    final List<SelectedContent> selections = <SelectedContent>[
      for (final Selectable selectable in selectables)
        if (selectable.getSelectedContent() case final SelectedContent data) data,
    ];
    if (selections.isEmpty) {
      return null;
    }
    final StringBuffer buffer = StringBuffer();
    for (final SelectedContent selection in selections) {
      buffer.write(selection.plainText);
    }
    return SelectedContent(plainText: buffer.toString());
  }

  /// The total length of the content under this [SelectionContainerDelegate].
  ///
  /// This value is derived from the [Selectable.contentLength] of each [Selectable]
  /// managed by this delegate.
  @override
  int get contentLength =>
      selectables.fold<int>(0, (int sum, Selectable selectable) => sum + selectable.contentLength);

  /// This method calculates a local [SelectedContentRange] based on the list
  /// of [selections] that are accumulated from the [Selectable] children under this
  /// delegate. This calculation takes into account the accumulated content
  /// length before the active selection, and returns null when either selection
  /// edge has not been set.
  SelectedContentRange? _calculateLocalRange(List<_SelectionInfo> selections) {
    if (currentSelectionStartIndex == -1 || currentSelectionEndIndex == -1) {
      return null;
    }
    int startOffset = 0;
    int endOffset = 0;
    bool foundStart = false;
    bool forwardSelection = currentSelectionEndIndex >= currentSelectionStartIndex;
    if (currentSelectionEndIndex == currentSelectionStartIndex) {
      // Determining selection direction is innacurate if currentSelectionStartIndex == currentSelectionEndIndex.
      // Use the range from the selectable within the selection as the source of truth for selection direction.
      final SelectedContentRange rangeAtSelectableInSelection =
          selectables[currentSelectionStartIndex].getSelection()!;
      forwardSelection =
          rangeAtSelectableInSelection.endOffset >= rangeAtSelectableInSelection.startOffset;
    }
    for (int index = 0; index < selections.length; index++) {
      final _SelectionInfo selection = selections[index];
      if (selection.range == null) {
        if (foundStart) {
          return SelectedContentRange(
            startOffset: forwardSelection ? startOffset : endOffset,
            endOffset: forwardSelection ? endOffset : startOffset,
          );
        }
        startOffset += selection.contentLength;
        endOffset = startOffset;
        continue;
      }
      final int selectionStartNormalized = min(
        selection.range!.startOffset,
        selection.range!.endOffset,
      );
      final int selectionEndNormalized = max(
        selection.range!.startOffset,
        selection.range!.endOffset,
      );
      if (!foundStart) {
        startOffset += selectionStartNormalized;
        endOffset = startOffset + (selectionEndNormalized - selectionStartNormalized).abs();
        foundStart = true;
      } else {
        endOffset += (selectionEndNormalized - selectionStartNormalized).abs();
      }
    }
    assert(
      foundStart,
      'The start of the selection has not been found despite this selection delegate having an existing currentSelectionStartIndex and currentSelectionEndIndex.',
    );
    return SelectedContentRange(
      startOffset: forwardSelection ? startOffset : endOffset,
      endOffset: forwardSelection ? endOffset : startOffset,
    );
  }

  /// Returns a [SelectedContentRange] considering the [SelectedContentRange]
  /// from each [Selectable] child managed under this delegate.
  ///
  /// When nothing is selected or either selection edge has not been set,
  /// this method will return `null`.
  @override
  SelectedContentRange? getSelection() {
    final List<_SelectionInfo> selections = <_SelectionInfo>[
      for (final Selectable selectable in selectables)
        (contentLength: selectable.contentLength, range: selectable.getSelection()),
    ];
    return _calculateLocalRange(selections);
  }

  // Clears the selection on all selectables not in the range of
  // currentSelectionStartIndex..currentSelectionEndIndex.
  //
  // If one of the edges does not exist, then this method will clear the selection
  // in all selectables except the existing edge.
  //
  // If neither of the edges exist this method immediately returns.
  void _flushInactiveSelections() {
    if (currentSelectionStartIndex == -1 && currentSelectionEndIndex == -1) {
      return;
    }
    if (currentSelectionStartIndex == -1 || currentSelectionEndIndex == -1) {
      final int skipIndex =
          currentSelectionStartIndex == -1 ? currentSelectionEndIndex : currentSelectionStartIndex;
      selectables
          .where((Selectable target) => target != selectables[skipIndex])
          .forEach(
            (Selectable target) =>
                dispatchSelectionEventToChild(target, const ClearSelectionEvent()),
          );
      return;
    }
    final int skipStart = min(currentSelectionStartIndex, currentSelectionEndIndex);
    final int skipEnd = max(currentSelectionStartIndex, currentSelectionEndIndex);
    for (int index = 0; index < selectables.length; index += 1) {
      if (index >= skipStart && index <= skipEnd) {
        continue;
      }
      dispatchSelectionEventToChild(selectables[index], const ClearSelectionEvent());
    }
  }

  /// Selects all contents of all [Selectable]s.
  @protected
  SelectionResult handleSelectAll(SelectAllSelectionEvent event) {
    for (final Selectable selectable in selectables) {
      dispatchSelectionEventToChild(selectable, event);
    }
    currentSelectionStartIndex = 0;
    currentSelectionEndIndex = selectables.length - 1;
    return SelectionResult.none;
  }

  SelectionResult _handleSelectBoundary(SelectionEvent event) {
    assert(
      event is SelectWordSelectionEvent || event is SelectParagraphSelectionEvent,
      'This method should only be given selection events that select text boundaries.',
    );
    late final Offset effectiveGlobalPosition;
    if (event.type == SelectionEventType.selectWord) {
      effectiveGlobalPosition = (event as SelectWordSelectionEvent).globalPosition;
    } else if (event.type == SelectionEventType.selectParagraph) {
      effectiveGlobalPosition = (event as SelectParagraphSelectionEvent).globalPosition;
    }
    SelectionResult? lastSelectionResult;
    for (int index = 0; index < selectables.length; index += 1) {
      bool globalRectsContainPosition = false;
      if (selectables[index].boundingBoxes.isNotEmpty) {
        for (final Rect rect in selectables[index].boundingBoxes) {
          final Rect globalRect = MatrixUtils.transformRect(
            selectables[index].getTransformTo(null),
            rect,
          );
          if (globalRect.contains(effectiveGlobalPosition)) {
            globalRectsContainPosition = true;
            break;
          }
        }
      }
      if (globalRectsContainPosition) {
        final SelectionGeometry existingGeometry = selectables[index].value;
        lastSelectionResult = dispatchSelectionEventToChild(selectables[index], event);
        if (index == selectables.length - 1 && lastSelectionResult == SelectionResult.next) {
          return SelectionResult.next;
        }
        if (lastSelectionResult == SelectionResult.next) {
          continue;
        }
        if (index == 0 && lastSelectionResult == SelectionResult.previous) {
          return SelectionResult.previous;
        }
        if (selectables[index].value != existingGeometry) {
          // Geometry has changed as a result of select word, need to clear the
          // selection of other selectables to keep selection in sync.
          selectables
              .where((Selectable target) => target != selectables[index])
              .forEach(
                (Selectable target) =>
                    dispatchSelectionEventToChild(target, const ClearSelectionEvent()),
              );
          currentSelectionStartIndex = currentSelectionEndIndex = index;
        }
        return SelectionResult.end;
      } else {
        if (lastSelectionResult == SelectionResult.next) {
          currentSelectionStartIndex = currentSelectionEndIndex = index - 1;
          return SelectionResult.end;
        }
      }
    }
    assert(lastSelectionResult == null);
    return SelectionResult.end;
  }

  /// Selects a word in a [Selectable] at the location
  /// [SelectWordSelectionEvent.globalPosition].
  @protected
  SelectionResult handleSelectWord(SelectWordSelectionEvent event) {
    return _handleSelectBoundary(event);
  }

  /// Selects a paragraph in a [Selectable] at the location
  /// [SelectParagraphSelectionEvent.globalPosition].
  @protected
  SelectionResult handleSelectParagraph(SelectParagraphSelectionEvent event) {
    return _handleSelectBoundary(event);
  }

  /// Removes the selection of all [Selectable]s this delegate manages.
  @protected
  SelectionResult handleClearSelection(ClearSelectionEvent event) {
    for (final Selectable selectable in selectables) {
      dispatchSelectionEventToChild(selectable, event);
    }
    currentSelectionEndIndex = -1;
    currentSelectionStartIndex = -1;
    return SelectionResult.none;
  }

  /// Extend current selection in a certain [TextGranularity].
  @protected
  SelectionResult handleGranularlyExtendSelection(GranularlyExtendSelectionEvent event) {
    assert((currentSelectionStartIndex == -1) == (currentSelectionEndIndex == -1));
    if (currentSelectionStartIndex == -1) {
      if (event.forward) {
        currentSelectionStartIndex = currentSelectionEndIndex = 0;
      } else {
        currentSelectionStartIndex = currentSelectionEndIndex = selectables.length - 1;
      }
    }
    int targetIndex = event.isEnd ? currentSelectionEndIndex : currentSelectionStartIndex;
    SelectionResult result = dispatchSelectionEventToChild(selectables[targetIndex], event);
    if (event.forward) {
      assert(result != SelectionResult.previous);
      while (targetIndex < selectables.length - 1 && result == SelectionResult.next) {
        targetIndex += 1;
        result = dispatchSelectionEventToChild(selectables[targetIndex], event);
        assert(result != SelectionResult.previous);
      }
    } else {
      assert(result != SelectionResult.next);
      while (targetIndex > 0 && result == SelectionResult.previous) {
        targetIndex -= 1;
        result = dispatchSelectionEventToChild(selectables[targetIndex], event);
        assert(result != SelectionResult.next);
      }
    }
    if (event.isEnd) {
      currentSelectionEndIndex = targetIndex;
    } else {
      currentSelectionStartIndex = targetIndex;
    }
    return result;
  }

  /// Extend current selection in a certain [TextGranularity].
  @protected
  SelectionResult handleDirectionallyExtendSelection(DirectionallyExtendSelectionEvent event) {
    assert((currentSelectionStartIndex == -1) == (currentSelectionEndIndex == -1));
    if (currentSelectionStartIndex == -1) {
      currentSelectionStartIndex =
          currentSelectionEndIndex = switch (event.direction) {
            SelectionExtendDirection.previousLine ||
            SelectionExtendDirection.backward => selectables.length - 1,
            SelectionExtendDirection.nextLine || SelectionExtendDirection.forward => 0,
          };
    }
    int targetIndex = event.isEnd ? currentSelectionEndIndex : currentSelectionStartIndex;
    SelectionResult result = dispatchSelectionEventToChild(selectables[targetIndex], event);
    switch (event.direction) {
      case SelectionExtendDirection.previousLine:
        assert(result == SelectionResult.end || result == SelectionResult.previous);
        if (result == SelectionResult.previous) {
          if (targetIndex > 0) {
            targetIndex -= 1;
            result = dispatchSelectionEventToChild(
              selectables[targetIndex],
              event.copyWith(direction: SelectionExtendDirection.backward),
            );
            assert(result == SelectionResult.end);
          }
        }
      case SelectionExtendDirection.nextLine:
        assert(result == SelectionResult.end || result == SelectionResult.next);
        if (result == SelectionResult.next) {
          if (targetIndex < selectables.length - 1) {
            targetIndex += 1;
            result = dispatchSelectionEventToChild(
              selectables[targetIndex],
              event.copyWith(direction: SelectionExtendDirection.forward),
            );
            assert(result == SelectionResult.end);
          }
        }
      case SelectionExtendDirection.forward:
      case SelectionExtendDirection.backward:
        assert(result == SelectionResult.end);
    }
    if (event.isEnd) {
      currentSelectionEndIndex = targetIndex;
    } else {
      currentSelectionStartIndex = targetIndex;
    }
    return result;
  }

  /// Updates the selection edges.
  @protected
  SelectionResult handleSelectionEdgeUpdate(SelectionEdgeUpdateEvent event) {
    if (event.type == SelectionEventType.endEdgeUpdate) {
      return currentSelectionEndIndex == -1
          ? _initSelection(event, isEnd: true)
          : _adjustSelection(event, isEnd: true);
    }
    return currentSelectionStartIndex == -1
        ? _initSelection(event, isEnd: false)
        : _adjustSelection(event, isEnd: false);
  }

  @override
  SelectionResult dispatchSelectionEvent(SelectionEvent event) {
    final bool selectionWillBeInProgress = event is! ClearSelectionEvent;
    if (!_selectionInProgress && selectionWillBeInProgress) {
      // Sort the selectable every time a selection start.
      selectables.sort(compareOrder);
    }
    _selectionInProgress = selectionWillBeInProgress;
    _isHandlingSelectionEvent = true;
    late SelectionResult result;
    switch (event.type) {
      case SelectionEventType.startEdgeUpdate:
      case SelectionEventType.endEdgeUpdate:
        _extendSelectionInProgress = false;
        result = handleSelectionEdgeUpdate(event as SelectionEdgeUpdateEvent);
      case SelectionEventType.clear:
        _extendSelectionInProgress = false;
        result = handleClearSelection(event as ClearSelectionEvent);
      case SelectionEventType.selectAll:
        _extendSelectionInProgress = false;
        result = handleSelectAll(event as SelectAllSelectionEvent);
      case SelectionEventType.selectWord:
        _extendSelectionInProgress = false;
        result = handleSelectWord(event as SelectWordSelectionEvent);
      case SelectionEventType.selectParagraph:
        _extendSelectionInProgress = false;
        result = handleSelectParagraph(event as SelectParagraphSelectionEvent);
      case SelectionEventType.granularlyExtendSelection:
        _extendSelectionInProgress = true;
        result = handleGranularlyExtendSelection(event as GranularlyExtendSelectionEvent);
      case SelectionEventType.directionallyExtendSelection:
        _extendSelectionInProgress = true;
        result = handleDirectionallyExtendSelection(event as DirectionallyExtendSelectionEvent);
    }
    _isHandlingSelectionEvent = false;
    _updateSelectionGeometry();
    return result;
  }

  @override
  void dispose() {
    for (final Selectable selectable in selectables) {
      selectable.removeListener(_handleSelectableGeometryChange);
    }
    selectables = const <Selectable>[];
    _scheduledSelectableUpdate = false;
    super.dispose();
  }

  /// Ensures the [Selectable] child has received up to date selection event.
  ///
  /// This method is called when a new [Selectable] is added to the delegate,
  /// and its screen location falls into the previous selection.
  ///
  /// Subclasses are responsible for updating the selection of this newly added
  /// [Selectable].
  @protected
  void ensureChildUpdated(Selectable selectable);

  /// Dispatches a selection event to a specific [Selectable].
  ///
  /// Override this method if subclasses need to generate additional events or
  /// treatments prior to sending the [SelectionEvent].
  @protected
  SelectionResult dispatchSelectionEventToChild(Selectable selectable, SelectionEvent event) {
    return selectable.dispatchSelectionEvent(event);
  }

  /// Initializes the selection of the selectable children.
  ///
  /// The goal is to find the selectable child that contains the selection edge.
  /// Returns [SelectionResult.end] if the selection edge ends on any of the
  /// children. Otherwise, it returns [SelectionResult.previous] if the selection
  /// does not reach any of its children. Returns [SelectionResult.next]
  /// if the selection reaches the end of its children.
  ///
  /// Ideally, this method should only be called twice at the beginning of the
  /// drag selection, once for start edge update event, once for end edge update
  /// event.
  SelectionResult _initSelection(SelectionEdgeUpdateEvent event, {required bool isEnd}) {
    assert(
      (isEnd && currentSelectionEndIndex == -1) || (!isEnd && currentSelectionStartIndex == -1),
    );
    int newIndex = -1;
    bool hasFoundEdgeIndex = false;
    SelectionResult? result;
    for (int index = 0; index < selectables.length && !hasFoundEdgeIndex; index += 1) {
      final Selectable child = selectables[index];
      final SelectionResult childResult = dispatchSelectionEventToChild(child, event);
      switch (childResult) {
        case SelectionResult.next:
        case SelectionResult.none:
          newIndex = index;
        case SelectionResult.end:
          newIndex = index;
          result = SelectionResult.end;
          hasFoundEdgeIndex = true;
        case SelectionResult.previous:
          hasFoundEdgeIndex = true;
          if (index == 0) {
            newIndex = 0;
            result = SelectionResult.previous;
          }
          result ??= SelectionResult.end;
        case SelectionResult.pending:
          newIndex = index;
          result = SelectionResult.pending;
          hasFoundEdgeIndex = true;
      }
    }

    if (newIndex == -1) {
      assert(selectables.isEmpty);
      return SelectionResult.none;
    }
    if (isEnd) {
      currentSelectionEndIndex = newIndex;
    } else {
      currentSelectionStartIndex = newIndex;
    }
    _flushInactiveSelections();
    // The result can only be null if the loop went through the entire list
    // without any of the selection returned end or previous. In this case, the
    // caller of this method needs to find the next selectable in their list.
    return result ?? SelectionResult.next;
  }

  /// Adjusts the selection based on the drag selection update event if there
  /// is already a selectable child that contains the selection edge.
  ///
  /// This method starts by sending the selection event to the current
  /// selectable that contains the selection edge, and finds forward or backward
  /// if that selectable no longer contains the selection edge.
  SelectionResult _adjustSelection(SelectionEdgeUpdateEvent event, {required bool isEnd}) {
    assert(() {
      if (isEnd) {
        assert(currentSelectionEndIndex < selectables.length && currentSelectionEndIndex >= 0);
        return true;
      }
      assert(currentSelectionStartIndex < selectables.length && currentSelectionStartIndex >= 0);
      return true;
    }());
    SelectionResult? finalResult;
    // Determines if the edge being adjusted is within the current viewport.
    //  - If so, we begin the search for the new selection edge position at the
    //    currentSelectionEndIndex/currentSelectionStartIndex.
    //  - If not, we attempt to locate the new selection edge starting from
    //    the opposite end.
    //  - If neither edge is in the current viewport, the search for the new
    //    selection edge position begins at 0.
    //
    // This can happen when there is a scrollable child and the edge being adjusted
    // has been scrolled out of view.
    final bool isCurrentEdgeWithinViewport =
        isEnd
            ? _selectionGeometry.endSelectionPoint != null
            : _selectionGeometry.startSelectionPoint != null;
    final bool isOppositeEdgeWithinViewport =
        isEnd
            ? _selectionGeometry.startSelectionPoint != null
            : _selectionGeometry.endSelectionPoint != null;
    int newIndex = switch ((isEnd, isCurrentEdgeWithinViewport, isOppositeEdgeWithinViewport)) {
      (true, true, true) => currentSelectionEndIndex,
      (true, true, false) => currentSelectionEndIndex,
      (true, false, true) => currentSelectionStartIndex,
      (true, false, false) => 0,
      (false, true, true) => currentSelectionStartIndex,
      (false, true, false) => currentSelectionStartIndex,
      (false, false, true) => currentSelectionEndIndex,
      (false, false, false) => 0,
    };
    bool? forward;
    late SelectionResult currentSelectableResult;
    // This loop sends the selection event to one of the following to determine
    // the direction of the search.
    //  - currentSelectionEndIndex/currentSelectionStartIndex if the current edge
    //    is in the current viewport.
    //  - The opposite edge index if the current edge is not in the current viewport.
    //  - Index 0 if neither edge is in the current viewport.
    //
    // If the result is `SelectionResult.next`, this loop look backward.
    // Otherwise, it looks forward.
    //
    // The terminate condition are:
    // 1. the selectable returns end, pending, none.
    // 2. the selectable returns previous when looking forward.
    // 2. the selectable returns next when looking backward.
    while (newIndex < selectables.length && newIndex >= 0 && finalResult == null) {
      currentSelectableResult = dispatchSelectionEventToChild(selectables[newIndex], event);
      switch (currentSelectableResult) {
        case SelectionResult.end:
        case SelectionResult.pending:
        case SelectionResult.none:
          finalResult = currentSelectableResult;
        case SelectionResult.next:
          if (forward == false) {
            newIndex += 1;
            finalResult = SelectionResult.end;
          } else if (newIndex == selectables.length - 1) {
            finalResult = currentSelectableResult;
          } else {
            forward = true;
            newIndex += 1;
          }
        case SelectionResult.previous:
          if (forward ?? false) {
            newIndex -= 1;
            finalResult = SelectionResult.end;
          } else if (newIndex == 0) {
            finalResult = currentSelectableResult;
          } else {
            forward = false;
            newIndex -= 1;
          }
      }
    }
    if (isEnd) {
      currentSelectionEndIndex = newIndex;
    } else {
      currentSelectionStartIndex = newIndex;
    }
    _flushInactiveSelections();
    return finalResult!;
  }
}

/// The length of the content that can be selected, and the range that is
/// selected.
typedef _SelectionInfo = ({int contentLength, SelectedContentRange? range});

/// Signature for a widget builder that builds a context menu for the given
/// [SelectableRegionState].
///
/// See also:
///
///  * [EditableTextContextMenuBuilder], which performs the same role for
///    [EditableText].
typedef SelectableRegionContextMenuBuilder =
    Widget Function(BuildContext context, SelectableRegionState selectableRegionState);

/// The status of the selection under a [SelectableRegion].
///
/// This value can be accessed for a [SelectableRegion] by using
/// [SelectableRegionSelectionStatusScope.maybeOf].
///
/// This value under a [SelectableRegion] is updated frequently
/// during selection gestures such as clicks and taps to select
/// and keyboard shortcuts.
enum SelectableRegionSelectionStatus {
  /// Indicates that the selection under a [SelectableRegion] is changing.
  ///
  /// A [SelectableRegion]s selection is changing when it is being
  /// updated by user through selection gestures and keyboard shortcuts.
  /// For example, during a text selection drag with a click + drag,
  /// a [SelectableRegion]s selection is considered changing until
  /// the user releases the click, then it will be considered finalized.
  changing,

  /// Indicates that the selection under a [SelectableRegion] is finalized.
  ///
  /// A [SelectableRegion]s selection is finalized when it is no longer
  /// being updated by the user through selection gestures or keyboard
  /// shortcuts. For example, the selection will be finalized on a mouse
  /// drag end, touch long press drag end, a single click to collapse the
  /// selection, a double click/tap to select a word, ctrl + A / cmd + A to
  /// select all, or a triple click/tap to select a paragraph.
  finalized,
}

/// Notifies its listeners when the [SelectableRegion] that created this object
/// is changing or finalizes its selection.
///
/// To access the [_SelectableRegionSelectionStatusNotifier] from the nearest [SelectableRegion]
/// ancestor, use [SelectableRegionSelectionStatusScope.maybeOf].
final class _SelectableRegionSelectionStatusNotifier extends ChangeNotifier
    implements ValueListenable<SelectableRegionSelectionStatus> {
  _SelectableRegionSelectionStatusNotifier._();

  SelectableRegionSelectionStatus _selectableRegionSelectionStatus =
      SelectableRegionSelectionStatus.finalized;

  /// The current value of the [SelectableRegionSelectionStatus] of the [SelectableRegion]
  /// that owns this object.
  ///
  /// Defaults to [SelectableRegionSelectionStatus.finalized].
  @override
  SelectableRegionSelectionStatus get value => _selectableRegionSelectionStatus;

  /// Sets the [SelectableRegionSelectionStatus] for the [SelectableRegion] that
  /// owns this object.
  ///
  /// Listeners are notified even if the value did not change.
  @protected
  set value(SelectableRegionSelectionStatus newStatus) {
    assert(
      newStatus == SelectableRegionSelectionStatus.finalized &&
              value == SelectableRegionSelectionStatus.changing ||
          newStatus == SelectableRegionSelectionStatus.changing,
      'Attempting to finalize the selection when it is already finalized.',
    );
    _selectableRegionSelectionStatus = newStatus;
    notifyListeners();
  }
}

/// Notifies its listeners when the selection under a [SelectableRegion] or
/// [SelectionArea] is being changed or finalized.
///
/// Use [SelectableRegionSelectionStatusScope.maybeOf], to access the [ValueListenable] of type
/// [SelectableRegionSelectionStatus] under a [SelectableRegion]. Its listeners
/// will be called even when the value of the [SelectableRegionSelectionStatus]
/// does not change.
final class SelectableRegionSelectionStatusScope extends InheritedWidget {
  const SelectableRegionSelectionStatusScope._({
    required this.selectionStatusNotifier,
    required super.child,
  });

  /// Tracks updates to the [SelectableRegionSelectionStatus] of the owning
  /// [SelectableRegion].
  ///
  /// Listeners will be called even when the value of the [SelectableRegionSelectionStatus]
  /// does not change. The selection under the [SelectableRegion] still may have changed.
  final ValueListenable<SelectableRegionSelectionStatus> selectionStatusNotifier;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [SelectableRegion] or [SelectionArea] widget, then null is
  /// returned.
  ///
  /// Calling this method will create a dependency on the closest
  /// [SelectableRegionSelectionStatusScope] in the [context], if there is one.
  static ValueListenable<SelectableRegionSelectionStatus>? maybeOf(BuildContext context) {
    return context
        .dependOnInheritedWidgetOfExactType<SelectableRegionSelectionStatusScope>()
        ?.selectionStatusNotifier;
  }

  @override
  bool updateShouldNotify(SelectableRegionSelectionStatusScope oldWidget) {
    return selectionStatusNotifier != oldWidget.selectionStatusNotifier;
  }
}

/// A [SelectionContainer] that allows the user to access the [SelectionDetails] and
/// listen to selection changes for the child subtree it wraps under a [SelectionArea]
/// or [SelectableRegion].
///
/// The selection updates are provided through the [selectionNotifier], to listen
/// to these updates attach a listener through [SelectionListenerNotifier.addListener].
///
/// This widget does not listen to selection changes of nested [SelectionArea]s
/// or [SelectableRegion]s in its subtree because those widgets are self-contained
/// and do not bubble up their selection. To listen to selection changes of a
/// [SelectionArea] or [SelectableRegion] under this [SelectionListener], add
/// an additional [SelectionListener] under each one.
///
/// {@tool dartpad}
/// This example shows how to use [SelectionListener] to access the [SelectionDetails]
/// under a [SelectionArea] or [SelectableRegion].
///
/// ** See code in examples/api/lib/material/selection_area/selection_area.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to color the active selection red under a
/// [SelectionArea] or [SelectableRegion].
///
/// ** See code in examples/api/lib/material/selection_area/selection_area.2.dart **
/// {@end-tool}
///
/// See also:
///
///   * [SelectableRegion], which provides an overview of the selection system.
class SelectionListener extends StatefulWidget {
  /// Create a new [SelectionListener] widget.
  const SelectionListener({super.key, required this.selectionNotifier, required this.child});

  /// Notifies listeners when the selection has changed.
  final SelectionListenerNotifier selectionNotifier;

  /// The child widget this selection listener applies to.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  @override
  State<SelectionListener> createState() => _SelectionListenerState();
}

class _SelectionListenerState extends State<SelectionListener> {
  late final _SelectionListenerDelegate _selectionDelegate = _SelectionListenerDelegate(
    selectionNotifier: widget.selectionNotifier,
  );

  @override
  void didUpdateWidget(SelectionListener oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.selectionNotifier != widget.selectionNotifier) {
      _selectionDelegate._setNotifier(widget.selectionNotifier);
    }
  }

  @override
  void dispose() {
    _selectionDelegate.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SelectionContainer(delegate: _selectionDelegate, child: widget.child);
  }
}

final class _SelectionListenerDelegate extends StaticSelectionContainerDelegate
    implements SelectionDetails {
  _SelectionListenerDelegate({required SelectionListenerNotifier selectionNotifier})
    : _selectionNotifier = selectionNotifier {
    _selectionNotifier._registerSelectionListenerDelegate(this);
  }

  SelectionGeometry? _initialSelectionGeometry;

  SelectionListenerNotifier _selectionNotifier;
  void _setNotifier(SelectionListenerNotifier newNotifier) {
    _selectionNotifier._unregisterSelectionListenerDelegate();
    _selectionNotifier = newNotifier;
    _selectionNotifier._registerSelectionListenerDelegate(this);
  }

  @override
  void notifyListeners() {
    super.notifyListeners();
    // Skip initial notification if selection is not valid.
    if (_initialSelectionGeometry == null && !value.hasSelection) {
      _initialSelectionGeometry = value;
      return;
    }
    _selectionNotifier.notifyListeners();
  }

  @override
  void dispose() {
    _selectionNotifier._unregisterSelectionListenerDelegate();
    _initialSelectionGeometry = null;
    super.dispose();
  }

  @override
  SelectedContentRange? get range => getSelection();

  @override
  SelectionStatus get status => value.status;
}

/// A read-only interface for accessing the details of a selection under a [SelectionListener].
///
/// This includes information such as the status of the selection indicating
/// if it is collapsed or uncollapsed, the [SelectedContentRange] that includes
/// the start and end offsets of the selection local to the [SelectionListener]
/// that reports this object.
///
/// This object is typically accessed by providing a [SelectionListenerNotifier]
/// to a [SelectionListener] and retrieving the value from [SelectionListenerNotifier.selection].
abstract final class SelectionDetails {
  /// The computed selection range of the owning [SelectionListener]s subtree.
  ///
  /// Returns `null` if there is nothing selected.
  SelectedContentRange? get range;

  /// The status that indicates whether there is a selection and whether the selection is collapsed.
  SelectionStatus get status;
}

/// Notifies listeners when the selection under a [SelectionListener] has been
/// changed.
///
/// This object is typically provided to a [SelectionListener].
final class SelectionListenerNotifier extends ChangeNotifier {
  _SelectionListenerDelegate? _selectionDelegate;

  /// The details of the selection under the [SelectionListener] that owns this notifier.
  ///
  /// Throws an exception if this notifier has not been registered to a [SelectionListener].
  /// To check if a notifier has been registered to a [SelectionListener] use [registered].
  SelectionDetails get selection =>
      _selectionDelegate ??
      (throw Exception('Selection client has not been registered to this notifier.'));

  /// Whether this [SelectionListenerNotifier] has been registered to a [SelectionListener].
  bool get registered => _selectionDelegate != null;

  void _registerSelectionListenerDelegate(_SelectionListenerDelegate selectionDelegate) {
    assert(
      !registered,
      'This SelectionListenerNotifier is already registered to another SelectionListener. Try providing a new SelectionListenerNotifier.',
    );
    _selectionDelegate = selectionDelegate;
  }

  void _unregisterSelectionListenerDelegate() {
    _selectionDelegate = null;
  }

  // From ChangeNotifier.
  @override
  void dispose() {
    _unregisterSelectionListenerDelegate();
    super.dispose();
  }

  /// Calls the listener every time the [SelectionGeometry] of the selection changes under
  /// a [SelectionListener].
  ///
  /// Listeners can be removed with [removeListener].
  @override
  void addListener(VoidCallback listener) {
    super.addListener(listener);
  }
}
