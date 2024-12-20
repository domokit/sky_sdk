// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'app.dart';
/// @docImport 'drag_target.dart';
/// @docImport 'implicit_animations.dart';
/// @docImport 'media_query.dart';
/// @docImport 'navigator.dart';
/// @docImport 'routes.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'sliver.dart';
/// @docImport 'text.dart';
library;

import 'dart:collection';

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';

import 'basic.dart';
import 'framework.dart';
import 'lookup_boundary.dart';
import 'ticker_provider.dart';

const String _flutterWidgetsLibrary = 'package:flutter/widgets.dart';

// Examples can assume:
// late BuildContext context;

// * OverlayEntry Implementation

/// A place in an [Overlay] that can contain a widget.
///
/// Overlay entries are inserted into an [Overlay] using the
/// [OverlayState.insert] or [OverlayState.insertAll] functions. To find the
/// closest enclosing overlay for a given [BuildContext], use the [Overlay.of]
/// function.
///
/// An overlay entry can be in at most one overlay at a time. To remove an entry
/// from its overlay, call the [remove] function on the overlay entry.
///
/// Because an [Overlay] uses a [Stack] layout, overlay entries can use
/// [Positioned] and [AnimatedPositioned] to position themselves within the
/// overlay.
///
/// For example, [Draggable] uses an [OverlayEntry] to show the drag avatar that
/// follows the user's finger across the screen after the drag begins. Using the
/// overlay to display the drag avatar lets the avatar float over the other
/// widgets in the app. As the user's finger moves, draggable calls
/// [markNeedsBuild] on the overlay entry to cause it to rebuild. In its build,
/// the entry includes a [Positioned] with its top and left property set to
/// position the drag avatar near the user's finger. When the drag is over,
/// [Draggable] removes the entry from the overlay to remove the drag avatar
/// from view.
///
/// By default, if there is an entirely [opaque] entry over this one, then this
/// one will not be included in the widget tree (in particular, stateful widgets
/// within the overlay entry will not be instantiated). To ensure that your
/// overlay entry is still built even if it is not visible, set [maintainState]
/// to true. This is more expensive, so should be done with care. In particular,
/// if widgets in an overlay entry with [maintainState] set to true repeatedly
/// call [State.setState], the user's battery will be drained unnecessarily.
///
/// [OverlayEntry] is a [Listenable] that notifies when the widget built by
/// [builder] is mounted or unmounted, whose exact state can be queried by
/// [mounted]. After the owner of the [OverlayEntry] calls [remove] and then
/// [dispose], the widget may not be immediately removed from the widget tree.
/// As a result listeners of the [OverlayEntry] can get notified for one last
/// time after the [dispose] call, when the widget is eventually unmounted.
///
/// {@macro flutter.widgets.overlayPortalVsOverlayEntry}
///
/// See also:
///
///  * [OverlayPortal], an alternative API for inserting widgets into an
///    [Overlay] using a builder callback.
///  * [Overlay], a stack of entries that can be managed independently.
///  * [OverlayState], the current state of an Overlay.
///  * [WidgetsApp], a convenience widget that wraps a number of widgets that
///    are commonly required for an application.
///  * [MaterialApp], a convenience widget that wraps a number of widgets that
///    are commonly required for Material Design applications.
class OverlayEntry implements Listenable {
  /// Creates an overlay entry.
  ///
  /// To insert the entry into an [Overlay], first find the overlay using
  /// [Overlay.of] and then call [OverlayState.insert]. To remove the entry,
  /// call [remove] on the overlay entry itself.
  OverlayEntry({
    required this.builder,
    bool opaque = false,
    bool maintainState = false,
    this.canSizeOverlay = false,
  }) : _opaque = opaque,
       _maintainState = maintainState {
    if (kFlutterMemoryAllocationsEnabled) {
      _maybeDispatchObjectCreation();
    }
  }

  /// This entry will include the widget built by this builder in the overlay at
  /// the entry's position.
  ///
  /// To cause this builder to be called again, call [markNeedsBuild] on this
  /// overlay entry.
  final WidgetBuilder builder;

  /// Whether this entry occludes the entire overlay.
  ///
  /// If an entry claims to be opaque, then, for efficiency, the overlay will
  /// skip building entries below that entry unless they have [maintainState]
  /// set.
  bool get opaque => _opaque;
  bool _opaque;
  set opaque(bool value) {
    assert(!_disposedByOwner);
    if (_opaque == value) {
      return;
    }
    _opaque = value;
    _overlay?._didChangeEntryOpacity();
  }

  /// Whether this entry must be included in the tree even if there is a fully
  /// [opaque] entry above it.
  ///
  /// By default, if there is an entirely [opaque] entry over this one, then this
  /// one will not be included in the widget tree (in particular, stateful widgets
  /// within the overlay entry will not be instantiated). To ensure that your
  /// overlay entry is still built even if it is not visible, set [maintainState]
  /// to true. This is more expensive, so should be done with care. In particular,
  /// if widgets in an overlay entry with [maintainState] set to true repeatedly
  /// call [State.setState], the user's battery will be drained unnecessarily.
  ///
  /// This is used by the [Navigator] and [Route] objects to ensure that routes
  /// are kept around even when in the background, so that [Future]s promised
  /// from subsequent routes will be handled properly when they complete.
  bool get maintainState => _maintainState;
  bool _maintainState;
  set maintainState(bool value) {
    assert(!_disposedByOwner);
    if (_maintainState == value) {
      return;
    }
    _maintainState = value;
    assert(_overlay != null);
    _overlay!._didChangeEntryOpacity();
  }

  /// Whether the content of this [OverlayEntry] can be used to size the
  /// [Overlay].
  ///
  /// In most situations the overlay sizes itself based on its incoming
  /// constraints to be as large as possible. However, if that would result in
  /// an infinite size, it has to rely on one of its children to size itself. In
  /// this situation, the overlay will consult the topmost non-[Positioned]
  /// overlay entry that has this property set to true, lay it out with the
  /// incoming [BoxConstraints] of the overlay, and force all other
  /// non-[Positioned] overlay entries to have the same size. The [Positioned]
  /// entries are laid out as usual based on the calculated size of the overlay.
  ///
  /// Overlay entries that set this to true must be able to handle unconstrained
  /// [BoxConstraints].
  ///
  /// Setting this to true has no effect if the overlay entry uses a [Positioned]
  /// widget to position itself in the overlay.
  final bool canSizeOverlay;

  /// Whether the [OverlayEntry] is currently mounted in the widget tree.
  ///
  /// The [OverlayEntry] notifies its listeners when this value changes.
  bool get mounted => _overlayEntryStateNotifier?.value != null;

  /// The currently mounted `_OverlayEntryWidgetState` built using this [OverlayEntry].
  ValueNotifier<_OverlayEntryWidgetState?>? _overlayEntryStateNotifier =
      ValueNotifier<_OverlayEntryWidgetState?>(null);

  // TODO(polina-c): stop duplicating code across disposables
  // https://github.com/flutter/flutter/issues/137435
  /// Dispatches event of object creation to [FlutterMemoryAllocations.instance].
  void _maybeDispatchObjectCreation() {
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: _flutterWidgetsLibrary,
        className: '$OverlayEntry',
        object: this,
      );
    }
  }

  @override
  void addListener(VoidCallback listener) {
    assert(!_disposedByOwner);
    _overlayEntryStateNotifier?.addListener(listener);
  }

  @override
  void removeListener(VoidCallback listener) {
    _overlayEntryStateNotifier?.removeListener(listener);
  }

  OverlayState? _overlay;
  final GlobalKey<_OverlayEntryWidgetState> _key = GlobalKey<_OverlayEntryWidgetState>();

  /// Remove this entry from the overlay.
  ///
  /// This should only be called once.
  ///
  /// This method removes this overlay entry from the overlay immediately. The
  /// UI will be updated in the same frame if this method is called before the
  /// overlay rebuild in this frame; otherwise, the UI will be updated in the
  /// next frame. This means that it is safe to call during builds, but also
  /// that if you do call this after the overlay rebuild, the UI will not update
  /// until the next frame (i.e. many milliseconds later).
  void remove() {
    assert(_overlay != null);
    assert(!_disposedByOwner);
    final OverlayState overlay = _overlay!;
    _overlay = null;
    if (!overlay.mounted) {
      return;
    }

    overlay._entries.remove(this);
    if (SchedulerBinding.instance.schedulerPhase == SchedulerPhase.persistentCallbacks) {
      SchedulerBinding.instance.addPostFrameCallback((Duration duration) {
        overlay._markDirty();
      }, debugLabel: 'OverlayEntry.markDirty');
    } else {
      overlay._markDirty();
    }
  }

  /// Cause this entry to rebuild during the next pipeline flush.
  ///
  /// You need to call this function if the output of [builder] has changed.
  void markNeedsBuild() {
    assert(!_disposedByOwner);
    _key.currentState?._markNeedsBuild();
  }

  void _didUnmount() {
    assert(!mounted);
    if (_disposedByOwner) {
      _overlayEntryStateNotifier?.dispose();
      _overlayEntryStateNotifier = null;
    }
  }

  bool _disposedByOwner = false;

  /// Discards any resources used by this [OverlayEntry].
  ///
  /// This method must be called after [remove] if the [OverlayEntry] is
  /// inserted into an [Overlay].
  ///
  /// After this is called, the object is not in a usable state and should be
  /// discarded (calls to [addListener] will throw after the object is disposed).
  /// However, the listeners registered may not be immediately released until
  /// the widget built using this [OverlayEntry] is unmounted from the widget
  /// tree.
  ///
  /// This method should only be called by the object's owner.
  void dispose() {
    assert(!_disposedByOwner);
    assert(
      _overlay == null,
      'An OverlayEntry must first be removed from the Overlay before dispose is called.',
    );
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    _disposedByOwner = true;
    if (!mounted) {
      // If we're still mounted when disposed, then this will be disposed in
      // _didUnmount, to allow notifications to occur until the entry is
      // unmounted.
      _overlayEntryStateNotifier?.dispose();
      _overlayEntryStateNotifier = null;
    }
  }

  @override
  String toString() =>
      '${describeIdentity(this)}(opaque: $opaque; maintainState: $maintainState)${_disposedByOwner ? "(DISPOSED)" : ""}';
}

class _OverlayEntryWidget extends StatefulWidget {
  const _OverlayEntryWidget({
    required Key super.key,
    required this.entry,
    required this.overlayState,
    this.tickerEnabled = true,
  });

  final OverlayEntry entry;
  final OverlayState overlayState;
  final bool tickerEnabled;

  @override
  _OverlayEntryWidgetState createState() => _OverlayEntryWidgetState();
}

class _OverlayEntryWidgetState extends State<_OverlayEntryWidget> {
  late _RenderTheater _theater;

  // Manages the stack of theater children whose paint order are sorted by their
  // _zOrderIndex. The children added by OverlayPortal are added to this linked
  // list, and they will be shown _above_ the OverlayEntry tied to this widget.
  // The children with larger zOrderIndex values (i.e. those called `show`
  // recently) will be painted last.
  //
  // This linked list is lazily created in `_add`, and the entries are added/removed
  // via `_add`/`_remove`, called by OverlayPortals lower in the tree. `_add` or
  // `_remove` does not cause this widget to rebuild, the linked list will be
  // read by _RenderTheater as part of its render child model. This would ideally
  // be in a RenderObject but there may not be RenderObjects between
  // _RenderTheater and the render subtree OverlayEntry builds.
  LinkedList<_OverlayEntryLocation>? _sortedTheaterSiblings;

  // Worst-case O(N), N being the number of children added to the top spot in
  // the same frame. This can be a bit expensive when there's a lot of global
  // key reparenting in the same frame but N is usually a small number.
  void _add(_OverlayEntryLocation child) {
    assert(mounted);
    final LinkedList<_OverlayEntryLocation> children =
        _sortedTheaterSiblings ??= LinkedList<_OverlayEntryLocation>();
    assert(!children.contains(child));
    _OverlayEntryLocation? insertPosition = children.isEmpty ? null : children.last;
    while (insertPosition != null && insertPosition._zOrderIndex > child._zOrderIndex) {
      insertPosition = insertPosition.previous;
    }
    if (insertPosition == null) {
      children.addFirst(child);
    } else {
      insertPosition.insertAfter(child);
    }
    assert(children.contains(child));
  }

  void _remove(_OverlayEntryLocation child) {
    assert(_sortedTheaterSiblings != null);
    final bool wasInCollection = _sortedTheaterSiblings?.remove(child) ?? false;
    assert(wasInCollection);
  }

  // Returns an Iterable that traverse the children in the child model in paint
  // order (from farthest to the user to the closest to the user).
  //
  // The iterator should be safe to use even when the child model is being
  // mutated. The reason for that is it's allowed to add/remove/move deferred
  // children to a _RenderTheater during performLayout, but the affected
  // children don't have to be laid out in the same performLayout call.
  late final Iterable<RenderBox> _paintOrderIterable = _createChildIterable(reversed: false);
  // An Iterable that traverse the children in the child model in
  // hit-test order (from closest to the user to the farthest to the user).
  late final Iterable<RenderBox> _hitTestOrderIterable = _createChildIterable(reversed: true);

  // The following uses sync* because hit-testing is lazy, and LinkedList as a
  // Iterable doesn't support concurrent modification.
  Iterable<RenderBox> _createChildIterable({required bool reversed}) sync* {
    final LinkedList<_OverlayEntryLocation>? children = _sortedTheaterSiblings;
    if (children == null || children.isEmpty) {
      return;
    }
    _OverlayEntryLocation? candidate = reversed ? children.last : children.first;
    while (candidate != null) {
      final RenderBox? renderBox = candidate._overlayChildRenderBox;
      candidate = reversed ? candidate.previous : candidate.next;
      if (renderBox != null) {
        yield renderBox;
      }
    }
  }

  @override
  void initState() {
    super.initState();
    widget.entry._overlayEntryStateNotifier!.value = this;
    _theater = context.findAncestorRenderObjectOfType<_RenderTheater>()!;
    assert(_sortedTheaterSiblings == null);
  }

  @override
  void didUpdateWidget(_OverlayEntryWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    // OverlayState's build method always returns a RenderObjectWidget _Theater,
    // so it's safe to assume that state equality implies render object equality.
    assert(oldWidget.entry == widget.entry);
    if (oldWidget.overlayState != widget.overlayState) {
      final _RenderTheater newTheater = context.findAncestorRenderObjectOfType<_RenderTheater>()!;
      assert(_theater != newTheater);
      _theater = newTheater;
    }
  }

  @override
  void dispose() {
    widget.entry._overlayEntryStateNotifier?.value = null;
    widget.entry._didUnmount();
    _sortedTheaterSiblings = null;
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return TickerMode(
      enabled: widget.tickerEnabled,
      child: _RenderTheaterMarker(
        theater: _theater,
        overlayEntryWidgetState: this,
        child: widget.entry.builder(context),
      ),
    );
  }

  void _markNeedsBuild() {
    setState(() {
      /* the state that changed is in the builder */
    });
  }
}

/// A stack of entries that can be managed independently.
///
/// Overlays let independent child widgets "float" visual elements on top of
/// other widgets by inserting them into the overlay's stack. The overlay lets
/// each of these widgets manage their participation in the overlay using
/// [OverlayEntry] objects.
///
/// Although you can create an [Overlay] directly, it's most common to use the
/// overlay created by the [Navigator] in a [WidgetsApp], [CupertinoApp] or a
/// [MaterialApp]. The navigator uses its overlay to manage the visual
/// appearance of its routes.
///
/// The [Overlay] widget uses a custom stack implementation, which is very
/// similar to the [Stack] widget. The main use case of [Overlay] is related to
/// navigation and being able to insert widgets on top of the pages in an app.
/// For layout purposes unrelated to navigation, consider using [Stack] instead.
///
/// An [Overlay] widget requires a [Directionality] widget to be in scope, so
/// that it can resolve direction-sensitive coordinates of any
/// [Positioned.directional] children.
///
/// For widgets drawn in an [OverlayEntry], do not assume that the size of the
/// [Overlay] is the size returned by [MediaQuery.sizeOf]. Nested overlays can
/// have different sizes.
///
/// {@tool dartpad}
/// This example shows how to use the [Overlay] to highlight the [NavigationBar]
/// destination.
///
/// ** See code in examples/api/lib/widgets/overlay/overlay.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [OverlayEntry], the class that is used for describing the overlay entries.
///  * [OverlayState], which is used to insert the entries into the overlay.
///  * [WidgetsApp], which inserts an [Overlay] widget indirectly via its [Navigator].
///  * [MaterialApp], which inserts an [Overlay] widget indirectly via its [Navigator].
///  * [CupertinoApp], which inserts an [Overlay] widget indirectly via its [Navigator].
///  * [Stack], which allows directly displaying a stack of widgets.
class Overlay extends StatefulWidget {
  /// Creates an overlay.
  ///
  /// The initial entries will be inserted into the overlay when its associated
  /// [OverlayState] is initialized.
  ///
  /// Rather than creating an overlay, consider using the overlay that is
  /// created by the [Navigator] in a [WidgetsApp], [CupertinoApp], or a
  /// [MaterialApp] for the application.
  const Overlay({
    super.key,
    this.initialEntries = const <OverlayEntry>[],
    this.clipBehavior = Clip.hardEdge,
  });

  /// Wrap the provided `child` in an [Overlay] to allow other visual elements
  /// (packed in [OverlayEntry]s) to float on top of the child.
  ///
  /// This is a convenience method over the regular [Overlay] constructor: It
  /// creates an [Overlay] and puts the provided `child` in an [OverlayEntry]
  /// at the bottom of that newly created Overlay.
  static Widget wrap({Key? key, Clip clipBehavior = Clip.hardEdge, required Widget child}) {
    return _WrappingOverlay(key: key, clipBehavior: clipBehavior, child: child);
  }

  /// The entries to include in the overlay initially.
  ///
  /// These entries are only used when the [OverlayState] is initialized. If you
  /// are providing a new [Overlay] description for an overlay that's already in
  /// the tree, then the new entries are ignored.
  ///
  /// To add entries to an [Overlay] that is already in the tree, use
  /// [Overlay.of] to obtain the [OverlayState] (or assign a [GlobalKey] to the
  /// [Overlay] widget and obtain the [OverlayState] via
  /// [GlobalKey.currentState]), and then use [OverlayState.insert] or
  /// [OverlayState.insertAll].
  ///
  /// To remove an entry from an [Overlay], use [OverlayEntry.remove].
  final List<OverlayEntry> initialEntries;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// The [OverlayState] from the closest instance of [Overlay] that encloses
  /// the given context within the closest [LookupBoundary], and, in debug mode,
  /// will throw if one is not found.
  ///
  /// In debug mode, if the `debugRequiredFor` argument is provided and an
  /// overlay isn't found, then this function will throw an exception containing
  /// the runtime type of the given widget in the error message. The exception
  /// attempts to explain that the calling [Widget] (the one given by the
  /// `debugRequiredFor` argument) needs an [Overlay] to be present to function.
  /// If `debugRequiredFor` is not supplied, then the error message is more
  /// generic.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// OverlayState overlay = Overlay.of(context);
  /// ```
  ///
  /// If `rootOverlay` is set to true, the state from the furthest instance of
  /// this class is given instead. Useful for installing overlay entries above
  /// all subsequent instances of [Overlay].
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// See also:
  ///
  /// * [Overlay.maybeOf] for a similar function that returns null if an
  ///   [Overlay] is not found.
  static OverlayState of(
    BuildContext context, {
    bool rootOverlay = false,
    Widget? debugRequiredFor,
  }) {
    final OverlayState? result = maybeOf(context, rootOverlay: rootOverlay);
    assert(() {
      if (result == null) {
        final bool hiddenByBoundary = LookupBoundary.debugIsHidingAncestorStateOfType<OverlayState>(
          context,
        );
        final List<DiagnosticsNode> information = <DiagnosticsNode>[
          ErrorSummary(
            'No Overlay widget found${hiddenByBoundary ? ' within the closest LookupBoundary' : ''}.',
          ),
          if (hiddenByBoundary)
            ErrorDescription(
              'There is an ancestor Overlay widget, but it is hidden by a LookupBoundary.',
            ),
          ErrorDescription(
            '${debugRequiredFor?.runtimeType ?? 'Some'} widgets require an Overlay widget ancestor for correct operation.',
          ),
          ErrorHint(
            'The most common way to add an Overlay to an application is to include a MaterialApp, CupertinoApp or Navigator widget in the runApp() call.',
          ),
          if (debugRequiredFor != null)
            DiagnosticsProperty<Widget>(
              'The specific widget that failed to find an overlay was',
              debugRequiredFor,
              style: DiagnosticsTreeStyle.errorProperty,
            ),
          if (context.widget != debugRequiredFor)
            context.describeElement(
              'The context from which that widget was searching for an overlay was',
            ),
        ];

        throw FlutterError.fromParts(information);
      }
      return true;
    }());
    return result!;
  }

  /// The [OverlayState] from the closest instance of [Overlay] that encloses
  /// the given context within the closest [LookupBoundary], if any.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// OverlayState? overlay = Overlay.maybeOf(context);
  /// ```
  ///
  /// If `rootOverlay` is set to true, the state from the furthest instance of
  /// this class is given instead. Useful for installing overlay entries above
  /// all subsequent instances of [Overlay].
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// See also:
  ///
  ///  * [Overlay.of] for a similar function that returns a non-nullable result
  ///    and throws if an [Overlay] is not found.

  static OverlayState? maybeOf(BuildContext context, {bool rootOverlay = false}) {
    return rootOverlay
        ? LookupBoundary.findRootAncestorStateOfType<OverlayState>(context)
        : LookupBoundary.findAncestorStateOfType<OverlayState>(context);
  }

  @override
  OverlayState createState() => OverlayState();
}

/// The current state of an [Overlay].
///
/// Used to insert [OverlayEntry]s into the overlay using the [insert] and
/// [insertAll] functions.
class OverlayState extends State<Overlay> with TickerProviderStateMixin {
  final List<OverlayEntry> _entries = <OverlayEntry>[];

  @protected
  @override
  void initState() {
    super.initState();
    insertAll(widget.initialEntries);
  }

  int _insertionIndex(OverlayEntry? below, OverlayEntry? above) {
    assert(above == null || below == null);
    if (below != null) {
      return _entries.indexOf(below);
    }
    if (above != null) {
      return _entries.indexOf(above) + 1;
    }
    return _entries.length;
  }

  bool _debugCanInsertEntry(OverlayEntry entry) {
    final List<DiagnosticsNode> operandsInformation = <DiagnosticsNode>[
      DiagnosticsProperty<OverlayEntry>(
        'The OverlayEntry was',
        entry,
        style: DiagnosticsTreeStyle.errorProperty,
      ),
      DiagnosticsProperty<OverlayState>(
        'The Overlay the OverlayEntry was trying to insert to was',
        this,
        style: DiagnosticsTreeStyle.errorProperty,
      ),
    ];

    if (!mounted) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('Attempted to insert an OverlayEntry to an already disposed Overlay.'),
        ...operandsInformation,
      ]);
    }

    final OverlayState? currentOverlay = entry._overlay;
    final bool alreadyContainsEntry = _entries.contains(entry);

    if (alreadyContainsEntry) {
      final bool inconsistentOverlayState = !identical(currentOverlay, this);
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('The specified entry is already present in the target Overlay.'),
        ...operandsInformation,
        if (inconsistentOverlayState)
          ErrorHint('This could be an error in the Flutter framework.')
        else
          ErrorHint(
            'Consider calling remove on the OverlayEntry before inserting it to a different Overlay, '
            'or switching to the OverlayPortal API to avoid manual OverlayEntry management.',
          ),
        if (inconsistentOverlayState)
          DiagnosticsProperty<OverlayState>(
            "The OverlayEntry's current Overlay was",
            currentOverlay,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
      ]);
    }

    if (currentOverlay == null) {
      return true;
    }

    throw FlutterError.fromParts(<DiagnosticsNode>[
      ErrorSummary('The specified entry is already present in a different Overlay.'),
      ...operandsInformation,
      DiagnosticsProperty<OverlayState>(
        "The OverlayEntry's current Overlay was",
        currentOverlay,
        style: DiagnosticsTreeStyle.errorProperty,
      ),
      ErrorHint(
        'Consider calling remove on the OverlayEntry before inserting it to a different Overlay, '
        'or switching to the OverlayPortal API to avoid manual OverlayEntry management.',
      ),
    ]);
  }

  /// Insert the given entry into the overlay.
  ///
  /// If `below` is non-null, the entry is inserted just below `below`.
  /// If `above` is non-null, the entry is inserted just above `above`.
  /// Otherwise, the entry is inserted on top.
  ///
  /// It is an error to specify both `above` and `below`.
  void insert(OverlayEntry entry, {OverlayEntry? below, OverlayEntry? above}) {
    assert(_debugVerifyInsertPosition(above, below));
    assert(_debugCanInsertEntry(entry));
    entry._overlay = this;
    setState(() {
      _entries.insert(_insertionIndex(below, above), entry);
    });
  }

  /// Insert all the entries in the given iterable.
  ///
  /// If `below` is non-null, the entries are inserted just below `below`.
  /// If `above` is non-null, the entries are inserted just above `above`.
  /// Otherwise, the entries are inserted on top.
  ///
  /// It is an error to specify both `above` and `below`.
  void insertAll(Iterable<OverlayEntry> entries, {OverlayEntry? below, OverlayEntry? above}) {
    assert(_debugVerifyInsertPosition(above, below));
    assert(entries.every(_debugCanInsertEntry));
    if (entries.isEmpty) {
      return;
    }
    for (final OverlayEntry entry in entries) {
      assert(entry._overlay == null);
      entry._overlay = this;
    }
    setState(() {
      _entries.insertAll(_insertionIndex(below, above), entries);
    });
  }

  bool _debugVerifyInsertPosition(
    OverlayEntry? above,
    OverlayEntry? below, {
    Iterable<OverlayEntry>? newEntries,
  }) {
    assert(above == null || below == null, 'Only one of `above` and `below` may be specified.');
    assert(
      above == null ||
          (above._overlay == this &&
              _entries.contains(above) &&
              (newEntries?.contains(above) ?? true)),
      'The provided entry used for `above` must be present in the Overlay${newEntries != null ? ' and in the `newEntriesList`' : ''}.',
    );
    assert(
      below == null ||
          (below._overlay == this &&
              _entries.contains(below) &&
              (newEntries?.contains(below) ?? true)),
      'The provided entry used for `below` must be present in the Overlay${newEntries != null ? ' and in the `newEntriesList`' : ''}.',
    );
    return true;
  }

  /// Remove all the entries listed in the given iterable, then reinsert them
  /// into the overlay in the given order.
  ///
  /// Entries mention in `newEntries` but absent from the overlay are inserted
  /// as if with [insertAll].
  ///
  /// Entries not mentioned in `newEntries` but present in the overlay are
  /// positioned as a group in the resulting list relative to the entries that
  /// were moved, as specified by one of `below` or `above`, which, if
  /// specified, must be one of the entries in `newEntries`:
  ///
  /// If `below` is non-null, the group is positioned just below `below`.
  /// If `above` is non-null, the group is positioned just above `above`.
  /// Otherwise, the group is left on top, with all the rearranged entries
  /// below.
  ///
  /// It is an error to specify both `above` and `below`.
  void rearrange(Iterable<OverlayEntry> newEntries, {OverlayEntry? below, OverlayEntry? above}) {
    final List<OverlayEntry> newEntriesList =
        newEntries is List<OverlayEntry> ? newEntries : newEntries.toList(growable: false);
    assert(_debugVerifyInsertPosition(above, below, newEntries: newEntriesList));
    assert(
      newEntriesList.every(
        (OverlayEntry entry) => entry._overlay == null || entry._overlay == this,
      ),
      'One or more of the specified entries are already present in another Overlay.',
    );
    assert(
      newEntriesList.every(
        (OverlayEntry entry) => _entries.indexOf(entry) == _entries.lastIndexOf(entry),
      ),
      'One or more of the specified entries are specified multiple times.',
    );
    if (newEntriesList.isEmpty) {
      return;
    }
    if (listEquals(_entries, newEntriesList)) {
      return;
    }
    final LinkedHashSet<OverlayEntry> old = LinkedHashSet<OverlayEntry>.of(_entries);
    for (final OverlayEntry entry in newEntriesList) {
      entry._overlay ??= this;
    }
    setState(() {
      _entries.clear();
      _entries.addAll(newEntriesList);
      old.removeAll(newEntriesList);
      _entries.insertAll(_insertionIndex(below, above), old);
    });
  }

  void _markDirty() {
    if (mounted) {
      setState(() {});
    }
  }

  /// (DEBUG ONLY) Check whether a given entry is visible (i.e., not behind an
  /// opaque entry).
  ///
  /// This is an O(N) algorithm, and should not be necessary except for debug
  /// asserts. To avoid people depending on it, this function is implemented
  /// only in debug mode, and always returns false in release mode.
  bool debugIsVisible(OverlayEntry entry) {
    bool result = false;
    assert(_entries.contains(entry));
    assert(() {
      for (int i = _entries.length - 1; i > 0; i -= 1) {
        final OverlayEntry candidate = _entries[i];
        if (candidate == entry) {
          result = true;
          break;
        }
        if (candidate.opaque) {
          break;
        }
      }
      return true;
    }());
    return result;
  }

  void _didChangeEntryOpacity() {
    setState(() {
      // We use the opacity of the entry in our build function, which means we
      // our state has changed.
    });
  }

  @protected
  @override
  Widget build(BuildContext context) {
    // This list is filled backwards and then reversed below before
    // it is added to the tree.
    final List<_OverlayEntryWidget> children = <_OverlayEntryWidget>[];
    bool onstage = true;
    int onstageCount = 0;
    for (final OverlayEntry entry in _entries.reversed) {
      if (onstage) {
        onstageCount += 1;
        children.add(_OverlayEntryWidget(key: entry._key, overlayState: this, entry: entry));
        if (entry.opaque) {
          onstage = false;
        }
      } else if (entry.maintainState) {
        children.add(
          _OverlayEntryWidget(
            key: entry._key,
            overlayState: this,
            entry: entry,
            tickerEnabled: false,
          ),
        );
      }
    }
    return _Theater(
      skipCount: children.length - onstageCount,
      clipBehavior: widget.clipBehavior,
      children: children.reversed.toList(growable: false),
    );
  }

  @protected
  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    // TODO(jacobr): use IterableProperty instead as that would
    // provide a slightly more consistent string summary of the List.
    properties.add(DiagnosticsProperty<List<OverlayEntry>>('entries', _entries));
  }
}

class _WrappingOverlay extends StatefulWidget {
  const _WrappingOverlay({super.key, this.clipBehavior = Clip.hardEdge, required this.child});

  final Clip clipBehavior;
  final Widget child;

  @override
  State<_WrappingOverlay> createState() => _WrappingOverlayState();
}

class _WrappingOverlayState extends State<_WrappingOverlay> {
  late final OverlayEntry _entry = OverlayEntry(
    canSizeOverlay: true,
    opaque: true,
    builder: (BuildContext context) {
      return widget.child;
    },
  );

  @override
  void didUpdateWidget(_WrappingOverlay oldWidget) {
    super.didUpdateWidget(oldWidget);
    _entry.markNeedsBuild();
  }

  @override
  void dispose() {
    _entry
      ..remove()
      ..dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Overlay(clipBehavior: widget.clipBehavior, initialEntries: <OverlayEntry>[_entry]);
  }
}

/// Special version of a [Stack], that doesn't layout and render the first
/// [skipCount] children.
///
/// The first [skipCount] children are considered "offstage".
class _Theater extends MultiChildRenderObjectWidget {
  const _Theater({
    this.skipCount = 0,
    this.clipBehavior = Clip.hardEdge,
    required List<_OverlayEntryWidget> super.children,
  }) : assert(skipCount >= 0),
       assert(children.length >= skipCount);

  final int skipCount;

  final Clip clipBehavior;

  @override
  _TheaterElement createElement() => _TheaterElement(this);

  @override
  _RenderTheater createRenderObject(BuildContext context) {
    return _RenderTheater(
      skipCount: skipCount,
      textDirection: Directionality.of(context),
      clipBehavior: clipBehavior,
    );
  }

  @override
  void updateRenderObject(BuildContext context, _RenderTheater renderObject) {
    renderObject
      ..skipCount = skipCount
      ..textDirection = Directionality.of(context)
      ..clipBehavior = clipBehavior;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IntProperty('skipCount', skipCount));
  }
}

class _TheaterElement extends MultiChildRenderObjectElement {
  _TheaterElement(_Theater super.widget);

  @override
  _RenderTheater get renderObject => super.renderObject as _RenderTheater;

  @override
  void insertRenderObjectChild(RenderBox child, IndexedSlot<Element?> slot) {
    super.insertRenderObjectChild(child, slot);
    final _TheaterParentData parentData = child.parentData! as _TheaterParentData;
    parentData.overlayEntry =
        ((widget as _Theater).children[slot.index] as _OverlayEntryWidget).entry;
    assert(parentData.overlayEntry != null);
  }

  @override
  void moveRenderObjectChild(
    RenderBox child,
    IndexedSlot<Element?> oldSlot,
    IndexedSlot<Element?> newSlot,
  ) {
    super.moveRenderObjectChild(child, oldSlot, newSlot);
    assert(() {
      final _TheaterParentData parentData = child.parentData! as _TheaterParentData;
      return parentData.overlayEntry ==
          ((widget as _Theater).children[newSlot.index] as _OverlayEntryWidget).entry;
    }());
  }

  @override
  void debugVisitOnstageChildren(ElementVisitor visitor) {
    final _Theater theater = widget as _Theater;
    assert(children.length >= theater.skipCount);
    children.skip(theater.skipCount).forEach(visitor);
  }
}

// A `RenderBox` that sizes itself to its parent's size, implements the stack
// layout algorithm and renders its children in the given `theater`.
mixin _RenderTheaterMixin on RenderBox {
  _RenderTheater get theater;

  Iterable<RenderBox> _childrenInPaintOrder();
  Iterable<RenderBox> _childrenInHitTestOrder();

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! StackParentData) {
      child.parentData = StackParentData();
    }
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    assert(!debugNeedsLayout);
    BaselineOffset baselineOffset = BaselineOffset.noBaseline;
    for (final RenderBox child in _childrenInPaintOrder()) {
      assert(!child.debugNeedsLayout);
      final StackParentData childParentData = child.parentData! as StackParentData;
      baselineOffset = baselineOffset.minOf(
        BaselineOffset(child.getDistanceToActualBaseline(baseline)) + childParentData.offset.dy,
      );
    }
    return baselineOffset.offset;
  }

  static double? baselineForChild(
    RenderBox child,
    Size theaterSize,
    BoxConstraints nonPositionedChildConstraints,
    Alignment alignment,
    TextBaseline baseline,
  ) {
    final StackParentData childParentData = child.parentData! as StackParentData;
    final BoxConstraints childConstraints =
        childParentData.isPositioned
            ? childParentData.positionedChildConstraints(theaterSize)
            : nonPositionedChildConstraints;
    final double? baselineOffset = child.getDryBaseline(childConstraints, baseline);
    if (baselineOffset == null) {
      return null;
    }
    final double y = switch (childParentData) {
      StackParentData(:final double top?) => top,
      StackParentData(:final double bottom?) =>
        theaterSize.height - bottom - child.getDryLayout(childConstraints).height,
      StackParentData() =>
        alignment.alongOffset(theaterSize - child.getDryLayout(childConstraints) as Offset).dy,
    };
    return baselineOffset + y;
  }

  void layoutChild(RenderBox child, BoxConstraints nonPositionedChildConstraints) {
    final StackParentData childParentData = child.parentData! as StackParentData;
    final Alignment alignment = theater._resolvedAlignment;
    if (!childParentData.isPositioned) {
      child.layout(nonPositionedChildConstraints, parentUsesSize: true);
      childParentData.offset = Offset.zero;
    } else {
      assert(
        child is! _RenderDeferredLayoutBox,
        'all _RenderDeferredLayoutBoxes must be non-positioned children.',
      );
      RenderStack.layoutPositionedChild(child, childParentData, size, alignment);
    }
    assert(child.parentData == childParentData);
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    final Iterator<RenderBox> iterator = _childrenInHitTestOrder().iterator;
    bool isHit = false;
    while (!isHit && iterator.moveNext()) {
      final RenderBox child = iterator.current;
      final StackParentData childParentData = child.parentData! as StackParentData;
      final RenderBox localChild = child;
      bool childHitTest(BoxHitTestResult result, Offset position) =>
          localChild.hitTest(result, position: position);
      isHit = result.addWithPaintOffset(
        offset: childParentData.offset,
        position: position,
        hitTest: childHitTest,
      );
    }
    return isHit;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    for (final RenderBox child in _childrenInPaintOrder()) {
      final StackParentData childParentData = child.parentData! as StackParentData;
      context.paintChild(child, childParentData.offset + offset);
    }
  }
}

class _TheaterParentData extends StackParentData {
  // The OverlayEntry that directly created this child. This field is null for
  // children that are created by an OverlayPortal.
  OverlayEntry? overlayEntry;

  /// A [OverlayPortal] makes its overlay child a render child of an ancestor
  /// [Overlay]. Currently, to make sure the overlay child is painted after its
  /// [OverlayPortal], and before the next [OverlayEntry] (which could be
  /// something that should obstruct the overlay child, such as a [ModalRoute])
  /// in the host [Overlay], the paint order of each overlay child is managed by
  /// the [OverlayEntry] that hosts its [OverlayPortal].
  ///
  /// The following methods are exposed to allow easy access to the overlay
  /// children's render objects whose order is managed by [overlayEntry], in the
  /// right order.

  // _overlayStateMounted is set to null in _OverlayEntryWidgetState's dispose
  // method. This property is only accessed during layout, paint and hit-test so
  // the `value!` should be safe.
  Iterator<RenderBox>? get paintOrderIterator =>
      overlayEntry?._overlayEntryStateNotifier?.value!._paintOrderIterable.iterator;
  Iterator<RenderBox>? get hitTestOrderIterator =>
      overlayEntry?._overlayEntryStateNotifier?.value!._hitTestOrderIterable.iterator;

  // A convenience method for traversing `paintOrderIterator` with a
  // [RenderObjectVisitor].
  void visitOverlayPortalChildrenOnOverlayEntry(RenderObjectVisitor visitor) =>
      overlayEntry?._overlayEntryStateNotifier?.value!._paintOrderIterable.forEach(visitor);
}

class _RenderTheater extends RenderBox
    with ContainerRenderObjectMixin<RenderBox, StackParentData>, _RenderTheaterMixin {
  _RenderTheater({
    List<RenderBox>? children,
    required TextDirection textDirection,
    int skipCount = 0,
    Clip clipBehavior = Clip.hardEdge,
  }) : assert(skipCount >= 0),
       _textDirection = textDirection,
       _skipCount = skipCount,
       _clipBehavior = clipBehavior {
    addAll(children);
  }

  @override
  _RenderTheater get theater => this;

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! _TheaterParentData) {
      child.parentData = _TheaterParentData();
    }
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    RenderBox? child = firstChild;
    while (child != null) {
      final _TheaterParentData childParentData = child.parentData! as _TheaterParentData;
      final Iterator<RenderBox>? iterator = childParentData.paintOrderIterator;
      if (iterator != null) {
        while (iterator.moveNext()) {
          iterator.current.attach(owner);
        }
      }
      child = childParentData.nextSibling;
    }
  }

  static void _detachChild(RenderObject child) => child.detach();

  @override
  void detach() {
    super.detach();
    RenderBox? child = firstChild;
    while (child != null) {
      final _TheaterParentData childParentData = child.parentData! as _TheaterParentData;
      childParentData.visitOverlayPortalChildrenOnOverlayEntry(_detachChild);
      child = childParentData.nextSibling;
    }
  }

  @override
  void redepthChildren() => visitChildren(redepthChild);

  Alignment? _alignmentCache;
  Alignment get _resolvedAlignment =>
      _alignmentCache ??= AlignmentDirectional.topStart.resolve(textDirection);

  void _markNeedResolution() {
    _alignmentCache = null;
    markNeedsLayout();
  }

  TextDirection get textDirection => _textDirection;
  TextDirection _textDirection;
  set textDirection(TextDirection value) {
    if (_textDirection == value) {
      return;
    }
    _textDirection = value;
    _markNeedResolution();
  }

  int get skipCount => _skipCount;
  int _skipCount;
  set skipCount(int value) {
    if (_skipCount != value) {
      _skipCount = value;
      markNeedsLayout();
    }
  }

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  Clip get clipBehavior => _clipBehavior;
  Clip _clipBehavior = Clip.hardEdge;
  set clipBehavior(Clip value) {
    if (value != _clipBehavior) {
      _clipBehavior = value;
      markNeedsPaint();
      markNeedsSemanticsUpdate();
    }
  }

  // Adding/removing deferred child does not affect the layout of other children,
  // or that of the Overlay, so there's no need to invalidate the layout of the
  // Overlay.
  //
  // When _skipMarkNeedsLayout is true, markNeedsLayout does not do anything.
  bool _skipMarkNeedsLayout = false;
  void _addDeferredChild(_RenderDeferredLayoutBox child) {
    assert(!_skipMarkNeedsLayout);
    _skipMarkNeedsLayout = true;
    adoptChild(child);
    // The Overlay still needs repainting when a deferred child is added. Usually
    // `markNeedsLayout` implies `markNeedsPaint`, but here `markNeedsLayout` is
    // skipped when the `_skipMarkNeedsLayout` flag is set.
    markNeedsPaint();
    _skipMarkNeedsLayout = false;

    // After adding `child` to the render tree, we want to make sure it will be
    // laid out in the same frame. This is done by calling markNeedsLayout on the
    // layout surrogate. This ensures `child` is reachable via tree walk (see
    // _RenderLayoutSurrogateProxyBox.performLayout).
    child._layoutSurrogate.markNeedsLayout();
  }

  void _removeDeferredChild(_RenderDeferredLayoutBox child) {
    assert(!_skipMarkNeedsLayout);
    _skipMarkNeedsLayout = true;
    dropChild(child);
    // The Overlay still needs repainting when a deferred child is dropped. See
    // the comment in `_addDeferredChild`.
    markNeedsPaint();
    _skipMarkNeedsLayout = false;
  }

  @override
  void markNeedsLayout() {
    if (!_skipMarkNeedsLayout) {
      super.markNeedsLayout();
    }
  }

  RenderBox? get _firstOnstageChild {
    if (skipCount == super.childCount) {
      return null;
    }
    RenderBox? child = super.firstChild;
    for (int toSkip = skipCount; toSkip > 0; toSkip--) {
      final StackParentData childParentData = child!.parentData! as StackParentData;
      child = childParentData.nextSibling;
      assert(child != null);
    }
    return child;
  }

  RenderBox? get _lastOnstageChild => skipCount == super.childCount ? null : lastChild;

  @override
  double computeMinIntrinsicWidth(double height) {
    return RenderStack.getIntrinsicDimension(
      _firstOnstageChild,
      (RenderBox child) => child.getMinIntrinsicWidth(height),
    );
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return RenderStack.getIntrinsicDimension(
      _firstOnstageChild,
      (RenderBox child) => child.getMaxIntrinsicWidth(height),
    );
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return RenderStack.getIntrinsicDimension(
      _firstOnstageChild,
      (RenderBox child) => child.getMinIntrinsicHeight(width),
    );
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return RenderStack.getIntrinsicDimension(
      _firstOnstageChild,
      (RenderBox child) => child.getMaxIntrinsicHeight(width),
    );
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    final Size size =
        constraints.biggest.isFinite
            ? constraints.biggest
            : _findSizeDeterminingChild().getDryLayout(constraints);
    final BoxConstraints nonPositionedChildConstraints = BoxConstraints.tight(size);
    final Alignment alignment = theater._resolvedAlignment;

    BaselineOffset baselineOffset = BaselineOffset.noBaseline;
    for (final RenderBox child in _childrenInPaintOrder()) {
      baselineOffset = baselineOffset.minOf(
        BaselineOffset(
          _RenderTheaterMixin.baselineForChild(
            child,
            size,
            nonPositionedChildConstraints,
            alignment,
            baseline,
          ),
        ),
      );
    }
    return baselineOffset.offset;
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    if (constraints.biggest.isFinite) {
      return constraints.biggest;
    }
    return _findSizeDeterminingChild().getDryLayout(constraints);
  }

  @override
  // The following uses sync* because concurrent modifications should be allowed
  // during layout.
  Iterable<RenderBox> _childrenInPaintOrder() sync* {
    RenderBox? child = _firstOnstageChild;
    while (child != null) {
      yield child;
      final _TheaterParentData childParentData = child.parentData! as _TheaterParentData;
      final Iterator<RenderBox>? innerIterator = childParentData.paintOrderIterator;
      if (innerIterator != null) {
        while (innerIterator.moveNext()) {
          yield innerIterator.current;
        }
      }
      child = childParentData.nextSibling;
    }
  }

  @override
  // The following uses sync* because hit testing should be lazy.
  Iterable<RenderBox> _childrenInHitTestOrder() sync* {
    RenderBox? child = _lastOnstageChild;
    int childLeft = childCount - skipCount;
    while (child != null) {
      final _TheaterParentData childParentData = child.parentData! as _TheaterParentData;
      final Iterator<RenderBox>? innerIterator = childParentData.hitTestOrderIterator;
      if (innerIterator != null) {
        while (innerIterator.moveNext()) {
          yield innerIterator.current;
        }
      }
      yield child;
      childLeft -= 1;
      child = childLeft <= 0 ? null : childParentData.previousSibling;
    }
  }

  @override
  bool get sizedByParent => false;

  bool _layingOutSizeDeterminingChild = false;
  @override
  void performLayout() {
    RenderBox? sizeDeterminingChild;
    if (constraints.biggest.isFinite) {
      size = constraints.biggest;
    } else {
      sizeDeterminingChild = _findSizeDeterminingChild();
      _layingOutSizeDeterminingChild = true;
      layoutChild(sizeDeterminingChild, constraints);
      _layingOutSizeDeterminingChild = false;
      size = sizeDeterminingChild.size;
    }

    // Equivalent to BoxConstraints used by RenderStack for StackFit.expand.
    final BoxConstraints nonPositionedChildConstraints = BoxConstraints.tight(size);
    for (final RenderBox child in _childrenInPaintOrder()) {
      if (child != sizeDeterminingChild) {
        layoutChild(child, nonPositionedChildConstraints);
      }
    }
  }

  RenderBox _findSizeDeterminingChild() {
    RenderBox? child = _lastOnstageChild;
    while (child != null) {
      final _TheaterParentData childParentData = child.parentData! as _TheaterParentData;
      if ((childParentData.overlayEntry?.canSizeOverlay ?? false) &&
          !childParentData.isPositioned) {
        return child;
      }
      child = childParentData.previousSibling;
    }
    throw FlutterError.fromParts(<DiagnosticsNode>[
      ErrorSummary(
        'Overlay was given infinite constraints and cannot be sized by a suitable child.',
      ),
      ErrorDescription(
        'The constraints given to the overlay ($constraints) would result in an illegal '
        'infinite size (${constraints.biggest}). To avoid that, the Overlay tried to size '
        'itself to one of its children, but no suitable non-positioned child that belongs to an '
        'OverlayEntry with canSizeOverlay set to true could be found.',
      ),
      ErrorHint(
        'Try wrapping the Overlay in a SizedBox to give it a finite size or '
        'use an OverlayEntry with canSizeOverlay set to true.',
      ),
    ]);
  }

  final LayerHandle<ClipRectLayer> _clipRectLayer = LayerHandle<ClipRectLayer>();

  @override
  void paint(PaintingContext context, Offset offset) {
    if (clipBehavior != Clip.none) {
      _clipRectLayer.layer = context.pushClipRect(
        needsCompositing,
        offset,
        Offset.zero & size,
        super.paint,
        clipBehavior: clipBehavior,
        oldLayer: _clipRectLayer.layer,
      );
    } else {
      _clipRectLayer.layer = null;
      super.paint(context, offset);
    }
  }

  @override
  void dispose() {
    _clipRectLayer.layer = null;
    super.dispose();
  }

  @override
  void visitChildren(RenderObjectVisitor visitor) {
    RenderBox? child = firstChild;
    while (child != null) {
      visitor(child);
      final _TheaterParentData childParentData = child.parentData! as _TheaterParentData;
      childParentData.visitOverlayPortalChildrenOnOverlayEntry(visitor);
      child = childParentData.nextSibling;
    }
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    RenderBox? child = _firstOnstageChild;
    while (child != null) {
      visitor(child);
      final _TheaterParentData childParentData = child.parentData! as _TheaterParentData;
      child = childParentData.nextSibling;
    }
  }

  @override
  Rect? describeApproximatePaintClip(RenderObject child) {
    switch (clipBehavior) {
      case Clip.none:
        return null;
      case Clip.hardEdge:
      case Clip.antiAlias:
      case Clip.antiAliasWithSaveLayer:
        return Offset.zero & size;
    }
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IntProperty('skipCount', skipCount));
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection));
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    final List<DiagnosticsNode> offstageChildren = <DiagnosticsNode>[];
    final List<DiagnosticsNode> onstageChildren = <DiagnosticsNode>[];

    int count = 1;
    bool onstage = false;
    RenderBox? child = firstChild;
    final RenderBox? firstOnstageChild = _firstOnstageChild;
    while (child != null) {
      final _TheaterParentData childParentData = child.parentData! as _TheaterParentData;
      if (child == firstOnstageChild) {
        onstage = true;
        count = 1;
      }

      if (onstage) {
        onstageChildren.add(child.toDiagnosticsNode(name: 'onstage $count'));
      } else {
        offstageChildren.add(
          child.toDiagnosticsNode(name: 'offstage $count', style: DiagnosticsTreeStyle.offstage),
        );
      }

      int subcount = 1;
      childParentData.visitOverlayPortalChildrenOnOverlayEntry((RenderObject renderObject) {
        final RenderBox child = renderObject as RenderBox;
        if (onstage) {
          onstageChildren.add(child.toDiagnosticsNode(name: 'onstage $count - $subcount'));
        } else {
          offstageChildren.add(
            child.toDiagnosticsNode(
              name: 'offstage $count - $subcount',
              style: DiagnosticsTreeStyle.offstage,
            ),
          );
        }
        subcount += 1;
      });

      child = childParentData.nextSibling;
      count += 1;
    }

    return <DiagnosticsNode>[
      ...onstageChildren,
      if (offstageChildren.isNotEmpty)
        ...offstageChildren
      else
        DiagnosticsNode.message('no offstage children', style: DiagnosticsTreeStyle.offstage),
    ];
  }
}

// * OverlayPortal Implementation
//  OverlayPortal is inspired by the
//  [flutter_portal](https://pub.dev/packages/flutter_portal) package.
//
// ** RenderObject hierarchy
// The widget works by inserting its overlay child's render subtree directly
// under [Overlay]'s render object (_RenderTheater).
// https://user-images.githubusercontent.com/31859944/171971838-62ed3975-4b5d-4733-a9c9-f79e263b8fcc.jpg
//
// To ensure the overlay child render subtree does not do layout twice, the
// subtree must only perform layout after both its _RenderTheater and the
// [OverlayPortal]'s render object (_RenderLayoutSurrogateProxyBox) have
// finished layout. This is handled by _RenderDeferredLayoutBox.
//
// ** Z-Index of an overlay child
// [_OverlayEntryLocation] is a (currently private) interface that allows an
// [OverlayPortal] to insert its overlay child into a specific [Overlay], as
// well as specifying the paint order between the overlay child and other
// children of the _RenderTheater.
//
// Since [OverlayPortal] is only allowed to target ancestor [Overlay]s
// (_RenderTheater must finish doing layout before _RenderDeferredLayoutBox),
// the _RenderTheater should typically be acquired using an [InheritedWidget]
// (currently, _RenderTheaterMarker) in case the [OverlayPortal] gets
// reparented.

/// A class to show, hide and bring to top an [OverlayPortal]'s overlay child
/// in the target [Overlay].
///
/// A [OverlayPortalController] can only be given to at most one [OverlayPortal]
/// at a time. When an [OverlayPortalController] is moved from one
/// [OverlayPortal] to another, its [isShowing] state does not carry over.
///
/// [OverlayPortalController.show] and [OverlayPortalController.hide] can be
/// called even before the controller is assigned to any [OverlayPortal], but
/// they typically should not be called while the widget tree is being rebuilt.
class OverlayPortalController {
  /// Creates an [OverlayPortalController], optionally with a String identifier
  /// `debugLabel`.
  OverlayPortalController({String? debugLabel}) : _debugLabel = debugLabel;

  _OverlayPortalState? _attachTarget;

  // A separate _zOrderIndex to allow `show()` or `hide()` to be called when the
  // controller is not yet attached. Once this controller is attached,
  // _attachTarget._zOrderIndex will be used as the source of truth, and this
  // variable will be set to null.
  int? _zOrderIndex;
  final String? _debugLabel;

  static int _wallTime =
      kIsWeb
          ? -9007199254740992 // -2^53
          : -1 << 63;

  // Returns a unique and monotonically increasing timestamp that represents
  // now.
  //
  // The value this method returns increments after each call.
  int _now() {
    final int now = _wallTime += 1;
    assert(_zOrderIndex == null || _zOrderIndex! < now);
    assert(_attachTarget?._zOrderIndex == null || _attachTarget!._zOrderIndex! < now);
    return now;
  }

  /// Show the overlay child of the [OverlayPortal] this controller is attached
  /// to, at the top of the target [Overlay].
  ///
  /// When there are more than one [OverlayPortal]s that target the same
  /// [Overlay], the overlay child of the last [OverlayPortal] to have called
  /// [show] appears at the top level, unobstructed.
  ///
  /// If [isShowing] is already true, calling this method brings the overlay
  /// child it controls to the top.
  ///
  /// This method should typically not be called while the widget tree is being
  /// rebuilt.
  void show() {
    final _OverlayPortalState? state = _attachTarget;
    if (state != null) {
      state.show(_now());
    } else {
      _zOrderIndex = _now();
    }
  }

  /// Hide the [OverlayPortal]'s overlay child.
  ///
  /// Once hidden, the overlay child will be removed from the widget tree the
  /// next time the widget tree rebuilds, and stateful widgets in the overlay
  /// child may lose states as a result.
  ///
  /// This method should typically not be called while the widget tree is being
  /// rebuilt.
  void hide() {
    final _OverlayPortalState? state = _attachTarget;
    if (state != null) {
      state.hide();
    } else {
      assert(_zOrderIndex != null);
      _zOrderIndex = null;
    }
  }

  /// Whether the associated [OverlayPortal] should build and show its overlay
  /// child, using its `overlayChildBuilder`.
  bool get isShowing {
    final _OverlayPortalState? state = _attachTarget;
    return state != null ? state._zOrderIndex != null : _zOrderIndex != null;
  }

  /// Convenience method for toggling the current [isShowing] status.
  ///
  /// This method should typically not be called while the widget tree is being
  /// rebuilt.
  void toggle() => isShowing ? hide() : show();

  @override
  String toString() {
    final String? debugLabel = _debugLabel;
    final String label = debugLabel == null ? '' : '($debugLabel)';
    final String isDetached = _attachTarget != null ? '' : ' DETACHED';
    return '${objectRuntimeType(this, 'OverlayPortalController')}$label$isDetached';
  }
}

/// A widget that renders its overlay child on an [Overlay].
///
/// The overlay child is initially hidden until [OverlayPortalController.show]
/// is called on the associated [controller]. The [OverlayPortal] uses
/// [overlayChildBuilder] to build its overlay child and renders it on the
/// specified [Overlay] as if it was inserted using an [OverlayEntry], while it
/// can depend on the same set of [InheritedWidget]s (such as [Theme]) that this
/// widget can depend on.
///
/// This widget requires an [Overlay] ancestor in the widget tree when its
/// overlay child is showing. The overlay child is rendered by the [Overlay]
/// ancestor, not by the widget itself. This allows the overlay child to float
/// above other widgets, independent of its position in the widget tree.
///
/// When [OverlayPortalController.hide] is called, the widget built using
/// [overlayChildBuilder] will be removed from the widget tree the next time the
/// widget rebuilds. Stateful descendants in the overlay child subtree may lose
/// states as a result.
///
/// {@tool dartpad}
/// This example uses an [OverlayPortal] to build a tooltip that becomes visible
/// when the user taps on the [child] widget. There's a [DefaultTextStyle] above
/// the [OverlayPortal] controlling the [TextStyle] of both the [child] widget
/// and the widget [overlayChildBuilder] builds, which isn't otherwise doable if
/// the tooltip was added as an [OverlayEntry].
///
/// ** See code in examples/api/lib/widgets/overlay/overlay_portal.0.dart **
/// {@end-tool}
///
/// ### Paint Order
///
/// In an [Overlay], an overlay child is painted after the [OverlayEntry]
/// associated with its [OverlayPortal] (that is, the [OverlayEntry] closest to
/// the [OverlayPortal] in the widget tree, which usually represents the
/// enclosing [Route]), and before the next [OverlayEntry].
///
/// When an [OverlayEntry] has multiple associated [OverlayPortal]s, the paint
/// order between their overlay children is the order in which
/// [OverlayPortalController.show] was called. The last [OverlayPortal] to have
/// called `show` gets to paint its overlay child in the foreground.
///
/// ### Semantics
///
/// The semantics subtree generated by the overlay child is considered attached
/// to [OverlayPortal] instead of the target [Overlay]. An [OverlayPortal]'s
/// semantics subtree can be dropped from the semantics tree due to invisibility
/// while the overlay child is still visible (for example, when the
/// [OverlayPortal] is completely invisible in a [ListView] but kept alive by
/// a [KeepAlive] widget). When this happens the semantics subtree generated by
/// the overlay child is also dropped, even if the overlay child is still visible
/// on screen.
///
/// {@template flutter.widgets.overlayPortalVsOverlayEntry}
/// ### Differences between [OverlayPortal] and [OverlayEntry]
///
/// The main difference between [OverlayEntry] and [OverlayPortal] is that
/// [OverlayEntry] builds its widget subtree as a child of the target [Overlay],
/// while [OverlayPortal] uses [OverlayPortal.overlayChildBuilder] to build a
/// child widget of itself. This allows [OverlayPortal]'s overlay child to depend
/// on the same set of [InheritedWidget]s as [OverlayPortal], and it's also
/// guaranteed that the overlay child will not outlive its [OverlayPortal].
///
/// On the other hand, [OverlayPortal]'s implementation is more complex. For
/// instance, it does a bit more work than a regular widget during global key
/// reparenting. If the content to be shown on the [Overlay] doesn't benefit
/// from being a part of [OverlayPortal]'s subtree, consider using an
/// [OverlayEntry] instead.
/// {@endtemplate}
///
/// See also:
///
///  * [OverlayEntry], an alternative API for inserting widgets into an
///    [Overlay].
///  * [Positioned], which can be used to size and position the overlay child in
///    relation to the target [Overlay]'s boundaries.
///  * [CompositedTransformFollower], which can be used to position the overlay
///    child in relation to the linked [CompositedTransformTarget] widget.
class OverlayPortal extends StatefulWidget {
  /// Creates an [OverlayPortal] that renders the widget [overlayChildBuilder]
  /// builds on the closest [Overlay] when [OverlayPortalController.show] is
  /// called.
  const OverlayPortal({
    super.key,
    required this.controller,
    required this.overlayChildBuilder,
    this.child,
  }) : _targetRootOverlay = false;

  /// Creates an [OverlayPortal] that renders the widget [overlayChildBuilder]
  /// builds on the root [Overlay] when [OverlayPortalController.show] is
  /// called.
  const OverlayPortal.targetsRootOverlay({
    super.key,
    required this.controller,
    required this.overlayChildBuilder,
    this.child,
  }) : _targetRootOverlay = true;

  /// The controller to show, hide and bring to top the overlay child.
  final OverlayPortalController controller;

  /// A [WidgetBuilder] used to build a widget below this widget in the tree,
  /// that renders on the closest [Overlay].
  ///
  /// The said widget will only be built and shown in the closest [Overlay] once
  /// [OverlayPortalController.show] is called on the associated [controller].
  /// It will be painted in front of the [OverlayEntry] closest to this widget
  /// in the widget tree (which is usually the enclosing [Route]).
  ///
  /// The built overlay child widget is inserted below this widget in the widget
  /// tree, allowing it to depend on [InheritedWidget]s above it, and be
  /// notified when the [InheritedWidget]s change.
  ///
  /// Unlike [child], the built overlay child can visually extend outside the
  /// bounds of this widget without being clipped, and receive hit-test events
  /// outside of this widget's bounds, as long as it does not extend outside of
  /// the [Overlay] on which it is rendered.
  final WidgetBuilder overlayChildBuilder;

  /// A widget below this widget in the tree.
  final Widget? child;

  final bool _targetRootOverlay;

  @override
  State<OverlayPortal> createState() => _OverlayPortalState();
}

class _OverlayPortalState extends State<OverlayPortal> {
  int? _zOrderIndex;
  // The location of the overlay child within the overlay. This object will be
  // used as the slot of the overlay child widget.
  //
  // The developer must call `show` to reveal the overlay so we can get a unique
  // timestamp of the user interaction for determining the z-index of the
  // overlay child in the overlay.
  //
  // Avoid invalidating the cache if possible, since the framework uses `==` to
  // compare slots, and _OverlayEntryLocation can't override that operator since
  // it's mutable. Changing slots can be relatively slow.
  bool _childModelMayHaveChanged = true;
  _OverlayEntryLocation? _locationCache;
  static bool _isTheSameLocation(_OverlayEntryLocation locationCache, _RenderTheaterMarker marker) {
    return locationCache._childModel == marker.overlayEntryWidgetState &&
        locationCache._theater == marker.theater;
  }

  _OverlayEntryLocation _getLocation(int zOrderIndex, bool targetRootOverlay) {
    final _OverlayEntryLocation? cachedLocation = _locationCache;
    late final _RenderTheaterMarker marker = _RenderTheaterMarker.of(
      context,
      targetRootOverlay: targetRootOverlay,
    );
    final bool isCacheValid =
        cachedLocation != null &&
        (!_childModelMayHaveChanged || _isTheSameLocation(cachedLocation, marker));
    _childModelMayHaveChanged = false;
    if (isCacheValid) {
      assert(cachedLocation._zOrderIndex == zOrderIndex);
      assert(cachedLocation._debugIsLocationValid());
      return cachedLocation;
    }
    // Otherwise invalidate the cache and create a new location.
    cachedLocation?._debugMarkLocationInvalid();
    final _OverlayEntryLocation newLocation = _OverlayEntryLocation(
      zOrderIndex,
      marker.overlayEntryWidgetState,
      marker.theater,
    );
    assert(newLocation._zOrderIndex == zOrderIndex);
    return _locationCache = newLocation;
  }

  @override
  void initState() {
    super.initState();
    _setupController(widget.controller);
  }

  void _setupController(OverlayPortalController controller) {
    assert(
      controller._attachTarget == null || controller._attachTarget == this,
      'Failed to attach $controller to $this. It is already attached to ${controller._attachTarget}.',
    );
    final int? controllerZOrderIndex = controller._zOrderIndex;
    final int? zOrderIndex = _zOrderIndex;
    if (zOrderIndex == null ||
        (controllerZOrderIndex != null && controllerZOrderIndex > zOrderIndex)) {
      _zOrderIndex = controllerZOrderIndex;
    }
    controller._zOrderIndex = null;
    controller._attachTarget = this;
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _childModelMayHaveChanged = true;
  }

  @override
  void didUpdateWidget(OverlayPortal oldWidget) {
    super.didUpdateWidget(oldWidget);
    _childModelMayHaveChanged =
        _childModelMayHaveChanged || oldWidget._targetRootOverlay != widget._targetRootOverlay;
    if (oldWidget.controller != widget.controller) {
      oldWidget.controller._attachTarget = null;
      _setupController(widget.controller);
    }
  }

  @override
  void dispose() {
    assert(widget.controller._attachTarget == this);
    widget.controller._attachTarget = null;
    _locationCache?._debugMarkLocationInvalid();
    _locationCache = null;
    super.dispose();
  }

  void show(int zOrderIndex) {
    assert(
      SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks,
      '${widget.controller.runtimeType}.show() should not be called during build.',
    );
    setState(() {
      _zOrderIndex = zOrderIndex;
    });
    _locationCache?._debugMarkLocationInvalid();
    _locationCache = null;
  }

  void hide() {
    assert(SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks);
    setState(() {
      _zOrderIndex = null;
    });
    _locationCache?._debugMarkLocationInvalid();
    _locationCache = null;
  }

  @override
  Widget build(BuildContext context) {
    final int? zOrderIndex = _zOrderIndex;
    if (zOrderIndex == null) {
      return _OverlayPortal(overlayLocation: null, overlayChild: null, child: widget.child);
    }
    return _OverlayPortal(
      overlayLocation: _getLocation(zOrderIndex, widget._targetRootOverlay),
      overlayChild: _DeferredLayout(child: Builder(builder: widget.overlayChildBuilder)),
      child: widget.child,
    );
  }
}

/// A location in an [Overlay].
///
/// An [_OverlayEntryLocation] determines the [Overlay] the associated
/// [OverlayPortal] should put its overlay child onto, as well as the overlay
/// child's paint order in relation to other contents painted on the [Overlay].
//
// An _OverlayEntryLocation is a cursor pointing to a location in a particular
// Overlay's child model, and provides methods to insert/remove/move a
// _RenderDeferredLayoutBox to/from its target _theater.
//
// The occupant (a `RenderBox`) will be painted above the associated
// [OverlayEntry], but below the [OverlayEntry] above that [OverlayEntry].
//
// Additionally, `_activate` and `_deactivate` are called when the overlay
// child's `_OverlayPortalElement` activates/deactivates (for instance, during
// global key reparenting).
// `_OverlayPortalElement` removes its overlay child's render object from the
// target `_RenderTheater` when it deactivates and puts it back on `activated`.
// These 2 methods can be used to "hide" a child in the child model without
// removing it, when the child is expensive/difficult to re-insert at the
// correct location on `activated`.
//
// ### Equality
//
// An `_OverlayEntryLocation` will be used as an Element's slot. These 3 parts
// uniquely identify a place in an overlay's child model:
// - _theater
// - _childModel (the OverlayEntry)
// - _zOrderIndex
//
// Since it can't implement operator== (it's mutable), the same `_OverlayEntryLocation`
// instance must not be used to represent more than one locations.
final class _OverlayEntryLocation extends LinkedListEntry<_OverlayEntryLocation> {
  _OverlayEntryLocation(this._zOrderIndex, this._childModel, this._theater);

  final int _zOrderIndex;
  final _OverlayEntryWidgetState _childModel;
  final _RenderTheater _theater;

  _RenderDeferredLayoutBox? _overlayChildRenderBox;
  void _addToChildModel(_RenderDeferredLayoutBox child) {
    assert(
      _overlayChildRenderBox == null,
      'Failed to add $child. This location ($this) is already occupied by $_overlayChildRenderBox.',
    );
    _overlayChildRenderBox = child;
    _childModel._add(this);
    _theater.markNeedsPaint();
    _theater.markNeedsCompositingBitsUpdate();
    _theater.markNeedsSemanticsUpdate();
  }

  void _removeFromChildModel(_RenderDeferredLayoutBox child) {
    assert(child == _overlayChildRenderBox);
    _overlayChildRenderBox = null;
    assert(_childModel._sortedTheaterSiblings?.contains(this) ?? false);
    _childModel._remove(this);
    _theater.markNeedsPaint();
    _theater.markNeedsCompositingBitsUpdate();
    _theater.markNeedsSemanticsUpdate();
  }

  void _addChild(_RenderDeferredLayoutBox child) {
    assert(_debugIsLocationValid());
    _addToChildModel(child);
    _theater._addDeferredChild(child);
    assert(child.parent == _theater);
  }

  void _removeChild(_RenderDeferredLayoutBox child) {
    // This call is allowed even when this location is disposed.
    _removeFromChildModel(child);
    _theater._removeDeferredChild(child);
    assert(child.parent == null);
  }

  void _moveChild(_RenderDeferredLayoutBox child, _OverlayEntryLocation fromLocation) {
    assert(fromLocation != this);
    assert(_debugIsLocationValid());
    final _RenderTheater fromTheater = fromLocation._theater;
    final _OverlayEntryWidgetState fromModel = fromLocation._childModel;

    if (fromTheater != _theater) {
      fromTheater._removeDeferredChild(child);
      _theater._addDeferredChild(child);
    }

    if (fromModel != _childModel || fromLocation._zOrderIndex != _zOrderIndex) {
      fromLocation._removeFromChildModel(child);
      _addToChildModel(child);
    }
  }

  void _activate(_RenderDeferredLayoutBox child) {
    // This call is allowed even when this location is invalidated.
    // See _OverlayPortalElement.activate.
    assert(_overlayChildRenderBox == null, '$_overlayChildRenderBox');
    _theater._addDeferredChild(child);
    _overlayChildRenderBox = child;
  }

  void _deactivate(_RenderDeferredLayoutBox child) {
    // This call is allowed even when this location is invalidated.
    _theater._removeDeferredChild(child);
    _overlayChildRenderBox = null;
  }

  // Throws a StateError if this location is already invalidated and shouldn't
  // be used as an OverlayPortal slot. Must be used in asserts.
  //
  // Generally, `assert(_debugIsLocationValid())` should be used to prevent
  // invalid accesses to an invalid `_OverlayEntryLocation` object. Exceptions
  // to this rule are _removeChild, _deactivate, which will be called when the
  // OverlayPortal is being removed from the widget tree and may use the
  // location information to perform cleanup tasks.
  //
  // Another exception is the _activate method which is called by
  // _OverlayPortalElement.activate. See the comment in _OverlayPortalElement.activate.
  bool _debugIsLocationValid() {
    if (_debugMarkLocationInvalidStackTrace == null) {
      return true;
    }
    throw StateError(
      '$this is already disposed. Stack trace: $_debugMarkLocationInvalidStackTrace',
    );
  }

  // The StackTrace of the first _debugMarkLocationInvalid call. It's only for
  // debugging purposes and the StackTrace will only be captured in debug builds.
  //
  // The effect of this method is not reversible. Once marked invalid, this
  // object can't be marked as valid again.
  StackTrace? _debugMarkLocationInvalidStackTrace;
  @mustCallSuper
  void _debugMarkLocationInvalid() {
    assert(_debugIsLocationValid());
    assert(() {
      _debugMarkLocationInvalidStackTrace = StackTrace.current;
      return true;
    }());
  }

  @override
  String toString() =>
      '${objectRuntimeType(this, '_OverlayEntryLocation')}[${shortHash(this)}] ${_debugMarkLocationInvalidStackTrace != null ? "(INVALID)" : ""}';
}

class _RenderTheaterMarker extends InheritedWidget {
  const _RenderTheaterMarker({
    required this.theater,
    required this.overlayEntryWidgetState,
    required super.child,
  });

  final _RenderTheater theater;
  final _OverlayEntryWidgetState overlayEntryWidgetState;

  @override
  bool updateShouldNotify(_RenderTheaterMarker oldWidget) {
    return oldWidget.theater != theater ||
        oldWidget.overlayEntryWidgetState != overlayEntryWidgetState;
  }

  static _RenderTheaterMarker of(BuildContext context, {bool targetRootOverlay = false}) {
    final _RenderTheaterMarker? marker;
    if (targetRootOverlay) {
      final InheritedElement? ancestor = _rootRenderTheaterMarkerOf(
        context.getElementForInheritedWidgetOfExactType<_RenderTheaterMarker>(),
      );
      assert(ancestor == null || ancestor.widget is _RenderTheaterMarker);
      marker =
          ancestor != null
              ? context.dependOnInheritedElement(ancestor) as _RenderTheaterMarker?
              : null;
    } else {
      marker = context.dependOnInheritedWidgetOfExactType<_RenderTheaterMarker>();
    }
    if (marker != null) {
      return marker;
    }
    throw FlutterError.fromParts(<DiagnosticsNode>[
      ErrorSummary('No Overlay widget found.'),
      ErrorDescription(
        '${context.widget.runtimeType} widgets require an Overlay widget ancestor.\n'
        'An overlay lets widgets float on top of other widget children.',
      ),
      ErrorHint(
        'To introduce an Overlay widget, you can either directly '
        'include one, or use a widget that contains an Overlay itself, '
        'such as a Navigator, WidgetApp, MaterialApp, or CupertinoApp.',
      ),
      ...context.describeMissingAncestor(expectedAncestorType: Overlay),
    ]);
  }

  static InheritedElement? _rootRenderTheaterMarkerOf(InheritedElement? theaterMarkerElement) {
    assert(theaterMarkerElement == null || theaterMarkerElement.widget is _RenderTheaterMarker);
    if (theaterMarkerElement == null) {
      return null;
    }
    InheritedElement? ancestor;
    theaterMarkerElement.visitAncestorElements((Element element) {
      ancestor = element.getElementForInheritedWidgetOfExactType<_RenderTheaterMarker>();
      return false;
    });
    return ancestor == null ? theaterMarkerElement : _rootRenderTheaterMarkerOf(ancestor);
  }
}

class _OverlayPortal extends RenderObjectWidget {
  /// Creates a widget that renders the given [overlayChild] in the [Overlay]
  /// specified by `overlayLocation`.
  ///
  /// The `overlayLocation` parameter must not be null when [overlayChild] is not
  /// null.
  _OverlayPortal({required this.overlayLocation, required this.overlayChild, required this.child})
    : assert(overlayChild == null || overlayLocation != null),
      assert(overlayLocation == null || overlayLocation._debugIsLocationValid());

  final Widget? overlayChild;

  /// A widget below this widget in the tree.
  final Widget? child;

  final _OverlayEntryLocation? overlayLocation;

  @override
  RenderObjectElement createElement() => _OverlayPortalElement(this);

  @override
  RenderObject createRenderObject(BuildContext context) => _RenderLayoutSurrogateProxyBox();
}

class _OverlayPortalElement extends RenderObjectElement {
  _OverlayPortalElement(_OverlayPortal super.widget);

  @override
  _RenderLayoutSurrogateProxyBox get renderObject =>
      super.renderObject as _RenderLayoutSurrogateProxyBox;

  Element? _overlayChild;
  Element? _child;

  @override
  void mount(Element? parent, Object? newSlot) {
    super.mount(parent, newSlot);
    final _OverlayPortal widget = this.widget as _OverlayPortal;
    _child = updateChild(_child, widget.child, null);
    _overlayChild = updateChild(_overlayChild, widget.overlayChild, widget.overlayLocation);
  }

  @override
  void update(_OverlayPortal newWidget) {
    super.update(newWidget);
    _child = updateChild(_child, newWidget.child, null);
    _overlayChild = updateChild(_overlayChild, newWidget.overlayChild, newWidget.overlayLocation);
  }

  @override
  void forgetChild(Element child) {
    // The _overlayChild Element does not have a key because the _DeferredLayout
    // widget does not take a Key, so only the regular _child can be taken
    // during global key reparenting.
    assert(child == _child);
    _child = null;
    super.forgetChild(child);
  }

  @override
  void visitChildren(ElementVisitor visitor) {
    final Element? child = _child;
    final Element? overlayChild = _overlayChild;
    if (child != null) {
      visitor(child);
    }
    if (overlayChild != null) {
      visitor(overlayChild);
    }
  }

  @override
  void activate() {
    super.activate();
    final _RenderDeferredLayoutBox? box = _overlayChild?.renderObject as _RenderDeferredLayoutBox?;
    if (box != null) {
      assert(!box.attached);
      assert(renderObject._deferredLayoutChild == box);
      // updateChild has not been called at this point so the RenderTheater in
      // the overlay location could be detached. Adding children to a detached
      // RenderObject is still allowed however this isn't the most efficient.
      (_overlayChild!.slot! as _OverlayEntryLocation)._activate(box);
    }
  }

  @override
  void deactivate() {
    // Instead of just detaching the render objects, removing them from the
    // render subtree entirely. This is a workaround for the
    // !renderObject.attached assert in the `super.deactivate()` method.
    final _RenderDeferredLayoutBox? box = _overlayChild?.renderObject as _RenderDeferredLayoutBox?;
    if (box != null) {
      (_overlayChild!.slot! as _OverlayEntryLocation)._deactivate(box);
    }
    super.deactivate();
  }

  @override
  void insertRenderObjectChild(RenderBox child, _OverlayEntryLocation? slot) {
    assert(child.parent == null, "$child's parent is not null: ${child.parent}");
    if (slot != null) {
      renderObject._deferredLayoutChild = child as _RenderDeferredLayoutBox;
      slot._addChild(child);
      renderObject.markNeedsSemanticsUpdate();
    } else {
      renderObject.child = child;
    }
  }

  // The [_DeferredLayout] widget does not have a key so there will be no
  // reparenting between _overlayChild and _child, thus the non-null-typed slots.
  @override
  void moveRenderObjectChild(
    _RenderDeferredLayoutBox child,
    _OverlayEntryLocation oldSlot,
    _OverlayEntryLocation newSlot,
  ) {
    assert(newSlot._debugIsLocationValid());
    newSlot._moveChild(child, oldSlot);
    renderObject.markNeedsSemanticsUpdate();
  }

  @override
  void removeRenderObjectChild(RenderBox child, _OverlayEntryLocation? slot) {
    if (slot == null) {
      renderObject.child = null;
      return;
    }
    assert(renderObject._deferredLayoutChild == child);
    slot._removeChild(child as _RenderDeferredLayoutBox);
    renderObject._deferredLayoutChild = null;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Element>('child', _child, defaultValue: null));
    properties.add(DiagnosticsProperty<Element>('overlayChild', _overlayChild, defaultValue: null));
    properties.add(
      DiagnosticsProperty<Object>('overlayLocation', _overlayChild?.slot, defaultValue: null),
    );
  }
}

class _DeferredLayout extends SingleChildRenderObjectWidget {
  const _DeferredLayout({
    // This widget must not be given a key: we currently do not support
    // reparenting between the overlayChild and child.
    required Widget child,
  }) : super(child: child);

  _RenderLayoutSurrogateProxyBox getLayoutParent(BuildContext context) {
    return context.findAncestorRenderObjectOfType<_RenderLayoutSurrogateProxyBox>()!;
  }

  @override
  _RenderDeferredLayoutBox createRenderObject(BuildContext context) {
    final _RenderLayoutSurrogateProxyBox parent = getLayoutParent(context);
    final _RenderDeferredLayoutBox renderObject = _RenderDeferredLayoutBox(parent);
    parent._deferredLayoutChild = renderObject;
    return renderObject;
  }

  @override
  void updateRenderObject(BuildContext context, _RenderDeferredLayoutBox renderObject) {
    assert(renderObject._layoutSurrogate == getLayoutParent(context));
    assert(getLayoutParent(context)._deferredLayoutChild == renderObject);
  }
}

// A `RenderProxyBox` that defers its layout until its `_layoutSurrogate` (which
// is not necessarily an ancestor of this RenderBox, but shares at least one
// `_RenderTheater` ancestor with this RenderBox) is laid out.
//
// This `RenderObject` must be a child of a `_RenderTheater`. It guarantees that:
//
// 1. It's a relayout boundary, so calling `markNeedsLayout` on it never dirties
//    its `_RenderTheater`.
//
// 2. Its `layout` implementation is overridden such that `performLayout` does
//    not do anything when its called from `layout`, preventing the parent
//    `_RenderTheater` from laying out this subtree prematurely (but this
//    `RenderObject` may still be resized). Instead, `markNeedsLayout` will be
//    called from within `layout` to schedule a layout update for this relayout
//    boundary when needed.
//
// 3. When invoked from `PipelineOwner.flushLayout`, or
//    `_layoutSurrogate.performLayout`, this `RenderObject` behaves like an
//    `Overlay` that has only one entry.
final class _RenderDeferredLayoutBox extends RenderProxyBox
    with _RenderTheaterMixin, LinkedListEntry<_RenderDeferredLayoutBox> {
  _RenderDeferredLayoutBox(this._layoutSurrogate);

  StackParentData get stackParentData => parentData! as StackParentData;
  final _RenderLayoutSurrogateProxyBox _layoutSurrogate;

  @override
  Iterable<RenderBox> _childrenInPaintOrder() {
    final RenderBox? child = this.child;
    return child == null
        ? const Iterable<RenderBox>.empty()
        : Iterable<RenderBox>.generate(1, (int i) => child);
  }

  @override
  Iterable<RenderBox> _childrenInHitTestOrder() => _childrenInPaintOrder();

  @override
  _RenderTheater get theater {
    final RenderObject? parent = this.parent;
    return parent is _RenderTheater
        ? parent
        : throw FlutterError('$parent of $this is not a _RenderTheater');
  }

  @override
  void redepthChildren() {
    _layoutSurrogate.redepthChild(this);
    super.redepthChildren();
  }

  @override
  bool get sizedByParent => true;

  bool _needsLayout = true;
  @override
  void markNeedsLayout() {
    _needsLayout = true;
    super.markNeedsLayout();
  }

  @override
  RenderObject? get semanticsParent => _layoutSurrogate;

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    if (child == null) {
      return null;
    }
    return _RenderTheaterMixin.baselineForChild(
      child,
      constraints.biggest,
      constraints,
      theater._resolvedAlignment,
      baseline,
    );
  }

  @override
  RenderObject? get debugLayoutParent => _layoutSurrogate;

  void layoutByLayoutSurrogate() {
    assert(!_theaterDoingThisLayout);
    final _RenderTheater? theater = parent as _RenderTheater?;
    if (theater == null || !attached) {
      assert(false, '$this is not attached to parent');
      return;
    }
    if (theater._layingOutSizeDeterminingChild) {
      theater.invokeLayoutCallback((BoxConstraints constraints) {
        markNeedsLayout();
      });
    } else {
      final BoxConstraints theaterConstraints = theater.constraints;
      final Size boxSize =
          theaterConstraints.biggest.isFinite
              ? theaterConstraints.biggest
              // Accessing the theater's size is only unsafe if it is laying out the
              // size-determining child.
              : theater.size;
      super.layout(BoxConstraints.tight(boxSize));
    }
  }

  bool _theaterDoingThisLayout = false;
  @override
  void layout(Constraints constraints, {bool parentUsesSize = false}) {
    assert(_needsLayout == debugNeedsLayout);
    // Only _RenderTheater calls this implementation.
    assert(parent != null);
    final bool scheduleDeferredLayout = _needsLayout || this.constraints != constraints;
    assert(!_theaterDoingThisLayout);
    _theaterDoingThisLayout = true;
    super.layout(constraints, parentUsesSize: parentUsesSize);
    assert(_theaterDoingThisLayout);
    _theaterDoingThisLayout = false;
    _needsLayout = false;
    assert(!debugNeedsLayout);
    if (scheduleDeferredLayout) {
      final _RenderTheater parent = this.parent! as _RenderTheater;
      // Invoking markNeedsLayout as a layout callback allows this node to be
      // merged back to the `PipelineOwner`'s dirty list in the right order, if
      // it's not already dirty. Otherwise this may cause some dirty descendants
      // to performLayout a second time.
      parent.invokeLayoutCallback((BoxConstraints constraints) {
        markNeedsLayout();
      });
    }
  }

  @override
  void performResize() {
    size = constraints.biggest;
  }

  bool _debugMutationsLocked = false;
  @override
  void performLayout() {
    assert(!_debugMutationsLocked);
    if (_theaterDoingThisLayout) {
      _needsLayout = false;
      return;
    }
    assert(() {
      _debugMutationsLocked = true;
      return true;
    }());
    // This method is directly being invoked from `PipelineOwner.flushLayout`,
    // or from `_layoutSurrogate`'s performLayout.
    assert(parent != null);
    final RenderBox? child = this.child;
    if (child == null) {
      _needsLayout = false;
      return;
    }
    assert(constraints.isTight);
    layoutChild(child, constraints);
    assert(() {
      _debugMutationsLocked = false;
      return true;
    }());
    _needsLayout = false;
  }

  @override
  void applyPaintTransform(RenderBox child, Matrix4 transform) {
    final BoxParentData childParentData = child.parentData! as BoxParentData;
    final Offset offset = childParentData.offset;
    transform.translate(offset.dx, offset.dy);
  }
}

// A RenderProxyBox that makes sure its `deferredLayoutChild` has a greater
// depth than itself.
class _RenderLayoutSurrogateProxyBox extends RenderProxyBox {
  _RenderDeferredLayoutBox? _deferredLayoutChild;

  @override
  void redepthChildren() {
    super.redepthChildren();
    final _RenderDeferredLayoutBox? child = _deferredLayoutChild;
    // If child is not attached, this method will be invoked by child's real
    // parent when it's attached.
    if (child != null && child.attached) {
      assert(child.attached);
      redepthChild(child);
    }
  }

  @override
  void performLayout() {
    super.performLayout();
    // Try to layout `_deferredLayoutChild` here now that its configuration
    // and constraints are up-to-date. Additionally, during the very first
    // layout, this makes sure that _deferredLayoutChild is reachable via tree
    // walk.
    _deferredLayoutChild?.layoutByLayoutSurrogate();
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    super.visitChildrenForSemantics(visitor);
    final _RenderDeferredLayoutBox? deferredChild = _deferredLayoutChild;
    if (deferredChild != null) {
      visitor(deferredChild);
    }
  }
}
