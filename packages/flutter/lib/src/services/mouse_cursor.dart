// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter/rendering.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';

import 'system_channels.dart';

export 'package:flutter/foundation.dart' show DiagnosticLevel, DiagnosticPropertiesBuilder;
export 'package:flutter/gestures.dart' show PointerEvent;

/// Maintains the state of mouse cursors and manages how cursors are searched
/// for.
///
/// This is typically created as a global singleton and owned by [MouseTracker].
class MouseCursorManager {
  /// Create a [MouseCursorManager] by specifying the fallback cursor.
  ///
  /// The `fallbackMouseCursor` must not be [MouseCursor.defer] (typically
  /// [SystemMouseCursors.basic]).
  MouseCursorManager(this.fallbackMouseCursor) : assert(fallbackMouseCursor != MouseCursor.defer);

  /// The mouse cursor to use if all cursor candidates choose to defer.
  ///
  /// See also:
  ///
  ///  * [MouseCursor.defer], the mouse cursor object to use to defer.
  final MouseCursor fallbackMouseCursor;

  /// Returns the active mouse cursor of a device.
  ///
  /// The return value is the last [MouseCursor] activated onto this
  /// device, even if the activation failed.
  ///
  /// Only valid when asserts are enabled. In release builds, always returns
  /// null.
  MouseCursor? debugDeviceActiveCursor(int device) {
    MouseCursor? result;
    assert(() {
      result = _lastSession[device]?.cursor;
      return true;
    }());
    return result;
  }

  final Map<int, MouseCursorSession> _lastSession = <int, MouseCursorSession>{};

  /// Handles the changes that cause a pointer device to have a new list of mouse
  /// cursor candidates.
  ///
  /// This change can be caused by a pointer event, in which case
  /// `triggeringEvent` should not be null, or by other changes, such as when a
  /// widget has moved under a still mouse, which is detected after the current
  /// frame is complete. In either case, `cursorCandidates` should be the list of
  /// cursors at the location of the mouse in hit-test order.
  void handleDeviceCursorUpdate(
    int device,
    PointerEvent? triggeringEvent,
    Iterable<MouseCursor> cursorCandidates,
  ) {
    if (triggeringEvent is PointerRemovedEvent) {
      _lastSession.remove(device);
      return;
    }

    final MouseCursorSession? lastSession = _lastSession[device];
    final MouseCursor nextCursor =
        _DeferringMouseCursor.firstNonDeferred(cursorCandidates) ?? fallbackMouseCursor;
    assert(nextCursor is! _DeferringMouseCursor);
    if (lastSession?.cursor == nextCursor) {
      return;
    }

    final MouseCursorSession nextSession = nextCursor.createSession(device);
    _lastSession[device] = nextSession;

    lastSession?.dispose();
    nextSession.activate();
  }
}

/// Manages the duration that a pointing device should display a specific mouse
/// cursor.
///
/// While [MouseCursor] classes describe the kind of cursors, [MouseCursorSession]
/// classes represents a continuous use of the cursor on a pointing device. The
/// [MouseCursorSession] classes can be stateful. For example, a cursor that
/// needs to load resources might want to set a temporary cursor first, then
/// switch to the correct cursor after the load is completed.
///
/// A [MouseCursorSession] has the following lifecycle:
///
///  * When a pointing device should start displaying a cursor, [MouseTracker]
///    creates a session by calling [MouseCursor.createSession] on the target
///    cursor, and stores it in a table associated with the device.
///  * [MouseTracker] then immediately calls the session's [activate], where the
///    session should fetch resources and make system calls.
///  * When the pointing device should start displaying a different cursor,
///    [MouseTracker] calls [dispose] on this session. After [dispose], this session
///    will no longer be used in the future.
abstract class MouseCursorSession {
  /// Create a session.
  MouseCursorSession(this.cursor, this.device);

  /// The cursor that created this session.
  final MouseCursor cursor;

  /// The device ID of the pointing device.
  final int device;

  /// Override this method to do the work of changing the cursor of the device.
  ///
  /// Called right after this session is created.
  ///
  /// This method has full control over the cursor until the [dispose] call, and
  /// can make system calls to change the pointer cursor as many times as
  /// necessary (usually through [SystemChannels.mouseCursor]). It can also
  /// collect resources, and store the result in this object.
  @protected
  Future<void> activate();

  /// Called when device stops displaying the cursor.
  ///
  /// After this call, this session instance will no longer be used in the
  /// future.
  ///
  /// When implementing this method in subclasses, you should release resources
  /// and prevent [activate] from causing side effects after disposal.
  @protected
  void dispose();
}

/// An interface for mouse cursor definitions.
///
/// A mouse cursor is a graphical image on the screen that echoes the movement
/// of a pointing device, such as a mouse or a stylus. A [MouseCursor] object
/// defines a kind of mouse cursor, such as an arrow, a pointing hand, or an
/// I-beam.
///
/// During the painting phase, [MouseCursor] objects are assigned to regions on
/// the screen via annotations. Later during a device update (e.g. when a mouse
/// moves), [MouseTracker] finds the _active cursor_ of each mouse device, which
/// is the front-most region associated with the position of each mouse cursor,
/// or defaults to [SystemMouseCursors.basic] if no cursors are associated with
/// the position. [MouseTracker] changes the cursor of the pointer if the new
/// active cursor is different from the previous active cursor, whose effect is
/// defined by the session created by [createSession].
///
/// ## Cursor classes
///
/// A [SystemMouseCursor] is a cursor that is natively supported by the
/// platform that the program is running on. All supported system mouse cursors
/// are enumerated in [SystemMouseCursors].
///
/// ## Using cursors
///
/// A [MouseCursor] object is used by being assigned to a [MouseRegion] or
/// another widget that exposes the [MouseRegion] API, such as
/// [InkResponse.mouseCursor].
///
/// {@tool dartpad}
/// This sample creates a rectangular region that is wrapped by a [MouseRegion]
/// with a system mouse cursor. The mouse pointer becomes an I-beam when
/// hovering over the region.
///
/// ** See code in examples/api/lib/services/mouse_cursor/mouse_cursor.0.dart **
/// {@end-tool}
///
/// Assigning regions with mouse cursors on platforms that do not support mouse
/// cursors, or when there are no mice connected, will have no effect.
///
/// ## Related classes
///
/// [MouseCursorSession] represents the duration when a pointing device displays
/// a cursor, and defines the states and behaviors of the cursor. Every mouse
/// cursor class usually has a corresponding [MouseCursorSession] class.
///
/// [MouseCursorManager] is a class that adds the feature of changing
/// cursors to [MouseTracker], which tracks the relationship between mouse
/// devices and annotations. [MouseCursorManager] is usually used as a part
/// of [MouseTracker].
@immutable
abstract class MouseCursor with Diagnosticable {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const MouseCursor();

  /// Associate a pointing device to this cursor.
  ///
  /// A mouse cursor class usually has a corresponding [MouseCursorSession]
  /// class, and instantiates such class in this method.
  ///
  /// This method is called each time a pointing device starts displaying this
  /// cursor. A given cursor can be displayed by multiple devices at the same
  /// time, in which case this method will be called separately for each device.
  @protected
  @factory
  MouseCursorSession createSession(int device);

  /// A very short description of the mouse cursor.
  ///
  /// The [debugDescription] should be a few words that can describe this cursor
  /// to make debug information more readable. It is returned as the [toString]
  /// when the diagnostic level is at or above [DiagnosticLevel.info].
  ///
  /// The [debugDescription] must not be empty.
  String get debugDescription;

  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.info}) {
    final String debugDescription = this.debugDescription;
    if (minLevel.index >= DiagnosticLevel.info.index) {
      return debugDescription;
    }
    return super.toString(minLevel: minLevel);
  }

  /// A special class that indicates that the region with this cursor defers the
  /// choice of cursor to the next region behind it.
  ///
  /// When an event occurs, [MouseTracker] will update each pointer's cursor by
  /// finding the list of regions that contain the pointer's location, from front
  /// to back in hit-test order. The pointer's cursor will be the first cursor in
  /// the list that is not a [MouseCursor.defer].
  static const MouseCursor defer = _DeferringMouseCursor._();

  /// A special value that doesn't change cursor by itself, but make a region
  /// that blocks other regions behind it from changing the cursor.
  ///
  /// When a pointer enters a region with a cursor of [uncontrolled], the pointer
  /// retains its previous cursor and keeps so until it moves out of the region.
  /// Technically, this region absorb the mouse cursor hit test without changing
  /// the pointer's cursor.
  ///
  /// This is useful in a region that displays a platform view, which let the
  /// operating system handle pointer events and change cursors accordingly. To
  /// achieve this, the region's cursor must not be any Flutter cursor, since
  /// that might overwrite the system request upon pointer entering; the cursor
  /// must not be null either, since that allows the widgets behind the region to
  /// change cursors.
  static const MouseCursor uncontrolled = _NoopMouseCursor._();
}

class _DeferringMouseCursor extends MouseCursor {
  const _DeferringMouseCursor._();

  @override
  MouseCursorSession createSession(int device) {
    assert(false, '_DeferringMouseCursor can not create a session');
    throw UnimplementedError();
  }

  @override
  String get debugDescription => 'defer';

  /// Returns the first cursor that is not a [MouseCursor.defer].
  static MouseCursor? firstNonDeferred(Iterable<MouseCursor> cursors) {
    for (final MouseCursor cursor in cursors) {
      if (cursor != MouseCursor.defer) {
        return cursor;
      }
    }
    return null;
  }
}

class _NoopMouseCursorSession extends MouseCursorSession {
  _NoopMouseCursorSession(_NoopMouseCursor super.cursor, super.device);

  @override
  Future<void> activate() async {
    /* Nothing */
  }

  @override
  void dispose() {
    /* Nothing */
  }
}

/// A mouse cursor that doesn't change the cursor when activated.
///
/// Although setting a region's cursor to [_NoopMouseCursor] doesn't change the
/// cursor, it blocks regions behind it from changing the cursor, in contrast to
/// setting the cursor to null. More information about the usage of this class
/// can be found at [MouseCursor.uncontrolled].
///
/// To use this class, use [MouseCursor.uncontrolled]. Directly
/// instantiating this class is not allowed.
class _NoopMouseCursor extends MouseCursor {
  // Application code shouldn't directly instantiate this class, since its only
  // instance is accessible at [SystemMouseCursors.releaseControl].
  const _NoopMouseCursor._();

  @override
  @protected
  _NoopMouseCursorSession createSession(int device) => _NoopMouseCursorSession(this, device);

  @override
  String get debugDescription => 'uncontrolled';
}

class _SystemMouseCursorSession extends MouseCursorSession {
  _SystemMouseCursorSession(SystemMouseCursor super.cursor, super.device);

  @override
  SystemMouseCursor get cursor => super.cursor as SystemMouseCursor;

  @override
  Future<void> activate() {
    return SystemChannels.mouseCursor.invokeMethod<void>('activateSystemCursor', <String, dynamic>{
      'device': device,
      'kind': cursor.kind,
    });
  }

  @override
  void dispose() {
    /* Nothing */
  }
}

/// A mouse cursor that is natively supported on the platform that the
/// application is running on.
///
/// System cursors can be used without external resources, and their appearances
/// match the experience of native apps. Examples of system cursors are a
/// pointing arrow, a pointing hand, a double arrow for resizing, or a text
/// I-beam, etc.
///
/// An instance of [SystemMouseCursor] refers to one cursor from each platform
/// that represents the same concept, such as being text, being clickable,
/// or being a forbidden operation. Since the set of system cursors supported by
/// each platform varies, multiple instances can correspond to the same system
/// cursor.
///
/// Each cursor is noted with its corresponding native cursors on each platform:
///
///  * Android: API name in Java
///  * Web: CSS cursor
///  * Windows: Win32 API
///  * Windows UWP: WinRT API, `winrt::Windows::UI::Core::CoreCursorType`
///  * Linux: GDK, `gdk_cursor_new_from_name`
///  * macOS: API name in Objective C
///
/// If the platform that the application is running on is not listed for a cursor,
/// using this cursor falls back to [SystemMouseCursors.basic].
///
/// [SystemMouseCursors] enumerates the complete set of system cursors supported
/// by Flutter, which are hard-coded in the engine. Therefore, manually
/// instantiating this class is not supported.
class SystemMouseCursor extends MouseCursor {
  // Application code shouldn't directly instantiate system mouse cursors, since
  // the supported system cursors are enumerated in [SystemMouseCursors].
  const SystemMouseCursor._({required this.kind});

  /// A string that identifies the kind of the cursor.
  ///
  /// The interpretation of [kind] is platform-dependent.
  final String kind;

  @override
  String get debugDescription => '${objectRuntimeType(this, 'SystemMouseCursor')}($kind)';

  @override
  @protected
  MouseCursorSession createSession(int device) => _SystemMouseCursorSession(this, device);

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SystemMouseCursor && other.kind == kind;
  }

  @override
  int get hashCode => kind.hashCode;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<String>('kind', kind, level: DiagnosticLevel.debug));
  }
}

/// A collection of system [MouseCursor]s.
///
/// System cursors are standard mouse cursors that are provided by the current
/// platform. They don't require external resources.
///
/// [SystemMouseCursors] is a superset of the system cursors of every platform
/// that Flutter supports, therefore some of these objects might map to the same
/// result, or fallback to the [basic] arrow. This mapping is defined by the
/// Flutter engine.
///
/// The cursors should be named based on the cursors' use cases instead of their
/// appearance, because different platforms might (although not commonly) use
/// different shapes for the same use case.
abstract final class SystemMouseCursors {
  // The mapping in this class must be kept in sync with the following files in
  // the engine:
  //
  // * Android: shell/platform/android/io/flutter/plugin/mouse/MouseCursorPlugin.java
  // * Web: lib/web_ui/lib/src/engine/mouse_cursor.dart
  // * Windows: shell/platform/windows/win32_flutter_window_win32.cc
  // * Windows UWP: shell/platform/windows/win32_flutter_window_winuwp.cc
  // * Linux: shell/platform/linux/fl_mouse_cursor_plugin.cc
  // * macOS: shell/platform/darwin/macos/framework/Source/FlutterMouseCursorPlugin.mm

  /// Hide the cursor.
  ///
  /// Any cursor other than [none] or [MouseCursor.uncontrolled] unhides the
  /// cursor.
  static const SystemMouseCursor none = SystemMouseCursor._(kind: 'none');

  // STATUS

  /// The platform-dependent basic cursor.
  ///
  /// Typically the shape of an arrow.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_DEFAULT, TYPE_ARROW
  ///  * Web: default
  ///  * Windows: IDC_ARROW
  ///  * Windows UWP: CoreCursorType::Arrow
  ///  * Linux: default
  ///  * macOS: arrowCursor
  static const SystemMouseCursor basic = SystemMouseCursor._(kind: 'basic');

  /// A cursor that emphasizes an element being clickable, such as a hyperlink.
  ///
  /// Typically the shape of a pointing hand.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_HAND
  ///  * Web: pointer
  ///  * Windows: IDC_HAND
  ///  * Windows UWP: CoreCursorType::Hand
  ///  * Linux: pointer
  ///  * macOS: pointingHandCursor
  static const SystemMouseCursor click = SystemMouseCursor._(kind: 'click');

  /// A cursor indicating an operation that will not be carried out.
  ///
  /// Typically the shape of a circle with a diagonal line. May fall back to
  /// [noDrop].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_NO_DROP
  ///  * Web: not-allowed
  ///  * Windows: IDC_NO
  ///  * Windows UWP: CoreCursorType::UniversalNo
  ///  * Linux: not-allowed
  ///  * macOS: operationNotAllowedCursor
  ///
  /// See also:
  ///
  ///  * [noDrop], which indicates somewhere that the current item may not be
  ///    dropped.
  static const SystemMouseCursor forbidden = SystemMouseCursor._(kind: 'forbidden');

  /// A cursor indicating the status that the program is busy and therefore
  /// can not be interacted with.
  ///
  /// Typically the shape of an hourglass or a watch.
  ///
  /// This cursor is not available as a system cursor on macOS. Although macOS
  /// displays a "spinning ball" cursor when busy, it's handled by the OS and not
  /// exposed for applications to choose.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_WAIT
  ///  * Windows: IDC_WAIT
  ///  * Web: wait
  ///  * Linux: wait
  ///
  /// See also:
  ///
  ///  * [progress], which is similar to [wait] but the program can still be
  ///    interacted with.
  static const SystemMouseCursor wait = SystemMouseCursor._(kind: 'wait');

  /// A cursor indicating the status that the program is busy but can still be
  /// interacted with.
  ///
  /// Typically the shape of an arrow with an hourglass or a watch at the corner.
  /// Does *not* fall back to [wait] if unavailable.
  ///
  /// Corresponds to:
  ///
  ///  * Web: progress
  ///  * Windows: IDC_APPSTARTING
  ///  * Linux: progress
  ///
  /// See also:
  ///
  ///  * [wait], which is similar to [progress] but the program can not be
  ///    interacted with.
  static const SystemMouseCursor progress = SystemMouseCursor._(kind: 'progress');

  /// A cursor indicating somewhere the user can trigger a context menu.
  ///
  /// Typically the shape of an arrow with a small menu at the corner.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_CONTEXT_MENU
  ///  * Web: context-menu
  ///  * Linux: context-menu
  ///  * macOS: contextualMenuCursor
  static const SystemMouseCursor contextMenu = SystemMouseCursor._(kind: 'contextMenu');

  /// A cursor indicating help information.
  ///
  /// Typically the shape of a question mark, or an arrow therewith.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_HELP
  ///  * Windows: IDC_HELP
  ///  * Windows UWP: CoreCursorType::Help
  ///  * Web: help
  ///  * Linux: help
  static const SystemMouseCursor help = SystemMouseCursor._(kind: 'help');

  // SELECTION

  /// A cursor indicating selectable text.
  ///
  /// Typically the shape of a capital I.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_TEXT
  ///  * Web: text
  ///  * Windows: IDC_IBEAM
  ///  * Windows UWP: CoreCursorType::IBeam
  ///  * Linux: text
  ///  * macOS: IBeamCursor
  static const SystemMouseCursor text = SystemMouseCursor._(kind: 'text');

  /// A cursor indicating selectable vertical text.
  ///
  /// Typically the shape of a capital I rotated to be horizontal. May fall back
  /// to [text].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_VERTICAL_TEXT
  ///  * Web: vertical-text
  ///  * Linux: vertical-text
  ///  * macOS: IBeamCursorForVerticalLayout
  static const SystemMouseCursor verticalText = SystemMouseCursor._(kind: 'verticalText');

  /// A cursor indicating selectable table cells.
  ///
  /// Typically the shape of a hollow plus sign.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_CELL
  ///  * Web: cell
  ///  * Linux: cell
  static const SystemMouseCursor cell = SystemMouseCursor._(kind: 'cell');

  /// A cursor indicating precise selection, such as selecting a pixel in a
  /// bitmap.
  ///
  /// Typically the shape of a crosshair.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_CROSSHAIR
  ///  * Web: crosshair
  ///  * Windows: IDC_CROSS
  ///  * Windows UWP: CoreCursorType::Cross
  ///  * Linux: crosshair
  ///  * macOS: crosshairCursor
  static const SystemMouseCursor precise = SystemMouseCursor._(kind: 'precise');

  // DRAG-AND-DROP

  /// A cursor indicating moving something.
  ///
  /// Typically the shape of four-way arrow. May fall back to [allScroll].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_ALL_SCROLL
  ///  * Windows: IDC_SIZEALL
  ///  * Windows UWP: CoreCursorType::SizeAll
  ///  * Web: move
  ///  * Linux: move
  static const SystemMouseCursor move = SystemMouseCursor._(kind: 'move');

  /// A cursor indicating something that can be dragged.
  ///
  /// Typically the shape of an open hand.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_GRAB
  ///  * Web: grab
  ///  * Linux: grab
  ///  * macOS: openHandCursor
  static const SystemMouseCursor grab = SystemMouseCursor._(kind: 'grab');

  /// A cursor indicating something that is being dragged.
  ///
  /// Typically the shape of a closed hand.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_GRABBING
  ///  * Web: grabbing
  ///  * Linux: grabbing
  ///  * macOS: closedHandCursor
  static const SystemMouseCursor grabbing = SystemMouseCursor._(kind: 'grabbing');

  /// A cursor indicating somewhere that the current item may not be dropped.
  ///
  /// Typically the shape of a hand with a [forbidden] sign at the corner. May
  /// fall back to [forbidden].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_NO_DROP
  ///  * Web: no-drop
  ///  * Windows: IDC_NO
  ///  * Windows UWP: CoreCursorType::UniversalNo
  ///  * Linux: no-drop
  ///  * macOS: operationNotAllowedCursor
  ///
  /// See also:
  ///
  ///  * [forbidden], which indicates an action that will not be carried out.
  static const SystemMouseCursor noDrop = SystemMouseCursor._(kind: 'noDrop');

  /// A cursor indicating that the current operation will create an alias of, or
  /// a shortcut of the item.
  ///
  /// Typically the shape of an arrow with a shortcut icon at the corner.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_ALIAS
  ///  * Web: alias
  ///  * Linux: alias
  ///  * macOS: dragLinkCursor
  static const SystemMouseCursor alias = SystemMouseCursor._(kind: 'alias');

  /// A cursor indicating that the current operation will copy the item.
  ///
  /// Typically the shape of an arrow with a boxed plus sign at the corner.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_COPY
  ///  * Web: copy
  ///  * Linux: copy
  ///  * macOS: dragCopyCursor
  static const SystemMouseCursor copy = SystemMouseCursor._(kind: 'copy');

  /// A cursor indicating that the current operation will result in the
  /// disappearance of the item.
  ///
  /// Typically the shape of an arrow with a cloud of smoke at the corner.
  ///
  /// Corresponds to:
  ///
  ///  * macOS: disappearingItemCursor
  static const SystemMouseCursor disappearing = SystemMouseCursor._(kind: 'disappearing');

  // RESIZING AND SCROLLING

  /// A cursor indicating scrolling in any direction.
  ///
  /// Typically the shape of a dot surrounded by 4 arrows.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_ALL_SCROLL
  ///  * Windows: IDC_SIZEALL
  ///  * Windows UWP: CoreCursorType::SizeAll
  ///  * Web: all-scroll
  ///  * Linux: all-scroll
  ///
  /// See also:
  ///
  ///  * [move], which indicates moving in any direction.
  static const SystemMouseCursor allScroll = SystemMouseCursor._(kind: 'allScroll');

  /// A cursor indicating resizing an object bidirectionally from its left or
  /// right edge.
  ///
  /// Typically the shape of a bidirectional arrow pointing left and right.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_HORIZONTAL_DOUBLE_ARROW
  ///  * Web: ew-resize
  ///  * Windows: IDC_SIZEWE
  ///  * Windows UWP: CoreCursorType::SizeWestEast
  ///  * Linux: ew-resize
  ///  * macOS: resizeLeftRightCursor
  static const SystemMouseCursor resizeLeftRight = SystemMouseCursor._(kind: 'resizeLeftRight');

  /// A cursor indicating resizing an object bidirectionally from its top or
  /// bottom edge.
  ///
  /// Typically the shape of a bidirectional arrow pointing up and down.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_VERTICAL_DOUBLE_ARROW
  ///  * Web: ns-resize
  ///  * Windows: IDC_SIZENS
  ///  * Windows UWP: CoreCursorType::SizeNorthSouth
  ///  * Linux: ns-resize
  ///  * macOS: resizeUpDownCursor
  static const SystemMouseCursor resizeUpDown = SystemMouseCursor._(kind: 'resizeUpDown');

  /// A cursor indicating resizing an object bidirectionally from its top left or
  /// bottom right corner.
  ///
  /// Typically the shape of a bidirectional arrow pointing upper left and lower right.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_TOP_LEFT_DIAGONAL_DOUBLE_ARROW
  ///  * Web: nwse-resize
  ///  * Windows: IDC_SIZENWSE
  ///  * Windows UWP: CoreCursorType::SizeNorthwestSoutheast
  ///  * Linux: nwse-resize
  static const SystemMouseCursor resizeUpLeftDownRight = SystemMouseCursor._(
    kind: 'resizeUpLeftDownRight',
  );

  /// A cursor indicating resizing an object bidirectionally from its top right or
  /// bottom left corner.
  ///
  /// Typically the shape of a bidirectional arrow pointing upper right and lower left.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_TOP_RIGHT_DIAGONAL_DOUBLE_ARROW
  ///  * Windows: IDC_SIZENESW
  ///  * Windows UWP: CoreCursorType::SizeNortheastSouthwest
  ///  * Web: nesw-resize
  ///  * Linux: nesw-resize
  static const SystemMouseCursor resizeUpRightDownLeft = SystemMouseCursor._(
    kind: 'resizeUpRightDownLeft',
  );

  /// A cursor indicating resizing an object from its top edge.
  ///
  /// Typically the shape of an arrow pointing up. May fallback to [resizeUpDown].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_VERTICAL_DOUBLE_ARROW
  ///  * Web: n-resize
  ///  * Windows: IDC_SIZENS
  ///  * Windows UWP: CoreCursorType::SizeNorthSouth
  ///  * Linux: n-resize
  ///  * macOS: resizeUpCursor
  static const SystemMouseCursor resizeUp = SystemMouseCursor._(kind: 'resizeUp');

  /// A cursor indicating resizing an object from its bottom edge.
  ///
  /// Typically the shape of an arrow pointing down. May fallback to [resizeUpDown].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_VERTICAL_DOUBLE_ARROW
  ///  * Web: s-resize
  ///  * Windows: IDC_SIZENS
  ///  * Windows UWP: CoreCursorType::SizeNorthSouth
  ///  * Linux: s-resize
  ///  * macOS: resizeDownCursor
  static const SystemMouseCursor resizeDown = SystemMouseCursor._(kind: 'resizeDown');

  /// A cursor indicating resizing an object from its left edge.
  ///
  /// Typically the shape of an arrow pointing left. May fallback to [resizeLeftRight].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_HORIZONTAL_DOUBLE_ARROW
  ///  * Web: w-resize
  ///  * Windows: IDC_SIZEWE
  ///  * Windows UWP: CoreCursorType::SizeWestEast
  ///  * Linux: w-resize
  ///  * macOS: resizeLeftCursor
  static const SystemMouseCursor resizeLeft = SystemMouseCursor._(kind: 'resizeLeft');

  /// A cursor indicating resizing an object from its right edge.
  ///
  /// Typically the shape of an arrow pointing right. May fallback to [resizeLeftRight].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_HORIZONTAL_DOUBLE_ARROW
  ///  * Web: e-resize
  ///  * Windows: IDC_SIZEWE
  ///  * Windows UWP: CoreCursorType::SizeWestEast
  ///  * Linux: e-resize
  ///  * macOS: resizeRightCursor
  static const SystemMouseCursor resizeRight = SystemMouseCursor._(kind: 'resizeRight');

  /// A cursor indicating resizing an object from its top-left corner.
  ///
  /// Typically the shape of an arrow pointing upper left. May fallback to [resizeUpLeftDownRight].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_TOP_LEFT_DIAGONAL_DOUBLE_ARROW
  ///  * Web: nw-resize
  ///  * Windows: IDC_SIZENWSE
  ///  * Windows UWP: CoreCursorType::SizeNorthwestSoutheast
  ///  * Linux: nw-resize
  static const SystemMouseCursor resizeUpLeft = SystemMouseCursor._(kind: 'resizeUpLeft');

  /// A cursor indicating resizing an object from its top-right corner.
  ///
  /// Typically the shape of an arrow pointing upper right. May fallback to [resizeUpRightDownLeft].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_TOP_RIGHT_DIAGONAL_DOUBLE_ARROW
  ///  * Web: ne-resize
  ///  * Windows: IDC_SIZENESW
  ///  * Windows UWP: CoreCursorType::SizeNortheastSouthwest
  ///  * Linux: ne-resize
  static const SystemMouseCursor resizeUpRight = SystemMouseCursor._(kind: 'resizeUpRight');

  /// A cursor indicating resizing an object from its bottom-left corner.
  ///
  /// Typically the shape of an arrow pointing lower left. May fallback to [resizeUpRightDownLeft].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_TOP_RIGHT_DIAGONAL_DOUBLE_ARROW
  ///  * Web: sw-resize
  ///  * Windows: IDC_SIZENESW
  ///  * Windows UWP: CoreCursorType::SizeNortheastSouthwest
  ///  * Linux: sw-resize
  static const SystemMouseCursor resizeDownLeft = SystemMouseCursor._(kind: 'resizeDownLeft');

  /// A cursor indicating resizing an object from its bottom-right corner.
  ///
  /// Typically the shape of an arrow pointing lower right. May fallback to [resizeUpLeftDownRight].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_TOP_LEFT_DIAGONAL_DOUBLE_ARROW
  ///  * Web: se-resize
  ///  * Windows: IDC_SIZENWSE
  ///  * Windows UWP: CoreCursorType::SizeNorthwestSoutheast
  ///  * Linux: se-resize
  static const SystemMouseCursor resizeDownRight = SystemMouseCursor._(kind: 'resizeDownRight');

  /// A cursor indicating resizing a column, or an item horizontally.
  ///
  /// Typically the shape of arrows pointing left and right with a vertical bar
  /// separating them. May fallback to [resizeLeftRight].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_HORIZONTAL_DOUBLE_ARROW
  ///  * Web: col-resize
  ///  * Windows: IDC_SIZEWE
  ///  * Windows UWP: CoreCursorType::SizeWestEast
  ///  * Linux: col-resize
  ///  * macOS: resizeLeftRightCursor
  static const SystemMouseCursor resizeColumn = SystemMouseCursor._(kind: 'resizeColumn');

  /// A cursor indicating resizing a row, or an item vertically.
  ///
  /// Typically the shape of arrows pointing up and down with a horizontal bar
  /// separating them. May fallback to [resizeUpDown].
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_VERTICAL_DOUBLE_ARROW
  ///  * Web: row-resize
  ///  * Windows: IDC_SIZENS
  ///  * Windows UWP: CoreCursorType::SizeNorthSouth
  ///  * Linux: row-resize
  ///  * macOS: resizeUpDownCursor
  static const SystemMouseCursor resizeRow = SystemMouseCursor._(kind: 'resizeRow');

  // OTHER OPERATIONS

  /// A cursor indicating zooming in.
  ///
  /// Typically a magnifying glass with a plus sign.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_ZOOM_IN
  ///  * Web: zoom-in
  ///  * Linux: zoom-in
  static const SystemMouseCursor zoomIn = SystemMouseCursor._(kind: 'zoomIn');

  /// A cursor indicating zooming out.
  ///
  /// Typically a magnifying glass with a minus sign.
  ///
  /// Corresponds to:
  ///
  ///  * Android: TYPE_ZOOM_OUT
  ///  * Web: zoom-out
  ///  * Linux: zoom-out
  static const SystemMouseCursor zoomOut = SystemMouseCursor._(kind: 'zoomOut');
}
