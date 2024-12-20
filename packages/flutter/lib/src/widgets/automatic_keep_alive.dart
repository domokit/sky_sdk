// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'scroll_view.dart';
library;

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';

import 'framework.dart';
import 'notification_listener.dart';
import 'sliver.dart';

/// Allows subtrees to request to be kept alive in lazy lists.
///
/// This widget is like [KeepAlive] but instead of being explicitly configured,
/// it listens to [KeepAliveNotification] messages from the [child] and other
/// descendants.
///
/// The subtree is kept alive whenever there is one or more descendant that has
/// sent a [KeepAliveNotification] and not yet triggered its
/// [KeepAliveNotification.handle].
///
/// To send these notifications, consider using [AutomaticKeepAliveClientMixin].
class AutomaticKeepAlive extends StatefulWidget {
  /// Creates a widget that listens to [KeepAliveNotification]s and maintains a
  /// [KeepAlive] widget appropriately.
  const AutomaticKeepAlive({super.key, required this.child});

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  @override
  State<AutomaticKeepAlive> createState() => _AutomaticKeepAliveState();
}

class _AutomaticKeepAliveState extends State<AutomaticKeepAlive> {
  Map<Listenable, VoidCallback>? _handles;
  // In order to apply parent data out of turn, the child of the KeepAlive
  // widget must be the same across frames.
  late Widget _child;
  bool _keepingAlive = false;

  @override
  void initState() {
    super.initState();
    _updateChild();
  }

  @override
  void didUpdateWidget(AutomaticKeepAlive oldWidget) {
    super.didUpdateWidget(oldWidget);
    _updateChild();
  }

  void _updateChild() {
    _child = NotificationListener<KeepAliveNotification>(
      onNotification: _addClient,
      child: widget.child,
    );
  }

  @override
  void dispose() {
    if (_handles != null) {
      for (final Listenable handle in _handles!.keys) {
        handle.removeListener(_handles![handle]!);
      }
    }
    super.dispose();
  }

  bool _addClient(KeepAliveNotification notification) {
    final Listenable handle = notification.handle;
    _handles ??= <Listenable, VoidCallback>{};
    assert(!_handles!.containsKey(handle));
    _handles![handle] = _createCallback(handle);
    handle.addListener(_handles![handle]!);
    if (!_keepingAlive) {
      _keepingAlive = true;
      final ParentDataElement<KeepAliveParentDataMixin>? childElement = _getChildElement();
      if (childElement != null) {
        // If the child already exists, update it synchronously.
        _updateParentDataOfChild(childElement);
      } else {
        // If the child doesn't exist yet, we got called during the very first
        // build of this subtree. Wait until the end of the frame to update
        // the child when the child is guaranteed to be present.
        SchedulerBinding.instance.addPostFrameCallback((Duration timeStamp) {
          if (!mounted) {
            return;
          }
          final ParentDataElement<KeepAliveParentDataMixin>? childElement = _getChildElement();
          assert(childElement != null);
          _updateParentDataOfChild(childElement!);
        }, debugLabel: 'AutomaticKeepAlive.updateParentData');
      }
    }
    return false;
  }

  /// Get the [Element] for the only [KeepAlive] child.
  ///
  /// While this widget is guaranteed to have a child, this may return null if
  /// the first build of that child has not completed yet.
  ParentDataElement<KeepAliveParentDataMixin>? _getChildElement() {
    assert(mounted);
    final Element element = context as Element;
    Element? childElement;
    // We use Element.visitChildren rather than context.visitChildElements
    // because we might be called during build, and context.visitChildElements
    // verifies that it is not called during build. Element.visitChildren does
    // not, instead it assumes that the caller will be careful. (See the
    // documentation for these methods for more details.)
    //
    // Here we know it's safe (with the exception outlined below) because we
    // just received a notification, which we wouldn't be able to do if we
    // hadn't built our child and its child -- our build method always builds
    // the same subtree and it always includes the node we're looking for
    // (KeepAlive) as the parent of the node that reports the notifications
    // (NotificationListener).
    //
    // If we are called during the first build of this subtree the links to the
    // children will not be hooked up yet. In that case this method returns
    // null despite the fact that we will have a child after the build
    // completes. It's the caller's responsibility to deal with this case.
    //
    // (We're only going down one level, to get our direct child.)
    element.visitChildren((Element child) {
      childElement = child;
    });
    assert(childElement == null || childElement is ParentDataElement<KeepAliveParentDataMixin>);
    return childElement as ParentDataElement<KeepAliveParentDataMixin>?;
  }

  void _updateParentDataOfChild(ParentDataElement<KeepAliveParentDataMixin> childElement) {
    childElement.applyWidgetOutOfTurn(build(context) as ParentDataWidget<KeepAliveParentDataMixin>);
  }

  VoidCallback _createCallback(Listenable handle) {
    late final VoidCallback callback;
    return callback = () {
      assert(() {
        if (!mounted) {
          throw FlutterError(
            'AutomaticKeepAlive handle triggered after AutomaticKeepAlive was disposed.\n'
            'Widgets should always trigger their KeepAliveNotification handle when they are '
            'deactivated, so that they (or their handle) do not send spurious events later '
            'when they are no longer in the tree.',
          );
        }
        return true;
      }());
      _handles!.remove(handle);
      handle.removeListener(callback);
      if (_handles!.isEmpty) {
        if (SchedulerBinding.instance.schedulerPhase.index <
            SchedulerPhase.persistentCallbacks.index) {
          // Build/layout haven't started yet so let's just schedule this for
          // the next frame.
          setState(() {
            _keepingAlive = false;
          });
        } else {
          // We were probably notified by a descendant when they were yanked out
          // of our subtree somehow. We're probably in the middle of build or
          // layout, so there's really nothing we can do to clean up this mess
          // short of just scheduling another build to do the cleanup. This is
          // very unfortunate, and means (for instance) that garbage collection
          // of these resources won't happen for another 16ms.
          //
          // The problem is there's really no way for us to distinguish these
          // cases:
          //
          //  * We haven't built yet (or missed out chance to build), but
          //    someone above us notified our descendant and our descendant is
          //    disconnecting from us. If we could mark ourselves dirty we would
          //    be able to clean everything this frame. (This is a pretty
          //    unlikely scenario in practice. Usually things change before
          //    build/layout, not during build/layout.)
          //
          //  * Our child changed, and as our old child went away, it notified
          //    us. We can't setState, since we _just_ built. We can't apply the
          //    parent data information to our child because we don't _have_ a
          //    child at this instant. We really want to be able to change our
          //    mind about how we built, so we can give the KeepAlive widget a
          //    new value, but it's too late.
          //
          //  * A deep descendant in another build scope just got yanked, and in
          //    the process notified us. We could apply new parent data
          //    information, but it may or may not get applied this frame,
          //    depending on whether said child is in the same layout scope.
          //
          //  * A descendant is being moved from one position under us to
          //    another position under us. They just notified us of the removal,
          //    at some point in the future they will notify us of the addition.
          //    We don't want to do anything. (This is why we check that
          //    _handles is still empty below.)
          //
          //  * We're being notified in the paint phase, or even in a post-frame
          //    callback. Either way it is far too late for us to make our
          //    parent lay out again this frame, so the garbage won't get
          //    collected this frame.
          //
          //  * We are being torn out of the tree ourselves, as is our
          //    descendant, and it notified us while it was being deactivated.
          //    We don't need to do anything, but we don't know yet because we
          //    haven't been deactivated yet. (This is why we check mounted
          //    below before calling setState.)
          //
          // Long story short, we have to schedule a new frame and request a
          // frame there, but this is generally a bad practice, and you should
          // avoid it if possible.
          _keepingAlive = false;
          scheduleMicrotask(() {
            if (mounted && _handles!.isEmpty) {
              // If mounted is false, we went away as well, so there's nothing to do.
              // If _handles is no longer empty, then another client (or the same
              // client in a new place) registered itself before we had a chance to
              // turn off keepalive, so again there's nothing to do.
              setState(() {
                assert(!_keepingAlive);
              });
            }
          });
        }
      }
    };
  }

  @override
  Widget build(BuildContext context) {
    return KeepAlive(keepAlive: _keepingAlive, child: _child);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(
      FlagProperty('_keepingAlive', value: _keepingAlive, ifTrue: 'keeping subtree alive'),
    );
    description.add(
      DiagnosticsProperty<Map<Listenable, VoidCallback>>(
        'handles',
        _handles,
        description:
            _handles != null
                ? '${_handles!.length} active client${_handles!.length == 1 ? "" : "s"}'
                : null,
        ifNull: 'no notifications ever received',
      ),
    );
  }
}

/// Indicates that the subtree through which this notification bubbles must be
/// kept alive even if it would normally be discarded as an optimization.
///
/// For example, a focused text field might fire this notification to indicate
/// that it should not be disposed even if the user scrolls the field off
/// screen.
///
/// Each [KeepAliveNotification] is configured with a [handle] that consists of
/// a [Listenable] that is triggered when the subtree no longer needs to be kept
/// alive.
///
/// The [handle] should be triggered any time the sending widget is removed from
/// the tree (in [State.deactivate]). If the widget is then rebuilt and still
/// needs to be kept alive, it should immediately send a new notification
/// (possible with the very same [Listenable]) during build.
///
/// This notification is listened to by the [AutomaticKeepAlive] widget, which
/// is added to the tree automatically by [SliverList] (and [ListView]) and
/// [SliverGrid] (and [GridView]) widgets.
///
/// Failure to trigger the [handle] in the manner described above will likely
/// cause the [AutomaticKeepAlive] to lose track of whether the widget should be
/// kept alive or not, leading to memory leaks or lost data. For example, if the
/// widget that requested keepalive is removed from the subtree but doesn't
/// trigger its [Listenable] on the way out, then the subtree will continue to
/// be kept alive until the list itself is disposed. Similarly, if the
/// [Listenable] is triggered while the widget needs to be kept alive, but a new
/// [KeepAliveNotification] is not immediately sent, then the widget risks being
/// garbage collected while it wants to be kept alive.
///
/// It is an error to use the same [handle] in two [KeepAliveNotification]s
/// within the same [AutomaticKeepAlive] without triggering that [handle] before
/// the second notification is sent.
///
/// For a more convenient way to interact with [AutomaticKeepAlive] widgets,
/// consider using [AutomaticKeepAliveClientMixin], which uses
/// [KeepAliveNotification] internally.
class KeepAliveNotification extends Notification {
  /// Creates a notification to indicate that a subtree must be kept alive.
  const KeepAliveNotification(this.handle);

  /// A [Listenable] that will inform its clients when the widget that fired the
  /// notification no longer needs to be kept alive.
  ///
  /// The [Listenable] should be triggered any time the sending widget is
  /// removed from the tree (in [State.deactivate]). If the widget is then
  /// rebuilt and still needs to be kept alive, it should immediately send a new
  /// notification (possible with the very same [Listenable]) during build.
  ///
  /// See also:
  ///
  ///  * [KeepAliveHandle], a convenience class for use with this property.
  final Listenable handle;
}

/// A [Listenable] which can be manually triggered.
///
/// Used with [KeepAliveNotification] objects as their
/// [KeepAliveNotification.handle].
///
/// For a more convenient way to interact with [AutomaticKeepAlive] widgets,
/// consider using [AutomaticKeepAliveClientMixin], which uses a
/// [KeepAliveHandle] internally.
class KeepAliveHandle extends ChangeNotifier {
  @override
  void dispose() {
    notifyListeners();
    super.dispose();
  }
}

/// A mixin with convenience methods for clients of [AutomaticKeepAlive]. Used
/// with [State] subclasses.
///
/// Subclasses must implement [wantKeepAlive], and their [build] methods must
/// call `super.build` (though the return value should be ignored).
///
/// Then, whenever [wantKeepAlive]'s value changes (or might change), the
/// subclass should call [updateKeepAlive].
///
/// The type argument `T` is the type of the [StatefulWidget] subclass of the
/// [State] into which this class is being mixed.
///
/// See also:
///
///  * [AutomaticKeepAlive], which listens to messages from this mixin.
///  * [KeepAliveNotification], the notifications sent by this mixin.
@optionalTypeArgs
mixin AutomaticKeepAliveClientMixin<T extends StatefulWidget> on State<T> {
  KeepAliveHandle? _keepAliveHandle;

  void _ensureKeepAlive() {
    assert(_keepAliveHandle == null);
    _keepAliveHandle = KeepAliveHandle();
    KeepAliveNotification(_keepAliveHandle!).dispatch(context);
  }

  void _releaseKeepAlive() {
    _keepAliveHandle!.dispose();
    _keepAliveHandle = null;
  }

  /// Whether the current instance should be kept alive.
  ///
  /// Call [updateKeepAlive] whenever this getter's value changes.
  @protected
  bool get wantKeepAlive;

  /// Ensures that any [AutomaticKeepAlive] ancestors are in a good state, by
  /// firing a [KeepAliveNotification] or triggering the [KeepAliveHandle] as
  /// appropriate.
  @protected
  void updateKeepAlive() {
    if (wantKeepAlive) {
      if (_keepAliveHandle == null) {
        _ensureKeepAlive();
      }
    } else {
      if (_keepAliveHandle != null) {
        _releaseKeepAlive();
      }
    }
  }

  @override
  void initState() {
    super.initState();
    if (wantKeepAlive) {
      _ensureKeepAlive();
    }
  }

  @override
  void deactivate() {
    if (_keepAliveHandle != null) {
      _releaseKeepAlive();
    }
    super.deactivate();
  }

  @mustCallSuper
  @override
  Widget build(BuildContext context) {
    if (wantKeepAlive && _keepAliveHandle == null) {
      _ensureKeepAlive();
      // Whenever wantKeepAlive's value changes (or might change), the
      // subclass should call [updateKeepAlive].
      // That will ensure that the keepalive is disabled (or enabled)
      // without requiring a rebuild.
    }
    return const _NullWidget();
  }
}

class _NullWidget extends StatelessWidget {
  const _NullWidget();

  @override
  Widget build(BuildContext context) {
    throw FlutterError(
      'Widgets that mix AutomaticKeepAliveClientMixin into their State must '
      'call super.build() but must ignore the return value of the superclass.',
    );
  }
}
