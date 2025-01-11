// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/animation.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';

import 'default_animation_style.dart';
import 'framework.dart';

export 'package:flutter/animation.dart' show AnimationProvider;
export 'package:flutter/scheduler.dart' show TickerProvider;

// Examples can assume:
// late BuildContext context;

/// Enables or disables tickers (and thus animation controllers) in the widget
/// subtree.
///
/// This only works if [AnimationController] objects are created using
/// widget-aware ticker providers. For example, using a
/// [TickerProviderStateMixin] or a [SingleTickerProviderStateMixin].
class TickerMode extends StatefulWidget {
  /// Creates a widget that enables or disables tickers.
  const TickerMode({super.key, required this.enabled, required this.child});

  /// The requested ticker mode for this subtree.
  ///
  /// The effective ticker mode of this subtree may differ from this value
  /// if there is an ancestor [TickerMode] with this field set to false.
  ///
  /// If true and all ancestor [TickerMode]s are also enabled, then tickers in
  /// this subtree will tick.
  ///
  /// If false, then tickers in this subtree will not tick regardless of any
  /// ancestor [TickerMode]s. Animations driven by such tickers are not paused,
  /// they just don't call their callbacks. Time still elapses.
  final bool enabled;

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// Whether tickers in the given subtree should be enabled or disabled.
  ///
  /// This is used automatically by [TickerProviderStateMixin] and
  /// [SingleTickerProviderStateMixin] to decide if their tickers should be
  /// enabled or disabled.
  ///
  /// In the absence of a [TickerMode] widget, this function defaults to true.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// bool tickingEnabled = TickerMode.of(context);
  /// ```
  static bool of(BuildContext context) {
    final _EffectiveTickerMode? widget =
        context.dependOnInheritedWidgetOfExactType<_EffectiveTickerMode>();
    return widget?.enabled ?? true;
  }

  /// Obtains a [ValueListenable] from the [TickerMode] surrounding the `context`,
  /// which indicates whether tickers are enabled in the given subtree.
  ///
  /// When that [TickerMode] enabled or disabled tickers, the listenable notifies
  /// its listeners.
  ///
  /// While the [ValueListenable] is stable for the lifetime of the surrounding
  /// [TickerMode], calling this method does not establish a dependency between
  /// the `context` and the [TickerMode] and the widget owning the `context`
  /// does not rebuild when the ticker mode changes from true to false or vice
  /// versa. This is preferable when the ticker mode does not impact what is
  /// currently rendered on screen, e.g. because it is only used to mute/unmute a
  /// [Ticker]. Since no dependency is established, the widget owning the
  /// `context` is also not informed when it is moved to a new location in the
  /// tree where it may have a different [TickerMode] ancestor. When this
  /// happens, the widget must manually unsubscribe from the old listenable,
  /// obtain a new one from the new ancestor [TickerMode] by calling this method
  /// again, and re-subscribe to it. [StatefulWidget]s can, for example, do this
  /// in [State.activate], which is called after the widget has been moved to
  /// a new location.
  ///
  /// Alternatively, [of] can be used instead of this method to create a
  /// dependency between the provided `context` and the ancestor [TickerMode].
  /// In this case, the widget automatically rebuilds when the ticker mode
  /// changes or when it is moved to a new [TickerMode] ancestor, which
  /// simplifies the management cost in the widget at the expensive of some
  /// potential unnecessary rebuilds.
  ///
  /// In the absence of a [TickerMode] widget, this function returns a
  /// [ValueListenable], whose [ValueListenable.value] is always true.
  static ValueListenable<bool> getNotifier(BuildContext context) {
    final _EffectiveTickerMode? widget =
        context.getInheritedWidgetOfExactType<_EffectiveTickerMode>();
    return widget?.notifier ?? const _ConstantValueListenable<bool>(true);
  }

  @override
  State<TickerMode> createState() => _TickerModeState();
}

class _TickerModeState extends State<TickerMode> {
  bool _ancestorTicketMode = true;
  final ValueNotifier<bool> _effectiveMode = ValueNotifier<bool>(true);

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _ancestorTicketMode = TickerMode.of(context);
    _updateEffectiveMode();
  }

  @override
  void didUpdateWidget(TickerMode oldWidget) {
    super.didUpdateWidget(oldWidget);
    _updateEffectiveMode();
  }

  @override
  void dispose() {
    _effectiveMode.dispose();
    super.dispose();
  }

  void _updateEffectiveMode() {
    _effectiveMode.value = _ancestorTicketMode && widget.enabled;
  }

  @override
  Widget build(BuildContext context) {
    return _EffectiveTickerMode(
      enabled: _effectiveMode.value,
      notifier: _effectiveMode,
      child: widget.child,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      FlagProperty(
        'requested mode',
        value: widget.enabled,
        ifTrue: 'enabled',
        ifFalse: 'disabled',
        showName: true,
      ),
    );
  }
}

class _EffectiveTickerMode extends InheritedWidget {
  const _EffectiveTickerMode({required this.enabled, required this.notifier, required super.child});

  final bool enabled;
  final ValueNotifier<bool> notifier;

  @override
  bool updateShouldNotify(_EffectiveTickerMode oldWidget) => enabled != oldWidget.enabled;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      FlagProperty(
        'effective mode',
        value: enabled,
        ifTrue: 'enabled',
        ifFalse: 'disabled',
        showName: true,
      ),
    );
  }
}

typedef _Animation = StyledAnimation<Object?>;
typedef _StyleNotifier = ValueListenable<AnimationStyle>;

/// Provides a single [Ticker] that is configured to only tick while the current
/// tree is enabled, as defined by [TickerMode].
///
/// To create the [AnimationController] in a [State] that only uses a single
/// [AnimationController], mix in this class, then pass `vsync: this`
/// to the animation controller constructor.
///
/// This mixin only supports vending a single ticker. If you might have multiple
/// [AnimationController] objects over the lifetime of the [State], use a full
/// [TickerProviderStateMixin] instead.
@optionalTypeArgs
mixin SingleTickerProviderStateMixin<T extends StatefulWidget> on State<T>
    implements AnimationProvider {
  Ticker? _ticker;

  @override
  Ticker createTicker(TickerCallback onTick) {
    assert(() {
      if (_ticker == null) {
        return true;
      }
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary(
          '$runtimeType is a SingleTickerProviderStateMixin but multiple tickers were created.',
        ),
        ErrorDescription(
          'A SingleTickerProviderStateMixin can only be used as a TickerProvider once.',
        ),
        ErrorHint(
          'If a State is used for multiple AnimationController objects, or if it is passed to other '
          'objects and those objects might use it more than one time in total, then instead of '
          'mixing in a SingleTickerProviderStateMixin, use a regular TickerProviderStateMixin.',
        ),
      ]);
    }());
    _ticker = Ticker(
      onTick,
      debugLabel: kDebugMode ? 'created by ${describeIdentity(this)}' : null,
    );
    _updateTickerModeNotifier();
    _updateTicker(); // Sets _ticker.mute correctly.
    return _ticker!;
  }

  _Animation? _animation;
  _StyleNotifier? _styleNotifier;
  void _updateAnimationStyle() {
    _animation!.updateStyle(_styleNotifier!.value);
  }

  @override
  void registerAnimation(StyledAnimation<Object?> animation) {
    assert(() {
      if (_animation == null) {
        return true;
      }
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary(
          '$runtimeType uses the SingleTickerProviderStateMixin '
          'but multiple animations were registered to it.',
        ),
        ErrorDescription(
          'A SingleTickerProviderStateMixin can only be used for a single Animation object.',
        ),
        ErrorHint(
          'If a State is used for multiple AnimationController objects, or if it is passed to other '
          'objects that might use it multiple times, consider using TickerProviderStateMixin instead.',
        ),
      ]);
    }());
    _animation = animation;
    (_styleNotifier ??= DefaultAnimationStyle.getNotifier(context)).addListener(
      _updateAnimationStyle..call(),
    );
  }

  @override
  void dispose() {
    assert(() {
      if (_ticker == null || !_ticker!.isActive) {
        return true;
      }
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('$this was disposed with an active Ticker.'),
        ErrorDescription(
          '$runtimeType created a Ticker via its SingleTickerProviderStateMixin, but at the time '
          'dispose() was called on the mixin, that Ticker was still active. The Ticker must '
          'be disposed before calling super.dispose().',
        ),
        ErrorHint(
          'Tickers used by AnimationControllers '
          'should be disposed by calling dispose() on the AnimationController itself. '
          'Otherwise, the ticker will leak.',
        ),
        _ticker!.describeForError('The offending ticker was'),
      ]);
    }());
    _tickerModeNotifier?.removeListener(_updateTicker);
    _tickerModeNotifier = null;
    _styleNotifier?.removeListener(_updateAnimationStyle);
    _styleNotifier = null;
    super.dispose();
  }

  ValueListenable<bool>? _tickerModeNotifier;

  @override
  void activate() {
    super.activate();
    // We may have a new TickerMode ancestor.
    _updateTickerModeNotifier();
    _updateTicker();

    final _StyleNotifier styleNotifier = DefaultAnimationStyle.getNotifier(context);
    if (styleNotifier != _styleNotifier) {
      _styleNotifier?.removeListener(_updateAnimationStyle);
      _styleNotifier = styleNotifier..addListener(_updateAnimationStyle);
      _updateAnimationStyle();
    }
  }

  void _updateTicker() => _ticker?.muted = !_tickerModeNotifier!.value;

  void _updateTickerModeNotifier() {
    final ValueListenable<bool> newNotifier = TickerMode.getNotifier(context);
    if (newNotifier == _tickerModeNotifier) {
      return;
    }
    _tickerModeNotifier?.removeListener(_updateTicker);
    newNotifier.addListener(_updateTicker);
    _tickerModeNotifier = newNotifier;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    final String? tickerDescription = switch ((_ticker?.isActive, _ticker?.muted)) {
      (true, true) => 'active but muted',
      (true, _) => 'active',
      (false, true) => 'inactive and muted',
      (false, _) => 'inactive',
      (null, _) => null,
    };
    properties.add(
      DiagnosticsProperty<Ticker>(
        'ticker',
        _ticker,
        description: tickerDescription,
        showSeparator: false,
        defaultValue: null,
      ),
    );
  }
}

/// Provides [Ticker] objects that are configured to only tick while the current
/// tree is enabled, as defined by [TickerMode].
///
/// To create an [AnimationController] in a class that uses this mixin, pass
/// `vsync: this` to the animation controller constructor whenever you
/// create a new animation controller.
///
/// If you only have a single [Ticker] (for example only a single
/// [AnimationController]) for the lifetime of your [State], then using a
/// [SingleTickerProviderStateMixin] is more efficient. This is the common case.
@optionalTypeArgs
mixin TickerProviderStateMixin<T extends StatefulWidget> on State<T> implements AnimationProvider {
  Set<Ticker>? _tickers;

  @override
  Ticker createTicker(TickerCallback onTick) {
    if (_tickerModeNotifier == null) {
      // Setup TickerMode notifier before we vend the first ticker.
      _updateTickerModeNotifier();
    }
    assert(_tickerModeNotifier != null);
    _tickers ??= <_WidgetTicker>{};
    final _WidgetTicker result = _WidgetTicker(
      onTick,
      this,
      debugLabel: kDebugMode ? 'created by ${describeIdentity(this)}' : null,
    )..muted = !_tickerModeNotifier!.value;
    _tickers!.add(result);
    return result;
  }

  void _removeTicker(_WidgetTicker ticker) {
    assert(_tickers != null);
    assert(_tickers!.contains(ticker));
    _tickers!.remove(ticker);
  }

  ValueListenable<bool>? _tickerModeNotifier;

  Set<_Animation>? _animations;
  _StyleNotifier? _styleNotifier;
  void _updateAnimations() {
    final Set<_Animation>? animations = _animations;
    if (animations == null) {
      return;
    }
    for (final _Animation animation in animations) {
      animation.updateStyle(_styleNotifier!.value);
    }
  }

  @override
  void registerAnimation(StyledAnimation<Object?> animation) {
    final bool firstAnimation = _animations == null;
    final _StyleNotifier notifier = _styleNotifier ??= DefaultAnimationStyle.getNotifier(context);
    (_animations ??= <_Animation>{}).add(animation..updateStyle(notifier.value));
    if (firstAnimation) {
      notifier.addListener(_updateAnimations);
    }
  }

  @override
  void activate() {
    super.activate();
    // We may have a new TickerMode ancestor, get its Notifier.
    _updateTickerModeNotifier();
    _updateTickers();

    late final _StyleNotifier newNotifier = DefaultAnimationStyle.getNotifier(context);
    if (_animations != null && newNotifier != _styleNotifier) {
      _styleNotifier?.removeListener(_updateAnimations);
      _styleNotifier = newNotifier..addListener(_updateAnimations);
      _updateAnimations();
    }
  }

  void _updateTickers() {
    if (_tickers != null) {
      final bool muted = !_tickerModeNotifier!.value;
      for (final Ticker ticker in _tickers!) {
        ticker.muted = muted;
      }
    }
  }

  void _updateTickerModeNotifier() {
    final ValueListenable<bool> newNotifier = TickerMode.getNotifier(context);
    if (newNotifier == _tickerModeNotifier) {
      return;
    }
    _tickerModeNotifier?.removeListener(_updateTickers);
    newNotifier.addListener(_updateTickers);
    _tickerModeNotifier = newNotifier;
  }

  @override
  void dispose() {
    assert(() {
      if (_tickers != null) {
        for (final Ticker ticker in _tickers!) {
          if (ticker.isActive) {
            throw FlutterError.fromParts(<DiagnosticsNode>[
              ErrorSummary('$this was disposed with an active Ticker.'),
              ErrorDescription(
                '$runtimeType created a Ticker via its TickerProviderStateMixin, but at the time '
                'dispose() was called on the mixin, that Ticker was still active. All Tickers must '
                'be disposed before calling super.dispose().',
              ),
              ErrorHint(
                'Tickers used by AnimationControllers '
                'should be disposed by calling dispose() on the AnimationController itself. '
                'Otherwise, the ticker will leak.',
              ),
              ticker.describeForError('The offending ticker was'),
            ]);
          }
        }
      }
      return true;
    }());
    _tickerModeNotifier?.removeListener(_updateTickers);
    _tickerModeNotifier = null;
    _styleNotifier?.removeListener(_updateAnimations);
    _styleNotifier = null;
    super.dispose();
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<Set<Ticker>>(
        'tickers',
        _tickers,
        description:
            _tickers != null
                ? 'tracking ${_tickers!.length} ticker${_tickers!.length == 1 ? "" : "s"}'
                : null,
        defaultValue: null,
      ),
    );
  }
}

// This class should really be called _DisposingTicker or some such, but this
// class name leaks into stack traces and error messages and that name would be
// confusing. Instead we use the less precise but more anodyne "_WidgetTicker",
// which attracts less attention.
class _WidgetTicker extends Ticker {
  _WidgetTicker(super.onTick, this._creator, {super.debugLabel});

  final TickerProviderStateMixin _creator;

  @override
  void dispose() {
    _creator._removeTicker(this);
    super.dispose();
  }
}

class _ConstantValueListenable<T> implements ValueListenable<T> {
  const _ConstantValueListenable(this.value);

  @override
  void addListener(VoidCallback listener) {
    // Intentionally left empty: Value cannot change, so we never have to
    // notify registered listeners.
  }

  @override
  void removeListener(VoidCallback listener) {
    // Intentionally left empty: Value cannot change, so we never have to
    // notify registered listeners.
  }

  @override
  final T value;
}
