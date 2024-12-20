// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'app.dart';
/// @docImport 'basic.dart';
library;

import 'dart:collection';

import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

import 'actions.dart';
import 'focus_manager.dart';
import 'focus_scope.dart';
import 'framework.dart';
import 'platform_menu_bar.dart';

final Set<LogicalKeyboardKey> _controlSynonyms = LogicalKeyboardKey.expandSynonyms(
  <LogicalKeyboardKey>{LogicalKeyboardKey.control},
);
final Set<LogicalKeyboardKey> _shiftSynonyms = LogicalKeyboardKey.expandSynonyms(
  <LogicalKeyboardKey>{LogicalKeyboardKey.shift},
);
final Set<LogicalKeyboardKey> _altSynonyms = LogicalKeyboardKey.expandSynonyms(<LogicalKeyboardKey>{
  LogicalKeyboardKey.alt,
});
final Set<LogicalKeyboardKey> _metaSynonyms = LogicalKeyboardKey.expandSynonyms(
  <LogicalKeyboardKey>{LogicalKeyboardKey.meta},
);

/// A set of [KeyboardKey]s that can be used as the keys in a [Map].
///
/// A key set contains the keys that are down simultaneously to represent a
/// shortcut.
///
/// This is a thin wrapper around a [Set], but changes the equality comparison
/// from an identity comparison to a contents comparison so that non-identical
/// sets with the same keys in them will compare as equal.
///
/// See also:
///
///  * [ShortcutManager], which uses [LogicalKeySet] (a [KeySet] subclass) to
///    define its key map.
@immutable
class KeySet<T extends KeyboardKey> {
  /// A constructor for making a [KeySet] of up to four keys.
  ///
  /// If you need a set of more than four keys, use [KeySet.fromSet].
  ///
  /// The same [KeyboardKey] may not be appear more than once in the set.
  KeySet(T key1, [T? key2, T? key3, T? key4]) : _keys = HashSet<T>()..add(key1) {
    int count = 1;
    if (key2 != null) {
      _keys.add(key2);
      assert(() {
        count++;
        return true;
      }());
    }
    if (key3 != null) {
      _keys.add(key3);
      assert(() {
        count++;
        return true;
      }());
    }
    if (key4 != null) {
      _keys.add(key4);
      assert(() {
        count++;
        return true;
      }());
    }
    assert(
      _keys.length == count,
      'Two or more provided keys are identical. Each key must appear only once.',
    );
  }

  /// Create a [KeySet] from a set of [KeyboardKey]s.
  ///
  /// Do not mutate the `keys` set after passing it to this object.
  ///
  /// The `keys` set must not be empty.
  KeySet.fromSet(Set<T> keys)
    : assert(keys.isNotEmpty),
      assert(!keys.contains(null)),
      _keys = HashSet<T>.of(keys);

  /// Returns a copy of the [KeyboardKey]s in this [KeySet].
  Set<T> get keys => _keys.toSet();
  final HashSet<T> _keys;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is KeySet<T> && setEquals<T>(other._keys, _keys);
  }

  // Cached hash code value. Improves [hashCode] performance by 27%-900%,
  // depending on key set size and read/write ratio.
  @override
  late final int hashCode = _computeHashCode(_keys);

  // Arrays used to temporarily store hash codes for sorting.
  static final List<int> _tempHashStore3 = <int>[0, 0, 0]; // used to sort exactly 3 keys
  static final List<int> _tempHashStore4 = <int>[0, 0, 0, 0]; // used to sort exactly 4 keys
  static int _computeHashCode<T>(Set<T> keys) {
    // Compute order-independent hash and cache it.
    final int length = keys.length;
    final Iterator<T> iterator = keys.iterator;

    // There's always at least one key. Just extract it.
    iterator.moveNext();
    final int h1 = iterator.current.hashCode;

    if (length == 1) {
      // Don't do anything fancy if there's exactly one key.
      return h1;
    }

    iterator.moveNext();
    final int h2 = iterator.current.hashCode;
    if (length == 2) {
      // No need to sort if there's two keys, just compare them.
      return h1 < h2 ? Object.hash(h1, h2) : Object.hash(h2, h1);
    }

    // Sort key hash codes and feed to Object.hashAll to ensure the aggregate
    // hash code does not depend on the key order.
    final List<int> sortedHashes = length == 3 ? _tempHashStore3 : _tempHashStore4;
    sortedHashes[0] = h1;
    sortedHashes[1] = h2;
    iterator.moveNext();
    sortedHashes[2] = iterator.current.hashCode;
    if (length == 4) {
      iterator.moveNext();
      sortedHashes[3] = iterator.current.hashCode;
    }
    sortedHashes.sort();
    return Object.hashAll(sortedHashes);
  }
}

/// Determines how the state of a lock key is used to accept a shortcut.
enum LockState {
  /// The lock key state is not used to determine [SingleActivator.accepts] result.
  ignored,

  /// The lock key must be locked to trigger the shortcut.
  locked,

  /// The lock key must be unlocked to trigger the shortcut.
  unlocked,
}

/// An interface to define the keyboard key combination to trigger a shortcut.
///
/// [ShortcutActivator]s are used by [Shortcuts] widgets, and are mapped to
/// [Intent]s, the intended behavior that the key combination should trigger.
/// When a [Shortcuts] widget receives a key event, its [ShortcutManager] looks
/// up the first matching [ShortcutActivator], and signals the corresponding
/// [Intent], which might trigger an action as defined by a hierarchy of
/// [Actions] widgets. For a detailed introduction on the mechanism and use of
/// the shortcut-action system, see [Actions].
///
/// The matching [ShortcutActivator] is looked up in the following way:
///
///  * Find the registered [ShortcutActivator]s whose [triggers] contain the
///    incoming event.
///  * Of the previous list, finds the first activator whose [accepts] returns
///    true in the order of insertion.
///
/// See also:
///
///  * [SingleActivator], an implementation that represents a single key combined
///    with modifiers (control, shift, alt, meta).
///  * [CharacterActivator], an implementation that represents key combinations
///    that result in the specified character, such as question mark.
///  * [LogicalKeySet], an implementation that requires one or more
///    [LogicalKeyboardKey]s to be pressed at the same time. Prefer
///    [SingleActivator] when possible.
abstract class ShortcutActivator {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const ShortcutActivator();

  /// An optional property to provide all the keys that might be the final event
  /// to trigger this shortcut.
  ///
  /// For example, for `Ctrl-A`, [LogicalKeyboardKey.keyA] is the only trigger,
  /// while [LogicalKeyboardKey.control] is not, because the shortcut should
  /// only work by pressing KeyA *after* Ctrl, but not before. For `Ctrl-A-E`,
  /// on the other hand, both KeyA and KeyE should be triggers, since either of
  /// them is allowed to trigger.
  ///
  /// If provided, trigger keys can be used as a first-pass filter for incoming
  /// events in order to optimize lookups, as [Intent]s are stored in a [Map]
  /// and indexed by trigger keys. It is up to the individual implementors of
  /// this interface to decide if they ignore triggers or not.
  ///
  /// Subclasses should make sure that the return value of this method does not
  /// change throughout the lifespan of this object.
  ///
  /// This method might also return null, which means this activator declares
  /// all keys as trigger keys. Activators whose [triggers] return null will be
  /// tested with [accepts] on every event. Since this becomes a linear search,
  /// and having too many might impact performance, it is preferred to return
  /// non-null [triggers] whenever possible.
  Iterable<LogicalKeyboardKey>? get triggers => null;

  /// Whether the triggering `event` and the keyboard `state` at the time of the
  /// event meet required conditions, providing that the event is a triggering
  /// event.
  ///
  /// For example, for `Ctrl-A`, it has to check if the event is a
  /// [KeyDownEvent], if either side of the Ctrl key is pressed, and none of the
  /// Shift keys, Alt keys, or Meta keys are pressed; it doesn't have to check
  /// if KeyA is pressed, since it's already guaranteed.
  ///
  /// As a possible performance improvement, implementers of this function are
  /// encouraged (but not required) to check the [triggers] member, if it is
  /// non-null, to see if it contains the event's logical key before doing more
  /// complicated work.
  ///
  /// This method must not cause any side effects for the `state`. Typically
  /// this is only used to query whether [HardwareKeyboard.logicalKeysPressed]
  /// contains a key.
  ///
  /// See also:
  ///
  /// * [LogicalKeyboardKey.collapseSynonyms], which helps deciding whether a
  ///   modifier key is pressed when the side variation is not important.
  bool accepts(KeyEvent event, HardwareKeyboard state);

  /// Returns true if the event and current [HardwareKeyboard] state would cause
  /// this [ShortcutActivator] to be activated.
  @Deprecated(
    'Call accepts on the activator instead. '
    'This feature was deprecated after v3.16.0-15.0.pre.',
  )
  static bool isActivatedBy(ShortcutActivator activator, KeyEvent event) {
    return activator.accepts(event, HardwareKeyboard.instance);
  }

  /// Returns a description of the key set that is short and readable.
  ///
  /// Intended to be used in debug mode for logging purposes.
  String debugDescribeKeys();
}

/// A set of [LogicalKeyboardKey]s that can be used as the keys in a map.
///
/// [LogicalKeySet] can be used as a [ShortcutActivator]. It is not recommended
/// to use [LogicalKeySet] for a common shortcut such as `Delete` or `Ctrl+C`,
/// prefer [SingleActivator] when possible, whose behavior more closely resembles
/// that of typical platforms.
///
/// When used as a [ShortcutActivator], [LogicalKeySet] will activate the intent
/// when all [keys] are pressed, and no others, except that modifier keys are
/// considered without considering sides (e.g. control left and control right are
/// considered the same).
///
/// {@tool dartpad}
/// In the following example, the counter is increased when the following key
/// sequences are pressed:
///
///  * Control left, then C.
///  * Control right, then C.
///  * C, then Control left.
///
/// But not when:
///
///  * Control left, then A, then C.
///
/// ** See code in examples/api/lib/widgets/shortcuts/logical_key_set.0.dart **
/// {@end-tool}
///
/// This is also a thin wrapper around a [Set], but changes the equality
/// comparison from an identity comparison to a contents comparison so that
/// non-identical sets with the same keys in them will compare as equal.

class LogicalKeySet extends KeySet<LogicalKeyboardKey>
    with Diagnosticable
    implements ShortcutActivator {
  /// A constructor for making a [LogicalKeySet] of up to four keys.
  ///
  /// If you need a set of more than four keys, use [LogicalKeySet.fromSet].
  ///
  /// The same [LogicalKeyboardKey] may not be appear more than once in the set.
  LogicalKeySet(super.key1, [super.key2, super.key3, super.key4]);

  /// Create a [LogicalKeySet] from a set of [LogicalKeyboardKey]s.
  ///
  /// Do not mutate the `keys` set after passing it to this object.
  LogicalKeySet.fromSet(super.keys) : super.fromSet();

  @override
  Iterable<LogicalKeyboardKey> get triggers => _triggers;
  late final Set<LogicalKeyboardKey> _triggers =
      keys
          .expand((LogicalKeyboardKey key) => _unmapSynonyms[key] ?? <LogicalKeyboardKey>[key])
          .toSet();

  bool _checkKeyRequirements(Set<LogicalKeyboardKey> pressed) {
    final Set<LogicalKeyboardKey> collapsedRequired = LogicalKeyboardKey.collapseSynonyms(keys);
    final Set<LogicalKeyboardKey> collapsedPressed = LogicalKeyboardKey.collapseSynonyms(pressed);
    return collapsedRequired.length == collapsedPressed.length &&
        collapsedRequired.difference(collapsedPressed).isEmpty;
  }

  @override
  bool accepts(KeyEvent event, HardwareKeyboard state) {
    if (event is! KeyDownEvent && event is! KeyRepeatEvent) {
      return false;
    }
    return triggers.contains(event.logicalKey) && _checkKeyRequirements(state.logicalKeysPressed);
  }

  static final Set<LogicalKeyboardKey> _modifiers = <LogicalKeyboardKey>{
    LogicalKeyboardKey.alt,
    LogicalKeyboardKey.control,
    LogicalKeyboardKey.meta,
    LogicalKeyboardKey.shift,
  };
  static final Map<LogicalKeyboardKey, List<LogicalKeyboardKey>> _unmapSynonyms =
      <LogicalKeyboardKey, List<LogicalKeyboardKey>>{
        LogicalKeyboardKey.control: <LogicalKeyboardKey>[
          LogicalKeyboardKey.controlLeft,
          LogicalKeyboardKey.controlRight,
        ],
        LogicalKeyboardKey.shift: <LogicalKeyboardKey>[
          LogicalKeyboardKey.shiftLeft,
          LogicalKeyboardKey.shiftRight,
        ],
        LogicalKeyboardKey.alt: <LogicalKeyboardKey>[
          LogicalKeyboardKey.altLeft,
          LogicalKeyboardKey.altRight,
        ],
        LogicalKeyboardKey.meta: <LogicalKeyboardKey>[
          LogicalKeyboardKey.metaLeft,
          LogicalKeyboardKey.metaRight,
        ],
      };

  @override
  String debugDescribeKeys() {
    final List<LogicalKeyboardKey> sortedKeys =
        keys.toList()..sort((LogicalKeyboardKey a, LogicalKeyboardKey b) {
          // Put the modifiers first. If it has a synonym, then it's something
          // like shiftLeft, altRight, etc.
          final bool aIsModifier = a.synonyms.isNotEmpty || _modifiers.contains(a);
          final bool bIsModifier = b.synonyms.isNotEmpty || _modifiers.contains(b);
          if (aIsModifier && !bIsModifier) {
            return -1;
          } else if (bIsModifier && !aIsModifier) {
            return 1;
          }
          return a.debugName!.compareTo(b.debugName!);
        });
    return sortedKeys.map<String>((LogicalKeyboardKey key) => key.debugName.toString()).join(' + ');
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<Set<LogicalKeyboardKey>>('keys', _keys, description: debugDescribeKeys()),
    );
  }
}

/// A [DiagnosticsProperty] which handles formatting a `Map<LogicalKeySet, Intent>`
/// (the same type as the [Shortcuts.shortcuts] property) so that its
/// diagnostic output is human-readable.
class ShortcutMapProperty extends DiagnosticsProperty<Map<ShortcutActivator, Intent>> {
  /// Create a diagnostics property for `Map<ShortcutActivator, Intent>` objects,
  /// which are the same type as the [Shortcuts.shortcuts] property.
  ShortcutMapProperty(
    String super.name,
    Map<ShortcutActivator, Intent> super.value, {
    super.showName,
    Object super.defaultValue,
    super.level,
    super.description,
  });

  @override
  Map<ShortcutActivator, Intent> get value => super.value!;

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    return '{${value.keys.map<String>((ShortcutActivator keySet) => '{${keySet.debugDescribeKeys()}}: ${value[keySet]}').join(', ')}}';
  }
}

/// A shortcut key combination of a single key and modifiers.
///
/// The [SingleActivator] implements typical shortcuts such as:
///
///  * ArrowLeft
///  * Shift + Delete
///  * Control + Alt + Meta + Shift + A
///
/// More specifically, it creates shortcut key combinations that are composed of a
/// [trigger] key, and zero, some, or all of the four modifiers (control, shift,
/// alt, meta). The shortcut is activated when the following conditions are met:
///
///  * The incoming event is a down event for a [trigger] key.
///  * If [control] is true, then at least one control key must be held.
///    Otherwise, no control keys must be held.
///  * Similar conditions apply for the [alt], [shift], and [meta] keys.
///
/// This resembles the typical behavior of most operating systems, and handles
/// modifier keys differently from [LogicalKeySet] in the following way:
///
///  * [SingleActivator]s allow additional non-modifier keys being pressed in
///    order to activate the shortcut. For example, pressing key X while holding
///    ControlLeft *and key A* will be accepted by
///    `SingleActivator(LogicalKeyboardKey.keyX, control: true)`.
///  * [SingleActivator]s do not consider modifiers to be a trigger key. For
///    example, pressing ControlLeft while holding key X *will not* activate a
///    `SingleActivator(LogicalKeyboardKey.keyX, control: true)`.
///
/// See also:
///
///  * [CharacterActivator], an activator that represents key combinations
///    that result in the specified character, such as question mark.
class SingleActivator with Diagnosticable, MenuSerializableShortcut implements ShortcutActivator {
  /// Triggered when the [trigger] key is pressed while the modifiers are held.
  ///
  /// The [trigger] should be the non-modifier key that is pressed after all the
  /// modifiers, such as [LogicalKeyboardKey.keyC] as in `Ctrl+C`. It must not
  /// be a modifier key (sided or unsided).
  ///
  /// The [control], [shift], [alt], and [meta] flags represent whether the
  /// respective modifier keys should be held (true) or released (false). They
  /// default to false.
  ///
  /// By default, the activator is checked on all [KeyDownEvent] events for the
  /// [trigger] key. If [includeRepeats] is false, only [trigger] key events
  /// which are not [KeyRepeatEvent]s will be considered.
  ///
  /// {@tool dartpad}
  /// In the following example, the shortcut `Control + C` increases the
  /// counter:
  ///
  /// ** See code in examples/api/lib/widgets/shortcuts/single_activator.0.dart **
  /// {@end-tool}
  const SingleActivator(
    this.trigger, {
    this.control = false,
    this.shift = false,
    this.alt = false,
    this.meta = false,
    this.numLock = LockState.ignored,
    this.includeRepeats = true,
  }) : // The enumerated check with `identical` is cumbersome but the only way
       // since const constructors can not call functions such as `==` or
       // `Set.contains`. Checking with `identical` might not work when the
       // key object is created from ID, but it covers common cases.
       assert(
         !identical(trigger, LogicalKeyboardKey.control) &&
             !identical(trigger, LogicalKeyboardKey.controlLeft) &&
             !identical(trigger, LogicalKeyboardKey.controlRight) &&
             !identical(trigger, LogicalKeyboardKey.shift) &&
             !identical(trigger, LogicalKeyboardKey.shiftLeft) &&
             !identical(trigger, LogicalKeyboardKey.shiftRight) &&
             !identical(trigger, LogicalKeyboardKey.alt) &&
             !identical(trigger, LogicalKeyboardKey.altLeft) &&
             !identical(trigger, LogicalKeyboardKey.altRight) &&
             !identical(trigger, LogicalKeyboardKey.meta) &&
             !identical(trigger, LogicalKeyboardKey.metaLeft) &&
             !identical(trigger, LogicalKeyboardKey.metaRight),
       );

  /// The non-modifier key of the shortcut that is pressed after all modifiers
  /// to activate the shortcut.
  ///
  /// For example, for `Control + C`, [trigger] should be
  /// [LogicalKeyboardKey.keyC].
  final LogicalKeyboardKey trigger;

  /// Whether either (or both) control keys should be held for [trigger] to
  /// activate the shortcut.
  ///
  /// It defaults to false, meaning all Control keys must be released when the
  /// event is received in order to activate the shortcut. If it's true, then
  /// either or both Control keys must be pressed.
  ///
  /// See also:
  ///
  ///  * [LogicalKeyboardKey.controlLeft], [LogicalKeyboardKey.controlRight].
  final bool control;

  /// Whether either (or both) shift keys should be held for [trigger] to
  /// activate the shortcut.
  ///
  /// It defaults to false, meaning all Shift keys must be released when the
  /// event is received in order to activate the shortcut. If it's true, then
  /// either or both Shift keys must be pressed.
  ///
  /// See also:
  ///
  ///  * [LogicalKeyboardKey.shiftLeft], [LogicalKeyboardKey.shiftRight].
  final bool shift;

  /// Whether either (or both) alt keys should be held for [trigger] to
  /// activate the shortcut.
  ///
  /// It defaults to false, meaning all Alt keys must be released when the
  /// event is received in order to activate the shortcut. If it's true, then
  /// either or both Alt keys must be pressed.
  ///
  /// See also:
  ///
  ///  * [LogicalKeyboardKey.altLeft], [LogicalKeyboardKey.altRight].
  final bool alt;

  /// Whether either (or both) meta keys should be held for [trigger] to
  /// activate the shortcut.
  ///
  /// It defaults to false, meaning all Meta keys must be released when the
  /// event is received in order to activate the shortcut. If it's true, then
  /// either or both Meta keys must be pressed.
  ///
  /// See also:
  ///
  ///  * [LogicalKeyboardKey.metaLeft], [LogicalKeyboardKey.metaRight].
  final bool meta;

  /// Whether the NumLock key state should be checked for [trigger] to activate
  /// the shortcut.
  ///
  /// It defaults to [LockState.ignored], meaning the NumLock state is ignored
  /// when the event is received in order to activate the shortcut.
  /// If it's [LockState.locked], then the NumLock key must be locked.
  /// If it's [LockState.unlocked], then the NumLock key must be unlocked.
  ///
  /// See also:
  ///
  ///  * [LogicalKeyboardKey.numLock].
  final LockState numLock;

  /// Whether this activator accepts repeat events of the [trigger] key.
  ///
  /// If [includeRepeats] is true, the activator is checked on all
  /// [KeyDownEvent] or [KeyRepeatEvent]s for the [trigger] key. If
  /// [includeRepeats] is false, only [trigger] key events which are
  /// [KeyDownEvent]s will be considered.
  final bool includeRepeats;

  @override
  Iterable<LogicalKeyboardKey> get triggers => <LogicalKeyboardKey>[trigger];

  bool _shouldAcceptModifiers(Set<LogicalKeyboardKey> pressed) {
    return control == pressed.intersection(_controlSynonyms).isNotEmpty &&
        shift == pressed.intersection(_shiftSynonyms).isNotEmpty &&
        alt == pressed.intersection(_altSynonyms).isNotEmpty &&
        meta == pressed.intersection(_metaSynonyms).isNotEmpty;
  }

  bool _shouldAcceptNumLock(HardwareKeyboard state) {
    return switch (numLock) {
      LockState.ignored => true,
      LockState.locked => state.lockModesEnabled.contains(KeyboardLockMode.numLock),
      LockState.unlocked => !state.lockModesEnabled.contains(KeyboardLockMode.numLock),
    };
  }

  @override
  bool accepts(KeyEvent event, HardwareKeyboard state) {
    return (event is KeyDownEvent || (includeRepeats && event is KeyRepeatEvent)) &&
        triggers.contains(event.logicalKey) &&
        _shouldAcceptModifiers(state.logicalKeysPressed) &&
        _shouldAcceptNumLock(state);
  }

  @override
  ShortcutSerialization serializeForMenu() {
    return ShortcutSerialization.modifier(
      trigger,
      shift: shift,
      alt: alt,
      meta: meta,
      control: control,
    );
  }

  /// Returns a short and readable description of the key combination.
  ///
  /// Intended to be used in debug mode for logging purposes. In release mode,
  /// [debugDescribeKeys] returns an empty string.
  @override
  String debugDescribeKeys() {
    String result = '';
    assert(() {
      final List<String> keys = <String>[
        if (control) 'Control',
        if (alt) 'Alt',
        if (meta) 'Meta',
        if (shift) 'Shift',
        trigger.debugName ?? trigger.toStringShort(),
      ];
      result = keys.join(' + ');
      return true;
    }());
    return result;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(MessageProperty('keys', debugDescribeKeys()));
    properties.add(
      FlagProperty('includeRepeats', value: includeRepeats, ifFalse: 'excluding repeats'),
    );
  }
}

/// A shortcut combination that is triggered by a key event that produces a
/// specific character.
///
/// Keys often produce different characters when combined with modifiers. For
/// example, it might be helpful for the user to bring up a help menu by
/// pressing the question mark ('?'). However, there is no logical key that
/// directly represents a question mark. Although 'Shift+Slash' produces a '?'
/// character on a US keyboard, its logical key is still considered a Slash key,
/// and hard-coding 'Shift+Slash' in this situation is unfriendly to other
/// keyboard layouts.
///
/// For example, `CharacterActivator('?')` is triggered when a key combination
/// results in a question mark, which is 'Shift+Slash' on a US keyboard, but
/// 'Shift+Comma' on a French keyboard.
///
/// {@tool dartpad}
/// In the following example, when a key combination results in a question mark,
/// the [SnackBar] gets shown:
///
/// ** See code in examples/api/lib/widgets/shortcuts/character_activator.0.dart **
/// {@end-tool}
///
/// The [alt], [control], and [meta] flags represent whether the respective
/// modifier keys should be held (true) or released (false). They default to
/// false. [CharacterActivator] cannot check shifted keys, since the Shift key
/// affects the resulting character, and will accept whether either of the
/// Shift keys are pressed or not, as long as the key event produces the
/// correct character.
///
/// By default, the activator is checked on all [KeyDownEvent] or
/// [KeyRepeatEvent]s for the [character] in combination with the requested
/// modifier keys. If `includeRepeats` is false, only the [character] events
/// with that are [KeyDownEvent]s will be considered.
///
/// {@template flutter.widgets.shortcuts.CharacterActivator.alt}
/// On macOS and iOS, the [alt] flag indicates that the Option key (⌥) is
/// pressed. Because the Option key affects the character generated on these
/// platforms, it can be unintuitive to define [CharacterActivator]s for them.
///
/// For instance, if you want the shortcut to trigger when Option+s (⌥-s) is
/// pressed, and what you intend is to trigger whenever the character 'ß' is
/// produced, you would use `CharacterActivator('ß')` or
/// `CharacterActivator('ß', alt: true)` instead of `CharacterActivator('s',
/// alt: true)`. This is because `CharacterActivator('s', alt: true)` will
/// never trigger, since the 's' character can't be produced when the Option
/// key is held down.
///
/// If what is intended is that the shortcut is triggered when Option+s (⌥-s)
/// is pressed, regardless of which character is produced, it is better to use
/// [SingleActivator], as in `SingleActivator(LogicalKeyboardKey.keyS, alt:
/// true)`.
/// {@endtemplate}
///
/// See also:
///
///  * [SingleActivator], an activator that represents a single key combined
///    with modifiers, such as `Ctrl+C` or `Ctrl-Right Arrow`.
class CharacterActivator
    with Diagnosticable, MenuSerializableShortcut
    implements ShortcutActivator {
  /// Triggered when the key event yields the given character.
  const CharacterActivator(
    this.character, {
    this.alt = false,
    this.control = false,
    this.meta = false,
    this.includeRepeats = true,
  });

  /// Whether either (or both) Alt keys should be held for the [character] to
  /// activate the shortcut.
  ///
  /// It defaults to false, meaning all Alt keys must be released when the event
  /// is received in order to activate the shortcut. If it's true, then either
  /// one or both Alt keys must be pressed.
  ///
  /// {@macro flutter.widgets.shortcuts.CharacterActivator.alt}
  ///
  /// See also:
  ///
  /// * [LogicalKeyboardKey.altLeft], [LogicalKeyboardKey.altRight].
  final bool alt;

  /// Whether either (or both) Control keys should be held for the [character]
  /// to activate the shortcut.
  ///
  /// It defaults to false, meaning all Control keys must be released when the
  /// event is received in order to activate the shortcut. If it's true, then
  /// either one or both Control keys must be pressed.
  ///
  /// See also:
  ///
  ///  * [LogicalKeyboardKey.controlLeft], [LogicalKeyboardKey.controlRight].
  final bool control;

  /// Whether either (or both) Meta keys should be held for the [character] to
  /// activate the shortcut.
  ///
  /// It defaults to false, meaning all Meta keys must be released when the
  /// event is received in order to activate the shortcut. If it's true, then
  /// either one or both Meta keys must be pressed.
  ///
  /// See also:
  ///
  ///  * [LogicalKeyboardKey.metaLeft], [LogicalKeyboardKey.metaRight].
  final bool meta;

  /// Whether this activator accepts repeat events of the [character].
  ///
  /// If [includeRepeats] is true, the activator is checked on all
  /// [KeyDownEvent] and [KeyRepeatEvent]s for the [character]. If
  /// [includeRepeats] is false, only the [character] events that are
  /// [KeyDownEvent]s will be considered.
  final bool includeRepeats;

  /// The character which triggers the shortcut.
  ///
  /// This is typically a single-character string, such as '?' or 'œ', although
  /// [CharacterActivator] doesn't check the length of [character] or whether it
  /// can be matched by any key combination at all. It is case-sensitive, since
  /// the [character] is directly compared by `==` to the character reported by
  /// the platform.
  ///
  /// See also:
  ///
  ///  * [KeyEvent.character], the character of a key event.
  final String character;

  @override
  Iterable<LogicalKeyboardKey>? get triggers => null;

  bool _shouldAcceptModifiers(Set<LogicalKeyboardKey> pressed) {
    // Doesn't look for shift, since the character will encode that.
    return control == pressed.intersection(_controlSynonyms).isNotEmpty &&
        alt == pressed.intersection(_altSynonyms).isNotEmpty &&
        meta == pressed.intersection(_metaSynonyms).isNotEmpty;
  }

  @override
  bool accepts(KeyEvent event, HardwareKeyboard state) {
    // Ignore triggers, since we're only interested in the character.
    return event.character == character &&
        (event is KeyDownEvent || (includeRepeats && event is KeyRepeatEvent)) &&
        _shouldAcceptModifiers(state.logicalKeysPressed);
  }

  @override
  String debugDescribeKeys() {
    String result = '';
    assert(() {
      final List<String> keys = <String>[
        if (alt) 'Alt',
        if (control) 'Control',
        if (meta) 'Meta',
        "'$character'",
      ];
      result = keys.join(' + ');
      return true;
    }());
    return result;
  }

  @override
  ShortcutSerialization serializeForMenu() {
    return ShortcutSerialization.character(character, alt: alt, control: control, meta: meta);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(MessageProperty('character', debugDescribeKeys()));
    properties.add(
      FlagProperty('includeRepeats', value: includeRepeats, ifFalse: 'excluding repeats'),
    );
  }
}

class _ActivatorIntentPair with Diagnosticable {
  const _ActivatorIntentPair(this.activator, this.intent);
  final ShortcutActivator activator;
  final Intent intent;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<String>('activator', activator.debugDescribeKeys()));
    properties.add(DiagnosticsProperty<Intent>('intent', intent));
  }
}

/// A manager of keyboard shortcut bindings used by [Shortcuts] to handle key
/// events.
///
/// The manager may be listened to (with [addListener]/[removeListener]) for
/// change notifications when the shortcuts change.
///
/// Typically, a [Shortcuts] widget supplies its own manager, but in uncommon
/// cases where overriding the usual shortcut manager behavior is desired, a
/// subclassed [ShortcutManager] may be supplied.
class ShortcutManager with Diagnosticable, ChangeNotifier {
  /// Constructs a [ShortcutManager].
  ShortcutManager({
    Map<ShortcutActivator, Intent> shortcuts = const <ShortcutActivator, Intent>{},
    this.modal = false,
  }) : _shortcuts = shortcuts {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  /// True if the [ShortcutManager] should not pass on keys that it doesn't
  /// handle to any key-handling widgets that are ancestors to this one.
  ///
  /// Setting [modal] to true will prevent any key event given to this manager
  /// from being given to any ancestor managers, even if that key doesn't appear
  /// in the [shortcuts] map.
  ///
  /// The net effect of setting [modal] to true is to return
  /// [KeyEventResult.skipRemainingHandlers] from [handleKeypress] if it does
  /// not exist in the shortcut map, instead of returning
  /// [KeyEventResult.ignored].
  final bool modal;

  /// Returns the shortcut map.
  ///
  /// When the map is changed, listeners to this manager will be notified.
  ///
  /// The returned map should not be modified.
  Map<ShortcutActivator, Intent> get shortcuts => _shortcuts;
  Map<ShortcutActivator, Intent> _shortcuts = <ShortcutActivator, Intent>{};
  set shortcuts(Map<ShortcutActivator, Intent> value) {
    if (!mapEquals<ShortcutActivator, Intent>(_shortcuts, value)) {
      _shortcuts = value;
      _indexedShortcutsCache = null;
      notifyListeners();
    }
  }

  static Map<LogicalKeyboardKey?, List<_ActivatorIntentPair>> _indexShortcuts(
    Map<ShortcutActivator, Intent> source,
  ) {
    final Map<LogicalKeyboardKey?, List<_ActivatorIntentPair>> result =
        <LogicalKeyboardKey?, List<_ActivatorIntentPair>>{};
    source.forEach((ShortcutActivator activator, Intent intent) {
      // This intermediate variable is necessary to comply with Dart analyzer.
      final Iterable<LogicalKeyboardKey?>? nullableTriggers = activator.triggers;
      for (final LogicalKeyboardKey? trigger in nullableTriggers ?? <LogicalKeyboardKey?>[null]) {
        result
            .putIfAbsent(trigger, () => <_ActivatorIntentPair>[])
            .add(_ActivatorIntentPair(activator, intent));
      }
    });
    return result;
  }

  Map<LogicalKeyboardKey?, List<_ActivatorIntentPair>> get _indexedShortcuts {
    return _indexedShortcutsCache ??= _indexShortcuts(shortcuts);
  }

  Map<LogicalKeyboardKey?, List<_ActivatorIntentPair>>? _indexedShortcutsCache;

  Iterable<_ActivatorIntentPair> _getCandidates(LogicalKeyboardKey key) {
    return <_ActivatorIntentPair>[
      ..._indexedShortcuts[key] ?? <_ActivatorIntentPair>[],
      ..._indexedShortcuts[null] ?? <_ActivatorIntentPair>[],
    ];
  }

  /// Returns the [Intent], if any, that matches the current set of pressed
  /// keys.
  ///
  /// Returns null if no intent matches the current set of pressed keys.
  Intent? _find(KeyEvent event, HardwareKeyboard state) {
    for (final _ActivatorIntentPair activatorIntent in _getCandidates(event.logicalKey)) {
      if (activatorIntent.activator.accepts(event, state)) {
        return activatorIntent.intent;
      }
    }
    return null;
  }

  /// Handles a key press `event` in the given `context`.
  ///
  /// If a key mapping is found, then the associated action will be invoked
  /// using the [Intent] activated by the [ShortcutActivator] in the [shortcuts]
  /// map, and the currently focused widget's context (from
  /// [FocusManager.primaryFocus]).
  ///
  /// Returns a [KeyEventResult.handled] if an action was invoked, otherwise a
  /// [KeyEventResult.skipRemainingHandlers] if [modal] is true, or if it maps
  /// to a [DoNothingAction] with [DoNothingAction.consumesKey] set to false,
  /// and in all other cases returns [KeyEventResult.ignored].
  ///
  /// In order for an action to be invoked (and [KeyEventResult.handled]
  /// returned), a [ShortcutActivator] must accept the given [KeyEvent], be
  /// mapped to an [Intent], the [Intent] must be mapped to an [Action], and the
  /// [Action] must be enabled.
  @protected
  KeyEventResult handleKeypress(BuildContext context, KeyEvent event) {
    // Marking some variables as "late" ensures that they aren't evaluated unless needed.
    late final Intent? intent = _find(event, HardwareKeyboard.instance);
    late final BuildContext? context = primaryFocus?.context;
    late final Action<Intent>? action = Actions.maybeFind<Intent>(context!, intent: intent);

    if (intent != null && context != null && action != null) {
      final (bool enabled, Object? invokeResult) = Actions.of(
        context,
      ).invokeActionIfEnabled(action, intent, context);

      if (enabled) {
        return action.toKeyEventResult(intent, invokeResult);
      }
    }
    return modal ? KeyEventResult.skipRemainingHandlers : KeyEventResult.ignored;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Map<ShortcutActivator, Intent>>('shortcuts', shortcuts));
    properties.add(FlagProperty('modal', value: modal, ifTrue: 'modal', defaultValue: false));
  }
}

/// A widget that creates key bindings to specific actions for its
/// descendants.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=6ZcQmdoz9N8}
///
/// This widget establishes a [ShortcutManager] to be used by its descendants
/// when invoking an [Action] via a keyboard key combination that maps to an
/// [Intent].
///
/// This is similar to but more powerful than the [CallbackShortcuts] widget.
/// Unlike [CallbackShortcuts], this widget separates key bindings and their
/// implementations. This separation allows [Shortcuts] to have key bindings
/// that adapt to the focused context. For example, the desired action for a
/// deletion intent may be to delete a character in a text input, or to delete
/// a file in a file menu.
///
/// See the article on
/// [Using Actions and Shortcuts](https://flutter.dev/to/actions-shortcuts)
/// for a detailed explanation.
///
/// {@tool dartpad}
/// Here, we will use the [Shortcuts] and [Actions] widgets to add and subtract
/// from a counter. When the child widget has keyboard focus, and a user presses
/// the keys that have been defined in [Shortcuts], the action that is bound
/// to the appropriate [Intent] for the key is invoked.
///
/// It also shows the use of a [CallbackAction] to avoid creating a new [Action]
/// subclass.
///
/// ** See code in examples/api/lib/widgets/shortcuts/shortcuts.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This slightly more complicated, but more flexible, example creates a custom
/// [Action] subclass to increment and decrement within a widget (a [Column])
/// that has keyboard focus. When the user presses the up and down arrow keys,
/// the counter will increment and decrement a data model using the custom
/// actions.
///
/// One thing that this demonstrates is passing arguments to the [Intent] to be
/// carried to the [Action]. This shows how actions can get data either from
/// their own construction (like the `model` in this example), or from the
/// intent passed to them when invoked (like the increment `amount` in this
/// example).
///
/// ** See code in examples/api/lib/widgets/shortcuts/shortcuts.1.dart **
/// {@end-tool}
///
/// See also:
///
///  * [CallbackShortcuts], a simpler but less flexible widget that defines key
///    bindings that invoke callbacks.
///  * [Intent], a class for containing a description of a user action to be
///    invoked.
///  * [Action], a class for defining an invocation of a user action.
///  * [CallbackAction], a class for creating an action from a callback.
class Shortcuts extends StatefulWidget {
  /// Creates a const [Shortcuts] widget that owns the map of shortcuts and
  /// creates its own manager.
  ///
  /// When using this constructor, [manager] will return null.
  ///
  /// The [child] and [shortcuts] arguments are required.
  ///
  /// See also:
  ///
  ///  * [Shortcuts.manager], a constructor that uses a [ShortcutManager] to
  ///    manage the shortcuts list instead.
  const Shortcuts({
    super.key,
    required Map<ShortcutActivator, Intent> shortcuts,
    required this.child,
    this.debugLabel,
    this.includeSemantics = true,
  }) : _shortcuts = shortcuts,
       manager = null;

  /// Creates a const [Shortcuts] widget that uses the [manager] to
  /// manage the map of shortcuts.
  ///
  /// If this constructor is used, [shortcuts] will return the contents of
  /// [ShortcutManager.shortcuts].
  ///
  /// The [child] and [manager] arguments are required.
  const Shortcuts.manager({
    super.key,
    required ShortcutManager this.manager,
    required this.child,
    this.debugLabel,
    this.includeSemantics = true,
  }) : _shortcuts = const <ShortcutActivator, Intent>{};

  /// The [ShortcutManager] that will manage the mapping between key
  /// combinations and [Action]s.
  ///
  /// If this widget was created with [Shortcuts.manager], then
  /// [ShortcutManager.shortcuts] will be used as the source for shortcuts. If
  /// the unnamed constructor is used, this manager will be null, and a
  /// default-constructed [ShortcutManager] will be used.
  final ShortcutManager? manager;

  /// {@template flutter.widgets.shortcuts.shortcuts}
  /// The map of shortcuts that describes the mapping between a key sequence
  /// defined by a [ShortcutActivator] and the [Intent] that will be emitted
  /// when that key sequence is pressed.
  /// {@endtemplate}
  Map<ShortcutActivator, Intent> get shortcuts {
    return manager == null ? _shortcuts : manager!.shortcuts;
  }

  final Map<ShortcutActivator, Intent> _shortcuts;

  /// The child widget for this [Shortcuts] widget.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// The debug label that is printed for this node when logged.
  ///
  /// If this label is set, then it will be displayed instead of the shortcut
  /// map when logged.
  ///
  /// This allows simplifying the diagnostic output to avoid cluttering it
  /// unnecessarily with large default shortcut maps.
  final String? debugLabel;

  /// {@macro flutter.widgets.Focus.includeSemantics}
  final bool includeSemantics;

  @override
  State<Shortcuts> createState() => _ShortcutsState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<ShortcutManager>('manager', manager, defaultValue: null));
    properties.add(
      ShortcutMapProperty(
        'shortcuts',
        shortcuts,
        description: debugLabel?.isNotEmpty ?? false ? debugLabel : null,
      ),
    );
  }
}

class _ShortcutsState extends State<Shortcuts> {
  ShortcutManager? _internalManager;
  ShortcutManager get manager => widget.manager ?? _internalManager!;

  @override
  void dispose() {
    _internalManager?.dispose();
    super.dispose();
  }

  @override
  void initState() {
    super.initState();
    if (widget.manager == null) {
      _internalManager = ShortcutManager();
      _internalManager!.shortcuts = widget.shortcuts;
    }
  }

  @override
  void didUpdateWidget(Shortcuts oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.manager != oldWidget.manager) {
      if (widget.manager != null) {
        _internalManager?.dispose();
        _internalManager = null;
      } else {
        _internalManager ??= ShortcutManager();
      }
    }
    _internalManager?.shortcuts = widget.shortcuts;
  }

  KeyEventResult _handleOnKeyEvent(FocusNode node, KeyEvent event) {
    if (node.context == null) {
      return KeyEventResult.ignored;
    }
    return manager.handleKeypress(node.context!, event);
  }

  @override
  Widget build(BuildContext context) {
    return Focus(
      debugLabel: '$Shortcuts',
      canRequestFocus: false,
      onKeyEvent: _handleOnKeyEvent,
      includeSemantics: widget.includeSemantics,
      child: widget.child,
    );
  }
}

/// A widget that binds key combinations to specific callbacks.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=VcQQ1ns_qNY}
///
/// This is similar to but simpler than the [Shortcuts] widget as it doesn't
/// require [Intent]s and [Actions] widgets. Instead, it accepts a map
/// of [ShortcutActivator]s to [VoidCallback]s.
///
/// Unlike [Shortcuts], this widget does not separate key bindings and their
/// implementations. This separation allows [Shortcuts] to have key bindings
/// that adapt to the focused context. For example, the desired action for a
/// deletion intent may be to delete a character in a text input, or to delete
/// a file in a file menu.
///
/// {@tool dartpad}
/// This example uses the [CallbackShortcuts] widget to add and subtract
/// from a counter when the up or down arrow keys are pressed.
///
/// ** See code in examples/api/lib/widgets/shortcuts/callback_shortcuts.0.dart **
/// {@end-tool}
///
/// [Shortcuts] and [CallbackShortcuts] can both be used in the same app. As
/// with any key handling widget, if this widget handles a key event then
/// widgets above it in the focus chain will not receive the event. This means
/// that if this widget handles a key, then an ancestor [Shortcuts] widget (or
/// any other key handling widget) will not receive that key. Similarly, if
/// a descendant of this widget handles the key, then the key event will not
/// reach this widget for handling.
///
/// See the article on
/// [Using Actions and Shortcuts](https://flutter.dev/to/actions-shortcuts)
/// for a detailed explanation.
///
/// See also:
///  * [Shortcuts], a more powerful widget for defining key bindings.
///  * [Focus], a widget that defines which widgets can receive keyboard focus.
class CallbackShortcuts extends StatelessWidget {
  /// Creates a const [CallbackShortcuts] widget.
  const CallbackShortcuts({super.key, required this.bindings, required this.child});

  /// A map of key combinations to callbacks used to define the shortcut
  /// bindings.
  ///
  /// If a descendant of this widget has focus, and a key is pressed, the
  /// activator keys of this map will be asked if they accept the key event. If
  /// they do, then the corresponding callback is invoked, and the key event
  /// propagation is halted. If none of the activators accept the key event,
  /// then the key event continues to be propagated up the focus chain.
  ///
  /// If more than one activator accepts the key event, then all of the
  /// callbacks associated with activators that accept the key event are
  /// invoked.
  ///
  /// Some examples of [ShortcutActivator] subclasses that can be used to define
  /// the key combinations here are [SingleActivator], [CharacterActivator], and
  /// [LogicalKeySet].
  final Map<ShortcutActivator, VoidCallback> bindings;

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  // A helper function to make the stack trace more useful if the callback
  // throws, by providing the activator and event as arguments that will appear
  // in the stack trace.
  bool _applyKeyEventBinding(ShortcutActivator activator, KeyEvent event) {
    if (activator.accepts(event, HardwareKeyboard.instance)) {
      bindings[activator]!.call();
      return true;
    }
    return false;
  }

  @override
  Widget build(BuildContext context) {
    return Focus(
      canRequestFocus: false,
      skipTraversal: true,
      onKeyEvent: (FocusNode node, KeyEvent event) {
        KeyEventResult result = KeyEventResult.ignored;
        // Activates all key bindings that match, returns "handled" if any handle it.
        for (final ShortcutActivator activator in bindings.keys) {
          result = _applyKeyEventBinding(activator, event) ? KeyEventResult.handled : result;
        }
        return result;
      },
      child: child,
    );
  }
}

/// A entry returned by [ShortcutRegistry.addAll] that allows the caller to
/// identify the shortcuts they registered with the [ShortcutRegistry] through
/// the [ShortcutRegistrar].
///
/// When the entry is no longer needed, [dispose] should be called, and the
/// entry should no longer be used.
class ShortcutRegistryEntry {
  // Tokens can only be created by the ShortcutRegistry.
  const ShortcutRegistryEntry._(this.registry);

  /// The [ShortcutRegistry] that this entry was issued by.
  final ShortcutRegistry registry;

  /// Replaces the given shortcut bindings in the [ShortcutRegistry] that this
  /// entry was created from.
  ///
  /// This method will assert in debug mode if another [ShortcutRegistryEntry]
  /// exists (i.e. hasn't been disposed of) that has already added a given
  /// shortcut.
  ///
  /// It will also assert if this entry has already been disposed.
  ///
  /// If two equivalent, but different, [ShortcutActivator]s are added, all of
  /// them will be executed when triggered. For example, if both
  /// `SingleActivator(LogicalKeyboardKey.keyA)` and `CharacterActivator('a')`
  /// are added, then both will be executed when an "a" key is pressed.
  void replaceAll(Map<ShortcutActivator, Intent> value) {
    registry._replaceAll(this, value);
  }

  /// Called when the entry is no longer needed.
  ///
  /// Call this will remove all shortcuts associated with this
  /// [ShortcutRegistryEntry] from the [registry].
  @mustCallSuper
  void dispose() {
    registry._disposeEntry(this);
  }
}

/// A class used by [ShortcutRegistrar] that allows adding or removing shortcut
/// bindings by descendants of the [ShortcutRegistrar].
///
/// You can reach the nearest [ShortcutRegistry] using [of] and [maybeOf].
///
/// The registry may be listened to (with [addListener]/[removeListener]) for
/// change notifications when the registered shortcuts change. Change
/// notifications take place after the current frame is drawn, so that
/// widgets that are not descendants of the registry can listen to it (e.g. in
/// overlays).
class ShortcutRegistry with ChangeNotifier {
  /// Creates an instance of [ShortcutRegistry].
  ShortcutRegistry() {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  bool _notificationScheduled = false;
  bool _disposed = false;

  @override
  void dispose() {
    super.dispose();
    _disposed = true;
  }

  /// Gets the combined shortcut bindings from all contexts that are registered
  /// with this [ShortcutRegistry].
  ///
  /// Listeners will be notified when the value returned by this getter changes.
  ///
  /// Returns a copy: modifying the returned map will have no effect.
  Map<ShortcutActivator, Intent> get shortcuts {
    assert(ChangeNotifier.debugAssertNotDisposed(this));
    return <ShortcutActivator, Intent>{
      for (final MapEntry<ShortcutRegistryEntry, Map<ShortcutActivator, Intent>> entry
          in _registeredShortcuts.entries)
        ...entry.value,
    };
  }

  final Map<ShortcutRegistryEntry, Map<ShortcutActivator, Intent>> _registeredShortcuts =
      <ShortcutRegistryEntry, Map<ShortcutActivator, Intent>>{};

  /// Adds all the given shortcut bindings to this [ShortcutRegistry], and
  /// returns a entry for managing those bindings.
  ///
  /// The entry should have [ShortcutRegistryEntry.dispose] called on it when
  /// these shortcuts are no longer needed. This will remove them from the
  /// registry, and invalidate the entry.
  ///
  /// This method will assert in debug mode if another entry exists (i.e. hasn't
  /// been disposed of) that has already added a given shortcut.
  ///
  /// If two equivalent, but different, [ShortcutActivator]s are added, all of
  /// them will be executed when triggered. For example, if both
  /// `SingleActivator(LogicalKeyboardKey.keyA)` and `CharacterActivator('a')`
  /// are added, then both will be executed when an "a" key is pressed.
  ///
  /// See also:
  ///
  ///  * [ShortcutRegistryEntry.replaceAll], a function used to replace the set of
  ///    shortcuts associated with a particular entry.
  ///  * [ShortcutRegistryEntry.dispose], a function used to remove the set of
  ///    shortcuts associated with a particular entry.
  ShortcutRegistryEntry addAll(Map<ShortcutActivator, Intent> value) {
    assert(ChangeNotifier.debugAssertNotDisposed(this));
    assert(value.isNotEmpty, 'Cannot register an empty map of shortcuts');
    final ShortcutRegistryEntry entry = ShortcutRegistryEntry._(this);
    _registeredShortcuts[entry] = value;
    assert(_debugCheckForDuplicates());
    _notifyListenersNextFrame();
    return entry;
  }

  // Subscriber notification has to happen in the next frame because shortcuts
  // are often registered that affect things in the overlay or different parts
  // of the tree, and so can cause build ordering issues if notifications happen
  // during the build. The _notificationScheduled check makes sure we only
  // notify once per frame.
  void _notifyListenersNextFrame() {
    if (!_notificationScheduled) {
      SchedulerBinding.instance.addPostFrameCallback((Duration _) {
        _notificationScheduled = false;
        if (!_disposed) {
          notifyListeners();
        }
      }, debugLabel: 'ShortcutRegistry.notifyListeners');
      _notificationScheduled = true;
    }
  }

  /// Returns the [ShortcutRegistry] that belongs to the [ShortcutRegistrar]
  /// which most tightly encloses the given [BuildContext].
  ///
  /// If no [ShortcutRegistrar] widget encloses the context given, [of] will
  /// throw an exception in debug mode.
  ///
  /// There is a default [ShortcutRegistrar] instance in [WidgetsApp], so if
  /// [WidgetsApp], [MaterialApp] or [CupertinoApp] are used, an additional
  /// [ShortcutRegistrar] isn't needed.
  ///
  /// See also:
  ///
  ///  * [maybeOf], which is similar to this function, but will return null if
  ///    it doesn't find a [ShortcutRegistrar] ancestor.
  static ShortcutRegistry of(BuildContext context) {
    final _ShortcutRegistrarScope? inherited =
        context.dependOnInheritedWidgetOfExactType<_ShortcutRegistrarScope>();
    assert(() {
      if (inherited == null) {
        throw FlutterError(
          'Unable to find a $ShortcutRegistrar widget in the context.\n'
          '$ShortcutRegistrar.of() was called with a context that does not contain a '
          '$ShortcutRegistrar widget.\n'
          'No $ShortcutRegistrar ancestor could be found starting from the context that was '
          'passed to $ShortcutRegistrar.of().\n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    return inherited!.registry;
  }

  /// Returns [ShortcutRegistry] of the [ShortcutRegistrar] that most tightly
  /// encloses the given [BuildContext].
  ///
  /// If no [ShortcutRegistrar] widget encloses the given context, [maybeOf]
  /// will return null.
  ///
  /// There is a default [ShortcutRegistrar] instance in [WidgetsApp], so if
  /// [WidgetsApp], [MaterialApp] or [CupertinoApp] are used, an additional
  /// [ShortcutRegistrar] isn't needed.
  ///
  /// See also:
  ///
  ///  * [of], which is similar to this function, but returns a non-nullable
  ///    result, and will throw an exception if it doesn't find a
  ///    [ShortcutRegistrar] ancestor.
  static ShortcutRegistry? maybeOf(BuildContext context) {
    final _ShortcutRegistrarScope? inherited =
        context.dependOnInheritedWidgetOfExactType<_ShortcutRegistrarScope>();
    return inherited?.registry;
  }

  // Replaces all the shortcuts associated with the given entry from this
  // registry.
  void _replaceAll(ShortcutRegistryEntry entry, Map<ShortcutActivator, Intent> value) {
    assert(ChangeNotifier.debugAssertNotDisposed(this));
    assert(_debugCheckEntryIsValid(entry));
    _registeredShortcuts[entry] = value;
    assert(_debugCheckForDuplicates());
    _notifyListenersNextFrame();
  }

  // Removes all the shortcuts associated with the given entry from this
  // registry.
  void _disposeEntry(ShortcutRegistryEntry entry) {
    assert(_debugCheckEntryIsValid(entry));
    if (_registeredShortcuts.remove(entry) != null) {
      _notifyListenersNextFrame();
    }
  }

  bool _debugCheckEntryIsValid(ShortcutRegistryEntry entry) {
    if (!_registeredShortcuts.containsKey(entry)) {
      if (entry.registry == this) {
        throw FlutterError(
          'entry ${describeIdentity(entry)} is invalid.\n'
          'The entry has already been disposed of. Tokens are not valid after '
          'dispose is called on them, and should no longer be used.',
        );
      } else {
        throw FlutterError(
          'Foreign entry ${describeIdentity(entry)} used.\n'
          'This entry was not created by this registry, it was created by '
          '${describeIdentity(entry.registry)}, and should be used with that '
          'registry instead.',
        );
      }
    }
    return true;
  }

  bool _debugCheckForDuplicates() {
    final Map<ShortcutActivator, ShortcutRegistryEntry?> previous =
        <ShortcutActivator, ShortcutRegistryEntry?>{};
    for (final MapEntry<ShortcutRegistryEntry, Map<ShortcutActivator, Intent>> tokenEntry
        in _registeredShortcuts.entries) {
      for (final ShortcutActivator shortcut in tokenEntry.value.keys) {
        if (previous.containsKey(shortcut)) {
          throw FlutterError(
            '$ShortcutRegistry: Received a duplicate registration for the '
            'shortcut $shortcut in ${describeIdentity(tokenEntry.key)} and ${previous[shortcut]}.',
          );
        }
        previous[shortcut] = tokenEntry.key;
      }
    }
    return true;
  }
}

/// A widget that holds a [ShortcutRegistry] which allows descendants to add,
/// remove, or replace shortcuts.
///
/// This widget holds a [ShortcutRegistry] so that its descendants can find it
/// with [ShortcutRegistry.of] or [ShortcutRegistry.maybeOf].
///
/// The registered shortcuts are valid whenever a widget below this one in the
/// hierarchy has focus.
///
/// To add shortcuts to the registry, call [ShortcutRegistry.of] or
/// [ShortcutRegistry.maybeOf] to get the [ShortcutRegistry], and then add them
/// using [ShortcutRegistry.addAll], which will return a [ShortcutRegistryEntry]
/// which must be disposed by calling [ShortcutRegistryEntry.dispose] when the
/// shortcuts are no longer needed.
///
/// To replace or update the shortcuts in the registry, call
/// [ShortcutRegistryEntry.replaceAll].
///
/// To remove previously added shortcuts from the registry, call
/// [ShortcutRegistryEntry.dispose] on the entry returned by
/// [ShortcutRegistry.addAll].
class ShortcutRegistrar extends StatefulWidget {
  /// Creates a const [ShortcutRegistrar].
  ///
  /// The [child] parameter is required.
  const ShortcutRegistrar({super.key, required this.child});

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  @override
  State<ShortcutRegistrar> createState() => _ShortcutRegistrarState();
}

class _ShortcutRegistrarState extends State<ShortcutRegistrar> {
  final ShortcutRegistry registry = ShortcutRegistry();
  final ShortcutManager manager = ShortcutManager();

  @override
  void initState() {
    super.initState();
    registry.addListener(_shortcutsChanged);
  }

  void _shortcutsChanged() {
    // This shouldn't need to update the widget, and avoids calling setState
    // during build phase.
    manager.shortcuts = registry.shortcuts;
  }

  @override
  void dispose() {
    registry.removeListener(_shortcutsChanged);
    registry.dispose();
    manager.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return _ShortcutRegistrarScope(
      registry: registry,
      child: Shortcuts.manager(manager: manager, child: widget.child),
    );
  }
}

class _ShortcutRegistrarScope extends InheritedWidget {
  const _ShortcutRegistrarScope({required this.registry, required super.child});

  final ShortcutRegistry registry;

  @override
  bool updateShouldNotify(covariant _ShortcutRegistrarScope oldWidget) {
    return registry != oldWidget.registry;
  }
}
