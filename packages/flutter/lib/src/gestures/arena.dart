// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'events.dart';
library;

import 'dart:async';

import 'package:flutter/foundation.dart';

import 'debug.dart';

/// Whether the gesture was accepted or rejected.
enum GestureDisposition {
  /// This gesture was accepted as the interpretation of the user's input.
  accepted,

  /// This gesture was rejected as the interpretation of the user's input.
  rejected,
}

/// Represents an object participating in an arena.
///
/// Receives callbacks from the GestureArena to notify the object when it wins
/// or loses a gesture negotiation. Exactly one of [acceptGesture] or
/// [rejectGesture] will be called for each arena this member was added to,
/// regardless of what caused the arena to be resolved. For example, if a
/// member resolves the arena itself, that member still receives an
/// [acceptGesture] callback.
abstract class GestureArenaMember {
  /// Called when this member wins the arena for the given pointer id.
  void acceptGesture(int pointer);

  /// Called when this member loses the arena for the given pointer id.
  void rejectGesture(int pointer);
}

/// An interface to pass information to an arena.
///
/// A given [GestureArenaMember] can have multiple entries in multiple arenas
/// with different pointer ids.
class GestureArenaEntry {
  GestureArenaEntry._(this._arena, this._pointer, this._member);

  final GestureArenaManager _arena;
  final int _pointer;
  final GestureArenaMember _member;

  /// Call this member to claim victory (with accepted) or admit defeat (with rejected).
  ///
  /// It's fine to attempt to resolve a gesture recognizer for an arena that is
  /// already resolved.
  void resolve(GestureDisposition disposition) {
    _arena._resolve(_pointer, _member, disposition);
  }
}

class _GestureArena {
  final List<GestureArenaMember> members = <GestureArenaMember>[];
  bool isOpen = true;
  bool isHeld = false;
  bool hasPendingSweep = false;

  /// If a member attempts to win while the arena is still open, it becomes the
  /// "eager winner". We look for an eager winner when closing the arena to new
  /// participants, and if there is one, we resolve the arena in its favor at
  /// that time.
  GestureArenaMember? eagerWinner;

  void add(GestureArenaMember member) {
    assert(isOpen);
    members.add(member);
  }

  @override
  String toString() {
    final StringBuffer buffer = StringBuffer();
    if (members.isEmpty) {
      buffer.write('<empty>');
    } else {
      buffer.write(
        members
            .map<String>((GestureArenaMember member) {
              if (member == eagerWinner) {
                return '$member (eager winner)';
              }
              return '$member';
            })
            .join(', '),
      );
    }
    if (isOpen) {
      buffer.write(' [open]');
    }
    if (isHeld) {
      buffer.write(' [held]');
    }
    if (hasPendingSweep) {
      buffer.write(' [hasPendingSweep]');
    }
    return buffer.toString();
  }
}

/// Used for disambiguating the meaning of sequences of pointer events.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=Q85LBtBdi0U}
///
/// The first member to accept or the last member to not reject wins.
///
/// See <https://flutter.dev/to/gesture-disambiguation> for more
/// information about the role this class plays in the gesture system.
///
/// To debug problems with gestures, consider using
/// [debugPrintGestureArenaDiagnostics].
class GestureArenaManager {
  final Map<int, _GestureArena> _arenas = <int, _GestureArena>{};

  /// Adds a new member (e.g., gesture recognizer) to the arena.
  GestureArenaEntry add(int pointer, GestureArenaMember member) {
    final _GestureArena state = _arenas.putIfAbsent(pointer, () {
      assert(_debugLogDiagnostic(pointer, '★ Opening new gesture arena.'));
      return _GestureArena();
    });
    state.add(member);
    assert(_debugLogDiagnostic(pointer, 'Adding: $member'));
    return GestureArenaEntry._(this, pointer, member);
  }

  /// Prevents new members from entering the arena.
  ///
  /// Called after the framework has finished dispatching the pointer down event.
  void close(int pointer) {
    final _GestureArena? state = _arenas[pointer];
    if (state == null) {
      return; // This arena either never existed or has been resolved.
    }
    state.isOpen = false;
    assert(_debugLogDiagnostic(pointer, 'Closing', state));
    _tryToResolveArena(pointer, state);
  }

  /// Forces resolution of the arena, giving the win to the first member.
  ///
  /// Sweep is typically after all the other processing for a [PointerUpEvent]
  /// have taken place. It ensures that multiple passive gestures do not cause a
  /// stalemate that prevents the user from interacting with the app.
  ///
  /// Recognizers that wish to delay resolving an arena past [PointerUpEvent]
  /// should call [hold] to delay sweep until [release] is called.
  ///
  /// See also:
  ///
  ///  * [hold]
  ///  * [release]
  void sweep(int pointer) {
    final _GestureArena? state = _arenas[pointer];
    if (state == null) {
      return; // This arena either never existed or has been resolved.
    }
    assert(!state.isOpen);
    if (state.isHeld) {
      state.hasPendingSweep = true;
      assert(_debugLogDiagnostic(pointer, 'Delaying sweep', state));
      return; // This arena is being held for a long-lived member.
    }
    assert(_debugLogDiagnostic(pointer, 'Sweeping', state));
    _arenas.remove(pointer);
    if (state.members.isNotEmpty) {
      // First member wins.
      assert(_debugLogDiagnostic(pointer, 'Winner: ${state.members.first}'));
      state.members.first.acceptGesture(pointer);
      // Give all the other members the bad news.
      for (int i = 1; i < state.members.length; i++) {
        state.members[i].rejectGesture(pointer);
      }
    }
  }

  /// Prevents the arena from being swept.
  ///
  /// Typically, a winner is chosen in an arena after all the other
  /// [PointerUpEvent] processing by [sweep]. If a recognizer wishes to delay
  /// resolving an arena past [PointerUpEvent], the recognizer can [hold] the
  /// arena open using this function. To release such a hold and let the arena
  /// resolve, call [release].
  ///
  /// See also:
  ///
  ///  * [sweep]
  ///  * [release]
  void hold(int pointer) {
    final _GestureArena? state = _arenas[pointer];
    if (state == null) {
      return; // This arena either never existed or has been resolved.
    }
    state.isHeld = true;
    assert(_debugLogDiagnostic(pointer, 'Holding', state));
  }

  /// Releases a hold, allowing the arena to be swept.
  ///
  /// If a sweep was attempted on a held arena, the sweep will be done
  /// on release.
  ///
  /// See also:
  ///
  ///  * [sweep]
  ///  * [hold]
  void release(int pointer) {
    final _GestureArena? state = _arenas[pointer];
    if (state == null) {
      return; // This arena either never existed or has been resolved.
    }
    state.isHeld = false;
    assert(_debugLogDiagnostic(pointer, 'Releasing', state));
    if (state.hasPendingSweep) {
      sweep(pointer);
    }
  }

  /// Reject or accept a gesture recognizer.
  ///
  /// This is called by calling [GestureArenaEntry.resolve] on the object returned from [add].
  void _resolve(int pointer, GestureArenaMember member, GestureDisposition disposition) {
    final _GestureArena? state = _arenas[pointer];
    if (state == null) {
      return; // This arena has already resolved.
    }
    assert(state.members.contains(member));
    switch (disposition) {
      case GestureDisposition.accepted:
        assert(_debugLogDiagnostic(pointer, 'Accepting: $member'));
        if (state.isOpen) {
          state.eagerWinner ??= member;
        } else {
          assert(_debugLogDiagnostic(pointer, 'Self-declared winner: $member'));
          _resolveInFavorOf(pointer, state, member);
        }
      case GestureDisposition.rejected:
        assert(_debugLogDiagnostic(pointer, 'Rejecting: $member'));
        state.members.remove(member);
        member.rejectGesture(pointer);
        if (!state.isOpen) {
          _tryToResolveArena(pointer, state);
        }
    }
  }

  void _tryToResolveArena(int pointer, _GestureArena state) {
    assert(_arenas[pointer] == state);
    assert(!state.isOpen);
    if (state.members.length == 1) {
      scheduleMicrotask(() => _resolveByDefault(pointer, state));
    } else if (state.members.isEmpty) {
      _arenas.remove(pointer);
      assert(_debugLogDiagnostic(pointer, 'Arena empty.'));
    } else if (state.eagerWinner != null) {
      assert(_debugLogDiagnostic(pointer, 'Eager winner: ${state.eagerWinner}'));
      _resolveInFavorOf(pointer, state, state.eagerWinner!);
    }
  }

  void _resolveByDefault(int pointer, _GestureArena state) {
    if (!_arenas.containsKey(pointer)) {
      return; // This arena has already resolved.
    }
    assert(_arenas[pointer] == state);
    assert(!state.isOpen);
    final List<GestureArenaMember> members = state.members;
    assert(members.length == 1);
    _arenas.remove(pointer);
    assert(_debugLogDiagnostic(pointer, 'Default winner: ${state.members.first}'));
    state.members.first.acceptGesture(pointer);
  }

  void _resolveInFavorOf(int pointer, _GestureArena state, GestureArenaMember member) {
    assert(state == _arenas[pointer]);
    assert(state.eagerWinner == null || state.eagerWinner == member);
    assert(!state.isOpen);
    _arenas.remove(pointer);
    for (final GestureArenaMember rejectedMember in state.members) {
      if (rejectedMember != member) {
        rejectedMember.rejectGesture(pointer);
      }
    }
    member.acceptGesture(pointer);
  }

  bool _debugLogDiagnostic(int pointer, String message, [_GestureArena? state]) {
    assert(() {
      if (debugPrintGestureArenaDiagnostics) {
        final int? count = state?.members.length;
        final String s = count != 1 ? 's' : '';
        debugPrint(
          'Gesture arena ${pointer.toString().padRight(4)} ❙ $message${count != null ? " with $count member$s." : ""}',
        );
      }
      return true;
    }());
    return true;
  }
}
