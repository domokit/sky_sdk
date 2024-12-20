// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// The platform that the browser is running on.
enum LayoutPlatform {
  /// Windows.
  win,

  /// Linux.
  linux,

  /// MacOS or iOS.
  darwin,
}

// The length of [LayoutEntry.printable].
const int _kPrintableLength = 4;

/// Describes the characters that a physical keyboard key will be mapped to
/// under different modifier states, for a given language on a given
/// platform.
class LayoutEntry {
  /// Create a layout entry.
  LayoutEntry(this.printables) : assert(printables.length == _kPrintableLength);

  /// The printable characters that a key should be mapped to under different
  /// modifier states.
  ///
  /// The [printables] always have a length of 4, corresponding to "without any
  /// modifiers", "with Shift", "with AltGr", and "with Shift and AltGr"
  /// respectively.
  ///
  /// Some values might be empty. It doesn't mean that these combinations will
  /// have an empty KeyboardKey.key, but usually these values are trivial,
  /// i.e. same as their non-modified counterparts.
  ///
  /// Some other values can be [kDeadKey]s. Dead keys are non-printable accents
  /// that will be combined into the following letter character.
  final List<String> printables;

  /// An empty [LayoutEntry] that produces dead keys under all conditions.
  static final LayoutEntry empty = LayoutEntry(const <String>['', '', '', '']);

  /// The value of KeyboardEvent.key for dead keys.
  static const String kDeadKey = 'Dead';
}

/// Describes the characters that all goal keys will be mapped to for a given
/// language on a given platform.
class Layout {
  /// Create a [Layout].
  const Layout(this.language, this.platform, this.entries);

  /// The language being used.
  final String language;

  /// The platform that the browser is running on.
  final LayoutPlatform platform;

  /// Maps from DOM `KeyboardKey.code`s to the characters they produce.
  final Map<String, LayoutEntry> entries;
}
