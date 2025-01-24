// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart' show immutable;

/// Data class that represents a range of versions in their String
/// representation.
///
/// Both the [versionMin] and [versionMax] are inclusive versions, and undefined
/// values represent an unknown minimum/maximum version.
@immutable
class VersionRange {
  const VersionRange(this.versionMin, this.versionMax);

  final String? versionMin;
  final String? versionMax;

  @override
  bool operator ==(Object other) =>
      other is VersionRange && other.versionMin == versionMin && other.versionMax == versionMax;

  @override
  int get hashCode => Object.hash(versionMin, versionMax);

  @override
  String toString() {
    return 'VersionRange(versionMin: $versionMin, versionMax: $versionMax)';
  }
}
