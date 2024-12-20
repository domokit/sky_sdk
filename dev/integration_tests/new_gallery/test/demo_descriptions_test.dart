// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:collection/collection.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:gallery/data/demos.dart';
import 'package:gallery/gallery_localizations_en.dart';

bool _isUnique(List<String> list) {
  final Set<String> covered = <String>{};
  for (final String element in list) {
    if (covered.contains(element)) {
      return false;
    } else {
      covered.add(element);
    }
  }
  return true;
}

const ListEquality<String> _stringListEquality = ListEquality<String>();

void main() {
  test('_isUnique works correctly', () {
    expect(_isUnique(<String>['a', 'b', 'c']), true);
    expect(_isUnique(<String>['a', 'c', 'a', 'b']), false);
    expect(_isUnique(<String>['a']), true);
    expect(_isUnique(<String>[]), true);
  });

  test('Demo descriptions are unique and correct', () {
    final List<GalleryDemo> allDemos = Demos.all(GalleryLocalizationsEn());
    final List<String> allDemoDescriptions = allDemos.map((GalleryDemo d) => d.describe).toList();

    expect(_isUnique(allDemoDescriptions), true);
    expect(_stringListEquality.equals(allDemoDescriptions, Demos.allDescriptions()), true);
  });

  test('Special demo descriptions are correct', () {
    final List<String> allDemos = Demos.allDescriptions();

    final List<String> specialDemos = <String>[
      'shrine@study',
      'rally@study',
      'crane@study',
      'fortnightly@study',
      'bottom-navigation@material',
      'button@material',
      'card@material',
      'chip@material',
      'dialog@material',
      'pickers@material',
      'cupertino-alerts@cupertino',
      'colors@other',
      'progress-indicator@material',
      'cupertino-activity-indicator@cupertino',
      'colors@other',
    ];

    for (final String specialDemo in specialDemos) {
      expect(allDemos.contains(specialDemo), true);
    }
  });
}
