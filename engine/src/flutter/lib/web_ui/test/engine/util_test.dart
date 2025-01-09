// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

final Float32List identityTransform = Matrix4.identity().storage;
final Float32List xTranslation = (Matrix4.identity()..translate(10)).storage;
final Float32List yTranslation = (Matrix4.identity()..translate(0, 10)).storage;
final Float32List zTranslation = (Matrix4.identity()..translate(0, 0, 10)).storage;
final Float32List scaleAndTranslate2d =
    (Matrix4.identity()
          ..scale(2, 3, 1)
          ..translate(4, 5))
        .storage;
final Float32List rotation2d = (Matrix4.identity()..rotateZ(0.2)).storage;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  test('transformKindOf and isIdentityFloat32ListTransform identify matrix kind', () {
    expect(transformKindOf(identityTransform), TransformKind.identity);
    expect(isIdentityFloat32ListTransform(identityTransform), isTrue);

    expect(transformKindOf(zTranslation), TransformKind.complex);
    expect(isIdentityFloat32ListTransform(zTranslation), isFalse);

    expect(transformKindOf(xTranslation), TransformKind.transform2d);
    expect(isIdentityFloat32ListTransform(xTranslation), isFalse);

    expect(transformKindOf(yTranslation), TransformKind.transform2d);
    expect(isIdentityFloat32ListTransform(yTranslation), isFalse);

    expect(transformKindOf(scaleAndTranslate2d), TransformKind.transform2d);
    expect(isIdentityFloat32ListTransform(scaleAndTranslate2d), isFalse);

    expect(transformKindOf(rotation2d), TransformKind.transform2d);
    expect(isIdentityFloat32ListTransform(rotation2d), isFalse);
  });

  test('canonicalizes font families correctly on iOS (not 15)', () {
    ui_web.browser.debugOperatingSystemOverride = ui_web.OperatingSystem.iOs;
    debugIsIOS15 = false;

    expect(canonicalizeFontFamily('sans-serif'), 'sans-serif');
    expect(canonicalizeFontFamily('foo'), '"foo", -apple-system, BlinkMacSystemFont, sans-serif');
    expect(canonicalizeFontFamily('.SF Pro Text'), '-apple-system, BlinkMacSystemFont');

    ui_web.browser.debugOperatingSystemOverride = null;
    debugIsIOS15 = null;
  });

  test('does not use -apple-system on iOS 15', () {
    ui_web.browser.debugOperatingSystemOverride = ui_web.OperatingSystem.iOs;
    debugIsIOS15 = true;

    expect(canonicalizeFontFamily('sans-serif'), 'sans-serif');
    expect(canonicalizeFontFamily('foo'), '"foo", BlinkMacSystemFont, sans-serif');
    expect(canonicalizeFontFamily('.SF Pro Text'), 'BlinkMacSystemFont');

    ui_web.browser.debugOperatingSystemOverride = null;
    debugIsIOS15 = null;
  });

  test('parseFloat basic tests', () {
    // Simple integers and doubles.
    expect(parseFloat('108'), 108.0);
    expect(parseFloat('.34'), 0.34);
    expect(parseFloat('108.34'), 108.34);

    // Number followed by text.
    expect(parseFloat('108.34px'), 108.34);
    expect(parseFloat('108.34px29'), 108.34);
    expect(parseFloat('108.34px 29'), 108.34);

    // Number followed by space and text.
    expect(parseFloat('108.34 px29'), 108.34);
    expect(parseFloat('108.34 px 29'), 108.34);

    // Invalid numbers.
    expect(parseFloat('text'), isNull);
    expect(parseFloat('text108'), isNull);
    expect(parseFloat('text 108'), isNull);
    expect(parseFloat('another text 108'), isNull);
  });

  test('can set style properties on elements', () {
    final DomElement element = domDocument.createElement('div');
    setElementStyle(element, 'color', 'red');
    expect(element.style.color, 'red');
  });

  test('can remove style properties from elements', () {
    final DomElement element = domDocument.createElement('div');
    setElementStyle(element, 'color', 'blue');
    expect(element.style.color, 'blue');
    setElementStyle(element, 'color', null);
    expect(element.style.color, '');
  });

  test('futurize turns a Callbacker into a Future', () async {
    final Future<String> stringFuture = futurize((Callback<String> callback) {
      scheduleMicrotask(() {
        callback('hello');
      });
      return null;
    });
    expect(await stringFuture, 'hello');
  });

  test('futurize converts error string to exception', () async {
    try {
      await futurize((Callback<String> callback) {
        return 'this is an error';
      });
      fail('Expected it to throw');
    } on Exception catch (exception) {
      expect('$exception', contains('this is an error'));
    }
  });

  test('futurize converts async null into an async operation failure', () async {
    final Future<String?> stringFuture = futurize((Callback<String?> callback) {
      scheduleMicrotask(() {
        callback(null);
      });
      return null;
    });

    try {
      await stringFuture;
      fail('Expected it to throw');
    } on Exception catch (exception) {
      expect('$exception', contains('operation failed'));
    }
  });

  test('futurize converts sync null into a sync operation failure', () async {
    try {
      await futurize((Callback<String?> callback) {
        callback(null);
        return null;
      });
      fail('Expected it to throw');
    } on Exception catch (exception) {
      expect('$exception', contains('operation failed'));
    }
  });
}
