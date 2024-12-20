// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../dom.dart';

DomHTMLElement _createContainer() {
  final DomHTMLElement container = createDomHTMLDivElement();
  container.style
    ..position = 'fixed'
    ..top = '0'
    ..right = '0'
    ..padding = '6px'
    ..color = '#fff'
    ..backgroundColor = '#000'
    ..opacity = '0.8';
  return container;
}

/// An overlay in the top right corner of the page that shows statistics
/// regarding canvas reuse.
///
/// This should only be used for development purposes and never included in
/// release builds.
class DebugCanvasReuseOverlay {
  DebugCanvasReuseOverlay._() {
    final DomHTMLElement container = _createContainer();
    final DomHTMLElement title = createDomHTMLDivElement();
    title.style
      ..fontWeight = 'bold'
      ..textDecoration = 'underline';
    title.text = 'Canvas Reuse';

    domDocument.body!.append(
      container
        ..append(title)
        ..append(
          createDomHTMLDivElement()
            ..appendText('Created: ')
            ..append(_created),
        )
        ..append(
          createDomHTMLDivElement()
            ..appendText('Kept: ')
            ..append(_kept),
        )
        ..append(
          createDomHTMLDivElement()
            ..appendText('Reused: ')
            ..append(_reused),
        )
        ..append(
          createDomHTMLDivElement()
            ..appendText('Disposed: ')
            ..append(_disposed),
        )
        ..append(
          createDomHTMLDivElement()
            ..appendText('In Recycle List: ')
            ..append(_inRecycle),
        )
        ..append(
          createDomHTMLDivElement()..append(
            createDomHTMLButtonElement()
              ..text = 'Reset'
              ..addEventListener('click', createDomEventListener((_) => _reset())),
          ),
        ),
    );
  }

  static DebugCanvasReuseOverlay? _instance;
  static DebugCanvasReuseOverlay get instance {
    if (_instance == null) {
      // Only call the constructor when assertions are enabled to guard against
      // mistakingly including this class in a release build.
      assert(() {
        _instance = DebugCanvasReuseOverlay._();
        return true;
      }());
    }
    return _instance!;
  }

  final DomText _created = createDomText('0');
  final DomText _kept = createDomText('0');
  final DomText _reused = createDomText('0');
  final DomText _disposed = createDomText('0');
  final DomText _inRecycle = createDomText('0');

  int _createdCount = 0;
  int get createdCount => _createdCount;
  set createdCount(int createdCount) {
    _createdCount = createdCount;
    _update();
  }

  int _keptCount = 0;
  int get keptCount => _keptCount;
  set keptCount(int keptCount) {
    _keptCount = keptCount;
    _update();
  }

  int _reusedCount = 0;
  int get reusedCount => _reusedCount;
  set reusedCount(int reusedCount) {
    _reusedCount = reusedCount;
    _update();
  }

  int _disposedCount = 0;
  int get disposedCount => _disposedCount;
  set disposedCount(int disposedCount) {
    _disposedCount = disposedCount;
    _update();
  }

  int _inRecycleCount = 0;
  int get inRecycleCount => _inRecycleCount;
  set inRecycleCount(int inRecycleCount) {
    _inRecycleCount = inRecycleCount;
    _update();
  }

  void _update() {
    _created.text = '$_createdCount';
    _kept.text = '$_keptCount';
    _reused.text = '$_reusedCount';
    _disposed.text = '$_disposedCount';
    _inRecycle.text = '$_inRecycleCount';
  }

  void _reset() {
    _createdCount = _keptCount = _reusedCount = _disposedCount = _inRecycleCount = 0;
    _update();
  }
}
