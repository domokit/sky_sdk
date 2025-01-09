// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart' hide window;
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../common/matchers.dart';
import '../common/test_initialization.dart';
import 'history_test.dart';

EngineFlutterWindow get myWindow => EnginePlatformDispatcher.instance.implicitView!;

Map<String, dynamic> _tagStateWithSerialCount(dynamic state, int serialCount) {
  return <String, dynamic>{'serialCount': serialCount, 'state': state};
}

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  setUpImplicitView();

  // For now, web always has an implicit view provided by the web engine.
  test('EnginePlatformDispatcher.instance.implicitView should be non-null', () async {
    expect(EnginePlatformDispatcher.instance.implicitView, isNotNull);
    expect(EnginePlatformDispatcher.instance.implicitView?.viewId, 0);
    expect(myWindow.viewId, 0);
  });

  test('window.defaultRouteName should work with a custom url strategy', () async {
    const String path = '/initial';
    const Object state = <dynamic, dynamic>{'origin': true};

    final _SampleUrlStrategy customStrategy = _SampleUrlStrategy(path, state);
    await myWindow.debugInitializeHistory(customStrategy, useSingle: true);
    expect(myWindow.defaultRouteName, '/initial');
    // Also make sure that the custom url strategy was actually used.
    expect(customStrategy.wasUsed, isTrue);
  });

  test('window.defaultRouteName should not change', () async {
    final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
      const TestHistoryEntry('initial state', null, '/initial'),
    );
    await myWindow.debugInitializeHistory(strategy, useSingle: true);
    expect(myWindow.defaultRouteName, '/initial');

    // Changing the URL in the address bar later shouldn't affect [window.defaultRouteName].
    strategy.replaceState(null, '', '/newpath');
    expect(myWindow.defaultRouteName, '/initial');
  });

  // window.defaultRouteName is now permanently decoupled from the history,
  // even in subsequent tests, because the PlatformDispatcher caches it.

  test('window.defaultRouteName should reset after navigation platform message', () async {
    await myWindow.debugInitializeHistory(
      TestUrlStrategy.fromEntry(
        // The URL here does not set the PlatformDispatcher's defaultRouteName,
        // since it got cached as soon as we read it above.
        const TestHistoryEntry('initial state', null, '/not-really-inital/THIS_IS_IGNORED'),
      ),
      useSingle: true,
    );
    // Reading it multiple times should return the same value.
    expect(myWindow.defaultRouteName, '/initial');
    expect(myWindow.defaultRouteName, '/initial');

    final Completer<void> callback = Completer<void>();
    myWindow.sendPlatformMessage(
      'flutter/navigation',
      const JSONMethodCodec().encodeMethodCall(
        const MethodCall('routeUpdated', <String, dynamic>{'routeName': '/bar'}),
      ),
      (_) {
        callback.complete();
      },
    );
    await callback.future;
    // After a navigation platform message, the PlatformDispatcher's
    // defaultRouteName resets to "/".
    expect(myWindow.defaultRouteName, '/');
  });

  // window.defaultRouteName is now '/'.

  test('can switch history mode', () async {
    Completer<void> callback;
    await myWindow.debugInitializeHistory(
      TestUrlStrategy.fromEntry(const TestHistoryEntry('initial state', null, '/initial')),
      useSingle: false,
    );
    expect(myWindow.browserHistory, isA<MultiEntriesBrowserHistory>());

    Future<void> check<T>(String method, Object? arguments) async {
      callback = Completer<void>();
      myWindow.sendPlatformMessage(
        'flutter/navigation',
        const JSONMethodCodec().encodeMethodCall(MethodCall(method, arguments)),
        (_) {
          callback.complete();
        },
      );
      await callback.future;
      expect(myWindow.browserHistory, isA<T>());
    }

    // These may be initialized as `null`
    // See https://github.com/flutter/flutter/issues/83158#issuecomment-847483010
    await check<SingleEntryBrowserHistory>('selectSingleEntryHistory', null); // -> single
    await check<MultiEntriesBrowserHistory>('selectMultiEntryHistory', null); // -> multi
    await check<SingleEntryBrowserHistory>(
      'selectSingleEntryHistory',
      <String, dynamic>{},
    ); // -> single
    await check<MultiEntriesBrowserHistory>(
      'selectMultiEntryHistory',
      <String, dynamic>{},
    ); // -> multi
    await check<SingleEntryBrowserHistory>('routeUpdated', <String, dynamic>{
      'routeName': '/bar',
    }); // -> single
    await check<SingleEntryBrowserHistory>('routeInformationUpdated', <String, dynamic>{
      'location': '/bar',
    }); // does not change mode
    await check<MultiEntriesBrowserHistory>(
      'selectMultiEntryHistory',
      <String, dynamic>{},
    ); // -> multi
    await check<MultiEntriesBrowserHistory>('routeInformationUpdated', <String, dynamic>{
      'location': '/bar',
    }); // does not change mode
  });

  test(
    'handleNavigationMessage throws for route update methods called with null arguments',
    () async {
      expect(() async {
        await myWindow.handleNavigationMessage(
          const JSONMethodCodec().encodeMethodCall(const MethodCall('routeUpdated')),
        );
      }, throwsAssertionError);

      expect(() async {
        await myWindow.handleNavigationMessage(
          const JSONMethodCodec().encodeMethodCall(const MethodCall('routeInformationUpdated')),
        );
      }, throwsAssertionError);
    },
  );

  test('handleNavigationMessage execute request in order.', () async {
    // Start with multi entries.
    await myWindow.debugInitializeHistory(
      TestUrlStrategy.fromEntry(const TestHistoryEntry('initial state', null, '/initial')),
      useSingle: false,
    );
    expect(myWindow.browserHistory, isA<MultiEntriesBrowserHistory>());
    final List<String> executionOrder = <String>[];
    await myWindow
        .handleNavigationMessage(
          const JSONMethodCodec().encodeMethodCall(const MethodCall('selectSingleEntryHistory')),
        )
        .then<void>((bool data) {
          executionOrder.add('1');
        });
    await myWindow
        .handleNavigationMessage(
          const JSONMethodCodec().encodeMethodCall(const MethodCall('selectMultiEntryHistory')),
        )
        .then<void>((bool data) {
          executionOrder.add('2');
        });
    await myWindow
        .handleNavigationMessage(
          const JSONMethodCodec().encodeMethodCall(const MethodCall('selectSingleEntryHistory')),
        )
        .then<void>((bool data) {
          executionOrder.add('3');
        });
    await myWindow
        .handleNavigationMessage(
          const JSONMethodCodec().encodeMethodCall(
            const MethodCall(
              'routeInformationUpdated',
              <String, dynamic>{'location': '/baz', 'state': null}, // boom
            ),
          ),
        )
        .then<void>((bool data) {
          executionOrder.add('4');
        });
    // The routeInformationUpdated should finish after the browser history
    // has been set to single entry.
    expect(executionOrder.length, 4);
    expect(executionOrder[0], '1');
    expect(executionOrder[1], '2');
    expect(executionOrder[2], '3');
    expect(executionOrder[3], '4');
  });

  test('should not throw when using nav1 and nav2 together', () async {
    await myWindow.debugInitializeHistory(
      TestUrlStrategy.fromEntry(const TestHistoryEntry('initial state', null, '/initial')),
      useSingle: false,
    );
    expect(myWindow.browserHistory, isA<MultiEntriesBrowserHistory>());

    // routeUpdated resets the history type
    Completer<void> callback = Completer<void>();
    myWindow.sendPlatformMessage(
      'flutter/navigation',
      const JSONMethodCodec().encodeMethodCall(
        const MethodCall('routeUpdated', <String, dynamic>{'routeName': '/bar'}),
      ),
      (_) {
        callback.complete();
      },
    );
    await callback.future;
    expect(myWindow.browserHistory, isA<SingleEntryBrowserHistory>());
    expect(myWindow.browserHistory.urlStrategy!.getPath(), '/bar');

    // routeInformationUpdated does not
    callback = Completer<void>();
    myWindow.sendPlatformMessage(
      'flutter/navigation',
      const JSONMethodCodec().encodeMethodCall(
        const MethodCall('routeInformationUpdated', <String, dynamic>{
          'location': '/baz',
          'state': null,
        }),
      ),
      (_) {
        callback.complete();
      },
    );
    await callback.future;
    expect(myWindow.browserHistory, isA<SingleEntryBrowserHistory>());
    expect(myWindow.browserHistory.urlStrategy!.getPath(), '/baz');

    // they can be interleaved safely
    await myWindow.handleNavigationMessage(
      const JSONMethodCodec().encodeMethodCall(
        const MethodCall('routeUpdated', <String, dynamic>{'routeName': '/foo'}),
      ),
    );
    expect(myWindow.browserHistory, isA<SingleEntryBrowserHistory>());
    expect(myWindow.browserHistory.urlStrategy!.getPath(), '/foo');
  });

  test('should not throw when state is complex json object', () async {
    // Regression test https://github.com/flutter/flutter/issues/87823.
    await myWindow.debugInitializeHistory(
      TestUrlStrategy.fromEntry(const TestHistoryEntry('initial state', null, '/initial')),
      useSingle: false,
    );
    expect(myWindow.browserHistory, isA<MultiEntriesBrowserHistory>());

    // routeInformationUpdated does not
    final Completer<void> callback = Completer<void>();
    myWindow.sendPlatformMessage(
      'flutter/navigation',
      const JSONMethodCodec().encodeMethodCall(
        const MethodCall('routeInformationUpdated', <String, dynamic>{
          'location': '/baz',
          'state': <String, dynamic>{
            'state1': true,
            'state2': 1,
            'state3': 'string',
            'state4': <String, dynamic>{'substate1': 1.0, 'substate2': 'string2'},
          },
        }),
      ),
      (_) {
        callback.complete();
      },
    );
    await callback.future;
    expect(myWindow.browserHistory, isA<MultiEntriesBrowserHistory>());
    expect(myWindow.browserHistory.urlStrategy!.getPath(), '/baz');
    final wrappedState = myWindow.browserHistory.urlStrategy!.getState()! as Map<Object?, Object?>;
    final actualState = wrappedState['state']! as Map<Object?, Object?>;
    expect(actualState['state1'], true);
    expect(actualState['state2'], 1);
    expect(actualState['state3'], 'string');

    final state4 = actualState['state4']! as Map<Object?, Object?>;
    expect(state4['substate1'], 1.0);
    expect(state4['substate2'], 'string2');
  });

  test('routeInformationUpdated can handle uri', () async {
    await myWindow.debugInitializeHistory(
      TestUrlStrategy.fromEntry(const TestHistoryEntry('initial state', null, '/initial')),
      useSingle: false,
    );
    expect(myWindow.browserHistory, isA<MultiEntriesBrowserHistory>());

    // routeInformationUpdated does not
    final Completer<void> callback = Completer<void>();
    myWindow.sendPlatformMessage(
      'flutter/navigation',
      const JSONMethodCodec().encodeMethodCall(
        const MethodCall('routeInformationUpdated', <String, dynamic>{
          'uri': 'http://myhostname.com/baz?abc=def#fragment',
        }),
      ),
      (_) {
        callback.complete();
      },
    );
    await callback.future;
    expect(myWindow.browserHistory, isA<MultiEntriesBrowserHistory>());
    expect(myWindow.browserHistory.urlStrategy!.getPath(), '/baz?abc=def#fragment');
  });

  test('can replace in MultiEntriesBrowserHistory', () async {
    await myWindow.debugInitializeHistory(
      TestUrlStrategy.fromEntry(const TestHistoryEntry('initial state', null, '/initial')),
      useSingle: false,
    );
    expect(myWindow.browserHistory, isA<MultiEntriesBrowserHistory>());

    Completer<void> callback = Completer<void>();
    myWindow.sendPlatformMessage(
      'flutter/navigation',
      const JSONMethodCodec().encodeMethodCall(
        const MethodCall('routeInformationUpdated', <String, dynamic>{
          'location': '/baz',
          'state': '/state',
        }),
      ),
      (_) {
        callback.complete();
      },
    );
    await callback.future;
    expect(myWindow.browserHistory.urlStrategy!.getPath(), '/baz');
    expect(myWindow.browserHistory.urlStrategy!.getState(), _tagStateWithSerialCount('/state', 1));

    callback = Completer<void>();
    myWindow.sendPlatformMessage(
      'flutter/navigation',
      const JSONMethodCodec().encodeMethodCall(
        const MethodCall('routeInformationUpdated', <String, dynamic>{
          'location': '/baz',
          'state': '/state1',
          'replace': true,
        }),
      ),
      (_) {
        callback.complete();
      },
    );
    await callback.future;
    expect(myWindow.browserHistory.urlStrategy!.getPath(), '/baz');
    expect(myWindow.browserHistory.urlStrategy!.getState(), _tagStateWithSerialCount('/state1', 1));

    callback = Completer<void>();
    myWindow.sendPlatformMessage(
      'flutter/navigation',
      const JSONMethodCodec().encodeMethodCall(
        const MethodCall('routeInformationUpdated', <String, dynamic>{
          'location': '/foo',
          'state': '/foostate1',
        }),
      ),
      (_) {
        callback.complete();
      },
    );
    await callback.future;
    expect(myWindow.browserHistory.urlStrategy!.getPath(), '/foo');
    expect(
      myWindow.browserHistory.urlStrategy!.getState(),
      _tagStateWithSerialCount('/foostate1', 2),
    );

    await myWindow.browserHistory.back();
    expect(myWindow.browserHistory.urlStrategy!.getPath(), '/baz');
    expect(myWindow.browserHistory.urlStrategy!.getState(), _tagStateWithSerialCount('/state1', 1));
  });

  test('initialize browser history with default url strategy (single)', () async {
    // On purpose, we don't initialize history on the window. We want to let the
    // window to self-initialize when it receives a navigation message.

    // Without initializing history, the default route name should be
    // initialized to "/" in tests.
    expect(myWindow.defaultRouteName, '/');

    final Completer<void> callback = Completer<void>();
    myWindow.sendPlatformMessage(
      'flutter/navigation',
      const JSONMethodCodec().encodeMethodCall(
        const MethodCall('routeUpdated', <String, dynamic>{'routeName': '/bar'}),
      ),
      (_) {
        callback.complete();
      },
    );
    await callback.future;
    expect(myWindow.browserHistory, isA<SingleEntryBrowserHistory>());
    // The url strategy should've been set to the default, and the path
    // should've been correctly set to "/bar".
    expect(myWindow.browserHistory.urlStrategy, isNot(isNull));
    expect(myWindow.browserHistory.urlStrategy!.getPath(), '/bar');
  }, skip: isSafari); // https://github.com/flutter/flutter/issues/50836

  test('initialize browser history with default url strategy (multiple)', () async {
    // On purpose, we don't initialize history on the window. We want to let the
    // window to self-initialize when it receives a navigation message.

    // Without initializing history, the default route name should be
    // initialized to "/" in tests.
    expect(myWindow.defaultRouteName, '/');

    final Completer<void> callback = Completer<void>();
    myWindow.sendPlatformMessage(
      'flutter/navigation',
      const JSONMethodCodec().encodeMethodCall(
        const MethodCall('routeInformationUpdated', <String, dynamic>{
          'location': '/baz',
          'state': null,
        }),
      ),
      (_) {
        callback.complete();
      },
    );
    await callback.future;
    expect(myWindow.browserHistory, isA<MultiEntriesBrowserHistory>());
    // The url strategy should've been set to the default, and the path
    // should've been correctly set to "/baz".
    expect(myWindow.browserHistory.urlStrategy, isNot(isNull));
    expect(myWindow.browserHistory.urlStrategy!.getPath(), '/baz');
  }, skip: isSafari); // https://github.com/flutter/flutter/issues/50836

  test('can disable location strategy', () async {
    // Disable URL strategy.
    expect(() {
      ui_web.urlStrategy = null;
    }, returnsNormally);
    // History should be initialized.
    expect(myWindow.browserHistory, isNotNull);
    // But without a URL strategy.
    expect(myWindow.browserHistory.urlStrategy, isNull);
    // Current path is always "/" in this case.
    expect(myWindow.browserHistory.currentPath, '/');

    // Perform some navigation operations.
    await routeInformationUpdated('/foo/bar', null);
    // Path should not be updated because URL strategy is disabled.
    expect(myWindow.browserHistory.currentPath, '/');
  });

  test('cannot set url strategy after it was initialized', () async {
    final TestUrlStrategy testStrategy = TestUrlStrategy.fromEntry(
      const TestHistoryEntry('initial state', null, '/'),
    );
    await myWindow.debugInitializeHistory(testStrategy, useSingle: true);

    expect(() {
      ui_web.urlStrategy = null;
    }, throwsA(isAssertionError));
  });

  test('cannot set url strategy more than once', () async {
    // First time is okay.
    expect(() {
      ui_web.urlStrategy = null;
    }, returnsNormally);
    // Second time is not allowed.
    expect(() {
      ui_web.urlStrategy = null;
    }, throwsA(isAssertionError));
  });

  // Regression test for https://github.com/flutter/flutter/issues/77817
  test('window.locale(s) are not nullable', () {
    // If the getters were nullable, these expressions would result in compiler errors.
    ui.PlatformDispatcher.instance.locale.countryCode;
    ui.PlatformDispatcher.instance.locales.first.countryCode;
  });
}

class _SampleUrlStrategy implements ui_web.UrlStrategy {
  _SampleUrlStrategy(this._path, this._state);

  final String _path;
  final Object? _state;

  bool wasUsed = false;

  @override
  String getPath() => _path;

  @override
  Object? getState() => _state;

  @override
  ui.VoidCallback addPopStateListener(DartDomEventListener listener) {
    wasUsed = true;
    return () {};
  }

  @override
  String prepareExternalUrl(String value) => '';

  @override
  void pushState(Object? newState, String title, String url) {
    wasUsed = true;
  }

  @override
  void replaceState(Object? newState, String title, String url) {
    wasUsed = true;
  }

  @override
  Future<void> go(int delta) async {
    wasUsed = true;
  }
}
