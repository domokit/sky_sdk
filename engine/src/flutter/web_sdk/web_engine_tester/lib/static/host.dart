// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: avoid_dynamic_calls

@JS()
library test.host;

import 'dart:async';
import 'dart:convert';
import 'dart:js_interop';

import 'package:stack_trace/stack_trace.dart';
import 'package:stream_channel/stream_channel.dart';
// ignore: implementation_imports
import 'package:ui/src/engine/dom.dart';

/// A class defined in content shell, used to control its behavior.
@JS()
@staticInterop
class _TestRunner {}

extension _TestRunnerExtension on _TestRunner {
  external JSVoid waitUntilDone();
}

/// Returns the current content shell runner, or `null` if none exists.
@JS()
external _TestRunner? get testRunner; // ignore: library_private_types_in_public_api

/// A class that exposes the test API to JS.
///
/// These are exposed so that tools like IDEs can interact with them via remote
/// debugging.
@JS()
@anonymous
@staticInterop
class _JSApi {
  external factory _JSApi({JSFunction resume, JSFunction restartCurrent});
}

extension _JSApiExtension on _JSApi {
  /// Causes the test runner to resume running, as though the user had clicked
  /// the "play" button.
  // ignore: unused_element
  external JSFunction get resume;

  /// Causes the test runner to restart the current test once it finishes
  /// running.
  // ignore: unused_element
  external JSFunction get restartCurrent;
}

/// Sets the top-level `dartTest` object so that it's visible to JS.
@JS('dartTest')
external set _jsApi(_JSApi api);

/// The iframes created for each loaded test suite, indexed by the suite id.
final Map<int, DomHTMLIFrameElement> _iframes = <int, DomHTMLIFrameElement>{};

/// Subscriptions created for each loaded test suite, indexed by the suite id.
final Map<int, List<DomSubscription>> _domSubscriptions = <int, List<DomSubscription>>{};
final Map<int, List<StreamSubscription<dynamic>>> _streamSubscriptions =
    <int, List<StreamSubscription<dynamic>>>{};

/// The URL for the current page.
final Uri _currentUrl = Uri.parse(domWindow.location.href);

/// Code that runs in the browser and loads test suites at the server's behest.
///
/// One instance of this runs for each browser. When the server tells it to load
/// a test, it starts an iframe pointing at that test's code; from then on, it
/// just relays messages between the two.
///
/// The browser uses two layers of [MultiChannel]s when communicating with the
/// server:
///
///                                       server
///                                         │
///                                    (WebSocket)
///                                         │
///                    ┏━ host.html ━━━━━━━━┿━━━━━━━━━━━━━━━━━┓
///                    ┃                    │                 ┃
///                    ┃    ┌──────┬───MultiChannel─────┐     ┃
///                    ┃    │      │      │      │      │     ┃
///                    ┃   host  suite  suite  suite  suite   ┃
///                    ┃           │      │      │      │     ┃
///                    ┗━━━━━━━━━━━┿━━━━━━┿━━━━━━┿━━━━━━┿━━━━━┛
///                                │      │      │      │
///                                │     ...    ...    ...
///                                │
///                         (MessageChannel)
///                                │
///      ┏━ suite.html (in iframe) ┿━━━━━━━━━━━━━━━━━━━━━━━━━━┓
///      ┃                         │                          ┃
///      ┃         ┌──────────MultiChannel┬─────────┐         ┃
///      ┃         │          │     │     │         │         ┃
///      ┃   IframeListener  test  test  test  running test   ┃
///      ┃                                                    ┃
///      ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
///
/// The host (this code) has a [MultiChannel] that splits the WebSocket
/// connection with the server. One connection is used for the host itself to
/// receive messages like "load a suite at this URL", and the rest are
/// connected to each test suite's iframe via a [MessageChannel].
///
/// Each iframe then has its own [MultiChannel] which takes its
/// [MessageChannel] connection and splits it again. One connection is used for
/// the [IframeListener], which sends messages like "here are all the tests in
/// this suite". The rest are used for each test, receiving messages like
/// "start running". A new connection is also created whenever a test begins
/// running to send status messages about its progress.
///
/// It's of particular note that the suite's [MultiChannel] connection uses the
/// host's purely as a transport layer; neither is aware that the other is also
/// using [MultiChannel]. This is necessary, since the host doesn't share memory
/// with the suites and thus can't share its [MultiChannel] with them, but it
/// does mean that the server needs to be sure to nest its [MultiChannel]s at
/// the same place the client does.
void main() {
  // This tells content_shell not to close immediately after the page has
  // rendered.
  testRunner?.waitUntilDone();

  if (_currentUrl.queryParameters['debug'] == 'true') {
    domDocument.body!.classList.add('debug');
  }

  runZonedGuarded(
    () {
      final MultiChannel<dynamic> serverChannel = _connectToServer();
      serverChannel.stream.listen((dynamic message) {
        if (message['command'] == 'loadSuite') {
          final int channelId = message['channel'] as int;
          final String url = message['url'] as String;
          final int messageId = message['id'] as int;
          final VirtualChannel<dynamic> suiteChannel = serverChannel.virtualChannel(channelId);
          final StreamChannel<dynamic> iframeChannel = _connectToIframe(url, messageId);
          suiteChannel.pipe(iframeChannel);
        } else if (message['command'] == 'displayPause') {
          domDocument.body!.classList.add('paused');
        } else if (message['command'] == 'resume') {
          domDocument.body!.classList.remove('paused');
        } else {
          assert(message['command'] == 'closeSuite');
          _iframes.remove(message['id'])!.remove();

          for (final DomSubscription subscription in _domSubscriptions.remove(message['id'])!) {
            subscription.cancel();
          }
          for (final StreamSubscription<dynamic> subscription
              in _streamSubscriptions.remove(message['id'])!) {
            subscription.cancel();
          }
        }
      });

      // Send periodic pings to the test runner so it can know when the browser is
      // paused for debugging.
      Timer.periodic(
        const Duration(seconds: 1),
        (_) => serverChannel.sink.add(<String, String>{'command': 'ping'}),
      );

      _jsApi = _JSApi(
        resume:
            () {
              if (!domDocument.body!.classList.contains('paused')) {
                return;
              }
              domDocument.body!.classList.remove('paused');
              serverChannel.sink.add(<String, String>{'command': 'resume'});
            }.toJS,
        restartCurrent:
            () {
              serverChannel.sink.add(<String, String>{'command': 'restart'});
            }.toJS,
      );
    },
    (dynamic error, StackTrace stackTrace) {
      print('$error\n${Trace.from(stackTrace).terse}'); // ignore: avoid_print
    },
  );
}

/// Creates a [MultiChannel] connection to the server, using a [WebSocket] as
/// the underlying protocol.
MultiChannel<dynamic> _connectToServer() {
  // The `managerUrl` query parameter contains the WebSocket URL of the remote
  // [BrowserManager] with which this communicates.
  final DomWebSocket webSocket = createDomWebSocket(_currentUrl.queryParameters['managerUrl']!);

  final StreamChannelController<dynamic> controller = StreamChannelController<dynamic>(sync: true);
  webSocket.addEventListener(
    'message',
    createDomEventListener((DomEvent message) {
      final String data = (message as DomMessageEvent).data as String;
      controller.local.sink.add(jsonDecode(data));
    }),
  );

  controller.local.stream.listen((dynamic message) => webSocket.send(jsonEncode(message)));

  return MultiChannel<dynamic>(controller.foreign);
}

/// Creates an iframe with `src` [url] and establishes a connection to it using
/// a [MessageChannel].
///
/// [id] identifies the suite loaded in this iframe.
StreamChannel<dynamic> _connectToIframe(String url, int id) {
  final DomHTMLIFrameElement iframe = createDomHTMLIFrameElement();
  _iframes[id] = iframe;
  iframe
    ..src = url
    ..width = '1000'
    ..height = '1000';
  domDocument.body!.appendChild(iframe);

  final StreamChannelController<dynamic> controller = StreamChannelController<dynamic>(sync: true);

  final List<DomSubscription> domSubscriptions = <DomSubscription>[];
  final List<StreamSubscription<dynamic>> streamSubscriptions = <StreamSubscription<dynamic>>[];
  _domSubscriptions[id] = domSubscriptions;
  _streamSubscriptions[id] = streamSubscriptions;
  domSubscriptions.add(
    DomSubscription(domWindow, 'message', (DomEvent event) {
      final DomMessageEvent message = event as DomMessageEvent;
      // A message on the Window can theoretically come from any website. It's
      // very unlikely that a malicious site would care about hacking someone's
      // unit tests, let alone be able to find the test server while it's
      // running, but it's good practice to check the origin anyway.
      if (message.origin != domWindow.location.origin) {
        return;
      }
      if (message.source.location?.href != iframe.src) {
        return;
      }

      message.stopPropagation();

      if (message.data == 'port') {
        final DomMessagePort port = message.ports.first;
        domSubscriptions.add(
          DomSubscription(port, 'message', (DomEvent event) {
            controller.local.sink.add((event as DomMessageEvent).data);
          }),
        );
        port.start();
        streamSubscriptions.add(controller.local.stream.listen(port.postMessage));
      } else if (message.data['ready'] == true) {
        // This message indicates that the iframe is actively listening for
        // events, so the message channel's second port can now be transferred.
        final DomMessageChannel channel = createDomMessageChannel();
        channel.port1.start();
        channel.port2.start();
        iframe.contentWindow.postMessage('port', domWindow.location.origin, <DomMessagePort>[
          channel.port2,
        ]);
        domSubscriptions.add(
          DomSubscription(channel.port1, 'message', (DomEvent message) {
            controller.local.sink.add((message as DomMessageEvent).data['data']);
          }),
        );

        streamSubscriptions.add(controller.local.stream.listen(channel.port1.postMessage));
      } else if (message.data['exception'] == true) {
        // This message from `dart.js` indicates that an exception occurred
        // loading the test.
        controller.local.sink.add(message.data['data']);
      }
    }),
  );

  return controller.foreign;
}
