// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:path_provider/path_provider.dart';

import 'android_platform_view.dart';
import 'future_data_handler.dart';
import 'motion_event_diff.dart';
import 'page.dart';

const String kEventsFileName = 'touchEvents';

class MotionEventsPage extends PageWidget {
  const MotionEventsPage({Key? key})
    : super('Motion Event Tests', const ValueKey<String>('MotionEventsListTile'), key: key);

  @override
  Widget build(BuildContext context) {
    return const MotionEventsBody();
  }
}

class MotionEventsBody extends StatefulWidget {
  const MotionEventsBody({super.key});

  @override
  State createState() => MotionEventsBodyState();
}

class MotionEventsBodyState extends State<MotionEventsBody> {
  static const int kEventsBufferSize = 1000;

  MethodChannel? viewChannel;

  /// The list of motion events that were passed to the FlutterView.
  List<Map<String, dynamic>> flutterViewEvents = <Map<String, dynamic>>[];

  /// The list of motion events that were passed to the embedded view.
  List<Map<String, dynamic>> embeddedViewEvents = <Map<String, dynamic>>[];

  @override
  Widget build(BuildContext context) {
    return Column(
      children: <Widget>[
        SizedBox(
          height: 300.0,
          child: AndroidPlatformView(
            key: const ValueKey<String>('PlatformView'),
            viewType: 'simple_view',
            onPlatformViewCreated: onPlatformViewCreated,
          ),
        ),
        Expanded(
          child: ListView.builder(itemBuilder: buildEventTile, itemCount: flutterViewEvents.length),
        ),
        Row(
          children: <Widget>[
            Expanded(
              child: ElevatedButton(
                onPressed: listenToFlutterViewEvents,
                child: const Text('RECORD'),
              ),
            ),
            Expanded(
              child: ElevatedButton(
                child: const Text('CLEAR'),
                onPressed: () {
                  setState(() {
                    flutterViewEvents.clear();
                    embeddedViewEvents.clear();
                  });
                },
              ),
            ),
            Expanded(
              child: ElevatedButton(
                child: const Text('SAVE'),
                onPressed: () {
                  const StandardMessageCodec codec = StandardMessageCodec();
                  saveRecordedEvents(codec.encodeMessage(flutterViewEvents)!, context);
                },
              ),
            ),
            Expanded(
              child: ElevatedButton(
                key: const ValueKey<String>('play'),
                child: const Text('PLAY FILE'),
                onPressed: () {
                  playEventsFile();
                },
              ),
            ),
            Expanded(
              child: ElevatedButton(
                key: const ValueKey<String>('back'),
                child: const Text('BACK'),
                onPressed: () {
                  Navigator.pop(context);
                },
              ),
            ),
          ],
        ),
      ],
    );
  }

  Future<String> playEventsFile() async {
    const StandardMessageCodec codec = StandardMessageCodec();
    try {
      final ByteData data = await rootBundle.load(
        'packages/assets_for_android_views/assets/touchEvents',
      );
      final List<dynamic> unTypedRecordedEvents = codec.decodeMessage(data) as List<dynamic>;
      final List<Map<String, dynamic>> recordedEvents =
          unTypedRecordedEvents
              .cast<Map<dynamic, dynamic>>()
              .map<Map<String, dynamic>>((Map<dynamic, dynamic> e) => e.cast<String, dynamic>())
              .toList();
      await viewChannel!.invokeMethod<void>('pipeTouchEvents');
      print('replaying ${recordedEvents.length} motion events');
      for (final Map<String, dynamic> event in recordedEvents.reversed) {
        await channel.invokeMethod<void>('synthesizeEvent', event);
      }

      await viewChannel!.invokeMethod<void>('stopTouchEvents');

      if (flutterViewEvents.length != embeddedViewEvents.length) {
        return 'Synthesized ${flutterViewEvents.length} events but the embedded view received ${embeddedViewEvents.length} events';
      }

      final StringBuffer diff = StringBuffer();
      for (int i = 0; i < flutterViewEvents.length; ++i) {
        final String currentDiff = diffMotionEvents(flutterViewEvents[i], embeddedViewEvents[i]);
        if (currentDiff.isEmpty) {
          continue;
        }
        if (diff.isNotEmpty) {
          diff.write(', ');
        }
        diff.write(currentDiff);
      }
      return diff.toString();
    } catch (e) {
      return e.toString();
    }
  }

  @override
  void initState() {
    super.initState();
    channel.setMethodCallHandler(onMethodChannelCall);
  }

  Future<void> saveRecordedEvents(ByteData data, BuildContext context) async {
    if (await channel.invokeMethod<bool>('getStoragePermission') != true) {
      if (context.mounted) {
        showMessage(context, 'External storage permissions are required to save events');
      }
      return;
    }
    try {
      final Directory outDir = (await getExternalStorageDirectory())!;
      // This test only runs on Android so we can assume path separator is '/'.
      final File file = File('${outDir.path}/$kEventsFileName');
      await file.writeAsBytes(data.buffer.asUint8List(0, data.lengthInBytes), flush: true);
      if (!context.mounted) {
        return;
      }
      showMessage(context, 'Saved original events to ${file.path}');
    } catch (e) {
      if (!context.mounted) {
        return;
      }
      showMessage(context, 'Failed saving $e');
    }
  }

  void showMessage(BuildContext context, String message) {
    ScaffoldMessenger.of(
      context,
    ).showSnackBar(SnackBar(content: Text(message), duration: const Duration(seconds: 3)));
  }

  void onPlatformViewCreated(int id) {
    viewChannel = MethodChannel('simple_view/$id')..setMethodCallHandler(onViewMethodChannelCall);
    driverDataHandler.registerHandler('run test').complete(playEventsFile);
  }

  void listenToFlutterViewEvents() {
    viewChannel!.invokeMethod<void>('pipeTouchEvents');
    Timer(const Duration(seconds: 3), () {
      viewChannel!.invokeMethod<void>('stopTouchEvents');
    });
  }

  Future<dynamic> onMethodChannelCall(MethodCall call) {
    switch (call.method) {
      case 'onTouch':
        final Map<dynamic, dynamic> map = call.arguments as Map<dynamic, dynamic>;
        flutterViewEvents.insert(0, map.cast<String, dynamic>());
        if (flutterViewEvents.length > kEventsBufferSize) {
          flutterViewEvents.removeLast();
        }
        setState(() {});
    }
    return Future<dynamic>.value();
  }

  Future<dynamic> onViewMethodChannelCall(MethodCall call) {
    switch (call.method) {
      case 'onTouch':
        final Map<dynamic, dynamic> map = call.arguments as Map<dynamic, dynamic>;
        embeddedViewEvents.insert(0, map.cast<String, dynamic>());
        if (embeddedViewEvents.length > kEventsBufferSize) {
          embeddedViewEvents.removeLast();
        }
        setState(() {});
    }
    return Future<dynamic>.value();
  }

  Widget buildEventTile(BuildContext context, int index) {
    if (embeddedViewEvents.length > index) {
      return TouchEventDiff(flutterViewEvents[index], embeddedViewEvents[index]);
    }
    return Text('Unmatched event, action: ${flutterViewEvents[index]['action']}');
  }
}

class TouchEventDiff extends StatelessWidget {
  const TouchEventDiff(this.originalEvent, this.synthesizedEvent, {super.key});

  final Map<String, dynamic> originalEvent;
  final Map<String, dynamic> synthesizedEvent;

  @override
  Widget build(BuildContext context) {
    Color color;
    final String diff = diffMotionEvents(originalEvent, synthesizedEvent);
    String msg;
    final int action = synthesizedEvent['action'] as int;
    final String actionName = getActionName(getActionMasked(action), action);
    if (diff.isEmpty) {
      color = Colors.green;
      msg = 'Matched event (action $actionName)';
    } else {
      color = Colors.red;
      msg = '[$actionName] $diff';
    }
    return GestureDetector(
      onLongPress: () {
        print('expected:');
        prettyPrintEvent(originalEvent);
        print('\nactual:');
        prettyPrintEvent(synthesizedEvent);
      },
      child: Container(color: color, margin: const EdgeInsets.only(bottom: 2.0), child: Text(msg)),
    );
  }

  void prettyPrintEvent(Map<String, dynamic> event) {
    final StringBuffer buffer = StringBuffer();
    final int action = event['action'] as int;
    final int maskedAction = getActionMasked(action);
    final String actionName = getActionName(maskedAction, action);

    buffer.write('$actionName ');
    if (maskedAction == 5 || maskedAction == 6) {
      buffer.write('pointer: ${getPointerIdx(action)} ');
    }

    final List<Map<dynamic, dynamic>> coords =
        (event['pointerCoords'] as List<dynamic>).cast<Map<dynamic, dynamic>>();
    for (int i = 0; i < coords.length; i++) {
      buffer.write(
        'p$i x: ${coords[i]['x']} y: ${coords[i]['y']}, pressure: ${coords[i]['pressure']} ',
      );
    }
    print(buffer);
  }
}
