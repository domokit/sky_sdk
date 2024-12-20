// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

import 'package:flutter_test/flutter_test.dart';

const String license1 = '''
L1Package1
L1Package2
L1Package3

L1Paragraph1

L1Paragraph2

L1Paragraph3''';

const String license2 = '''
L2Package1
L2Package2
L2Package3

L2Paragraph1

L2Paragraph2

L2Paragraph3''';

const String combinedLicenses = '''
$license1
--------------------------------------------------------------------------------
$license2
''';

class TestBinding extends BindingBase with SchedulerBinding, ServicesBinding {
  ViewFocusEvent? lastFocusEvent;

  @override
  TestDefaultBinaryMessenger get defaultBinaryMessenger =>
      super.defaultBinaryMessenger as TestDefaultBinaryMessenger;

  @override
  TestDefaultBinaryMessenger createBinaryMessenger() {
    Future<ByteData?> keyboardHandler(ByteData? message) async {
      return const StandardMethodCodec().encodeSuccessEnvelope(<int, int>{1: 1});
    }

    return TestDefaultBinaryMessenger(
      super.createBinaryMessenger(),
      outboundHandlers: <String, MessageHandler>{'flutter/keyboard': keyboardHandler},
    );
  }

  @override
  void handleViewFocusChanged(ViewFocusEvent event) {
    super.handleViewFocusChanged(event);
    lastFocusEvent = event;
  }
}

void main() {
  final TestBinding binding = TestBinding();

  test('Adds rootBundle LICENSES to LicenseRegistry', () async {
    binding.defaultBinaryMessenger.setMockMessageHandler('flutter/assets', (
      ByteData? message,
    ) async {
      if (const StringCodec().decodeMessage(message) == 'NOTICES.Z' && !kIsWeb) {
        return Uint8List.fromList(gzip.encode(utf8.encode(combinedLicenses))).buffer.asByteData();
      }
      if (const StringCodec().decodeMessage(message) == 'NOTICES' && kIsWeb) {
        return const StringCodec().encodeMessage(combinedLicenses);
      }
      return null;
    });

    final List<LicenseEntry> licenses = await LicenseRegistry.licenses.toList();

    expect(licenses[0].packages, equals(<String>['L1Package1', 'L1Package2', 'L1Package3']));
    expect(
      licenses[0].paragraphs.map((LicenseParagraph p) => p.text),
      equals(<String>['L1Paragraph1', 'L1Paragraph2', 'L1Paragraph3']),
    );

    expect(licenses[1].packages, equals(<String>['L2Package1', 'L2Package2', 'L2Package3']));
    expect(
      licenses[1].paragraphs.map((LicenseParagraph p) => p.text),
      equals(<String>['L2Paragraph1', 'L2Paragraph2', 'L2Paragraph3']),
    );
  });

  test('didHaveMemoryPressure clears asset caches', () async {
    int flutterAssetsCallCount = 0;
    binding.defaultBinaryMessenger.setMockMessageHandler('flutter/assets', (
      ByteData? message,
    ) async {
      flutterAssetsCallCount += 1;
      return ByteData.sublistView(utf8.encode('test_asset_data'));
    });

    await rootBundle.loadString('test_asset');
    expect(flutterAssetsCallCount, 1);
    await rootBundle.loadString('test_asset2');
    expect(flutterAssetsCallCount, 2);
    await rootBundle.loadString('test_asset');
    expect(flutterAssetsCallCount, 2);
    await rootBundle.loadString('test_asset2');
    expect(flutterAssetsCallCount, 2);

    final ByteData message =
        const JSONMessageCodec().encodeMessage(<String, dynamic>{'type': 'memoryPressure'})!;
    await binding.defaultBinaryMessenger.handlePlatformMessage('flutter/system', message, (_) {});

    await rootBundle.loadString('test_asset');
    expect(flutterAssetsCallCount, 3);
    await rootBundle.loadString('test_asset2');
    expect(flutterAssetsCallCount, 4);
    await rootBundle.loadString('test_asset');
    expect(flutterAssetsCallCount, 4);
    await rootBundle.loadString('test_asset2');
    expect(flutterAssetsCallCount, 4);
  });

  test('initInstances sets a default method call handler for SystemChannels.textInput', () async {
    final ByteData message =
        const JSONMessageCodec().encodeMessage(<String, dynamic>{
          'method': 'TextInput.requestElementsInRect',
          'args': null,
        })!;
    await binding.defaultBinaryMessenger.handlePlatformMessage('flutter/textinput', message, (
      ByteData? data,
    ) {
      expect(data, isNotNull);
    });
  });

  test('Calling exitApplication sends a method call to the engine', () async {
    bool sentMessage = false;
    MethodCall? methodCall;
    binding.defaultBinaryMessenger.setMockMessageHandler('flutter/platform', (
      ByteData? message,
    ) async {
      methodCall = const JSONMethodCodec().decodeMethodCall(message);
      sentMessage = true;
      return const JSONMethodCodec().encodeSuccessEnvelope(<String, String>{'response': 'cancel'});
    });
    final AppExitResponse response = await binding.exitApplication(AppExitType.required);
    expect(sentMessage, isTrue);
    expect(methodCall, isNotNull);
    expect((methodCall!.arguments as Map<String, dynamic>)['type'], equals('required'));
    expect(response, equals(AppExitResponse.cancel));
  });

  test('Default handleRequestAppExit returns exit', () async {
    const MethodCall incomingCall = MethodCall('System.requestAppExit', <dynamic>[
      <String, dynamic>{'type': 'cancelable'},
    ]);
    bool receivedReply = false;
    Map<String, dynamic>? result;
    await binding.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/platform',
      const JSONMethodCodec().encodeMethodCall(incomingCall),
      (ByteData? message) async {
        result =
            (const JSONMessageCodec().decodeMessage(message) as List<dynamic>)[0]
                as Map<String, dynamic>;
        receivedReply = true;
      },
    );

    expect(receivedReply, isTrue);
    expect(result, isNotNull);
    expect(result!['response'], equals('exit'));
  });

  test('initInstances synchronizes keyboard state', () async {
    final Set<PhysicalKeyboardKey> physicalKeys = HardwareKeyboard.instance.physicalKeysPressed;
    final Set<LogicalKeyboardKey> logicalKeys = HardwareKeyboard.instance.logicalKeysPressed;

    expect(physicalKeys.length, 1);
    expect(logicalKeys.length, 1);
    expect(physicalKeys.first, const PhysicalKeyboardKey(1));
    expect(logicalKeys.first, const LogicalKeyboardKey(1));
  });

  test('Default handleViewFocusChanged propagates event', () async {
    const ViewFocusEvent event = ViewFocusEvent(
      viewId: 0,
      direction: ViewFocusDirection.forward,
      state: ViewFocusState.focused,
    );
    PlatformDispatcher.instance.onViewFocusChange?.call(event);
    expect(binding.lastFocusEvent, equals(event));
  });
}
