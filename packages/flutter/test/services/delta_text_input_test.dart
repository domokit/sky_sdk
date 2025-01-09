// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show jsonDecode;

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import 'text_input_utils.dart';

void main() {
  final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();

  group('DeltaTextInputClient', () {
    late FakeTextChannel fakeTextChannel;

    setUp(() {
      fakeTextChannel = FakeTextChannel((MethodCall call) async {});
      TextInput.setChannel(fakeTextChannel);
    });

    tearDown(() {
      TextInputConnection.debugResetId();
      TextInput.setChannel(SystemChannels.textInput);
    });

    test(
      'DeltaTextInputClient send the correct configuration to the platform and responds to updateEditingValueWithDeltas method correctly',
      () async {
        // Assemble a TextInputConnection so we can verify its change in state.
        final FakeDeltaTextInputClient client = FakeDeltaTextInputClient(TextEditingValue.empty);
        const TextInputConfiguration configuration = TextInputConfiguration(enableDeltaModel: true);
        TextInput.attach(client, configuration);
        expect(client.configuration.enableDeltaModel, true);

        expect(client.latestMethodCall, isEmpty);

        const String jsonDelta =
            '{'
            '"oldText": "",'
            ' "deltaText": "let there be text",'
            ' "deltaStart": 0,'
            ' "deltaEnd": 0,'
            ' "selectionBase": 17,'
            ' "selectionExtent": 17,'
            ' "selectionAffinity" : "TextAffinity.downstream" ,'
            ' "selectionIsDirectional": false,'
            ' "composingBase": -1,'
            ' "composingExtent": -1}';

        // Send updateEditingValueWithDeltas message.
        final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
          'args': <dynamic>[1, jsonDecode('{"deltas": [$jsonDelta]}')],
          'method': 'TextInputClient.updateEditingStateWithDeltas',
        });
        await binding.defaultBinaryMessenger.handlePlatformMessage(
          'flutter/textinput',
          messageBytes,
          (ByteData? _) {},
        );

        expect(client.latestMethodCall, 'updateEditingValueWithDeltas');
      },
    );

    test('Invalid TextRange fails loudly when being converted to JSON - NonTextUpdate', () async {
      final List<FlutterErrorDetails> record = <FlutterErrorDetails>[];
      FlutterError.onError = (FlutterErrorDetails details) {
        record.add(details);
      };

      final FakeDeltaTextInputClient client = FakeDeltaTextInputClient(
        const TextEditingValue(text: '1'),
      );
      const TextInputConfiguration configuration = TextInputConfiguration(enableDeltaModel: true);
      TextInput.attach(client, configuration);

      const String jsonDelta =
          '{'
          '"oldText": "1",'
          ' "deltaText": "",'
          ' "deltaStart": -1,'
          ' "deltaEnd": -1,'
          ' "selectionBase": 3,'
          ' "selectionExtent": 3,'
          ' "selectionAffinity" : "TextAffinity.downstream" ,'
          ' "selectionIsDirectional": false,'
          ' "composingBase": -1,'
          ' "composingExtent": -1}';

      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'method': 'TextInputClient.updateEditingStateWithDeltas',
        'args': <dynamic>[-1, jsonDecode('{"deltas": [$jsonDelta]}')],
      });

      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );
      expect(record.length, 1);
      // Verify the error message in parts because Web formats the message
      // differently from others.
      expect(
        record[0].exception.toString(),
        matches(
          RegExp(
            r'\bThe selection range: TextSelection.collapsed\(offset: 3, affinity: TextAffinity.downstream, isDirectional: false\)(?!\w)',
          ),
        ),
      );
      expect(
        record[0].exception.toString(),
        matches(RegExp(r'\bis not within the bounds of text: 1 of length: 1\b')),
      );
    });

    test(
      'Invalid TextRange fails loudly when being converted to JSON - Faulty deltaStart and deltaEnd',
      () async {
        final List<FlutterErrorDetails> record = <FlutterErrorDetails>[];
        FlutterError.onError = (FlutterErrorDetails details) {
          record.add(details);
        };

        final FakeDeltaTextInputClient client = FakeDeltaTextInputClient(TextEditingValue.empty);
        const TextInputConfiguration configuration = TextInputConfiguration(enableDeltaModel: true);
        TextInput.attach(client, configuration);

        const String jsonDelta =
            '{'
            '"oldText": "",'
            ' "deltaText": "hello",'
            ' "deltaStart": 0,'
            ' "deltaEnd": 1,'
            ' "selectionBase": 5,'
            ' "selectionExtent": 5,'
            ' "selectionAffinity" : "TextAffinity.downstream" ,'
            ' "selectionIsDirectional": false,'
            ' "composingBase": -1,'
            ' "composingExtent": -1}';

        final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
          'method': 'TextInputClient.updateEditingStateWithDeltas',
          'args': <dynamic>[-1, jsonDecode('{"deltas": [$jsonDelta]}')],
        });

        await binding.defaultBinaryMessenger.handlePlatformMessage(
          'flutter/textinput',
          messageBytes,
          (ByteData? _) {},
        );
        expect(record.length, 1);
        // Verify the error message in parts because Web formats the message
        // differently from others.
        expect(
          record[0].exception.toString(),
          matches(RegExp(r'\bThe delta range: TextRange\(start: 0, end: 5\)(?!\w)')),
        );
        expect(
          record[0].exception.toString(),
          matches(RegExp(r'\bis not within the bounds of text:  of length: 0\b')),
        );
      },
    );

    test('Invalid TextRange fails loudly when being converted to JSON - Faulty Selection', () async {
      final List<FlutterErrorDetails> record = <FlutterErrorDetails>[];
      FlutterError.onError = (FlutterErrorDetails details) {
        record.add(details);
      };

      final FakeDeltaTextInputClient client = FakeDeltaTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration(enableDeltaModel: true);
      TextInput.attach(client, configuration);

      const String jsonDelta =
          '{'
          '"oldText": "",'
          ' "deltaText": "hello",'
          ' "deltaStart": 0,'
          ' "deltaEnd": 0,'
          ' "selectionBase": 6,'
          ' "selectionExtent": 6,'
          ' "selectionAffinity" : "TextAffinity.downstream" ,'
          ' "selectionIsDirectional": false,'
          ' "composingBase": -1,'
          ' "composingExtent": -1}';

      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'method': 'TextInputClient.updateEditingStateWithDeltas',
        'args': <dynamic>[-1, jsonDecode('{"deltas": [$jsonDelta]}')],
      });

      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );
      expect(record.length, 1);
      // Verify the error message in parts because Web formats the message
      // differently from others.
      expect(
        record[0].exception.toString(),
        matches(
          RegExp(
            r'\bThe selection range: TextSelection.collapsed\(offset: 6, affinity: TextAffinity.downstream, isDirectional: false\)(?!\w)',
          ),
        ),
      );
      expect(
        record[0].exception.toString(),
        matches(RegExp(r'\bis not within the bounds of text: hello of length: 5\b')),
      );
    });

    test(
      'Invalid TextRange fails loudly when being converted to JSON - Faulty Composing Region',
      () async {
        final List<FlutterErrorDetails> record = <FlutterErrorDetails>[];
        FlutterError.onError = (FlutterErrorDetails details) {
          record.add(details);
        };

        final FakeDeltaTextInputClient client = FakeDeltaTextInputClient(
          const TextEditingValue(text: 'worl'),
        );
        const TextInputConfiguration configuration = TextInputConfiguration(enableDeltaModel: true);
        TextInput.attach(client, configuration);

        const String jsonDelta =
            '{'
            '"oldText": "worl",'
            ' "deltaText": "world",'
            ' "deltaStart": 0,'
            ' "deltaEnd": 4,'
            ' "selectionBase": 5,'
            ' "selectionExtent": 5,'
            ' "selectionAffinity" : "TextAffinity.downstream" ,'
            ' "selectionIsDirectional": false,'
            ' "composingBase": 0,'
            ' "composingExtent": 6}';

        final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
          'method': 'TextInputClient.updateEditingStateWithDeltas',
          'args': <dynamic>[-1, jsonDecode('{"deltas": [$jsonDelta]}')],
        });

        await binding.defaultBinaryMessenger.handlePlatformMessage(
          'flutter/textinput',
          messageBytes,
          (ByteData? _) {},
        );
        expect(record.length, 1);
        // Verify the error message in parts because Web formats the message
        // differently from others.
        expect(
          record[0].exception.toString(),
          matches(RegExp(r'\bThe composing range: TextRange\(start: 0, end: 6\)(?!\w)')),
        );
        expect(
          record[0].exception.toString(),
          matches(RegExp(r'\bis not within the bounds of text: world of length: 5\b')),
        );
      },
    );
  });
}

class FakeDeltaTextInputClient implements DeltaTextInputClient {
  FakeDeltaTextInputClient(this.currentTextEditingValue);

  String latestMethodCall = '';

  @override
  TextEditingValue currentTextEditingValue;

  @override
  AutofillScope? get currentAutofillScope => null;

  @override
  void performAction(TextInputAction action) {
    latestMethodCall = 'performAction';
  }

  @override
  void performPrivateCommand(String action, Map<String, dynamic> data) {
    latestMethodCall = 'performPrivateCommand';
  }

  @override
  void insertContent(KeyboardInsertedContent content) {
    latestMethodCall = 'commitContent';
  }

  @override
  void updateEditingValue(TextEditingValue value) {
    latestMethodCall = 'updateEditingValue';
  }

  @override
  void updateEditingValueWithDeltas(List<TextEditingDelta> textEditingDeltas) {
    latestMethodCall = 'updateEditingValueWithDeltas';
  }

  @override
  void updateFloatingCursor(RawFloatingCursorPoint point) {
    latestMethodCall = 'updateFloatingCursor';
  }

  @override
  void connectionClosed() {
    latestMethodCall = 'connectionClosed';
  }

  @override
  void showAutocorrectionPromptRect(int start, int end) {
    latestMethodCall = 'showAutocorrectionPromptRect';
  }

  @override
  void insertTextPlaceholder(Size size) {
    latestMethodCall = 'insertTextPlaceholder';
  }

  @override
  void removeTextPlaceholder() {
    latestMethodCall = 'removeTextPlaceholder';
  }

  @override
  void showToolbar() {
    latestMethodCall = 'showToolbar';
  }

  @override
  void performSelector(String selectorName) {
    latestMethodCall = 'performSelector';
  }

  TextInputConfiguration get configuration => const TextInputConfiguration(enableDeltaModel: true);

  @override
  void didChangeInputControl(TextInputControl? oldControl, TextInputControl? newControl) {
    latestMethodCall = 'didChangeInputControl';
  }
}
