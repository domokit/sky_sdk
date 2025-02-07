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

  group('TextSelection', () {
    test('The invalid selection is a singleton', () {
      const TextSelection invalidSelection1 = TextSelection(
        baseOffset: -1,
        extentOffset: 0,
        isDirectional: true,
      );
      const TextSelection invalidSelection2 = TextSelection(
        baseOffset: 123,
        extentOffset: -1,
        affinity: TextAffinity.upstream,
      );
      expect(invalidSelection1, invalidSelection2);
      expect(invalidSelection1.hashCode, invalidSelection2.hashCode);
    });

    test('TextAffinity does not affect equivalence when the selection is not collapsed', () {
      const TextSelection selection1 = TextSelection(baseOffset: 1, extentOffset: 2);
      const TextSelection selection2 = TextSelection(
        baseOffset: 1,
        extentOffset: 2,
        affinity: TextAffinity.upstream,
      );
      expect(selection1, selection2);
      expect(selection1.hashCode, selection2.hashCode);
    });
  });

  group('TextEditingValue', () {
    group('replaced', () {
      const String testText = 'From a false proposition, anything follows.';

      test('selection deletion', () {
        const TextSelection selection = TextSelection(baseOffset: 5, extentOffset: 13);
        expect(
          const TextEditingValue(text: testText, selection: selection).replaced(selection, ''),
          const TextEditingValue(
            text: 'From proposition, anything follows.',
            selection: TextSelection.collapsed(offset: 5),
          ),
        );
      });

      test('reversed selection deletion', () {
        const TextSelection selection = TextSelection(baseOffset: 13, extentOffset: 5);
        expect(
          const TextEditingValue(text: testText, selection: selection).replaced(selection, ''),
          const TextEditingValue(
            text: 'From proposition, anything follows.',
            selection: TextSelection.collapsed(offset: 5),
          ),
        );
      });

      test('insert', () {
        const TextSelection selection = TextSelection.collapsed(offset: 5);
        expect(
          const TextEditingValue(text: testText, selection: selection).replaced(selection, 'AA'),
          const TextEditingValue(
            text: 'From AAa false proposition, anything follows.',
            // The caret moves to the end of the text inserted.
            selection: TextSelection.collapsed(offset: 7),
          ),
        );
      });

      test('replace before selection', () {
        const TextSelection selection = TextSelection(baseOffset: 13, extentOffset: 5);
        expect(
          // From |a false |proposition, anything follows.
          // Replace the first whitespace with "AA".
          const TextEditingValue(
            text: testText,
            selection: selection,
          ).replaced(const TextRange(start: 4, end: 5), 'AA'),
          const TextEditingValue(
            text: 'FromAAa false proposition, anything follows.',
            selection: TextSelection(baseOffset: 14, extentOffset: 6),
          ),
        );
      });

      test('replace after selection', () {
        const TextSelection selection = TextSelection(baseOffset: 13, extentOffset: 5);
        expect(
          // From |a false |proposition, anything follows.
          // replace the first "p" with "AA".
          const TextEditingValue(
            text: testText,
            selection: selection,
          ).replaced(const TextRange(start: 13, end: 14), 'AA'),
          const TextEditingValue(
            text: 'From a false AAroposition, anything follows.',
            selection: selection,
          ),
        );
      });

      test('replace inside selection - start boundary', () {
        const TextSelection selection = TextSelection(baseOffset: 13, extentOffset: 5);
        expect(
          // From |a false |proposition, anything follows.
          // replace the first "a" with "AA".
          const TextEditingValue(
            text: testText,
            selection: selection,
          ).replaced(const TextRange(start: 5, end: 6), 'AA'),
          const TextEditingValue(
            text: 'From AA false proposition, anything follows.',
            selection: TextSelection(baseOffset: 14, extentOffset: 5),
          ),
        );
      });

      test('replace inside selection - end boundary', () {
        const TextSelection selection = TextSelection(baseOffset: 13, extentOffset: 5);
        expect(
          // From |a false |proposition, anything follows.
          // replace the second whitespace with "AA".
          const TextEditingValue(
            text: testText,
            selection: selection,
          ).replaced(const TextRange(start: 12, end: 13), 'AA'),
          const TextEditingValue(
            text: 'From a falseAAproposition, anything follows.',
            selection: TextSelection(baseOffset: 14, extentOffset: 5),
          ),
        );
      });

      test('delete after selection', () {
        const TextSelection selection = TextSelection(baseOffset: 13, extentOffset: 5);
        expect(
          // From |a false |proposition, anything follows.
          // Delete the first "p".
          const TextEditingValue(
            text: testText,
            selection: selection,
          ).replaced(const TextRange(start: 13, end: 14), ''),
          const TextEditingValue(
            text: 'From a false roposition, anything follows.',
            selection: selection,
          ),
        );
      });

      test('delete inside selection - start boundary', () {
        const TextSelection selection = TextSelection(baseOffset: 13, extentOffset: 5);
        expect(
          // From |a false |proposition, anything follows.
          // Delete the first "a".
          const TextEditingValue(
            text: testText,
            selection: selection,
          ).replaced(const TextRange(start: 5, end: 6), ''),
          const TextEditingValue(
            text: 'From  false proposition, anything follows.',
            selection: TextSelection(baseOffset: 12, extentOffset: 5),
          ),
        );
      });

      test('delete inside selection - end boundary', () {
        const TextSelection selection = TextSelection(baseOffset: 13, extentOffset: 5);
        expect(
          // From |a false |proposition, anything follows.
          // Delete the second whitespace.
          const TextEditingValue(
            text: testText,
            selection: selection,
          ).replaced(const TextRange(start: 12, end: 13), ''),
          const TextEditingValue(
            text: 'From a falseproposition, anything follows.',
            selection: TextSelection(baseOffset: 12, extentOffset: 5),
          ),
        );
      });
    });
  });

  group('TextInput message channels', () {
    late FakeTextChannel fakeTextChannel;

    setUp(() {
      fakeTextChannel = FakeTextChannel((MethodCall call) async {});
      TextInput.setChannel(fakeTextChannel);
    });

    tearDown(() {
      TextInputConnection.debugResetId();
      TextInput.setChannel(SystemChannels.textInput);
    });

    test('text input client handler responds to reattach with setClient', () async {
      final FakeTextInputClient client = FakeTextInputClient(const TextEditingValue(text: 'test1'));
      TextInput.attach(client, client.configuration);
      fakeTextChannel.validateOutgoingMethodCalls(<MethodCall>[
        MethodCall('TextInput.setClient', <dynamic>[1, client.configuration.toJson()]),
      ]);

      fakeTextChannel.incoming!(const MethodCall('TextInputClient.requestExistingInputState'));

      expect(fakeTextChannel.outgoingCalls.length, 3);
      fakeTextChannel.validateOutgoingMethodCalls(<MethodCall>[
        // From original attach
        MethodCall('TextInput.setClient', <dynamic>[1, client.configuration.toJson()]),
        // From requestExistingInputState
        MethodCall('TextInput.setClient', <dynamic>[1, client.configuration.toJson()]),
        MethodCall('TextInput.setEditingState', client.currentTextEditingValue.toJSON()),
      ]);
    });

    test(
      'text input client handler responds to reattach with setClient (null TextEditingValue)',
      () async {
        final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
        TextInput.attach(client, client.configuration);
        fakeTextChannel.validateOutgoingMethodCalls(<MethodCall>[
          MethodCall('TextInput.setClient', <dynamic>[1, client.configuration.toJson()]),
        ]);

        fakeTextChannel.incoming!(const MethodCall('TextInputClient.requestExistingInputState'));

        expect(fakeTextChannel.outgoingCalls.length, 3);
        fakeTextChannel.validateOutgoingMethodCalls(<MethodCall>[
          // From original attach
          MethodCall('TextInput.setClient', <dynamic>[1, client.configuration.toJson()]),
          // From original attach
          MethodCall('TextInput.setClient', <dynamic>[1, client.configuration.toJson()]),
          // From requestExistingInputState
          const MethodCall('TextInput.setEditingState', <String, dynamic>{
            'text': '',
            'selectionBase': -1,
            'selectionExtent': -1,
            'selectionAffinity': 'TextAffinity.downstream',
            'selectionIsDirectional': false,
            'composingBase': -1,
            'composingExtent': -1,
          }),
        ]);
      },
    );

    test('Invalid TextRange fails loudly when being converted to JSON', () async {
      final List<FlutterErrorDetails> record = <FlutterErrorDetails>[];
      FlutterError.onError = (FlutterErrorDetails details) {
        record.add(details);
      };

      final FakeTextInputClient client = FakeTextInputClient(const TextEditingValue(text: 'test3'));
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'method': 'TextInputClient.updateEditingState',
        'args': <dynamic>[
          -1,
          <String, dynamic>{'text': '1', 'selectionBase': 2, 'selectionExtent': 3},
        ],
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
        matches(RegExp(r'\brange.start >= 0 && range.start <= text.length\b')),
      );
      expect(
        record[0].exception.toString(),
        matches(RegExp(r'\bRange start 2 is out of text of length 1\b')),
      );
    });

    test('FloatingCursor coordinates type-casting', () async {
      // Regression test for https://github.com/flutter/flutter/issues/109632.
      final List<FlutterErrorDetails> errors = <FlutterErrorDetails>[];
      FlutterError.onError = errors.add;

      final FakeTextInputClient client = FakeTextInputClient(const TextEditingValue(text: 'test3'));
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'method': 'TextInputClient.updateFloatingCursor',
        'args': <dynamic>[
          -1,
          'FloatingCursorDragState.update',
          <String, dynamic>{'X': 2, 'Y': 3},
        ],
      });

      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(errors, isEmpty);
    });
  });

  group('TextInputConfiguration', () {
    late TextInputConfiguration fakeTextInputConfiguration;
    late TextInputConfiguration fakeTextInputConfiguration2;

    setUp(() {
      // If you create two objects with `const` with the same values, the second object will be equal to the first one by reference.
      // This means that even without overriding the `equals` method, the test will pass.
      // ignore: prefer_const_constructors
      fakeTextInputConfiguration = TextInputConfiguration(
        viewId: 1,
        actionLabel: 'label1',
        smartDashesType: SmartDashesType.enabled,
        smartQuotesType: SmartQuotesType.enabled,
        // ignore: prefer_const_literals_to_create_immutables
        allowedMimeTypes: <String>['text/plain', 'application/pdf'],
      );
      fakeTextInputConfiguration2 = fakeTextInputConfiguration.copyWith();
    });

    tearDown(() {
      TextInputConnection.debugResetId();
    });

    test('equality operator works correctly', () {
      expect(fakeTextInputConfiguration, equals(fakeTextInputConfiguration2));
      expect(fakeTextInputConfiguration.viewId, equals(fakeTextInputConfiguration2.viewId));
      expect(fakeTextInputConfiguration.inputType, equals(fakeTextInputConfiguration2.inputType));
      expect(
        fakeTextInputConfiguration.inputAction,
        equals(fakeTextInputConfiguration2.inputAction),
      );
      expect(
        fakeTextInputConfiguration.autocorrect,
        equals(fakeTextInputConfiguration2.autocorrect),
      );
      expect(
        fakeTextInputConfiguration.enableSuggestions,
        equals(fakeTextInputConfiguration2.enableSuggestions),
      );
      expect(
        fakeTextInputConfiguration.obscureText,
        equals(fakeTextInputConfiguration2.obscureText),
      );
      expect(fakeTextInputConfiguration.readOnly, equals(fakeTextInputConfiguration2.readOnly));
      expect(
        fakeTextInputConfiguration.smartDashesType,
        equals(fakeTextInputConfiguration2.smartDashesType),
      );
      expect(
        fakeTextInputConfiguration.smartQuotesType,
        equals(fakeTextInputConfiguration2.smartQuotesType),
      );
      expect(
        fakeTextInputConfiguration.enableInteractiveSelection,
        equals(fakeTextInputConfiguration2.enableInteractiveSelection),
      );
      expect(
        fakeTextInputConfiguration.actionLabel,
        equals(fakeTextInputConfiguration2.actionLabel),
      );
      expect(
        fakeTextInputConfiguration.keyboardAppearance,
        equals(fakeTextInputConfiguration2.keyboardAppearance),
      );
      expect(
        fakeTextInputConfiguration.textCapitalization,
        equals(fakeTextInputConfiguration2.textCapitalization),
      );
      expect(
        fakeTextInputConfiguration.autofillConfiguration,
        equals(fakeTextInputConfiguration2.autofillConfiguration),
      );
      expect(
        fakeTextInputConfiguration.enableIMEPersonalizedLearning,
        equals(fakeTextInputConfiguration2.enableIMEPersonalizedLearning),
      );
      expect(
        fakeTextInputConfiguration.allowedMimeTypes,
        equals(fakeTextInputConfiguration2.allowedMimeTypes),
      );
      expect(
        fakeTextInputConfiguration.enableDeltaModel,
        equals(fakeTextInputConfiguration2.enableDeltaModel),
      );
    });

    test('copyWith method works correctly', () {
      fakeTextInputConfiguration2 = fakeTextInputConfiguration.copyWith();

      expect(fakeTextInputConfiguration, equals(fakeTextInputConfiguration2));
      expect(fakeTextInputConfiguration.viewId, equals(fakeTextInputConfiguration2.viewId));
      expect(fakeTextInputConfiguration.inputType, equals(fakeTextInputConfiguration2.inputType));
      expect(
        fakeTextInputConfiguration.inputAction,
        equals(fakeTextInputConfiguration2.inputAction),
      );
      expect(
        fakeTextInputConfiguration.autocorrect,
        equals(fakeTextInputConfiguration2.autocorrect),
      );
      expect(
        fakeTextInputConfiguration.enableSuggestions,
        equals(fakeTextInputConfiguration2.enableSuggestions),
      );
      expect(
        fakeTextInputConfiguration.obscureText,
        equals(fakeTextInputConfiguration2.obscureText),
      );
      expect(fakeTextInputConfiguration.readOnly, equals(fakeTextInputConfiguration2.readOnly));
      expect(
        fakeTextInputConfiguration.smartDashesType,
        equals(fakeTextInputConfiguration2.smartDashesType),
      );
      expect(
        fakeTextInputConfiguration.smartQuotesType,
        equals(fakeTextInputConfiguration2.smartQuotesType),
      );
      expect(
        fakeTextInputConfiguration.enableInteractiveSelection,
        equals(fakeTextInputConfiguration2.enableInteractiveSelection),
      );
      expect(
        fakeTextInputConfiguration.actionLabel,
        equals(fakeTextInputConfiguration2.actionLabel),
      );
      expect(
        fakeTextInputConfiguration.keyboardAppearance,
        equals(fakeTextInputConfiguration2.keyboardAppearance),
      );
      expect(
        fakeTextInputConfiguration.textCapitalization,
        equals(fakeTextInputConfiguration2.textCapitalization),
      );
      expect(
        fakeTextInputConfiguration.autofillConfiguration,
        equals(fakeTextInputConfiguration2.autofillConfiguration),
      );
      expect(
        fakeTextInputConfiguration.enableIMEPersonalizedLearning,
        equals(fakeTextInputConfiguration2.enableIMEPersonalizedLearning),
      );
      expect(
        fakeTextInputConfiguration.allowedMimeTypes,
        equals(fakeTextInputConfiguration2.allowedMimeTypes),
      );
      expect(
        fakeTextInputConfiguration.enableDeltaModel,
        equals(fakeTextInputConfiguration2.enableDeltaModel),
      );
    });

    test('hashCode works correctly', () {
      expect(fakeTextInputConfiguration.hashCode, equals(fakeTextInputConfiguration2.hashCode));

      expect(
        fakeTextInputConfiguration.viewId.hashCode,
        equals(fakeTextInputConfiguration2.viewId.hashCode),
      );
      expect(
        fakeTextInputConfiguration.inputType.hashCode,
        equals(fakeTextInputConfiguration2.inputType.hashCode),
      );
      expect(
        fakeTextInputConfiguration.inputAction.hashCode,
        equals(fakeTextInputConfiguration2.inputAction.hashCode),
      );
      expect(
        fakeTextInputConfiguration.autocorrect.hashCode,
        equals(fakeTextInputConfiguration2.autocorrect.hashCode),
      );
      expect(
        fakeTextInputConfiguration.enableSuggestions.hashCode,
        equals(fakeTextInputConfiguration2.enableSuggestions.hashCode),
      );
      expect(
        fakeTextInputConfiguration.obscureText.hashCode,
        equals(fakeTextInputConfiguration2.obscureText.hashCode),
      );
      expect(
        fakeTextInputConfiguration.readOnly.hashCode,
        equals(fakeTextInputConfiguration2.readOnly.hashCode),
      );
      expect(
        fakeTextInputConfiguration.smartDashesType.hashCode,
        equals(fakeTextInputConfiguration2.smartDashesType.hashCode),
      );
      expect(
        fakeTextInputConfiguration.smartQuotesType.hashCode,
        equals(fakeTextInputConfiguration2.smartQuotesType.hashCode),
      );
      expect(
        fakeTextInputConfiguration.enableInteractiveSelection.hashCode,
        equals(fakeTextInputConfiguration2.enableInteractiveSelection.hashCode),
      );
      expect(
        fakeTextInputConfiguration.actionLabel.hashCode,
        equals(fakeTextInputConfiguration2.actionLabel.hashCode),
      );
      expect(
        fakeTextInputConfiguration.keyboardAppearance.hashCode,
        equals(fakeTextInputConfiguration2.keyboardAppearance.hashCode),
      );
      expect(
        fakeTextInputConfiguration.textCapitalization.hashCode,
        equals(fakeTextInputConfiguration2.textCapitalization.hashCode),
      );
      expect(
        fakeTextInputConfiguration.autofillConfiguration.hashCode,
        equals(fakeTextInputConfiguration2.autofillConfiguration.hashCode),
      );
      expect(
        fakeTextInputConfiguration.enableIMEPersonalizedLearning.hashCode,
        equals(fakeTextInputConfiguration2.enableIMEPersonalizedLearning.hashCode),
      );
      expect(
        Object.hashAll(fakeTextInputConfiguration.allowedMimeTypes),
        equals(Object.hashAll(fakeTextInputConfiguration2.allowedMimeTypes)),
      );
      expect(
        fakeTextInputConfiguration.enableDeltaModel.hashCode,
        equals(fakeTextInputConfiguration2.enableDeltaModel.hashCode),
      );
    });

    test('sets expected defaults', () {
      const TextInputConfiguration configuration = TextInputConfiguration();
      expect(configuration.inputType, TextInputType.text);
      expect(configuration.readOnly, false);
      expect(configuration.obscureText, false);
      expect(configuration.enableDeltaModel, false);
      expect(configuration.autocorrect, true);
      expect(configuration.actionLabel, null);
      expect(configuration.textCapitalization, TextCapitalization.none);
      expect(configuration.keyboardAppearance, Brightness.light);
    });

    test('text serializes to JSON', () async {
      const TextInputConfiguration configuration = TextInputConfiguration(
        readOnly: true,
        obscureText: true,
        autocorrect: false,
        actionLabel: 'xyzzy',
      );
      final Map<String, dynamic> json = configuration.toJson();
      expect(json['inputType'], <String, dynamic>{
        'name': 'TextInputType.text',
        'signed': null,
        'decimal': null,
      });
      expect(json['readOnly'], true);
      expect(json['obscureText'], true);
      expect(json['autocorrect'], false);
      expect(json['actionLabel'], 'xyzzy');
    });

    test('number serializes to JSON', () async {
      const TextInputConfiguration configuration = TextInputConfiguration(
        inputType: TextInputType.numberWithOptions(decimal: true),
        obscureText: true,
        autocorrect: false,
        actionLabel: 'xyzzy',
      );
      final Map<String, dynamic> json = configuration.toJson();
      expect(json['inputType'], <String, dynamic>{
        'name': 'TextInputType.number',
        'signed': false,
        'decimal': true,
      });
      expect(json['readOnly'], false);
      expect(json['obscureText'], true);
      expect(json['autocorrect'], false);
      expect(json['actionLabel'], 'xyzzy');
    });

    test('basic structure', () async {
      const TextInputType text = TextInputType.text;
      const TextInputType number = TextInputType.number;
      const TextInputType number2 = TextInputType.number;
      const TextInputType signed = TextInputType.numberWithOptions(signed: true);
      const TextInputType signed2 = TextInputType.numberWithOptions(signed: true);
      const TextInputType decimal = TextInputType.numberWithOptions(decimal: true);
      const TextInputType signedDecimal = TextInputType.numberWithOptions(
        signed: true,
        decimal: true,
      );

      expect(
        text.toString(),
        'TextInputType(name: TextInputType.text, signed: null, decimal: null)',
      );
      expect(
        number.toString(),
        'TextInputType(name: TextInputType.number, signed: false, decimal: false)',
      );
      expect(
        signed.toString(),
        'TextInputType(name: TextInputType.number, signed: true, decimal: false)',
      );
      expect(
        decimal.toString(),
        'TextInputType(name: TextInputType.number, signed: false, decimal: true)',
      );
      expect(
        signedDecimal.toString(),
        'TextInputType(name: TextInputType.number, signed: true, decimal: true)',
      );
      expect(
        TextInputType.multiline.toString(),
        'TextInputType(name: TextInputType.multiline, signed: null, decimal: null)',
      );
      expect(
        TextInputType.phone.toString(),
        'TextInputType(name: TextInputType.phone, signed: null, decimal: null)',
      );
      expect(
        TextInputType.datetime.toString(),
        'TextInputType(name: TextInputType.datetime, signed: null, decimal: null)',
      );
      expect(
        TextInputType.emailAddress.toString(),
        'TextInputType(name: TextInputType.emailAddress, signed: null, decimal: null)',
      );
      expect(
        TextInputType.url.toString(),
        'TextInputType(name: TextInputType.url, signed: null, decimal: null)',
      );
      expect(
        TextInputType.visiblePassword.toString(),
        'TextInputType(name: TextInputType.visiblePassword, signed: null, decimal: null)',
      );
      expect(
        TextInputType.name.toString(),
        'TextInputType(name: TextInputType.name, signed: null, decimal: null)',
      );
      expect(
        TextInputType.streetAddress.toString(),
        'TextInputType(name: TextInputType.address, signed: null, decimal: null)',
      );
      expect(
        TextInputType.none.toString(),
        'TextInputType(name: TextInputType.none, signed: null, decimal: null)',
      );
      expect(
        TextInputType.webSearch.toString(),
        'TextInputType(name: TextInputType.webSearch, signed: null, decimal: null)',
      );
      expect(
        TextInputType.twitter.toString(),
        'TextInputType(name: TextInputType.twitter, signed: null, decimal: null)',
      );

      expect(text == number, false);
      expect(number == number2, true);
      expect(number == signed, false);
      expect(signed == signed2, true);
      expect(signed == decimal, false);
      expect(signed == signedDecimal, false);
      expect(decimal == signedDecimal, false);

      expect(text.hashCode == number.hashCode, false);
      expect(number.hashCode == number2.hashCode, true);
      expect(number.hashCode == signed.hashCode, false);
      expect(signed.hashCode == signed2.hashCode, true);
      expect(signed.hashCode == decimal.hashCode, false);
      expect(signed.hashCode == signedDecimal.hashCode, false);
      expect(decimal.hashCode == signedDecimal.hashCode, false);

      expect(TextInputType.text.index, 0);
      expect(TextInputType.multiline.index, 1);
      expect(TextInputType.number.index, 2);
      expect(TextInputType.phone.index, 3);
      expect(TextInputType.datetime.index, 4);
      expect(TextInputType.emailAddress.index, 5);
      expect(TextInputType.url.index, 6);
      expect(TextInputType.visiblePassword.index, 7);
      expect(TextInputType.name.index, 8);
      expect(TextInputType.streetAddress.index, 9);
      expect(TextInputType.none.index, 10);
      expect(TextInputType.webSearch.index, 11);
      expect(TextInputType.twitter.index, 12);

      expect(
        TextEditingValue.empty.toString(),
        'TextEditingValue(text: \u2524\u251C, selection: ${const TextSelection.collapsed(offset: -1)}, composing: ${TextRange.empty})',
      );
      expect(
        const TextEditingValue(text: 'Sample Text').toString(),
        'TextEditingValue(text: \u2524Sample Text\u251C, selection: ${const TextSelection.collapsed(offset: -1)}, composing: ${TextRange.empty})',
      );
    });

    test('TextInputClient onConnectionClosed method is called', () async {
      // Assemble a TextInputConnection so we can verify its change in state.
      final FakeTextInputClient client = FakeTextInputClient(const TextEditingValue(text: 'test3'));
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      expect(client.latestMethodCall, isEmpty);

      // Send onConnectionClosed message.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[1],
        'method': 'TextInputClient.onConnectionClosed',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(client.latestMethodCall, 'connectionClosed');
    });

    test('TextInputClient insertContent method is called', () async {
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      expect(client.latestMethodCall, isEmpty);

      // Send commitContent message with fake GIF data.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[
          1,
          'TextInputAction.commitContent',
          jsonDecode(
            '{"mimeType": "image/gif", "data": [0,1,0,1,0,1,0,0,0], "uri": "content://com.google.android.inputmethod.latin.fileprovider/test.gif"}',
          ),
        ],
        'method': 'TextInputClient.performAction',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(client.latestMethodCall, 'commitContent');
    });

    test('TextInputClient performSelectors method is called', () async {
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      expect(client.performedSelectors, isEmpty);
      expect(client.latestMethodCall, isEmpty);

      // Send performSelectors message.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[
          1,
          <dynamic>['selector1', 'selector2'],
        ],
        'method': 'TextInputClient.performSelectors',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(client.latestMethodCall, 'performSelector');
      expect(client.performedSelectors, <String>['selector1', 'selector2']);
    });

    test('TextInputClient performPrivateCommand method is called', () async {
      // Assemble a TextInputConnection so we can verify its change in state.
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      expect(client.latestMethodCall, isEmpty);

      // Send performPrivateCommand message.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[
          1,
          jsonDecode('{"action": "actionCommand", "data": {"input_context" : "abcdefg"}}'),
        ],
        'method': 'TextInputClient.performPrivateCommand',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(client.latestMethodCall, 'performPrivateCommand');
    });

    test('TextInputClient performPrivateCommand method is called with float', () async {
      // Assemble a TextInputConnection so we can verify its change in state.
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      expect(client.latestMethodCall, isEmpty);

      // Send performPrivateCommand message.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[
          1,
          jsonDecode('{"action": "actionCommand", "data": {"input_context" : 0.5}}'),
        ],
        'method': 'TextInputClient.performPrivateCommand',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(client.latestMethodCall, 'performPrivateCommand');
    });

    test(
      'TextInputClient performPrivateCommand method is called with CharSequence array',
      () async {
        // Assemble a TextInputConnection so we can verify its change in state.
        final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
        const TextInputConfiguration configuration = TextInputConfiguration();
        TextInput.attach(client, configuration);

        expect(client.latestMethodCall, isEmpty);

        // Send performPrivateCommand message.
        final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
          'args': <dynamic>[
            1,
            jsonDecode('{"action": "actionCommand", "data": {"input_context" : ["abc", "efg"]}}'),
          ],
          'method': 'TextInputClient.performPrivateCommand',
        });
        await binding.defaultBinaryMessenger.handlePlatformMessage(
          'flutter/textinput',
          messageBytes,
          (ByteData? _) {},
        );

        expect(client.latestMethodCall, 'performPrivateCommand');
      },
    );

    test('TextInputClient performPrivateCommand method is called with CharSequence', () async {
      // Assemble a TextInputConnection so we can verify its change in state.
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      expect(client.latestMethodCall, isEmpty);

      // Send performPrivateCommand message.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[
          1,
          jsonDecode('{"action": "actionCommand", "data": {"input_context" : "abc"}}'),
        ],
        'method': 'TextInputClient.performPrivateCommand',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(client.latestMethodCall, 'performPrivateCommand');
    });

    test('TextInputClient performPrivateCommand method is called with float array', () async {
      // Assemble a TextInputConnection so we can verify its change in state.
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      expect(client.latestMethodCall, isEmpty);

      // Send performPrivateCommand message.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[
          1,
          jsonDecode('{"action": "actionCommand", "data": {"input_context" : [0.5, 0.8]}}'),
        ],
        'method': 'TextInputClient.performPrivateCommand',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(client.latestMethodCall, 'performPrivateCommand');
    });

    test('TextInputClient performPrivateCommand method is called with no data at all', () async {
      // Assemble a TextInputConnection so we can verify its change in state.
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      expect(client.latestMethodCall, isEmpty);

      // Send performPrivateCommand message.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[
          1,
          jsonDecode('{"action": "actionCommand"}'), // No `data` parameter.
        ],
        'method': 'TextInputClient.performPrivateCommand',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(client.latestMethodCall, 'performPrivateCommand');
      expect(client.latestPrivateCommandData, <String, dynamic>{});
    });

    test('TextInputClient showAutocorrectionPromptRect method is called', () async {
      // Assemble a TextInputConnection so we can verify its change in state.
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      expect(client.latestMethodCall, isEmpty);

      // Send onConnectionClosed message.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[1, 0, 1],
        'method': 'TextInputClient.showAutocorrectionPromptRect',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(client.latestMethodCall, 'showAutocorrectionPromptRect');
    });

    test('TextInputClient showToolbar method is called', () async {
      // Assemble a TextInputConnection so we can verify its change in state.
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      expect(client.latestMethodCall, isEmpty);

      // Send showToolbar message.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[1, 0, 1],
        'method': 'TextInputClient.showToolbar',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(client.latestMethodCall, 'showToolbar');
    });
  });

  group('Scribble interactions', () {
    tearDown(() {
      TextInputConnection.debugResetId();
    });

    test('TextInputClient scribbleInteractionBegan and scribbleInteractionFinished', () async {
      // Assemble a TextInputConnection so we can verify its change in state.
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      final TextInputConnection connection = TextInput.attach(client, configuration);

      expect(connection.scribbleInProgress, false);

      // Send scribbleInteractionBegan message.
      ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[1, 0, 1],
        'method': 'TextInputClient.scribbleInteractionBegan',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(connection.scribbleInProgress, true);

      // Send scribbleInteractionFinished message.
      messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[1, 0, 1],
        'method': 'TextInputClient.scribbleInteractionFinished',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      expect(connection.scribbleInProgress, false);
    });

    test('TextInputClient focusElement', () async {
      // Assemble a TextInputConnection so we can verify its change in state.
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      final FakeScribbleElement targetElement = FakeScribbleElement(elementIdentifier: 'target');
      TextInput.registerScribbleElement(targetElement.elementIdentifier, targetElement);
      final FakeScribbleElement otherElement = FakeScribbleElement(elementIdentifier: 'other');
      TextInput.registerScribbleElement(otherElement.elementIdentifier, otherElement);

      expect(targetElement.latestMethodCall, isEmpty);
      expect(otherElement.latestMethodCall, isEmpty);

      // Send focusElement message.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[targetElement.elementIdentifier, 0.0, 0.0],
        'method': 'TextInputClient.focusElement',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );

      TextInput.unregisterScribbleElement(targetElement.elementIdentifier);
      TextInput.unregisterScribbleElement(otherElement.elementIdentifier);

      expect(targetElement.latestMethodCall, 'onScribbleFocus');
      expect(otherElement.latestMethodCall, isEmpty);
    });

    test('TextInputClient requestElementsInRect', () async {
      // Assemble a TextInputConnection so we can verify its change in state.
      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      const TextInputConfiguration configuration = TextInputConfiguration();
      TextInput.attach(client, configuration);

      final List<FakeScribbleElement> targetElements = <FakeScribbleElement>[
        FakeScribbleElement(
          elementIdentifier: 'target1',
          bounds: const Rect.fromLTWH(0.0, 0.0, 100.0, 100.0),
        ),
        FakeScribbleElement(
          elementIdentifier: 'target2',
          bounds: const Rect.fromLTWH(0.0, 100.0, 100.0, 100.0),
        ),
      ];
      final List<FakeScribbleElement> otherElements = <FakeScribbleElement>[
        FakeScribbleElement(
          elementIdentifier: 'other1',
          bounds: const Rect.fromLTWH(100.0, 0.0, 100.0, 100.0),
        ),
        FakeScribbleElement(
          elementIdentifier: 'other2',
          bounds: const Rect.fromLTWH(100.0, 100.0, 100.0, 100.0),
        ),
      ];

      void registerElements(FakeScribbleElement element) =>
          TextInput.registerScribbleElement(element.elementIdentifier, element);
      void unregisterElements(FakeScribbleElement element) =>
          TextInput.unregisterScribbleElement(element.elementIdentifier);

      <FakeScribbleElement>[...targetElements, ...otherElements].forEach(registerElements);

      // Send requestElementsInRect message.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'args': <dynamic>[0.0, 50.0, 50.0, 100.0],
        'method': 'TextInputClient.requestElementsInRect',
      });
      ByteData? responseBytes;
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? response) {
          responseBytes = response;
        },
      );

      <FakeScribbleElement>[...targetElements, ...otherElements].forEach(unregisterElements);

      final List<List<dynamic>> responses =
          (const JSONMessageCodec().decodeMessage(responseBytes) as List<dynamic>)
              .cast<List<dynamic>>();
      expect(responses.first.length, 2);
      expect(
        responses.first.first,
        containsAllInOrder(<dynamic>[
          targetElements.first.elementIdentifier,
          0.0,
          0.0,
          100.0,
          100.0,
        ]),
      );
      expect(
        responses.first.last,
        containsAllInOrder(<dynamic>[
          targetElements.last.elementIdentifier,
          0.0,
          100.0,
          100.0,
          100.0,
        ]),
      );
    });
  });

  test('TextEditingValue.isComposingRangeValid', () async {
    // The composing range is empty.
    expect(TextEditingValue.empty.isComposingRangeValid, isFalse);

    expect(
      const TextEditingValue(
        text: 'test',
        composing: TextRange(start: 1, end: 0),
      ).isComposingRangeValid,
      isFalse,
    );

    // The composing range is out of range for the text.
    expect(
      const TextEditingValue(
        text: 'test',
        composing: TextRange(start: 1, end: 5),
      ).isComposingRangeValid,
      isFalse,
    );

    // The composing range is out of range for the text.
    expect(
      const TextEditingValue(
        text: 'test',
        composing: TextRange(start: -1, end: 4),
      ).isComposingRangeValid,
      isFalse,
    );

    expect(
      const TextEditingValue(
        text: 'test',
        composing: TextRange(start: 1, end: 4),
      ).isComposingRangeValid,
      isTrue,
    );
  });

  group('TextInputControl', () {
    late FakeTextChannel fakeTextChannel;

    setUp(() {
      fakeTextChannel = FakeTextChannel((MethodCall call) async {});
      TextInput.setChannel(fakeTextChannel);
    });

    tearDown(() {
      TextInput.restorePlatformInputControl();
      TextInputConnection.debugResetId();
      TextInput.setChannel(SystemChannels.textInput);
    });

    test('gets attached and detached', () {
      final FakeTextInputControl control = FakeTextInputControl();
      TextInput.setInputControl(control);

      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      final TextInputConnection connection = TextInput.attach(
        client,
        const TextInputConfiguration(),
      );

      final List<String> expectedMethodCalls = <String>['attach'];
      expect(control.methodCalls, expectedMethodCalls);

      connection.close();
      expectedMethodCalls.add('detach');
      expect(control.methodCalls, expectedMethodCalls);
    });

    test('receives text input state changes', () {
      final FakeTextInputControl control = FakeTextInputControl();
      TextInput.setInputControl(control);

      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      final TextInputConnection connection = TextInput.attach(
        client,
        const TextInputConfiguration(),
      );
      control.methodCalls.clear();

      final List<String> expectedMethodCalls = <String>[];

      connection.updateConfig(const TextInputConfiguration());
      expectedMethodCalls.add('updateConfig');
      expect(control.methodCalls, expectedMethodCalls);

      connection.setEditingState(TextEditingValue.empty);
      expectedMethodCalls.add('setEditingState');
      expect(control.methodCalls, expectedMethodCalls);

      connection.close();
      expectedMethodCalls.add('detach');
      expect(control.methodCalls, expectedMethodCalls);
    });

    test('does not interfere with platform text input', () {
      final FakeTextInputControl control = FakeTextInputControl();
      TextInput.setInputControl(control);

      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      TextInput.attach(client, const TextInputConfiguration());

      fakeTextChannel.outgoingCalls.clear();

      fakeTextChannel.incoming!(
        MethodCall('TextInputClient.updateEditingState', <dynamic>[
          1,
          TextEditingValue.empty.toJSON(),
        ]),
      );

      expect(client.latestMethodCall, 'updateEditingValue');
      expect(control.methodCalls, <String>['attach', 'setEditingState']);
      expect(fakeTextChannel.outgoingCalls, isEmpty);
    });

    test('both input controls receive requests', () async {
      final FakeTextInputControl control = FakeTextInputControl();
      TextInput.setInputControl(control);

      const TextInputConfiguration textConfig = TextInputConfiguration();
      const TextInputConfiguration numberConfig = TextInputConfiguration(
        inputType: TextInputType.number,
      );
      const TextInputConfiguration multilineConfig = TextInputConfiguration(
        inputType: TextInputType.multiline,
      );
      const TextInputConfiguration noneConfig = TextInputConfiguration(
        inputType: TextInputType.none,
      );

      // Test for https://github.com/flutter/flutter/issues/125875.
      // When there's a custom text input control installed on Web, the platform text
      // input control receives TextInputType.none and isMultiline flag.
      // isMultiline flag is set to true when the input type is multiline.
      // isMultiline flag is set to false when the input type is not multiline.
      final Map<String, dynamic> noneIsMultilineFalseJson = noneConfig.toJson();
      final Map<String, dynamic> noneInputType =
          noneIsMultilineFalseJson['inputType'] as Map<String, dynamic>;
      if (kIsWeb) {
        noneInputType['isMultiline'] = false;
      }
      final Map<String, dynamic> noneIsMultilineTrueJson = noneConfig.toJson();
      final Map<String, dynamic> noneInputType1 =
          noneIsMultilineTrueJson['inputType'] as Map<String, dynamic>;
      if (kIsWeb) {
        noneInputType1['isMultiline'] = true;
      }

      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      final TextInputConnection connection = TextInput.attach(client, textConfig);

      final List<String> expectedMethodCalls = <String>['attach'];
      expect(control.methodCalls, expectedMethodCalls);
      expect(control.inputType, TextInputType.text);
      fakeTextChannel.validateOutgoingMethodCalls(<MethodCall>[
        // When there's a custom text input control installed, the platform text
        // input control receives TextInputType.none with isMultiline flag
        MethodCall('TextInput.setClient', <dynamic>[1, noneIsMultilineFalseJson]),
      ]);

      connection.show();
      expectedMethodCalls.add('show');
      expect(control.methodCalls, expectedMethodCalls);
      expect(fakeTextChannel.outgoingCalls.length, 2);
      expect(fakeTextChannel.outgoingCalls.last.method, 'TextInput.show');

      connection.updateConfig(numberConfig);
      expectedMethodCalls.add('updateConfig');
      expect(control.methodCalls, expectedMethodCalls);
      expect(control.inputType, TextInputType.number);
      expect(fakeTextChannel.outgoingCalls.length, 3);
      fakeTextChannel.validateOutgoingMethodCalls(<MethodCall>[
        // When there's a custom text input control installed, the platform text
        // input control receives TextInputType.none with isMultiline flag
        MethodCall('TextInput.setClient', <dynamic>[1, noneIsMultilineFalseJson]),
        const MethodCall('TextInput.show'),
        MethodCall('TextInput.updateConfig', noneIsMultilineFalseJson),
      ]);

      connection.updateConfig(multilineConfig);
      expectedMethodCalls.add('updateConfig');
      expect(control.methodCalls, expectedMethodCalls);
      expect(control.inputType, TextInputType.multiline);
      expect(fakeTextChannel.outgoingCalls.length, 4);

      fakeTextChannel.validateOutgoingMethodCalls(<MethodCall>[
        // When there's a custom text input control installed, the platform text
        // input control receives TextInputType.none with isMultiline flag
        MethodCall('TextInput.setClient', <dynamic>[1, noneIsMultilineFalseJson]),
        const MethodCall('TextInput.show'),
        MethodCall('TextInput.updateConfig', noneIsMultilineFalseJson),
        MethodCall('TextInput.updateConfig', noneIsMultilineTrueJson),
      ]);

      connection.setComposingRect(Rect.zero);
      expectedMethodCalls.add('setComposingRect');
      expect(control.methodCalls, expectedMethodCalls);
      expect(fakeTextChannel.outgoingCalls.length, 5);
      expect(fakeTextChannel.outgoingCalls.last.method, 'TextInput.setMarkedTextRect');

      connection.setCaretRect(Rect.zero);
      expectedMethodCalls.add('setCaretRect');
      expect(control.methodCalls, expectedMethodCalls);
      expect(fakeTextChannel.outgoingCalls.length, 6);
      expect(fakeTextChannel.outgoingCalls.last.method, 'TextInput.setCaretRect');

      connection.setEditableSizeAndTransform(Size.zero, Matrix4.identity());
      expectedMethodCalls.add('setEditableSizeAndTransform');
      expect(control.methodCalls, expectedMethodCalls);
      expect(fakeTextChannel.outgoingCalls.length, 7);
      expect(fakeTextChannel.outgoingCalls.last.method, 'TextInput.setEditableSizeAndTransform');

      connection.setSelectionRects(const <SelectionRect>[
        SelectionRect(position: 1, bounds: Rect.fromLTWH(2, 3, 4, 5), direction: TextDirection.rtl),
      ]);
      expectedMethodCalls.add('setSelectionRects');
      expect(control.methodCalls, expectedMethodCalls);
      expect(fakeTextChannel.outgoingCalls.length, 8);
      expect(fakeTextChannel.outgoingCalls.last.arguments, const TypeMatcher<List<List<num>>>());
      final List<List<num>> sentList =
          fakeTextChannel.outgoingCalls.last.arguments as List<List<num>>;
      expect(sentList.length, 1);
      expect(sentList[0].length, 6);
      expect(sentList[0][0], 2); // left
      expect(sentList[0][1], 3); // top
      expect(sentList[0][2], 4); // width
      expect(sentList[0][3], 5); // height
      expect(sentList[0][4], 1); // position
      expect(sentList[0][5], TextDirection.rtl.index); // direction
      expect(fakeTextChannel.outgoingCalls.last.method, 'TextInput.setSelectionRects');

      connection.setStyle(
        fontFamily: null,
        fontSize: null,
        fontWeight: null,
        textDirection: TextDirection.ltr,
        textAlign: TextAlign.left,
      );
      expectedMethodCalls.add('setStyle');
      expect(control.methodCalls, expectedMethodCalls);
      expect(fakeTextChannel.outgoingCalls.length, 9);
      expect(fakeTextChannel.outgoingCalls.last.method, 'TextInput.setStyle');

      connection.close();
      expectedMethodCalls.add('detach');
      expect(control.methodCalls, expectedMethodCalls);
      expect(fakeTextChannel.outgoingCalls.length, 10);
      expect(fakeTextChannel.outgoingCalls.last.method, 'TextInput.clearClient');

      expectedMethodCalls.add('hide');
      final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();
      await binding.runAsync(() async {});
      await expectLater(control.methodCalls, expectedMethodCalls);
      expect(fakeTextChannel.outgoingCalls.length, 11);
      expect(fakeTextChannel.outgoingCalls.last.method, 'TextInput.hide');
    });

    test('the platform input control receives isMultiline true on attach', () async {
      final FakeTextInputControl control = FakeTextInputControl();
      TextInput.setInputControl(control);

      const TextInputConfiguration multilineConfig = TextInputConfiguration(
        inputType: TextInputType.multiline,
      );
      const TextInputConfiguration noneConfig = TextInputConfiguration(
        inputType: TextInputType.none,
      );

      // Test for https://github.com/flutter/flutter/issues/125875.
      // When there's a custom text input control installed, the platform text
      // input control receives TextInputType.none and isMultiline flag.
      // isMultiline flag is set to true when the input type is multiline.
      // isMultiline flag is set to false when the input type is not multiline.
      final Map<String, dynamic> noneIsMultilineTrueJson = noneConfig.toJson();
      final Map<String, dynamic> noneInputType =
          noneIsMultilineTrueJson['inputType'] as Map<String, dynamic>;
      noneInputType['isMultiline'] = true;

      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      TextInput.attach(client, multilineConfig);

      final List<String> expectedMethodCalls = <String>['attach'];
      expect(control.methodCalls, expectedMethodCalls);
      expect(control.inputType, TextInputType.multiline);
      fakeTextChannel.validateOutgoingMethodCalls(<MethodCall>[
        // When there's a custom text input control installed, the platform text
        // input control receives TextInputType.none with isMultiline flag
        MethodCall('TextInput.setClient', <dynamic>[1, noneIsMultilineTrueJson]),
      ]);
    }, skip: !kIsWeb); // https://github.com/flutter/flutter/issues/125875

    test('notifies changes to the attached client', () async {
      final FakeTextInputControl control = FakeTextInputControl();
      TextInput.setInputControl(control);

      final FakeTextInputClient client = FakeTextInputClient(TextEditingValue.empty);
      final TextInputConnection connection = TextInput.attach(
        client,
        const TextInputConfiguration(),
      );

      TextInput.setInputControl(null);
      expect(client.latestMethodCall, 'didChangeInputControl');

      connection.show();
      expect(client.latestMethodCall, 'didChangeInputControl');
    });
  });
}

class FakeTextInputClient with TextInputClient {
  FakeTextInputClient(this.currentTextEditingValue);

  String latestMethodCall = '';
  final List<String> performedSelectors = <String>[];
  late Map<String, dynamic>? latestPrivateCommandData;

  @override
  TextEditingValue currentTextEditingValue;

  @override
  AutofillScope? get currentAutofillScope => null;

  @override
  void performAction(TextInputAction action) {
    latestMethodCall = 'performAction';
  }

  @override
  void performPrivateCommand(String action, Map<String, dynamic>? data) {
    latestMethodCall = 'performPrivateCommand';
    latestPrivateCommandData = data;
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
  void showToolbar() {
    latestMethodCall = 'showToolbar';
  }

  TextInputConfiguration get configuration => const TextInputConfiguration();

  @override
  void didChangeInputControl(TextInputControl? oldControl, TextInputControl? newControl) {
    latestMethodCall = 'didChangeInputControl';
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
  void performSelector(String selectorName) {
    latestMethodCall = 'performSelector';
    performedSelectors.add(selectorName);
  }
}

class FakeTextInputControl with TextInputControl {
  final List<String> methodCalls = <String>[];
  late TextInputType inputType;

  @override
  void attach(TextInputClient client, TextInputConfiguration configuration) {
    methodCalls.add('attach');
    inputType = configuration.inputType;
  }

  @override
  void detach(TextInputClient client) {
    methodCalls.add('detach');
  }

  @override
  void setEditingState(TextEditingValue value) {
    methodCalls.add('setEditingState');
  }

  @override
  void updateConfig(TextInputConfiguration configuration) {
    methodCalls.add('updateConfig');
    inputType = configuration.inputType;
  }

  @override
  void show() {
    methodCalls.add('show');
  }

  @override
  void hide() {
    methodCalls.add('hide');
  }

  @override
  void setComposingRect(Rect rect) {
    methodCalls.add('setComposingRect');
  }

  @override
  void setCaretRect(Rect rect) {
    methodCalls.add('setCaretRect');
  }

  @override
  void setEditableSizeAndTransform(Size editableBoxSize, Matrix4 transform) {
    methodCalls.add('setEditableSizeAndTransform');
  }

  @override
  void setSelectionRects(List<SelectionRect> selectionRects) {
    methodCalls.add('setSelectionRects');
  }

  @override
  void setStyle({
    required String? fontFamily,
    required double? fontSize,
    required FontWeight? fontWeight,
    required TextDirection textDirection,
    required TextAlign textAlign,
  }) {
    methodCalls.add('setStyle');
  }

  @override
  void finishAutofillContext({bool shouldSave = true}) {
    methodCalls.add('finishAutofillContext');
  }

  @override
  void requestAutofill() {
    methodCalls.add('requestAutofill');
  }
}
