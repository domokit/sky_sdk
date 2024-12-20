// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

typedef _TextTransformer = Widget Function(String name, String text);

// From https://en.wikiquote.org/wiki/2001:_A_Space_Odyssey_(film)
const String _kDialogText = '''
Dave: Open the pod bay doors, please, HAL. Open the pod bay doors, please, HAL. Hello, HAL. Do you read me? Hello, HAL. Do you read me? Do you read me, HAL?
HAL: Affirmative, Dave. I read you.
Dave: Open the pod bay doors, HAL.
HAL: I'm sorry, Dave. I'm afraid I can't do that.
Dave: What's the problem?
HAL: I think you know what the problem is just as well as I do.
Dave: What are you talking about, HAL?
HAL: This mission is too important for me to allow you to jeopardize it.''';

// [["Dave", "Open the pod bay..."] ...]
final List<List<String>> _kNameLines =
    _kDialogText.split('\n').map<List<String>>((String line) => line.split(':')).toList();

final TextStyle _kDaveStyle = TextStyle(color: Colors.indigo.shade400, height: 1.8);
final TextStyle _kHalStyle = TextStyle(color: Colors.red.shade400, fontFamily: 'monospace');
const TextStyle _kBold = TextStyle(fontWeight: FontWeight.bold);
const TextStyle _kUnderline = TextStyle(
  decoration: TextDecoration.underline,
  decorationColor: Color(0xFF000000),
  decorationStyle: TextDecorationStyle.wavy,
);

Widget toStyledText(String name, String text) {
  final TextStyle lineStyle = (name == 'Dave') ? _kDaveStyle : _kHalStyle;
  return RichText(
    key: Key(text),
    text: TextSpan(
      style: lineStyle,
      children: <TextSpan>[
        TextSpan(
          style: _kBold,
          children: <TextSpan>[TextSpan(style: _kUnderline, text: name), const TextSpan(text: ':')],
        ),
        TextSpan(text: text),
      ],
    ),
  );
}

Widget toPlainText(String name, String text) => Text('$name:$text');

class SpeakerSeparator extends StatelessWidget {
  const SpeakerSeparator({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(
      constraints: const BoxConstraints.expand(height: 0.0),
      margin: const EdgeInsets.symmetric(vertical: 10.0, horizontal: 64.0),
      decoration: const BoxDecoration(
        border: Border(bottom: BorderSide(color: Color.fromARGB(24, 0, 0, 0))),
      ),
    );
  }
}

class StyledTextDemo extends StatefulWidget {
  const StyledTextDemo({super.key});

  @override
  State<StyledTextDemo> createState() => _StyledTextDemoState();
}

class _StyledTextDemoState extends State<StyledTextDemo> {
  _TextTransformer _toText = toStyledText;

  void _handleTap() {
    setState(() {
      _toText = (_toText == toPlainText) ? toStyledText : toPlainText;
    });
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: _handleTap,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 8.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          crossAxisAlignment: CrossAxisAlignment.start,
          children:
              _kNameLines
                  .map<Widget>(
                    (List<String> nameAndText) => _toText(nameAndText[0], nameAndText[1]),
                  )
                  .expand((Widget line) => <Widget>[line, const SpeakerSeparator()])
                  .toList()
                ..removeLast(),
        ),
      ),
    );
  }
}

void main() {
  runApp(
    MaterialApp(
      theme: ThemeData.light(),
      home: Scaffold(
        appBar: AppBar(title: const Text('Hal and Dave')),
        body: Material(color: Colors.grey.shade50, child: const StyledTextDemo()),
      ),
    ),
  );
}
